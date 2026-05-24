# teach_record_replay.py
# ─────────────────────────────────────────────────────────────────────────────
#  TEACH MODE  →  Record  →  Replay  for AgileX Piper (pyAgxArm, Windows gs_usb)
#
#  IMPORTANT — how teach mode actually works on the Piper:
#    • Teach mode (gravity-compensated backdrive) is triggered by the PHYSICAL
#      BUTTON on the arm.  The firmware ignores all software commands to enter
#      it (confirmed in piper_sdk GitHub issue #11 — button has highest priority).
#    • HOWEVER, get_joint_angles() works perfectly while the button is held.
#    • So the workflow is:
#        1. Run this script.
#        2. When prompted, hold the teach button on the arm — the arm goes limp.
#        3. Physically move the arm through the plug/unplug motion.
#        4. Release the button when done.
#        5. The script saves every joint reading at ~20 Hz as a trajectory CSV.
#        6. Run replay to repeat the motion as many times as you like.
#
#  USAGE:
#    python teach_record_replay.py record            ← teach & record
#    python teach_record_replay.py replay            ← replay latest recording
#    python teach_record_replay.py replay <file.csv> ← replay specific file
#    python teach_record_replay.py replay <file.csv> --loop   ← loop forever
#
#  TRAJECTORY FORMAT (CSV — one row per ~50 ms sample):
#    dt, j1, j2, j3, j4, j5, j6, gripper_m
#    (dt = elapsed seconds since recording started — used to pace replay)
# ─────────────────────────────────────────────────────────────────────────────

import ctypes, logging, time, math, csv, sys, os, threading
from datetime import datetime
from pathlib import Path

logging.getLogger('usb').setLevel(logging.WARNING)

# ── libusb (Windows) ──────────────────────────────────────────────────────────
DLL_PATH = r"C:\Users\pv\robot-arm\venv\Lib\site-packages\libusb\_platform\windows\x86_64\libusb-1.0.dll"
ctypes.CDLL(DLL_PATH)
import usb.backend.libusb1
usb.backend.libusb1.get_backend(find_library=lambda x: DLL_PATH)

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel

# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────
RECORD_HZ        = 20        # samples per second while teaching
REPLAY_SPEED_PCT = 15        # arm speed during replay (keep low — USB insertion is delicate)
HOME_WAIT        = 4.0       # seconds to reach safe home position
RECORDINGS_DIR   = Path(__file__).parent / "recordings"

JOINT_LIMITS = [
    (-math.radians(154),  math.radians(154)),
    ( 0.0,                math.radians(195)),
    (-math.radians(175),  0.0),
    (-math.radians(102),  math.radians(102)),
    (-math.radians(75),   math.radians(75)),
    (-math.pi,            math.pi),
]
GRIPPER_OPEN_M = 0.07   # 70 mm = fully open

def deg(d):           return math.radians(d)
def clamp(v, lo, hi): return max(lo, min(hi, v))
def clamp_joints(j):  return [clamp(a, lo, hi) for a, (lo, hi) in zip(j, JOINT_LIMITS)]

# ─────────────────────────────────────────────────────────────────────────────
# Robot init  (identical to your working demo_sequence.py)
# ─────────────────────────────────────────────────────────────────────────────
def init_robot():
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER, interface="gs_usb", channel=0, bitrate=1_000_000)
    arm = AgxArmFactory.create_arm(cfg)
    arm.connect()
    time.sleep(0.5)
    arm.reset()
    time.sleep(1.5)
    arm.set_motion_mode('j')
    time.sleep(0.5)
    print("Enabling arm...")
    for _ in range(200):
        if arm.enable():
            break
        time.sleep(0.05)
    time.sleep(0.5)
    print(f"ctrl_mode: {arm.get_arm_status().msg.ctrl_mode}")
    return arm

def get_joints(robot):
    for _ in range(50):
        ja = robot.get_joint_angles()
        if ja is not None:
            return list(ja.msg)
        time.sleep(0.05)
    raise RuntimeError("Could not read joint angles after 50 retries")

def move_to(robot, joints, label, wait=3.0):
    joints = clamp_joints(joints)
    print(f"  → {label}  {[round(v,3) for v in joints]}")
    robot.move_j(joints)
    time.sleep(wait)
    actual = robot.get_joint_angles()
    return list(actual.msg) if actual else joints

def safe_home(robot, current):
    home = current.copy()
    home[1] = deg(30)
    home[2] = deg(-30)
    return move_to(robot, home, "Safe home", wait=HOME_WAIT)

# ─────────────────────────────────────────────────────────────────────────────
# RECORD  — sample joints at RECORD_HZ while user holds teach button
# ─────────────────────────────────────────────────────────────────────────────
def record(robot, effector):
    RECORDINGS_DIR.mkdir(exist_ok=True)
    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = RECORDINGS_DIR / f"trajectory_{ts}.csv"

    print("""
╔══════════════════════════════════════════════════════════════╗
║                  TEACH + RECORD MODE                         ║
╠══════════════════════════════════════════════════════════════╣
║  1. Hold the physical TEACH BUTTON on the arm.               ║
║     The arm goes limp — you can now move it freely.          ║
║                                                              ║
║  2. Physically guide the arm through the plug/unplug motion. ║
║     Move slowly and smoothly for best replay quality.        ║
║     The script samples joints at 20 Hz automatically.        ║
║                                                              ║
║  3. For the GRIPPER:                                         ║
║     Press 'o' + Enter → open gripper (to grasp cable)       ║
║     Press 'c' + Enter → close gripper (grip the cable)      ║
║     You can do this while still holding the teach button.    ║
║                                                              ║
║  4. Release the teach button — the arm holds position.       ║
║                                                              ║
║  5. Press Enter in this terminal to STOP recording and save. ║
╚══════════════════════════════════════════════════════════════╝
""")

    gripper_m     = 0.0
    samples       = []
    recording     = False
    stop_event    = threading.Event()

    # ── Background thread: samples joints continuously ────────────────────────
    def sampler():
        nonlocal gripper_m
        interval = 1.0 / RECORD_HZ
        t_start  = None
        while not stop_event.is_set():
            t0 = time.perf_counter()
            ja = robot.get_joint_angles()
            if ja and recording:
                now = time.perf_counter()
                if t_start is None:
                    t_start = now
                dt = now - t_start
                samples.append([round(dt, 4)] + [round(v, 6) for v in list(ja.msg)] + [round(gripper_m, 5)])
            elapsed = time.perf_counter() - t0
            sleep_for = interval - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    sampler_thread = threading.Thread(target=sampler, daemon=True)
    sampler_thread.start()

    # ── Gripper input on main thread ──────────────────────────────────────────
    # We run a non-blocking input loop using a second thread so user can type
    # 'o', 'c', or just Enter to stop, while sampling continues.

    print("When ready: press Enter to START recording, then guide the arm.")
    input()

    recording = True
    print(f"▶ Recording at {RECORD_HZ} Hz — move the arm now...")
    print("  (type 'o' + Enter = open gripper,  'c' + Enter = close,  Enter alone = STOP)\n")

    GRIPPER_STEP = 0.005   # 5 mm
    while True:
        cmd = input("").strip().lower()
        if cmd == 'o':
            gripper_m = clamp(gripper_m + GRIPPER_STEP, 0.0, GRIPPER_OPEN_M)
            effector.move_gripper_m(value=gripper_m, force=1.0)
            print(f"  Gripper → {gripper_m*1000:.1f} mm open")
        elif cmd == 'c':
            gripper_m = clamp(gripper_m - GRIPPER_STEP, 0.0, GRIPPER_OPEN_M)
            effector.move_gripper_m(value=gripper_m, force=1.0)
            print(f"  Gripper → {gripper_m*1000:.1f} mm open")
        else:
            # Any other input (including bare Enter) stops recording
            break

    stop_event.set()
    sampler_thread.join(timeout=1.0)

    if not samples:
        print("⚠ No samples captured — nothing saved.")
        return None

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["dt", "j1", "j2", "j3", "j4", "j5", "j6", "gripper_m"])
        w.writerows(samples)

    duration = samples[-1][0] if samples else 0
    print(f"\n✓ Saved {len(samples)} samples ({duration:.1f} s) → {out_path}")
    return out_path

# ─────────────────────────────────────────────────────────────────────────────
# REPLAY  — play back trajectory, optionally looping
# ─────────────────────────────────────────────────────────────────────────────
def replay(robot, effector, csv_path: Path, loop: bool = False):
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows   = list(reader)

    if not rows:
        print("ERROR: empty trajectory file"); return

    trajectory = []
    for r in rows:
        joints    = [float(r[f"j{i}"]) for i in range(1, 7)]
        gripper_m = float(r["gripper_m"])
        dt        = float(r["dt"])
        trajectory.append((dt, joints, gripper_m))

    duration = trajectory[-1][0]
    print(f"\n▶ Trajectory: {len(trajectory)} points, {duration:.1f} s")
    print(f"   File: {csv_path}")
    print(f"   Speed: {REPLAY_SPEED_PCT}%  |  Loop: {loop}")

    # ── Move to the FIRST position of the recording before we start ──────────
    print("\nMoving to trajectory start position...")
    _, first_joints, first_gripper = trajectory[0]
    move_to(robot, first_joints, "Trajectory start", wait=4.0)
    effector.move_gripper_m(value=first_gripper, force=1.0)
    time.sleep(1.0)

    input("\nPress Enter to begin replay (Ctrl+C to abort)...\n")

    play_count = 0
    try:
        while True:
            play_count += 1
            label = f"Loop {play_count}" if loop else "Replay"
            print(f"\n{'─'*40}")
            print(f"  {label} — {len(trajectory)} moves over {duration:.1f} s")

            t_start    = time.perf_counter()
            prev_dt    = 0.0
            prev_gripper = first_gripper

            for (dt, joints, gripper_m) in trajectory:
                # Wait until this sample's scheduled time
                target_wall = t_start + dt
                now = time.perf_counter()
                if target_wall > now:
                    time.sleep(target_wall - now)

                robot.move_j(clamp_joints(joints))

                # Only send gripper command when it changes (saves CAN bandwidth)
                if abs(gripper_m - prev_gripper) > 0.001:
                    effector.move_gripper_m(value=gripper_m, force=1.0)
                    prev_gripper = gripper_m

            print(f"  ✓ {label} complete.")

            if not loop:
                break

            rest = 2.0
            print(f"  Pausing {rest} s before next loop...")
            time.sleep(rest)

    except KeyboardInterrupt:
        print("\n  Stopped by user.")

# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
def find_latest() -> Path | None:
    if not RECORDINGS_DIR.exists(): return None
    files = sorted(RECORDINGS_DIR.glob("trajectory_*.csv"))
    return files[-1] if files else None

def main():
    args = sys.argv[1:]
    if not args or args[0] not in ("record", "replay"):
        print("Usage:")
        print("  python teach_record_replay.py record")
        print("  python teach_record_replay.py replay [file.csv] [--loop]")
        sys.exit(0)

    mode = args[0]
    do_loop = "--loop" in args

    # Find explicit CSV path if given
    csv_arg = None
    for a in args[1:]:
        if a.endswith(".csv"):
            csv_arg = Path(a)

    print("Initialising robot...")
    robot = init_robot()

    print("Initialising gripper...")
    effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    time.sleep(0.5)
    effector.reset_gripper()
    time.sleep(2.0)

    # Gripper open/close test
    effector.move_gripper_m(value=0.03, force=1.0); time.sleep(1.5)
    effector.move_gripper_m(value=0.0,  force=1.0); time.sleep(1.5)
    print("✓ Gripper ready")

    robot.set_speed_percent(REPLAY_SPEED_PCT)

    current = get_joints(robot)
    print(f"Start joints: {[round(v,3) for v in current]}")

    # Always go to safe home first
    current = safe_home(robot, current)

    try:
        if mode == "record":
            record(robot, effector)

        elif mode == "replay":
            path = csv_arg or find_latest()
            if path is None:
                print("No trajectory files found in ./recordings/ — run 'record' first.")
                sys.exit(1)
            if not path.exists():
                print(f"File not found: {path}")
                sys.exit(1)
            replay(robot, effector, path, loop=do_loop)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        print("\nReturning to safe home...")
        try:
            current = get_joints(robot)
            safe_home(robot, current)
        except Exception:
            pass
        effector.move_gripper_m(value=0.0, force=1.0)
        time.sleep(0.5)
        robot.disable()
        robot.disconnect()
        print("✓ Arm disabled and disconnected.")

if __name__ == "__main__":
    main()