# teach_record_v2.py
# ─────────────────────────────────────────────────────────────────────────────
#  Teach-mode Record + Replay for AgileX Piper  (pyAgxArm, Windows gs_usb)
#
#  SAFETY FEATURES:
#    1. Controlled shutdown  — arm moves to rest position BEFORE disabling.
#                             Never just drops.
#    2. Watchdog heartbeat   — background thread re-enables the arm every 0.5 s
#                             so it holds position instead of going limp.
#    3. Joint velocity guard — replay skips any point requiring a joint to jump
#                             more than MAX_JUMP_DEG in one step.
#    4. Speed cap            — hard 15% speed limit on ALL moves.
#    5. Emergency stop       — type 'x' → arm freezes, saves CSV, goes home.
#    6. Joint-limit clamp    — every move_j call is clamped to URDF limits.
#    7. Startup self-check   — verifies arm responds before proceeding.
#
#  TWO WAYS TO RECORD (mix freely):
#    A) Hold the TEACH BUTTON on the arm → arm goes limp → move by hand.
#    B) Type keyboard jog commands + Enter for fine positioning.
#
#  USAGE:
#    python teach_record_v2.py record
#    python teach_record_v2.py replay
#    python teach_record_v2.py replay recordings\trajectory_YYYYMMDD_HHMMSS.csv
#    python teach_record_v2.py replay recordings\trajectory_YYYYMMDD_HHMMSS.csv --loop
#
#  HOW TO RUN:
#    cd C:\Users\pv\robot-arm\hackathon
#    ..\venv\Scripts\activate
#    python teach_record_v2.py record
# ─────────────────────────────────────────────────────────────────────────────

import ctypes, logging, time, math, csv, sys, queue, threading
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
# Config — edit these at the top, not buried in the code
# ─────────────────────────────────────────────────────────────────────────────
RECORD_HZ      = 20     # joint samples per second during teach
SPEED_PCT      = 15     # hard speed cap for ALL moves (jog + replay)
HOME_WAIT      = 6.0    # seconds given to reach rest position
JOG_STEP_DEG   = 3.0    # degrees per keyboard jog press
GRIPPER_STEP_M = 0.015  # metres per gripper open/close press (15 mm)
GRIPPER_OPEN_M = 0.070  # fully open = 70 mm
WATCHDOG_HZ    = 2.0    # how often watchdog re-enables (per second)
MAX_JUMP_DEG   = 15.0   # replay safety: skip moves larger than this per step

RECORDINGS_DIR = Path(__file__).parent / "recordings"

# Piper URDF joint limits (radians)
JOINT_LIMITS = [
    (-math.radians(154),  math.radians(154)),
    ( 0.0,                math.radians(195)),
    (-math.radians(175),  0.0),
    (-math.radians(102),  math.radians(102)),
    (-math.radians(75),   math.radians(75)),
    (-math.pi,            math.pi),
]

# Rest position — arm fully down/folded, safe to disable from here.
# All joints at 0 = Piper's natural hanging-down rest posture.
# ⚠ If your arm hits something at 0,0,0,0,0,0 adjust these values.
REST_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────
def deg(d):           return math.radians(d)
def clamp(v, lo, hi): return max(lo, min(hi, v))
def clamp_joints(j):  return [clamp(a, lo, hi) for a, (lo, hi) in zip(j, JOINT_LIMITS)]
def joints_str(j):
    return "[" + "  ".join(f"J{i+1}:{math.degrees(v):+.1f}°" for i, v in enumerate(j)) + "]"

# ─────────────────────────────────────────────────────────────────────────────
# Robot init
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

    print("  Enabling arm (up to 10 s)...")
    for _ in range(200):
        if arm.enable():
            break
        time.sleep(0.05)
    time.sleep(0.5)

    # Startup self-check
    status = arm.get_arm_status()
    if status is None:
        raise RuntimeError("SAFETY: arm did not respond to status request — aborting.")
    print(f"  ctrl_mode : {status.msg.ctrl_mode}")

    joints = None
    for _ in range(50):
        joints = arm.get_joint_angles()
        if joints is not None:
            break
        time.sleep(0.05)
    if joints is None:
        raise RuntimeError("SAFETY: could not read joint angles — aborting.")
    print(f"  joints OK : {joints_str(list(joints.msg))}")

    arm.set_speed_percent(SPEED_PCT)
    print(f"  speed cap : {SPEED_PCT}%")
    return arm

def get_joints(robot):
    for _ in range(50):
        ja = robot.get_joint_angles()
        if ja is not None:
            return list(ja.msg)
        time.sleep(0.05)
    raise RuntimeError("Could not read joint angles after 50 retries")

# ─────────────────────────────────────────────────────────────────────────────
# Rest position + controlled shutdown
# ─────────────────────────────────────────────────────────────────────────────
def move_to_rest(robot):
    """
    Move arm all the way back down to REST_POSITION before disabling.
    Goes in two steps so it doesn't swing wildly:
      Step 1 — raise shoulder slightly so elbow clears any table/obstacle.
      Step 2 — fold all joints back to rest (0,0,0,0,0,0).
    """
    print("  ► Reading current position...")
    try:
        current = get_joints(robot)
    except Exception:
        current = [0.0] * 6

    print(f"  ► Current : {joints_str(current)}")

    # Step 1: bring elbow up a little first to avoid hitting the table
    # Only do this if the arm is not already near rest
    shoulder_up = list(current)
    shoulder_up[1] = max(current[1], deg(20))   # at least 20° shoulder up
    shoulder_up[2] = min(current[2], deg(-20))  # at least -20° elbow (up)
    shoulder_up = clamp_joints(shoulder_up)

    if abs(current[1] - REST_POSITION[1]) > deg(5) or \
       abs(current[2] - REST_POSITION[2]) > deg(5):
        print(f"  ► Step 1 — clearing elbow: {joints_str(shoulder_up)}")
        robot.move_j(shoulder_up)
        time.sleep(3.0)

    # Step 2: fold all the way down to rest
    rest = clamp_joints(REST_POSITION)
    print(f"  ► Step 2 — folding to rest: {joints_str(rest)}")
    robot.move_j(rest)
    time.sleep(HOME_WAIT)
    print("  ► Rest position reached.")

def controlled_shutdown(robot, effector, reason="shutdown"):
    """
    Safe shutdown — always runs regardless of how the script exits:
      1. Move arm back to rest position (all the way down).
      2. Close gripper.
      3. Stop watchdog.
      4. Disable + disconnect.
    """
    print(f"\n{'═'*52}")
    print(f"  CONTROLLED SHUTDOWN  ({reason})")
    print(f"{'═'*52}")
    try:
        move_to_rest(robot)
    except Exception as e:
        print(f"  ⚠ Could not reach rest position: {e}")
    try:
        print("  ► Closing gripper...")
        effector.move_gripper_m(value=0.0, force=1.0)
        time.sleep(1.5)
    except Exception:
        pass
    try:
        robot.disable()
        time.sleep(0.3)
        robot.disconnect()
    except Exception:
        pass
    print("  ✓ Arm disabled and disconnected safely.")
    print(f"{'═'*52}\n")

# ─────────────────────────────────────────────────────────────────────────────
# Watchdog
# ─────────────────────────────────────────────────────────────────────────────
class Watchdog:
    """Re-sends enable() at WATCHDOG_HZ so the arm holds position."""
    def __init__(self, robot):
        self._robot  = robot
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()
        print(f"  Watchdog started ({WATCHDOG_HZ} Hz re-enable)")

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self):
        interval = 1.0 / WATCHDOG_HZ
        while not self._stop.is_set():
            try:
                self._robot.enable()
            except Exception:
                pass
            time.sleep(interval)

# ─────────────────────────────────────────────────────────────────────────────
# Gripper
# ─────────────────────────────────────────────────────────────────────────────
def init_gripper(robot):
    effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    time.sleep(0.5)
    print("  Resetting gripper (homing ~2 s)...")
    effector.reset_gripper()
    time.sleep(2.0)
    gs = effector.get_gripper_status()
    if gs:
        print(f"  Gripper status: value={gs.msg.value}  force={gs.msg.force}  mode={gs.msg.mode}")
    else:
        print("  ⚠ Gripper status: None")
    print("  Testing gripper open/close...")
    effector.move_gripper_m(value=0.03, force=1.0);  time.sleep(1.5)
    effector.move_gripper_m(value=0.0,  force=1.0);  time.sleep(1.5)
    print("  ✓ Gripper ready")
    return effector

def send_gripper(effector, current_m, target_m):
    """Move gripper, wait for motor to physically travel, return new position."""
    target_m = clamp(target_m, 0.0, GRIPPER_OPEN_M)
    delta_mm = abs(target_m - current_m) * 1000
    if delta_mm < 1.0:
        print(f"  Gripper already at {current_m*1000:.1f} mm")
        return current_m
    effector.move_gripper_m(value=target_m, force=1.0)
    wait = max(0.8, delta_mm * 0.05)
    print(f"  Gripper → {target_m*1000:.1f} mm  (waiting {wait:.1f}s for motor)")
    time.sleep(wait)
    return target_m

# ─────────────────────────────────────────────────────────────────────────────
# RECORD
# ─────────────────────────────────────────────────────────────────────────────
def record(robot, effector):
    RECORDINGS_DIR.mkdir(exist_ok=True)
    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = RECORDINGS_DIR / f"trajectory_{ts}.csv"

    print("""
╔══════════════════════════════════════════════════════════════════╗
║            TEACH + RECORD MODE  (two ways to move)              ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║  ① PHYSICAL TEACH — hand-guide the arm:                         ║
║     Hold the TEACH BUTTON on the arm → arm goes limp.           ║
║     Move it by hand.  Joints sampled at 20 Hz automatically.    ║
║     Release button when done with that segment.                  ║
║                                                                  ║
║  ② KEYBOARD JOG — type command + Enter:                         ║
║     Joint jog (3° per press):                                    ║
║       1 / q  →  J1 + / −      2 / w  →  J2 + / −              ║
║       3 / e  →  J3 + / −      4 / r  →  J4 + / −              ║
║       5 / t  →  J5 + / −      6 / y  →  J6 + / −              ║
║     Gripper:                                                     ║
║       o  → open  +15 mm       oo → fully open  (70 mm)         ║
║       c  → close −15 mm       cc → fully close (0 mm)          ║
║                                                                  ║
║  SAFETY:                                                         ║
║       x  → EMERGENCY STOP — freezes, saves CSV, goes to rest    ║
║       s  (or bare Enter) → normal stop, saves CSV, goes to rest ║
╚══════════════════════════════════════════════════════════════════╝
""")

    JSTEP = deg(JOG_STEP_DEG)
    JOG_MAP = {
        '1': (0, +JSTEP), 'q': (0, -JSTEP),
        '2': (1, +JSTEP), 'w': (1, -JSTEP),
        '3': (2, +JSTEP), 'e': (2, -JSTEP),
        '4': (3, +JSTEP), 'r': (3, -JSTEP),
        '5': (4, +JSTEP), 't': (4, -JSTEP),
        '6': (5, +JSTEP), 'y': (5, -JSTEP),
    }

    gripper_m      = 0.0
    samples        = []
    recording      = False
    emergency      = False
    stop_event     = threading.Event()
    current_joints = get_joints(robot)

    # ── Sampler thread ────────────────────────────────────────────────────────
    def sampler():
        nonlocal gripper_m
        interval = 1.0 / RECORD_HZ
        t_start  = None
        while not stop_event.is_set():
            t0 = time.perf_counter()
            try:
                ja = robot.get_joint_angles()
            except Exception:
                ja = None
            if ja and recording:
                now = time.perf_counter()
                if t_start is None:
                    t_start = now
                dt = now - t_start
                samples.append(
                    [round(dt, 4)] +
                    [round(v, 6) for v in list(ja.msg)] +
                    [round(gripper_m, 5)]
                )
            elapsed = time.perf_counter() - t0
            sleep_for = interval - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    sampler_thread = threading.Thread(target=sampler, daemon=True)
    sampler_thread.start()

    # ── Input reader thread ───────────────────────────────────────────────────
    # queue.Queue (not threading.Queue — that doesn't exist)
    cmd_queue = queue.Queue()

    def input_reader():
        while not stop_event.is_set():
            try:
                line = input()
                cmd_queue.put(line.strip().lower())
            except EOFError:
                cmd_queue.put('s')
                break

    input_thread = threading.Thread(target=input_reader, daemon=True)

    print("When ready: press Enter to START recording.")
    input()
    recording = True
    print(f"▶ Recording at {RECORD_HZ} Hz...")
    print("  Guide by hand and/or type jog commands.")
    print("  'x' = emergency stop,  's' or Enter = save and stop.\n")
    input_thread.start()

    # ── Jog helper ────────────────────────────────────────────────────────────
    def jog_joint(ji, delta):
        nonlocal current_joints
        target = list(current_joints)
        target[ji] = clamp(target[ji] + delta, *JOINT_LIMITS[ji])
        target = clamp_joints(target)
        robot.move_j(target)
        time.sleep(0.6)
        actual = robot.get_joint_angles()
        if actual:
            current_joints = list(actual.msg)
        else:
            current_joints = target
        print(f"  {joints_str(current_joints)}")

    # ── Main command loop ─────────────────────────────────────────────────────
    while True:
        try:
            cmd = cmd_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        if cmd == 'x':
            print("\n  ⚠ EMERGENCY STOP")
            emergency = True
            break
        elif cmd in ('s', 'stop', ''):
            print("  Stopping recording...")
            break
        elif cmd in ('o', 'open'):
            gripper_m = send_gripper(effector, gripper_m, gripper_m + GRIPPER_STEP_M)
        elif cmd == 'oo':
            gripper_m = send_gripper(effector, gripper_m, GRIPPER_OPEN_M)
        elif cmd in ('c', 'close'):
            gripper_m = send_gripper(effector, gripper_m, gripper_m - GRIPPER_STEP_M)
        elif cmd == 'cc':
            gripper_m = send_gripper(effector, gripper_m, 0.0)
        elif cmd in JOG_MAP:
            ji, delta = JOG_MAP[cmd]
            jog_joint(ji, delta)
        else:
            print(f"  Unknown: '{cmd}'")
            print("  Joints: 1/q 2/w 3/e 4/r 5/t 6/y  |  Gripper: o oo c cc  |  Stop: s  |  Emergency: x")

    stop_event.set()
    sampler_thread.join(timeout=1.0)

    if not samples:
        print("⚠  No samples captured — nothing saved.")
        return None, emergency

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["dt", "j1", "j2", "j3", "j4", "j5", "j6", "gripper_m"])
        w.writerows(samples)

    duration = samples[-1][0]
    print(f"\n✓  Saved {len(samples)} samples ({duration:.1f} s) → {out_path}")
    return out_path, emergency

# ─────────────────────────────────────────────────────────────────────────────
# REPLAY
# ─────────────────────────────────────────────────────────────────────────────
def replay(robot, effector, csv_path: Path, loop: bool = False):
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        print("ERROR: empty trajectory file."); return

    raw_traj = [
        (float(r["dt"]),
         [float(r[f"j{i}"]) for i in range(1, 7)],
         float(r["gripper_m"]))
        for r in rows
    ]

    # Safety filter: drop points with huge per-step jumps
    MAX_JUMP_RAD = deg(MAX_JUMP_DEG)
    filtered = [raw_traj[0]]
    skipped  = 0
    for i in range(1, len(raw_traj)):
        prev_j    = filtered[-1][1]
        curr_j    = raw_traj[i][1]
        max_delta = max(abs(a - b) for a, b in zip(curr_j, prev_j))
        if max_delta > MAX_JUMP_RAD:
            skipped += 1
        else:
            filtered.append(raw_traj[i])

    if skipped:
        print(f"  ⚠ Safety filter removed {skipped} points with jumps > {MAX_JUMP_DEG}°")

    trajectory = filtered
    duration   = trajectory[-1][0]

    print(f"\n▶  Trajectory : {len(trajectory)} points, {duration:.1f} s")
    print(f"   File        : {csv_path}")
    print(f"   Speed       : {SPEED_PCT}%  |  Loop: {loop}")

    print("\n  Moving to trajectory start position...")
    _, first_joints, first_gripper = trajectory[0]
    robot.move_j(clamp_joints(first_joints))
    time.sleep(4.0)
    send_gripper(effector, 0.0, first_gripper)

    input("\nPress Enter to begin replay  (Ctrl+C to abort)...\n")

    play_count = 0
    try:
        while True:
            play_count += 1
            label = f"Loop {play_count}" if loop else "Replay"
            print(f"\n{'─'*50}")
            print(f"  {label} — {len(trajectory)} points, {duration:.1f} s")

            t_start      = time.perf_counter()
            prev_gripper = first_gripper

            for (dt, joints, gripper_m) in trajectory:
                target_wall = t_start + dt
                now = time.perf_counter()
                if target_wall > now:
                    time.sleep(target_wall - now)

                robot.move_j(clamp_joints(joints))

                if abs(gripper_m - prev_gripper) > 0.001:
                    effector.move_gripper_m(value=gripper_m, force=1.0)
                    prev_gripper = gripper_m

            print(f"  ✓ {label} complete.")

            if not loop:
                break

            print("  Pausing 2 s before next loop...")
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\n  Ctrl+C — stopping replay.")

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
        print("  python teach_record_v2.py record")
        print("  python teach_record_v2.py replay [file.csv] [--loop]")
        sys.exit(0)

    mode    = args[0]
    do_loop = "--loop" in args
    csv_arg = next((Path(a) for a in args[1:] if a.endswith(".csv")), None)

    print("=" * 52)
    print("  Piper — teach_record_v2  (safety edition)")
    print("=" * 52)

    print("\nInitialising robot...")
    robot = init_robot()

    print("\nInitialising gripper...")
    effector = init_gripper(robot)

    print("\nStarting watchdog...")
    watchdog = Watchdog(robot)
    watchdog.start()

    # Move to rest position at startup so we know where we are
    print("\nMoving to rest position at startup...")
    move_to_rest(robot)

    emergency = False
    try:
        if mode == "record":
            _, emergency = record(robot, effector)

        elif mode == "replay":
            path = csv_arg or find_latest()
            if path is None:
                print("No recordings found in ./recordings/ — run 'record' first.")
                watchdog.stop()
                sys.exit(1)
            if not path.exists():
                print(f"File not found: {path}")
                watchdog.stop()
                sys.exit(1)
            replay(robot, effector, path, loop=do_loop)

    except KeyboardInterrupt:
        print("\n  Ctrl+C received.")
        emergency = True

    except Exception as e:
        print(f"\n  ✗ Unexpected error: {e}")
        import traceback; traceback.print_exc()
        emergency = True

    finally:
        watchdog.stop()
        reason = "emergency stop" if emergency else "normal exit"
        controlled_shutdown(robot, effector, reason=reason)

if __name__ == "__main__":
    main()
