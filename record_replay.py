# record_replay.py
# ─────────────────────────────────────────────────────────────────────────────
# Record and replay robot arm movements for repeatable tasks (e.g. plug USB)
#
# USAGE:
#   python record_replay.py record    ← you physically guide the arm; saves moves
#   python record_replay.py replay    ← plays back the saved recording
#   python record_replay.py replay recordings/my_recording.json   ← specific file
#
# HOW TO RECORD:
#   1. Run:  python record_replay.py record
#   2. The arm initialises and enables — do NOT touch it yet.
#   3. You'll be prompted to press Enter to start recording.
#   4. Use the console menu to jog joints step by step, OR
#      put the arm in "teach mode" (if your Piper supports it) and move it by hand.
#   5. Press 's' + Enter after each key waypoint to save that position.
#   6. Press 'q' + Enter when done — file is saved to recordings/
#
# HOW TO REPLAY:
#   1. Run:  python record_replay.py replay
#   2. The arm initialises, raises to safe home, then executes each waypoint.
#   3. Speed is set to REPLAY_SPEED_PCT (default 15% — safe and slow).
#
# RECORDING FORMAT (JSON — easy to hand-edit):
#   { "waypoints": [
#       { "joints": [j1,j2,j3,j4,j5,j6], "gripper_m": 0.0, "wait": 3.0, "label": "approach" },
#       ...
#   ] }
# ─────────────────────────────────────────────────────────────────────────────

import ctypes, logging, time, math, json, sys, os
from datetime import datetime
from pathlib import Path

logging.getLogger('usb').setLevel(logging.WARNING)

# ── libusb (Windows) ──────────────────────────────────────────────────────────
DLL_PATH = r"C:\Users\pv\robot-arm\venv\Lib\site-packages\libusb\_platform\windows\x86_64\libusb-1.0.dll"
ctypes.CDLL(DLL_PATH)
import usb.backend.libusb1
usb.backend.libusb1.get_backend(find_library=lambda x: DLL_PATH)

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel

# ── Config ────────────────────────────────────────────────────────────────────
RECORD_SPEED_PCT  = 15   # speed during recording jog moves
REPLAY_SPEED_PCT  = 15   # speed during replay (keep low for safety)
HOME_WAIT         = 4.0  # seconds to wait after moving to home
RECORDINGS_DIR    = Path(__file__).parent / "recordings"

JOINT_LIMITS = [
    (-math.radians(154),  math.radians(154)),
    ( 0.0,                math.radians(195)),
    (-math.radians(175),  0.0),
    (-math.radians(102),  math.radians(102)),
    (-math.radians(75),   math.radians(75)),
    (-math.pi,            math.pi),
]
GRIPPER_OPEN_M = 0.07   # 70 mm = fully open

def deg(d):     return math.radians(d)
def clamp(v, lo, hi): return max(lo, min(hi, v))
def clamp_joints(j):
    return [clamp(a, lo, hi) for a, (lo, hi) in zip(j, JOINT_LIMITS)]

# ── Robot init (identical pattern to working demo_sequence.py) ────────────────
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
    raise RuntimeError("Could not read joint angles")

def move_to(robot, target, label, wait=3.0):
    target = clamp_joints(target)
    print(f"\n  → {label}")
    print(f"    joints: {[round(v,3) for v in target]}")
    robot.move_j(target)
    time.sleep(wait)
    actual = robot.get_joint_angles()
    if actual:
        print(f"    actual: {[round(v,3) for v in actual.msg]}")
    return list(actual.msg) if actual else target

def safe_home(robot, current):
    """Raise to a safe upright home before any sequence."""
    home = current.copy()
    home[1] = deg(30)
    home[2] = deg(-30)
    return move_to(robot, home, "Moving to safe home", wait=HOME_WAIT)

# ── RECORDING ─────────────────────────────────────────────────────────────────
JSTEP = deg(3)   # 3° per jog press
GSTEP = 0.005    # 5 mm per gripper press

def record_session(robot, effector):
    RECORDINGS_DIR.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path  = RECORDINGS_DIR / f"recording_{timestamp}.json"

    joints   = get_joints(robot)
    gripper_m = 0.0
    waypoints = []

    print("""
╔════════════════════════════════════════════════╗
║           RECORD MODE — Jog the arm            ║
╠════════════════════════════════════════════════╣
║  Joint jog:                                    ║
║    1/q → J1 +/-    2/w → J2 +/-               ║
║    3/e → J3 +/-    4/r → J4 +/-               ║
║    5/t → J5 +/-    6/y → J6 +/-               ║
║  Gripper:                                      ║
║    o → open (+5 mm)   c → close (-5 mm)        ║
║  Recording:                                    ║
║    s → save current position as waypoint       ║
║    q alone → quit and save recording           ║
╚════════════════════════════════════════════════╝
""")
    print("  Current joints:", [round(v,3) for v in joints])
    input("\nPress Enter when ready to start recording...\n")

    JOG_MAP = {
        '1': (0, +JSTEP), 'q': (0, -JSTEP),
        '2': (1, +JSTEP), 'w': (1, -JSTEP),
        '3': (2, +JSTEP), 'e': (2, -JSTEP),
        '4': (3, +JSTEP), 'r': (3, -JSTEP),
        '5': (4, +JSTEP), 't': (4, -JSTEP),
        '6': (5, +JSTEP), 'y': (5, -JSTEP),
    }

    wp_count = 0
    while True:
        cmd = input("jog> ").strip().lower()

        if cmd == 'q':
            break

        elif cmd == 's':
            label = input(f"  Label for waypoint {wp_count} (e.g. 'approach', 'insert'): ").strip()
            wait  = input(f"  Wait time after this waypoint during replay (default 3.0 s): ").strip()
            try:    wait = float(wait)
            except: wait = 3.0
            wp = {
                "joints":    [round(v, 5) for v in joints],
                "gripper_m": round(gripper_m, 5),
                "wait":      wait,
                "label":     label or f"waypoint_{wp_count}",
            }
            waypoints.append(wp)
            wp_count += 1
            print(f"  ✓ Saved waypoint {wp_count}: {wp['label']}")

        elif cmd == 'o':
            gripper_m = clamp(gripper_m + GSTEP, 0.0, GRIPPER_OPEN_M)
            effector.move_gripper_m(value=gripper_m, force=1.0)
            print(f"  Gripper → {gripper_m*1000:.1f} mm")

        elif cmd == 'c':
            gripper_m = clamp(gripper_m - GSTEP, 0.0, GRIPPER_OPEN_M)
            effector.move_gripper_m(value=gripper_m, force=1.0)
            print(f"  Gripper → {gripper_m*1000:.1f} mm")

        elif cmd in JOG_MAP:
            ji, delta = JOG_MAP[cmd]
            joints[ji] = clamp(joints[ji] + delta, *JOINT_LIMITS[ji])
            joints = clamp_joints(joints)
            robot.move_j(joints)
            time.sleep(0.5)
            actual = robot.get_joint_angles()
            if actual:
                joints = list(actual.msg)
            print(f"  J{ji+1} → {round(joints[ji], 3)} rad  |  all: {[round(v,3) for v in joints]}")

        else:
            print("  Unknown command. Use 1-6, q-y, o, c, s, q.")

    if not waypoints:
        print("No waypoints recorded — nothing saved.")
        return None

    data = {"waypoints": waypoints}
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n✓ Recording saved → {out_path}  ({len(waypoints)} waypoints)")
    return out_path

# ── REPLAY ────────────────────────────────────────────────────────────────────
def replay_session(robot, effector, recording_path: Path):
    with open(recording_path) as f:
        data = json.load(f)
    waypoints = data["waypoints"]
    print(f"\n▶ Replaying {len(waypoints)} waypoints from: {recording_path}")
    print("   Speed:", REPLAY_SPEED_PCT, "%")
    input("   Press Enter to start replay (Ctrl+C to abort)...\n")

    for i, wp in enumerate(waypoints):
        joints    = wp["joints"]
        gripper_m = wp.get("gripper_m", 0.0)
        wait      = wp.get("wait", 3.0)
        label     = wp.get("label", f"waypoint_{i}")

        print(f"\n[{i+1}/{len(waypoints)}] {label}")
        robot.move_j(clamp_joints(joints))
        time.sleep(wait)
        actual = robot.get_joint_angles()
        if actual:
            print(f"  actual joints: {[round(v,3) for v in actual.msg]}")

        effector.move_gripper_m(value=gripper_m, force=1.0)
        time.sleep(0.5)
        print(f"  gripper: {gripper_m*1000:.1f} mm")

    print("\n✓ Replay complete.")

def find_latest_recording() -> Path | None:
    if not RECORDINGS_DIR.exists():
        return None
    files = sorted(RECORDINGS_DIR.glob("recording_*.json"))
    return files[-1] if files else None

# ── MAIN ──────────────────────────────────────────────────────────────────────
def main():
    mode = sys.argv[1].lower() if len(sys.argv) > 1 else "help"
    if mode not in ("record", "replay"):
        print(__doc__ or "")
        print("Usage:  python record_replay.py record")
        print("        python record_replay.py replay [optional_file.json]")
        sys.exit(0)

    print("Initialising robot...")
    robot = init_robot()

    print("Initialising gripper...")
    effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    time.sleep(0.5)
    effector.reset_gripper()
    time.sleep(2.0)
    print("✓ Gripper ready")

    robot.set_speed_percent(RECORD_SPEED_PCT if mode == "record" else REPLAY_SPEED_PCT)

    current = get_joints(robot)
    print(f"Start joints: {[round(v,3) for v in current]}")

    # Always go home first
    current = safe_home(robot, current)

    try:
        if mode == "record":
            record_session(robot, effector)

        elif mode == "replay":
            if len(sys.argv) > 2:
                path = Path(sys.argv[2])
            else:
                path = find_latest_recording()
                if path is None:
                    print("No recordings found in ./recordings/ — run 'record' first.")
                    sys.exit(1)
                print(f"Using latest recording: {path}")
            replay_session(robot, effector, path)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        # Return to safe home
        print("\nReturning to home position...")
        current = get_joints(robot)
        safe_home(robot, current)
        effector.move_gripper_m(value=0.0, force=1.0)
        time.sleep(0.5)
        robot.disable()
        robot.disconnect()
        print("✓ Disabled and disconnected.")

if __name__ == "__main__":
    main()