 # mqtt_sub.py  v9 FIXED
# ─────────────────────────────────────────────────────────────────────────────
# FIXES: robot/arm now accepts {"angles": [...]} format
# ─────────────────────────────────────────────────────────────────────────────

import ctypes, logging, time, math, json, csv, os, threading, ssl, queue
from datetime import datetime, timezone
from pathlib import Path
import paho.mqtt.client as mqtt

logging.getLogger('usb').setLevel(logging.WARNING)

dll_path = r"C:\Users\pv\robot-arm\venv\Lib\site-packages\libusb\_platform\windows\x86_64\libusb-1.0.dll"
ctypes.CDLL(dll_path)
import usb.backend.libusb1
usb.backend.libusb1.get_backend(find_library=lambda x: dll_path)

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel

# ── EMQX broker ───────────────────────────────────────────────────────────────
BROKER_HOST = "m"
BROKER_PORT = 8084
MQTT_USER   = " "
MQTT_PASS   = " "

TOPICS = [
    ("robot/drive",   0),
    ("robot/arm",      0),
    ("arm/joystick",  0),
    ("robot/gripper", 0),
    ("drive/dpad",    0),
]

# ── Joint limits (Piper URDF, radians) ───────────────────────────────────────
JOINT_LIMITS = [
    (-math.radians(154),  math.radians(154)),
    ( 0.0,                math.radians(195)),
    (-math.radians(175),  0.0),
    (-math.radians(102),  math.radians(102)),
    (-math.radians(75),   math.radians(75)),
    (-math.pi,            math.pi),
]
JOYSTICK_STEP  = math.radians(3)   # 3° per joystick tick
GRIPPER_OPEN_M = 0.07              # Piper full open = 70 mm
GRIPPER_MIN, GRIPPER_MAX = 0.0, 1.0
SPEED_PCT = 20

# ── Episode / dataset paths (LeRobot-compatible layout) ──────────────────────
DATASET_ROOT = Path(__file__).parent / "dataset"

# ── Helpers ───────────────────────────────────────────────────────────────────
def _clamp(v, lo, hi): return max(lo, min(hi, v))

def _clamp_joints(j):
    return [_clamp(a, lo, hi) for a, (lo, hi) in zip(j, JOINT_LIMITS)]

def _deg(d): return math.radians(d)

# ── Robot init ────────────────────────────────────────────────────────────────
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
    arm.set_speed_percent(SPEED_PCT)
    status = arm.get_arm_status()
    if status:
        print(f"ctrl_mode: {status.msg.ctrl_mode}")
    else:
        print("ctrl_mode: (status not yet available)")
    return arm

print("Initialising robot...")
robot        = init_robot() 
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
time.sleep(0.5)

# Gripper must be reset before it responds to move commands
print("Resetting gripper...")
result = end_effector.reset_gripper()
print(f"  reset_gripper() → {result}")
time.sleep(2.0)   # gripper homing takes ~1-2 seconds

# Check gripper status
gs = end_effector.get_gripper_status()
if gs:
    print(f"  gripper status: value={gs.msg.value}  force={gs.msg.force}  mode={gs.msg.mode}")
else:
    print("  gripper status: None — gripper not communicating")

# Verify it responds
print("Testing gripper open...")
end_effector.move_gripper_m(value=0.03, force=1.0)
time.sleep(2.0)
print("Testing gripper close...")
end_effector.move_gripper_m(value=0.0, force=1.0)
time.sleep(2.0)
print("✓ Gripper ready")

time.sleep(0.5)
print("✓ Gripper effector ready")

# ── Seed current joints ───────────────────────────────────────────────────────
_current_joints = [0.0] * 6
for _ in range(50):
    ja = robot.get_joint_angles()
    if ja is not None:
        _current_joints = list(ja.msg)
        break
    time.sleep(0.05)
print(f"Start joints (rad): {[round(v,3) for v in _current_joints]}")

# ── Startup sequence: raise arm straight up ───────────────────────────────────
def startup_raise():
    print("\n── Startup: raising arm to home position ──")
    cur = list(_current_joints)

    # Step 1: raise shoulder + elbow to safe upright position
    up = cur.copy()
    up[1] = _deg(30)    # J2 shoulder up 30°
    up[2] = _deg(-30)   # J3 elbow up
    print(f"  → Raising shoulder/elbow  target={[round(v,3) for v in up]}")
    robot.move_j(up)
    time.sleep(4.0)

    actual = robot.get_joint_angles()
    if actual:
        _current_joints[:] = list(actual.msg)
        print(f"  ✓ Home position reached: {[round(v,3) for v in _current_joints]}")
    print("── Startup complete ──\n")

startup_raise()

# ── Command queue ─────────────────────────────────────────────────────────────
_cmd_queue = queue.Queue()

# ── LeRobot-style episode recorder ────────────────────────────────────────────
class EpisodeRecorder:
    """Records robot state + actions to parquet files in LeRobot dataset format."""
    def __init__(self, root: Path, fps: int = 20):
        self.root    = root
        self.fps     = fps
        self._frames = []
        self._ep_idx = self._next_episode_index()
        self._frame_idx = 0
        self._recording = False
        self._lock = threading.Lock()
        (root / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
        (root / "meta").mkdir(parents=True, exist_ok=True)
        print(f"✓ EpisodeRecorder ready — next episode: {self._ep_idx:06d}")

    def _next_episode_index(self) -> int:
        chunk = self.root / "data" / "chunk-000"
        if not chunk.exists():
            return 0
        existing = sorted(chunk.glob("episode_*.parquet"))
        return len(existing)

    def start_episode(self):
        with self._lock:
            self._frames    = []
            self._frame_idx = 0
            self._recording = True
        print(f"▶ Episode {self._ep_idx:06d} started")

    def record_frame(self, joints, gripper_m, action_joints, action_gripper,
                     linear=0.0, angular=0.0, topic=""):
        """Call this at ~20 Hz to log state + action."""
        if not self._recording:
            return
        ts = time.time()
        frame = {
            "observation.state":          joints + [gripper_m],
            "action":                     action_joints + [action_gripper],
            "observation.drive.linear":   linear,
            "observation.drive.angular":  angular,
            "timestamp":                  ts,
            "frame_index":                self._frame_idx,
            "episode_index":              self._ep_idx,
            "index":                      self._ep_idx * 10000 + self._frame_idx,
            "task_index":                 0,
            "topic":                      topic,
        }
        with self._lock:
            self._frames.append(frame)
            self._frame_idx += 1

    def stop_episode(self) -> Path | None:
        with self._lock:
            self._recording = False
            frames = list(self._frames)

        if not frames:
            print("⚠ No frames recorded — episode not saved")
            return None

        try:
            import pyarrow as pa
            import pyarrow.parquet as pq
        except ImportError:
            print("⚠ pyarrow not installed — saving as CSV instead")
            return self._save_csv(frames)

        obs_state  = pa.array([f["observation.state"]  for f in frames],
                               type=pa.list_(pa.float32()))
        action     = pa.array([f["action"]              for f in frames],
                               type=pa.list_(pa.float32()))
        table = pa.table({
            "observation.state":         obs_state,
            "action":                    action,
            "observation.drive.linear":  pa.array([f["observation.drive.linear"]  for f in frames], pa.float32()),
            "observation.drive.angular": pa.array([f["observation.drive.angular"] for f in frames], pa.float32()),
            "timestamp":                 pa.array([f["timestamp"]    for f in frames], pa.float64()),
            "frame_index":               pa.array([f["frame_index"]  for f in frames], pa.int64()),
            "episode_index":             pa.array([f["episode_index"]for f in frames], pa.int64()),
            "index":                     pa.array([f["index"]        for f in frames], pa.int64()),
            "task_index":                pa.array([f["task_index"]   for f in frames], pa.int64()),
        })

        out = self.root / "data" / "chunk-000" / f"episode_{self._ep_idx:06d}.parquet"
        pq.write_table(table, out)
        print(f"✓ Episode {self._ep_idx:06d} saved → {out}  ({len(frames)} frames)")
        self._ep_idx += 1
        self._update_meta()
        return out

    def _save_csv(self, frames) -> Path:
        out = self.root / "data" / "chunk-000" / f"episode_{self._ep_idx:06d}.csv"
        with open(out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=frames[0].keys())
            w.writeheader()
            w.writerows(frames)
        print(f"✓ Episode {self._ep_idx:06d} saved as CSV → {out}")
        self._ep_idx += 1
        return out

    def _update_meta(self):
        info = {
            "codebase_version": "v2.1",
            "robot_type": "piper",
            "total_episodes": self._ep_idx,
            "fps": self.fps,
            "features": {
                "observation.state": {"dtype": "float32", "shape": [7],
                    "names": ["j1","j2","j3","j4","j5","j6","gripper"]},
                "action": {"dtype": "float32", "shape": [7],
                    "names": ["j1","j2","j3","j4","j5","j6","gripper"]},
            }
        }
        with open(self.root / "meta" / "info.json", "w") as f:
            json.dump(info, f, indent=2)

recorder = EpisodeRecorder(DATASET_ROOT)

# ── CSV log ───────────────────────────────────────────────────────────────────
LOG_FILE   = Path(__file__).parent / "robot_log.csv"
LOG_FIELDS = ["timestamp","topic","payload_raw",
              "j1","j2","j3","j4","j5","j6",
              "linear","angular","gripper_value","event"]
_log_lock  = threading.Lock()

def log_row(topic, raw, joints=None, linear=None, angular=None,
            gripper=None, event=""):
    row = {f: "" for f in LOG_FIELDS}
    row.update({"timestamp": datetime.now().isoformat(timespec="milliseconds"),
                "topic": topic, "payload_raw": raw, "event": event})
    if joints:
        for i, v in enumerate(joints[:6]):
            row[f"j{i+1}"] = round(v, 5)
    if linear  is not None: row["linear"]        = linear
    if angular is not None: row["angular"]       = angular
    if gripper is not None: row["gripper_value"] = gripper
    exists = LOG_FILE.exists()
    with _log_lock:
        with open(LOG_FILE, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=LOG_FIELDS)
            if not exists:
                w.writeheader()
            w.writerow(row)

# ── Payload parsers ───────────────────────────────────────────────────────────
def _parse_gripper(raw: str):
    s = raw.strip()
    if s.lower().startswith("g "):
        try: return _clamp(float(s[2:]), GRIPPER_MIN, GRIPPER_MAX)
        except ValueError: return None
    try: return _clamp(float(s), GRIPPER_MIN, GRIPPER_MAX)
    except ValueError: pass
    try: return _clamp(float(json.loads(s)["value"]), GRIPPER_MIN, GRIPPER_MAX)
    except Exception: return None

def _parse_text_arm_command(raw: str):
    """
    Parse text-based arm commands:
      s <angle>    → shoulder (J2)
      e <angle>    → elbow (J3)
      b <angle>    → base/yaw (J1)
      w <angle>    → wrist pitch (J4)
      r <angle>    → rotate/roll (J5)
    All angles in degrees.
    """
    parts = raw.strip().split()
    if len(parts) != 2:
        return None, None
    
    cmd, val_str = parts[0].lower(), parts[1]
    try:
        angle_deg = float(val_str)
        angle_rad = math.radians(angle_deg)
    except ValueError:
        return None, None
    
    return cmd, angle_rad

# ── State tracking ────────────────────────────────────────────────────────────
_last_gripper_m  = 0.0
_last_linear     = 0.0
_last_angular    = 0.0

# ── MQTT callbacks ────────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"✓ Connected to EMQX: {BROKER_HOST}:{BROKER_PORT}")
        for topic, qos in TOPICS:
            client.subscribe(topic, qos)
            print(f"  ↳ Subscribed: {topic}")
    else:
        print(f"Connection failed rc={rc}")

def on_disconnect(client, userdata, flags, rc, properties=None):
    print(f"Disconnected rc={rc}")
    log_row("system", "", event=f"disconnect_rc{rc}")

def on_message(client, userdata, msg):
    topic = msg.topic
    raw   = msg.payload.decode("utf-8", errors="replace").strip()
    print(f"\n[RECV thread={threading.current_thread().name}] [{topic}] {raw}")
    
    # Handle text-based arm commands first (before JSON parsing)
    if topic == "robot/arm" and not raw.startswith("{"):
        cmd, angle_rad = _parse_text_arm_command(raw)
        if cmd is not None:
            target = list(_current_joints)
            
            joint_map = {
                "b": 0,  # base/yaw (J1)
                "s": 1,  # shoulder (J2)
                "e": 2,  # elbow (J3)
                "w": 4,  # wrist pitch (J4)
                "r": 5,  # rotate/roll (J5)
            }
            
            if cmd in joint_map:
                ji = joint_map[cmd]
                target[ji] = angle_rad
                target = _clamp_joints(target)
                print(f"  Text arm cmd '{cmd}' → J{ji+1}={math.degrees(angle_rad):.1f}°")
                log_row(topic, raw, joints=target, event=f"arm_text_{cmd}")
                _cmd_queue.put(("move_j", target, 1.0, raw))
                return
        
        print(f"  ✗ Unrecognized text command: '{raw}'")
        log_row(topic, raw, event="bad_text_arm_cmd")
        return
    
    if topic == "robot/gripper":
        value = _parse_gripper(raw)
        if value is None:
            print(f"  ✗ Cannot parse: '{raw}'")
            log_row(topic, raw, event="parse_error")
            return
        metres = value * GRIPPER_OPEN_M
        print(f"  Gripper → {value:.3f}  ({metres*1000:.1f} mm)")
        log_row(topic, raw, gripper=value, event="gripper_cmd")
        _cmd_queue.put(("gripper", metres, value, raw))
        return

    try:
        data = json.loads(raw)
    except json.JSONDecodeError as e:
        print(f"  ✗ Bad JSON: {e}")
        log_row(topic, raw, event=f"bad_json:{e}")
        return

    if topic == "robot/drive":
        linear  = _clamp(float(data.get("linear",  0.0)), -1.0, 1.0)
        angular = _clamp(float(data.get("angular", 0.0)), -1.0, 1.0)
        print(f"  Drive → linear={linear:.3f}  angular={angular:.3f}")
        log_row(topic, raw, linear=linear, angular=angular, event="drive_cmd")
        _cmd_queue.put(("drive", linear, angular, raw))

    elif topic == "arm/joystick":
        jx = _clamp(float(data.get("x", 0.0)), -1.0, 1.0)
        jy = _clamp(float(data.get("y", 0.0)), -1.0, 1.0)
        target    = list(_current_joints)
        target[0] += jx * JOYSTICK_STEP
        target[1] += jy * JOYSTICK_STEP
        target     = _clamp_joints(target)
        print(f"  Joystick J1+={jx*JOYSTICK_STEP:.3f} J2+={jy*JOYSTICK_STEP:.3f}")
        log_row(topic, raw, event="joystick_increment")
        _cmd_queue.put(("move_j", target, 0.5, raw))

    elif topic == "robot/arm":
        target = list(_current_joints)
        
        # Try format 1: direct joint angles
        if "angles" in data:
            try:
                angles = data["angles"]
                if isinstance(angles, list) and len(angles) >= 6:
                    target = [float(a) for a in angles[:6]]
                    target = _clamp_joints(target)
                    print(f"  Direct angles → {[round(v,3) for v in target]}")
                    log_row(topic, raw, joints=target, event="arm_angles_cmd")
                    _cmd_queue.put(("move_j", target, 1.0, raw))
                    return  # Success — exit handler
            except (ValueError, TypeError) as e:
                print(f"  ✗ angles parse error: {e}")
        
        # Try format 2: x,y,z cartesian + pitch/yaw (IK fallback)
        if "x" in data and "y" in data and "z" in data:
            try:
                x, y, z = float(data["x"]), float(data["y"]), float(data["z"])
                pitch = float(data.get("pitch", 0.0))
                yaw   = float(data.get("yaw",   0.0))
                
                target[0] = math.atan2(y, x)
                reach     = math.sqrt(x*x + y*y)
                target[1] = _clamp( reach * 0.8, *JOINT_LIMITS[1])
                target[2] = _clamp(-reach * 0.6, *JOINT_LIMITS[2])
                target[4] = _clamp(pitch,         *JOINT_LIMITS[4])
                target[5] = _clamp(yaw,           *JOINT_LIMITS[5])
                target     = _clamp_joints(target)
                print(f"  Cartesian IK → {[round(v,3) for v in target]}")
                log_row(topic, raw, joints=target, event="arm_ik_cmd")
                _cmd_queue.put(("move_j", target, 3.0, raw))
                return
            except (ValueError, TypeError, KeyError) as e:
                print(f"  ✗ x,y,z parse error: {e}")
        
        # Neither format matched
        print(f"  ✗ Expected 'angles' list or 'x','y','z' fields in: {raw}")
        log_row(topic, raw, event="invalid_arm_format")

    elif topic == "drive/dpad":
        action = data.get("action", "").lower()
        DPAD_DRIVE = {"up":(0.3,0.),"down":(-0.3,0.),
                      "left":(0.,0.3),"right":(0.,-0.3)}
        DPAD_ARM   = {"arm_up":(1, JOYSTICK_STEP),
                      "arm_down":(1,-JOYSTICK_STEP)}
        if action in DPAD_DRIVE:
            lin, ang = DPAD_DRIVE[action]
            log_row(topic, raw, linear=lin, angular=ang, event=f"dpad_{action}")
            _cmd_queue.put(("drive", lin, ang, raw))
        elif action in DPAD_ARM:
            ji, delta = DPAD_ARM[action]
            target = list(_current_joints)
            target[ji] += delta
            target = _clamp_joints(target)
            log_row(topic, raw, event=f"dpad_{action}")
            _cmd_queue.put(("move_j", target, 0.5, raw))
        else:
            print(f"  ✗ Unknown dpad: '{action}'")

# ── MQTT client ───────────────────────────────────────────────────────────────
mqtt_client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id="win10-piper-v9",
    transport="websockets",
)
mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
mqtt_client.tls_set_context(ssl.create_default_context())
mqtt_client.on_connect    = on_connect
mqtt_client.on_message    = on_message
mqtt_client.on_disconnect = on_disconnect

mqtt_client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
mqtt_client.loop_start()
print(f"Connecting to {BROKER_HOST}:{BROKER_PORT} (WSS)…")
print("Recording episode — press Ctrl+C to stop and save.\n")

# ── Start recording episode 0 ─────────────────────────────────────────────────
recorder.start_episode()

# ── MAIN LOOP ─────────────────────────────────────────────────────────────────
try:
    while True:
        try:
            cmd = _cmd_queue.get(timeout=0.05)
        except queue.Empty:
            continue

        print(f"\n⚙ EXECUTING cmd={cmd[0]} on thread={threading.current_thread().name}")

        if cmd[0] == "move_j":
            _, target, wait, raw = cmd
            print(f"  → calling robot.move_j({[round(v,3) for v in target]})")
            try:
                robot.move_j(target)
                print(f"  ✓ move_j returned, sleeping {wait}s")
                time.sleep(wait)
                actual = robot.get_joint_angles()
                print(f"  ✓ get_joint_angles returned: {actual}")
                if actual:
                    aj = list(actual.msg)
                    _current_joints[:] = aj
                    print(f"  ✓ actual={[round(v,3) for v in aj]}")
                    log_row("feedback", "", joints=aj, event="actual_joints")
            except Exception as e:
                print(f"  ✗ move_j EXCEPTION: {e}")

        elif cmd[0] == "gripper":
            _, metres, normalized_value, raw = cmd
            print(f"  → calling end_effector.move_gripper_m({metres:.4f}m = {normalized_value:.2f})")
            
            # Strategy: use higher force for full open/close, lower for gentle positioning
            if normalized_value < 0.15 or normalized_value > 0.85:
                force = 100.0  # Maximum force for full open/close
                print(f"    (full open/close mode, force={force})")
            else:
                force = 1.0    # Gentle grip
                print(f"    (gentle mode, force={force})")
            
            try:
                end_effector.move_gripper_m(value=metres, force=force)
                print(f"  ✓ move_gripper_m returned")
                _last_gripper_m = metres
            except Exception as e:
                print(f"  ✗ gripper EXCEPTION: {e}")

        elif cmd[0] == "drive":
            _, linear, angular, raw = cmd
            print(f"  ✓ drive linear={linear:.3f} angular={angular:.3f}")

        elif cmd[0] == "stop":
            break

except KeyboardInterrupt:
    print("\n── Ctrl+C — saving episode and shutting down ──")

finally:
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

    # Save episode
    recorder.stop_episode()

    # Return arm to safe position
    print("Returning arm to start position...")
    safe = list(_current_joints)
    safe[1] = _deg(30)
    safe[2] = _deg(-30)
    robot.move_j(safe)
    time.sleep(3.0)

    end_effector.move_gripper_m(value=0.0, force=1.0)
    time.sleep(0.5)
    robot.disable()
    robot.disconnect()
    log_row("system", "", event="shutdown")
    print("Arm disabled. Dataset saved. Bye.")
