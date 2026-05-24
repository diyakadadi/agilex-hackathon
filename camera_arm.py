import ctypes
import logging
import threading
import time
import math
import numpy as np
import cv2

logging.getLogger('usb').setLevel(logging.WARNING)

# ── libusb for CAN ──────────────────────────────────────────
dll_path = r"C:\Users\pv\robot-arm\venv\Lib\site-packages\libusb\_platform\windows\x86_64\libusb-1.0.dll"
ctypes.CDLL(dll_path)
import usb.backend.libusb1
usb.backend.libusb1.get_backend(find_library=lambda x: dll_path)

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel
from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat

# ── Config ──────────────────────────────────────────────────
FRAME_W, FRAME_H = 640, 480
CENTER_X = FRAME_W // 2

# Detection: if an object is closer than this (mm) in the centre zone → react
TRIGGER_DEPTH_MM   = 600
# Centre zone: horizontal band where we watch for objects
CENTRE_ZONE_X      = (220, 420)
CENTRE_ZONE_Y      = (160, 320)

# Arm rotation amounts
DODGE_ANGLE_DEG    = 60    # how far to swing away
RETURN_DELAY_S     = 3.0   # how long to wait before returning to centre

# ── Shared state ────────────────────────────────────────────
state = {
    "object_detected": False,
    "object_x": 0,          # pixel X of detected object
    "depth_mm": 0,
    "arm_busy": False,
    "current_j1": 0.0,      # base joint angle
    "dodged": False,
}
lock = threading.Lock()

# ── ARM SETUP ───────────────────────────────────────────────
def setup_arm():
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        interface="gs_usb",
        channel=0,
        bitrate=1000000,
    )
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    time.sleep(0.5)
    robot.reset()
    time.sleep(1.0)
    robot.set_motion_mode('j')
    time.sleep(0.3)
    print("Enabling arm...")
    for _ in range(200):
        if robot.enable():
            break
        time.sleep(0.05)
    robot.set_speed_percent(25)
    time.sleep(0.5)

    # Read starting position
    ja = None
    for _ in range(50):
        ja = robot.get_joint_angles()
        if ja is not None:
            break
        time.sleep(0.05)

    home = list(ja.msg)
    print("Arm ready. Home:", [round(v, 3) for v in home])
    return robot, home

# ── ARM CONTROL THREAD ──────────────────────────────────────
def arm_thread(robot, home):
    while True:
        time.sleep(0.1)

        with lock:
            detected  = state["object_detected"]
            obj_x     = state["object_x"]
            arm_busy  = state["arm_busy"]
            dodged    = state["dodged"]

        if detected and not arm_busy and not dodged:
            # Object is on the LEFT side of frame → dodge RIGHT, and vice versa
            if obj_x < CENTER_X:
                dodge_rad = math.radians(DODGE_ANGLE_DEG)   # swing right
                direction = "RIGHT"
            else:
                dodge_rad = -math.radians(DODGE_ANGLE_DEG)  # swing left
                direction = "LEFT"

            print(f"⚠ Object detected at x={obj_x}! Dodging {direction}...")

            with lock:
                state["arm_busy"] = True
                state["dodged"] = True

            target = home.copy()
            target[0] = home[0] + dodge_rad
            robot.move_j(target)
            time.sleep(2.5)

            with lock:
                state["arm_busy"] = False

        elif not detected and dodged and not arm_busy:
            # Object gone — return to home after delay
            print("✓ Object cleared. Returning home...")
            with lock:
                state["arm_busy"] = True

            time.sleep(RETURN_DELAY_S)
            robot.move_j(home)
            time.sleep(2.5)

            with lock:
                state["arm_busy"] = False
                state["dodged"] = False

# ── CAMERA LOOP ─────────────────────────────────────────────
def run_camera():
    pipeline = Pipeline()
    config = Config()
    config.enable_stream(OBSensorType.COLOR_SENSOR, FRAME_W, FRAME_H, OBFormat.RGB888, 30)
    config.enable_stream(OBSensorType.DEPTH_SENSOR, FRAME_W, FRAME_H, OBFormat.Y16,   30)
    pipeline.start(config)

    print("Camera started. Press Q to quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames(100)
            if frames is None:
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if color_frame is None or depth_frame is None:
                continue

            color_data = np.asanyarray(color_frame.get_data())
            color_img  = cv2.cvtColor(color_data, cv2.COLOR_RGB2BGR)
            depth_data = np.asanyarray(depth_frame.get_data()).astype(np.float32)

            # Sample depth in the centre zone
            zx1, zx2 = CENTRE_ZONE_X
            zy1, zy2 = CENTRE_ZONE_Y
            zone_depth = depth_data[zy1:zy2, zx1:zx2]
            valid      = zone_depth[zone_depth > 0]
            min_depth  = float(np.percentile(valid, 10)) if len(valid) > 0 else 9999

            # Detect closest object in zone
            object_detected = min_depth < TRIGGER_DEPTH_MM

            # Find X position of closest point
            obj_x = CENTER_X
            if object_detected:
                mask = (zone_depth > 0) & (zone_depth < TRIGGER_DEPTH_MM)
                if mask.any():
                    ys, xs = np.where(mask)
                    obj_x = int(np.mean(xs)) + zx1

            with lock:
                state["object_detected"] = object_detected
                state["object_x"]        = obj_x
                state["depth_mm"]        = int(min_depth)

            # ── Draw HUD ────────────────────────────────────
            # Centre zone box
            color = (0, 0, 255) if object_detected else (0, 255, 0)
            cv2.rectangle(color_img, (zx1, zy1), (zx2, zy2), color, 2)

            # Depth reading
            cv2.putText(color_img, f"Depth: {int(min_depth)}mm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # Status
            with lock:
                status = "DODGING" if state["arm_busy"] else ("DETECTED" if object_detected else "CLEAR")
            cv2.putText(color_img, f"Status: {status}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # Object marker
            if object_detected:
                cv2.circle(color_img, (obj_x, (zy1+zy2)//2), 10, (0, 0, 255), -1)
                cv2.putText(color_img, f"Trigger <{TRIGGER_DEPTH_MM}mm",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("Orbbec + Piper Arm", color_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# ── MAIN ────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Setting up arm...")
    robot, home = setup_arm()

    # Start arm control in background thread
    t = threading.Thread(target=arm_thread, args=(robot, home), daemon=True)
    t.start()

    try:
        run_camera()          # camera runs on main thread (OpenCV needs this)
    finally:
        print("Shutting down...")
        robot.disable()
        robot.disconnect()
        print("Done.")