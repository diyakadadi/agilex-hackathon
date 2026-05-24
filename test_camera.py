from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat
import cv2
import numpy as np

pipeline = Pipeline()
config = Config()

# Enable color + depth
config.enable_stream(OBSensorType.COLOR_SENSOR, 640, 480, OBFormat.RGB888, 30)
config.enable_stream(OBSensorType.DEPTH_SENSOR, 640, 480, OBFormat.Y16, 30)

pipeline.start(config)
print("Camera started — press Q to quit")

try:
    while True:
        frames = pipeline.wait_for_frames(100)
        if frames is None:
            continue

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if color_frame:
            color_data = np.asanyarray(color_frame.get_data())
            color_img = cv2.cvtColor(color_data, cv2.COLOR_RGB2BGR)
            cv2.imshow("Color", color_img)

        if depth_frame:
            depth_data = np.asanyarray(depth_frame.get_data())
            depth_vis = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imshow("Depth", depth_colored)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()