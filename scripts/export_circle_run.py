#!/usr/bin/env python3
import sys
import os
import argparse
from pathlib import Path
import cv2
import numpy as np
from rosbags.highlevel import AnyReader

def main():
    parser = argparse.ArgumentParser(description="Export raw stereo IR camera and IMU data from MCAP bag without ROS")
    parser.add_argument("--bag", required=True, help="Path to bag folder")
    parser.add_argument("--output", required=True, help="Output directory")
    args = parser.parse_args()

    out_dir = Path(args.output)
    cam0_dir = out_dir / "cam0" / "data"
    cam1_dir = out_dir / "cam1" / "data"
    cam0_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir.mkdir(parents=True, exist_ok=True)

    times_file = out_dir / "times.txt"
    imu_file = out_dir / "imu.txt"

    print(f"Reading bag from {args.bag} using rosbags AnyReader...")
    bag_path = Path(args.bag)

    infra1_msgs = {}
    infra2_msgs = {}
    imu_msgs = []

    with AnyReader([bag_path]) as reader:
        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic
            if topic == "/camera/camera/infra1/image_rect_raw":
                msg = reader.deserialize(rawdata, connection.msgtype)
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                infra1_msgs[ts] = msg
            elif topic == "/camera/camera/infra2/image_rect_raw":
                msg = reader.deserialize(rawdata, connection.msgtype)
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                infra2_msgs[ts] = msg
            elif topic in ["/camera/camera/imu", "/imu"]:
                msg = reader.deserialize(rawdata, connection.msgtype)
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
                ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                imu_msgs.append((ts, wx, wy, wz, ax, ay, az))

    print(f"Read {len(infra1_msgs)} infra1 images, {len(infra2_msgs)} infra2 images, {len(imu_msgs)} IMU messages.")

    # Match stereo pairs by nearest timestamp within 5ms
    infra1_ts_sorted = sorted(infra1_msgs.keys())
    stereo_pairs = []
    for ts1 in infra1_ts_sorted:
        # Find nearest infra2 timestamp
        matching_ts2 = None
        min_diff = 0.005 # 5ms
        for ts2 in infra2_msgs.keys():
            diff = abs(ts1 - ts2)
            if diff < min_diff:
                min_diff = diff
                matching_ts2 = ts2
        if matching_ts2 is not None:
            stereo_pairs.append((ts1, matching_ts2))

    print(f"Matched {len(stereo_pairs)} stereo frame pairs.")

    def process_image(msg):
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ["mono8", "8UC1", "y8"]:
            return img_data.reshape((msg.height, msg.width))
        elif msg.encoding in ["bgr8", "rgb8"]:
            img = img_data.reshape((msg.height, msg.width, 3))
            if msg.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            return img_data.reshape((msg.height, msg.width))

    with open(times_file, "w") as times_fd:
        for idx, (ts1, ts2) in enumerate(stereo_pairs):
            msg1 = infra1_msgs[ts1]
            msg2 = infra2_msgs[ts2]

            img1 = process_image(msg1)
            img2 = process_image(msg2)

            filename = f"{ts1:.6f}.png"
            cv2.imwrite(str(cam0_dir / filename), img1)
            cv2.imwrite(str(cam1_dir / filename), img2)
            times_fd.write(f"{ts1:.6f}\n")

    with open(imu_file, "w") as imu_fd:
        for (ts, wx, wy, wz, ax, ay, az) in imu_msgs:
            imu_fd.write(f"{ts:.6f} {wx} {wy} {wz} {ax} {ay} {az}\n")

    # Generate camera.yaml for Stereo IR
    cam_yaml = out_dir / "camera.yaml"
    with open(cam_yaml, "w") as f:
        f.write("""%YAML:1.0
File.version: "1.0"
Camera.type: "PinHole"

# Left Camera (infra1)
Camera1.fx: 426.98403931
Camera1.fy: 426.98403931
Camera1.cx: 430.81121826
Camera1.cy: 238.95848083

Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0

# Right Camera (infra2)
Camera2.fx: 426.98403931
Camera2.fy: 426.98403931
Camera2.cx: 430.81121826
Camera2.cy: 238.95848083

Camera2.k1: 0.0
Camera2.k2: 0.0
Camera2.p1: 0.0
Camera2.p2: 0.0

Camera.width: 848
Camera.height: 480
Camera.fps: 30

# Stereo baseline * fx
Camera.bf: 21.42953682
Camera.b: 0.050186

# Depth threshold: close/far threshold
Stereo.ThDepth: 40.0
Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [1.0, 0.0, 0.0, 0.050186,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0]

Camera.RGB: 0

ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0

WebViewer.Enabled: 1
WebViewer.Port: 8080
WebViewer.StaticRoot: "./web_viewer/dist"
""")
    print("Stereo dataset export complete.")

if __name__ == "__main__":
    main()
