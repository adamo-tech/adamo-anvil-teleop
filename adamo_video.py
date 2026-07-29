#!/usr/bin/env python3
"""Adamo video publisher for the OpenArm v2 anvil — 3 ROS camera tracks.

Publishes the workcell's three ROS cameras over the adamo SDK using native
ros2dds ingestion (no OAK / shared-memory shims). Each camera is a
sensor_msgs/CompressedImage (MJPEG); the SDK decodes it (jpegdec) and
re-encodes to H.264 with VA-API for low-latency transport.

Requires an adamo wheel built WITH ros2dds (>= 0.4.50 on PyPI):
    python -c "from adamo._native import ros_native_supported; print(ros_native_supported())"
must print True.

Environment (set by the systemd unit / .env):
  ADAMO_API_KEY     ak_...    org auth (required)
  ADAMO_ROBOT_NAME  openarm   participant name shown in the fleet
  ROS_DISTRO        jazzy     REQUIRED — if unset the ros2dds bridge assumes
                              'humble' and discovers no topics
  ROS_DOMAIN_ID     0         must match the workcell's ROS graph
  RUST_LOG          adamo_network=info   without this the track/ROS diagnostics
                              are silent and a healthy run can look like a stall
"""

import os
import sys

import adamo
from adamo._native import ros_native_supported

if not ros_native_supported():
    sys.exit("This adamo wheel lacks ros2dds. Install the ros2dds wheel: "
             "pip install 'adamo==0.4.50'")
if not os.environ.get("ROS_DISTRO"):
    sys.exit("Set ROS_DISTRO=jazzy (unset -> the ros2dds bridge assumes 'humble' "
             "and discovers no ROS topics).")

API_KEY = os.environ.get("ADAMO_API_KEY") or sys.exit("Set ADAMO_API_KEY=ak_...")
ROBOT_NAME = os.environ.get("ADAMO_ROBOT_NAME", "openarm")

robot = adamo.Robot(api_key=API_KEY, name=ROBOT_NAME, protocol="quic")

# Enable the operator-control fan-out (zenoh control envelopes -> native ROS
# topics like /controller/*), which the WebXR relay consumes. This is REDUNDANT
# while ROS cameras are attached below (any ros_* video track already starts
# the embedded ros2dds bridge + fan-out), but we call it explicitly so the
# control plane keeps working even if the camera list is ever emptied.
robot.enable_ros_control()

# The workcell's three ROS cameras (sensor_msgs/CompressedImage, MJPEG).
CAMERAS = (
    ("wrist_left",  "/cam_wrist_l/image_raw/compressed"),
    ("wrist_right", "/cam_wrist_r/image_raw/compressed"),
    ("chest",       "/cam_chest/image_raw/compressed"),
)

for track_name, topic in CAMERAS:
    robot.attach_video(
        track_name,
        ros=topic,
        ros_format="ros_compressed_image",
        encoder="vah264enc",
        bitrate_kbps=4000,
        fps=30,
        keyframe_distance=1.0,  # VA-API has no GDR; the -1.0 default -> ~17s keyframes -> slow joins
        allow_missing=True,     # tolerate a camera not being up yet
    )

print(f"[adamo_video] publishing {', '.join(n for n, _ in CAMERAS)} as '{ROBOT_NAME}'",
      flush=True)
robot.run()  # blocks: drives ROS ingest + VA-API encode + transport
