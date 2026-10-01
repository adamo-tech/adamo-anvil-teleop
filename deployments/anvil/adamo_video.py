#!/usr/bin/env python3
"""Anvil v1 service: native DDS control/recording plus independent SHM stereo.

Requires a qualified adamo 1.0.1 wheel built with ros2dds and ROS_DISTRO=jazzy.
The application XR relay currently retains its separate rosbridge dependency.
Wrist tracks are intentionally disabled. The existing relay owns calibration,
engage gating and commanded-EE behavior.
"""
import os
from pathlib import Path

import adamo

if not adamo.ros_native_supported():
    raise RuntimeError("Install the qualified ROS-enabled adamo wheel; this build lacks native DDS")
if not os.environ.get("ROS_DISTRO"):
    raise RuntimeError("Set ROS_DISTRO=jazzy before starting the Anvil service")
if os.environ.get("ADAMO_ROS_CONTROL_PREFIX"):
    raise RuntimeError("For an isolated trial, use ADAMO_ROS_NAMESPACE and ROS_DOMAIN_ID; it prefixes all ROS routes")

SENSORS = {
    "/joint_states": "sensor_msgs/msg/JointState",
    "/commanded_ee_left": "anvil_msgs/msg/CommandedEEPose",
    "/commanded_ee_right": "anvil_msgs/msg/CommandedEEPose",
}
robot = adamo.Robot(
    api_key=os.environ["ADAMO_API_KEY"],
    name=os.environ.get("ADAMO_ROBOT_NAME", "openarm"),
    protocol="quic",
    record=adamo.Recording(),
)
ros = robot.enable_ros_control(
    namespace=os.environ.get("ADAMO_ROS_NAMESPACE", ""),
    subscribe_topics=SENSORS,
)
robot.record_ros(
    list(SENSORS),
    definitions={
        "anvil_msgs/msg/CommandedEEPose":
            (Path(__file__).parent / "interfaces" / "CommandedEEPose.msg").read_text(),
    },
    max_age=1.0,
)
robot.attach_video(
    "oak_stereo", shm="camera/oak_stereo_nv12", pixel_format="NV12",
    width=2560, height=2880, fps=30, encoder="vah264enc",
    bitrate_kbps=10000, max_bitrate_kbps=15000, adaptive_bitrate=True,
    keyframe_distance=1.0, stereo=True, allow_missing=True,
)
print(f"[adamo_video] native ROS ready; SHM stereo as {robot.name}; wrist tracks disabled", flush=True)
try:
    robot.run()
finally:
    robot.close()
    ros.close()
