# Native ROS service candidate

This service requires the ROS-enabled SDK wheel from
[SDK PR #86](https://github.com/adamo-tech/adamo/pull/86), including main PR #85.
It replaces the service-specific websocket/CDR/camera machinery with native SDK
control and recording setup. The independent NV12 shared-memory stereo path
retains its existing resolution, frame rate, encoder and bitrate settings.
Wrist-camera tracks remain disabled.

The existing `adamo_xr_relay.py` still uses rosbridge. Keep it and rosbridge
running: this change removes the video service's websocket dependency only.
Calibration, arm mapping, engagement guards and commanded-EE behavior remain
owned by that relay. A later rclpy migration must preserve those behaviors.

`interfaces/CommandedEEPose.msg` matches the installed Anvil Jazzy interface:
Header, Pose, float64 gripper. `record_ros` retains the original ROS headers and
the existing `joint_states`, `commanded_ee_left`, `commanded_ee_right` recording
keys plus the SDK capture timestamp. Missing sensor values expire after one
second instead of being repeated forever.

## Qualification and deployment

1. Record the live service path and wheel hash. Back up the script and exact
   wheel; do not copy credentials into this repository.
2. Install the candidate wheel in a separate environment. Run SDK
   `tests/ros/dds_probe.py` and `tests/ros/robot_probe.py` in disposable Jazzy
   containers with the installed Anvil interfaces. These scripts restrict
   themselves to domains 179/181 and `/adamo_sdk_test`.
3. Install the qualified wheel into the service's existing venv; copy this
   script and its sibling `interfaces` directory to the active service path.
   Retain `ROS_DISTRO=jazzy`, the production `ROS_DOMAIN_ID`, and the existing
   credentials environment file. `ADAMO_ROS_NAMESPACE` prefixes all routes;
   leave it empty for production. Do not use the old control-only prefix.
4. Restart only `adamo-video` and inspect its status, source/encoder health and
   recording subscriptions. Native extensions require a process restart.
   Do not reboot the host, restart the XR relay or send synthetic arm commands.
5. Roll back by reinstalling the saved wheel with `pip --force-reinstall
   --no-deps`, restoring the saved service script, and restarting `adamo-video`.

The SDK parity matrix lists unresolved lifecycle, ownership, encoded-camera
passthrough and matched-latency checks. A healthy service startup alone does not
establish full legacy parity.
