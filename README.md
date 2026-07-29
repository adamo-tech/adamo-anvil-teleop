# Adamo Teleop — robot workcell package

Streams the anvil's ROS cameras over the adamo SDK and relays WebXR
teleoperation into the arms. Two systemd services, one installer.

## Install
```
./install.sh
```
**Interactive**. Answer two prompts (adamo API key, robot name).
You can get your API key from operate.adamohq.com.
Your organization is derived from the API key; it is never entered or stored.

## What it installs
- **adamo-video** (the *adamo-service*) — publishes
  `/cam_{wrist_l,wrist_r,chest}/image_raw/compressed` over the adamo SDK (native
  ros2dds ingestion, VA-API H.264), registers the robot on the adamo mesh, and
  runs the SDK's control fan-out that exposes the operator's controllers as
  native ROS topics (`/controller/*`). This is the only process that touches the
  adamo SDK / zenoh / your API key.
- **adamo-xr-relay** — a **plain ROS node** (rosbridge only; no adamo import, 
  no API key) that subscribes to the fanned `/controller/{left,right}`
  (+`/joy`), applies the WebXR→robot transform + gripper mapping, and publishes
  engage-gated `/commanded_ee_{left,right}`. Grip engages (deadman: the arm holds
  when you release); it anchors to the robot's live `/ee_pose_*` so re-engaging
  never jumps. The rosbridge connection **self-heals** — if the workcell
  container/rosbridge restarts, it reconnects and re-subscribes automatically, no
  relay restart needed. Because it's ordinary ROS, you can read it and re-map it
  to a different robot's command topic/message.

## Requirements (installed automatically)
`python3-venv`; GStreamer plugins base/good/bad; a VA-API driver for hardware
H.264 (Intel → `intel-media-va-driver-non-free`, AMD → `mesa-va-drivers`);
`adamo>=0.4.50`, `numpy`, `websockets`. The workcell must already expose
**rosbridge on `ws://localhost:9090`** and publish the three compressed camera
topics. Camera topic names differing? Edit the `CAMERAS` tuple in
`adamo_video.py` and the topic subscribe in the relay.

## Operate / debug
```
journalctl -u adamo-video -f
journalctl -u adamo-xr-relay -f
```
Safe dry-run (logs commanded poses, publishes nothing → arms stay still):
```
sudo systemctl stop adamo-xr-relay
~/adamo-teleop/venv/bin/python ~/adamo-teleop/adamo_xr_relay.py --dry-run
```

## Tune / reconfigure
Re-run `./install.sh`, or edit `~/adamo-teleop/.env`
(`ADAMO_ROBOT_NAME`, `ANVIL_ARM_TYPE`, `ROS_DOMAIN_ID`, `RUST_LOG`) then
`sudo systemctl restart adamo-video adamo-xr-relay`.
Relay motion knobs: `--scale`, `--rate`, `--max-ee-step`
(`adamo_xr_relay.py --help`).
