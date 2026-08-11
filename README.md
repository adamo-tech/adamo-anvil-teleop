# Adamo Teleop for Anvil

Use this package to connect an Anvil robot workcell to Adamo for camera streaming
and WebXR teleoperation.

## Before you start

You need:

- An Anvil workcell with Docker, systemd, and ROS running
- rosbridge available at `ws://localhost:9090`
- An Adamo API key from [operate.adamohq.com](https://operate.adamohq.com)
- `sudo` access on the workcell

The workcell should publish these compressed camera topics:

```text
/cam_wrist_l/image_raw/compressed
/cam_wrist_r/image_raw/compressed
/cam_chest/image_raw/compressed
```

## Install

Clone this repository on the Anvil workcell and run the installer as your normal
workcell user:

```bash
./install.sh
```

The installer asks for:

- Adamo API key
- Robot name as it should appear in Adamo
- Anvil arm type
- ROS domain ID
- ROS distribution

Press Enter to retain an existing value when reinstalling. Your Adamo
organization is determined automatically from the API key.

The installer adds the required system packages and Python dependencies,
creates `~/adamo-teleop`, and enables the Adamo services. It also installs the
compatibility support required for the current Anvil ROS release.

## Verify the installation

Check that the services are running:

```bash
systemctl status adamo-video adamo-xr-relay
systemctl status adamo-anvil-container-patch.timer
```

Then sign in to [operate.adamohq.com](https://operate.adamohq.com) and confirm
that the configured robot is online and its camera feeds are available.

Follow service logs when troubleshooting:

```bash
journalctl -u adamo-video -f
journalctl -u adamo-xr-relay -f
journalctl -u adamo-anvil-container-patch -f
```

## Reconfigure or update

Re-run the installer to update the installed files or change configuration:

```bash
./install.sh
```

Existing values are offered as defaults. Configuration is stored in
`~/adamo-teleop/.env`.

After manually editing that file, restart the Adamo services:

```bash
sudo systemctl restart adamo-video adamo-xr-relay
```

Common settings are:

```text
ADAMO_ROBOT_NAME
ANVIL_ARM_TYPE
ROS_DOMAIN_ID
ROS_DISTRO
RUST_LOG
```

## Safe relay test

To inspect incoming controller commands without publishing arm targets, stop the
managed relay and run it in dry-run mode:

```bash
sudo systemctl stop adamo-xr-relay
~/adamo-teleop/venv/bin/python ~/adamo-teleop/adamo_xr_relay.py --dry-run
```

Press Ctrl+C when finished, then restart the service:

```bash
sudo systemctl restart adamo-xr-relay
```

Run the relay with `--help` to view the available customer-facing tuning
options:

```bash
~/adamo-teleop/venv/bin/python ~/adamo-teleop/adamo_xr_relay.py --help
```
