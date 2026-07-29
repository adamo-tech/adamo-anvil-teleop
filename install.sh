#!/usr/bin/env bash
#
# Adamo teleop package — one-shot installer.
#
# Sets up two systemd services on a robot workcell:
#   adamo-video      : publishes the workcell's ROS cameras over the adamo SDK
#                      (native ros2dds ingestion) and registers the robot on the
#                      adamo mesh so operators can connect.
#   adamo-xr-relay   : a pure-ROS node (rosbridge only, no adamo/zenoh, no API
#                      key) that consumes the operator's controllers as native
#                      /controller/* topics — fanned out by the adamo-service —
#                      and republishes engage-gated /commanded_ee_* (WebXR->robot
#                      transform + gripper). Self-heals across rosbridge restarts.
#
# Run as your normal user (it calls sudo where needed):   ./install.sh
#
# You provide only your adamo API key + robot name. Your organization is derived
# from the API key at runtime by the SDK — it is never entered or stored here.
#
set -euo pipefail

ADAMO_VERSION="${ADAMO_VERSION:-0.4.50}"     # first wheel with ros2dds (glibc-2.17)
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say()  { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[!]\033[0m %s\n'  "$*" >&2; }
die()  { printf '\033[1;31m[x]\033[0m %s\n'  "$*" >&2; exit 1; }

if [ "${SUDO_USER:-}" ] && [ "${EUID:-$(id -u)}" -eq 0 ]; then RUN_USER="$SUDO_USER"; else RUN_USER="$(id -un)"; fi
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
INSTALL_DIR="${INSTALL_DIR:-$RUN_HOME/adamo-teleop}"
ENV_FILE="$INSTALL_DIR/.env"
SUDO="sudo"; [ "${EUID:-$(id -u)}" -eq 0 ] && SUDO=""
run_as() { if [ "$(id -un)" = "$RUN_USER" ]; then "$@"; else $SUDO -u "$RUN_USER" "$@"; fi; }

[ -f "$SRC_DIR/adamo_video.py" ]    || die "adamo_video.py not found next to install.sh"
[ -f "$SRC_DIR/adamo_xr_relay.py" ] || die "adamo_xr_relay.py not found next to install.sh"
say "Adamo teleop installer  (user=$RUN_USER  dir=$INSTALL_DIR  adamo==$ADAMO_VERSION)"

# --- 1. system prerequisites -------------------------------------------------
say "Updating apt and detecting the GPU…"
$SUDO apt-get update -qq
$SUDO DEBIAN_FRONTEND=noninteractive apt-get install -y -qq pciutils >/dev/null
GPU="$(lspci 2>/dev/null | grep -iE 'vga|3d|display' | head -1 | sed 's/.*: //' || true)"
case "$GPU" in
  *Intel*)              VA_PKG="intel-media-va-driver-non-free" ;;
  *AMD*|*Radeon*|*ATI*) VA_PKG="mesa-va-drivers" ;;
  *NVIDIA*)             VA_PKG=""; warn "NVIDIA GPU: VA-API H.264 encode is unavailable on NVIDIA desktop GPUs (vah264enc will be missing). On a Jetson use the aarch64 wheel." ;;
  *)                    VA_PKG=""; warn "Unrecognized GPU ('${GPU:-none}'); relying on va-driver-all." ;;
esac
say "GPU: ${GPU:-unknown}  ->  VA driver: ${VA_PKG:-va-driver-all}"

say "Installing python venv + GStreamer + VA-API + rosbridge deps…"
# base=appsrc, good=jpegdec, bad=vah264enc(va); va-driver-all+vainfo for VA-API.
$SUDO DEBIAN_FRONTEND=noninteractive apt-get install -y \
  python3-venv \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  va-driver-all vainfo ${VA_PKG:-}
$SUDO usermod -aG render,video "$RUN_USER" 2>/dev/null || true
if command -v gst-inspect-1.0 >/dev/null && gst-inspect-1.0 vah264enc >/dev/null 2>&1; then
  say "vah264enc available (hardware H.264 encode ready)."
else
  warn "vah264enc not found — check 'vainfo'; hardware H.264 may be unavailable."
fi

# --- 2. venv + SDK + relay deps ---------------------------------------------
say "Creating venv and installing adamo==$ADAMO_VERSION + numpy + websockets…"
run_as mkdir -p "$INSTALL_DIR"
run_as python3 -m venv "$INSTALL_DIR/venv"
run_as "$INSTALL_DIR/venv/bin/python" -m pip install -q --upgrade pip
run_as "$INSTALL_DIR/venv/bin/pip" install -q "adamo==$ADAMO_VERSION" numpy websockets
run_as "$INSTALL_DIR/venv/bin/python" - <<'PY'
from adamo._native import ros_native_supported
assert ros_native_supported(), "installed adamo wheel lacks ros2dds"
print("adamo ros2dds support: OK")
PY

# --- 3. the scripts ----------------------------------------------------------
say "Installing adamo_video.py + adamo_xr_relay.py…"
run_as cp "$SRC_DIR/adamo_video.py" "$SRC_DIR/adamo_xr_relay.py" "$INSTALL_DIR/"
# Self-contained zenoh-direct relay, kept as a fallback (needs no fan-out; runs
# even if the video service is down). Not wired to a service by default.
[ -f "$SRC_DIR/adamo_xr_relay_zenoh.py" ] && run_as cp "$SRC_DIR/adamo_xr_relay_zenoh.py" "$INSTALL_DIR/" || true

# --- 4. interactive config (.env) -------------------------------------------
old_key=""; old_name="openarm"; old_arm="openarm_v2"; old_domain="0"; old_distro="jazzy"
if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE" 2>/dev/null || true
  old_key="${ADAMO_API_KEY:-}"; old_name="${ADAMO_ROBOT_NAME:-openarm}"
  old_arm="${ANVIL_ARM_TYPE:-openarm_v2}"; old_domain="${ROS_DOMAIN_ID:-0}"; old_distro="${ROS_DISTRO:-jazzy}"
fi
echo; say "Configuration (press Enter to accept the [default])"
kp="  ADAMO_API_KEY"; [ -n "$old_key" ] && kp="$kp (Enter to keep existing)"
read -rs -p "$kp: " in_key; echo; API_KEY="${in_key:-$old_key}"
[ -n "$API_KEY" ] || die "An API key is required."
read -r -p "  ADAMO_ROBOT_NAME [$old_name]: "  x; ROBOT_NAME="${x:-$old_name}"
read -r -p "  ANVIL_ARM_TYPE   [$old_arm]: "   x; ARM_TYPE="${x:-$old_arm}"
read -r -p "  ROS_DOMAIN_ID    [$old_domain]: " x; ROS_DOMAIN_ID="${x:-$old_domain}"
read -r -p "  ROS_DISTRO       [$old_distro]: " x; ROS_DISTRO="${x:-$old_distro}"

run_as tee "$ENV_FILE" >/dev/null <<EOF
# Adamo teleop configuration. Org is derived from the API key — do not add it.
ADAMO_API_KEY=$API_KEY
ADAMO_ROBOT_NAME=$ROBOT_NAME
ANVIL_ARM_TYPE=$ARM_TYPE
ROS_DISTRO=$ROS_DISTRO
ROS_DOMAIN_ID=$ROS_DOMAIN_ID
# Lower to adamo_network=warn to quiet the once-per-second diagnostics.
RUST_LOG=adamo_network=info
EOF
run_as chmod 600 "$ENV_FILE"

# --- 5. systemd services -----------------------------------------------------
say "Installing systemd services…"
$SUDO tee /etc/systemd/system/adamo-video.service >/dev/null <<EOF
[Unit]
Description=Adamo video publisher (ROS cameras -> adamo SDK)
After=network-online.target docker.service
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
SupplementaryGroups=render video
EnvironmentFile=$ENV_FILE
ExecStart=$INSTALL_DIR/venv/bin/python $INSTALL_DIR/adamo_video.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

# The relay drives the arms; it only publishes while an operator holds the grip
# (deadman), so auto-start is safe — nothing moves until someone engages in VR.
# Runs after the video service — which is the adamo-service: it registers robot
# presence AND runs the ros2dds control fan-out the relay depends on for the
# /controller/* topics — and after docker (rosbridge is in the workcell container).
$SUDO tee /etc/systemd/system/adamo-xr-relay.service >/dev/null <<EOF
[Unit]
Description=Adamo WebXR -> commanded-EE teleop relay
After=network-online.target docker.service adamo-video.service
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
EnvironmentFile=$ENV_FILE
ExecStart=$INSTALL_DIR/venv/bin/python $INSTALL_DIR/adamo_xr_relay.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

$SUDO systemctl daemon-reload
$SUDO systemctl enable adamo-video adamo-xr-relay >/dev/null 2>&1 || true
$SUDO systemctl restart adamo-video adamo-xr-relay
sleep 3
echo; say "Installed. Status:"
$SUDO systemctl --no-pager --lines=0 status adamo-video adamo-xr-relay || true
echo
say "Cameras: wrist_left, wrist_right, chest   Teleop: /commanded_ee_{left,right} (grip to engage)."
say "Logs:    journalctl -u adamo-video -f   |   journalctl -u adamo-xr-relay -f"
say "Safe test: sudo systemctl stop adamo-xr-relay; then run it by hand with --dry-run:"
say "           $INSTALL_DIR/venv/bin/python $INSTALL_DIR/adamo_xr_relay.py --dry-run"
say "Reconfigure: re-run ./install.sh   (or edit $ENV_FILE, then restart the services)."
