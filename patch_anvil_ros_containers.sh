#!/usr/bin/env bash
# Apply the temporary Anvil rehoming handoff fix to running Compose ros2 services.
# Safe to run repeatedly: containers are restarted only when their patch differs.
set -euo pipefail

PATCH_DIR="${ANVIL_PATCH_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/anvil_ros_patches}"
RESETTER_PATCH="$PATCH_DIR/arms_resetter_node.py"
PUBLISHER_PATCH="$PATCH_DIR/quest_publisher"
RESETTER_TARGET="/workspace/install/control/lib/control/arms_resetter_node.py"
PUBLISHER_TARGET="/workspace/install/quest_teleop/lib/quest_teleop/publisher"
RESTART_CONTAINERS="${ANVIL_PATCH_RESTART_CONTAINERS:-1}"
IMAGE_PATTERN="${ANVIL_PATCH_IMAGE_PATTERN:-*:1.2.4}"

say()  { printf '[anvil-patch] %s\n' "$*"; }
warn() { printf '[anvil-patch] WARNING: %s\n' "$*" >&2; }

command -v docker >/dev/null 2>&1 || { say "Docker is unavailable; nothing to patch."; exit 0; }
[ -r "$RESETTER_PATCH" ] || { warn "Missing $RESETTER_PATCH"; exit 1; }
[ -r "$PUBLISHER_PATCH" ] || { warn "Missing $PUBLISHER_PATCH"; exit 1; }

file_hash() { sha256sum "$1" | awk '{print $1}'; }
container_hash() {
  docker exec "$1" sha256sum "$2" 2>/dev/null | awk '{print $1}'
}

resetter_hash="$(file_hash "$RESETTER_PATCH")"
publisher_hash="$(file_hash "$PUBLISHER_PATCH")"
found=0
failed=0

# Anvil's loader is a Compose project and consistently labels this service ros2.
# Verify both known install paths as an additional guard before changing it.
while IFS= read -r container; do
  [ -n "$container" ] || continue
  image="$(docker inspect --format '{{.Config.Image}}' "$container" 2>/dev/null || true)"
  case "$image" in
    $IMAGE_PATTERN) ;;
    *)
      say "Skipping $container ($image); patch is validated for $IMAGE_PATTERN only."
      continue
      ;;
  esac
  if ! docker exec "$container" test -f "$RESETTER_TARGET" 2>/dev/null ||
     ! docker exec "$container" test -f "$PUBLISHER_TARGET" 2>/dev/null; then
    continue
  fi
  found=$((found + 1))

  current_resetter="$(container_hash "$container" "$RESETTER_TARGET" || true)"
  current_publisher="$(container_hash "$container" "$PUBLISHER_TARGET" || true)"
  if [ "$current_resetter" = "$resetter_hash" ] &&
     [ "$current_publisher" = "$publisher_hash" ]; then
    say "$container already has the rehoming patch."
    continue
  fi

  say "Patching $container..."
  if ! docker cp "$RESETTER_PATCH" "$container:$RESETTER_TARGET" ||
     ! docker cp "$PUBLISHER_PATCH" "$container:$PUBLISHER_TARGET"; then
    warn "Could not write $container (its paths may be read-only bind mounts)."
    failed=$((failed + 1))
    continue
  fi

  current_resetter="$(container_hash "$container" "$RESETTER_TARGET" || true)"
  current_publisher="$(container_hash "$container" "$PUBLISHER_TARGET" || true)"
  if [ "$current_resetter" != "$resetter_hash" ] ||
     [ "$current_publisher" != "$publisher_hash" ]; then
    warn "Hash verification failed for $container."
    failed=$((failed + 1))
    continue
  fi

  if [ "$RESTART_CONTAINERS" = "1" ]; then
    say "Restarting $container so its Python entrypoints reload..."
    docker restart "$container" >/dev/null || { warn "Restart failed for $container."; failed=$((failed + 1)); }
  else
    warn "$container is patched on disk but needs a restart before the fix is active."
  fi
done < <(docker ps --filter label=com.docker.compose.service=ros2 --format '{{.ID}}')

if [ "$found" -eq 0 ]; then
  say "No running Compose ros2 containers found."
fi
[ "$failed" -eq 0 ] || exit 1
