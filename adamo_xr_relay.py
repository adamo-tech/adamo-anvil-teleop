#!/usr/bin/env python3
"""Adamo WebXR -> commanded-EE teleop relay (pure ROS / rosbridge).

This relay lives entirely on the public ROS surface. It talks to a single
rosbridge websocket (localhost:9090) and:

  * SUBSCRIBES to the operator's controller stream as native ROS topics —
    /controller/{left,right} (geometry_msgs/PoseStamped) and
    /controller/{left,right}/joy (sensor_msgs/Joy). These are produced by the
    adamo SDK's ros2dds control fan-out (the "adamo-service", adamo_video.py,
    is an `adamo.Robot()` streaming ROS cameras — which starts the embedded
    ros2dds bridge and thus the fan-out). Subscribing here is what makes the
    bridge route them.
  * ANCHORS to /ee_pose_{left,right} for jump-free engage.
  * PUBLISHES engage-gated /commanded_ee_{left,right} (anvil_msgs/CommandedEEPose).

There is NO adamo import, NO zenoh session, NO CDR parsing, and NO API key in
this process — the SDK owns all of that. It is an ordinary ROS node you can
read, fork, and re-map to a different robot. Deps: numpy, websockets.

The rosbridge connection self-heals: if the workcell container / rosbridge
restarts, the relay reconnects with backoff and re-subscribes automatically —
no relay restart needed.

Config (env, overridable by flags):
  ANVIL_ARM_TYPE  openarm_v2   gripper calibration profile (--arm-type)
"""

import argparse
import json
import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from websockets.sync.client import connect as ws_connect

# -- XR button/axis layout (WebXR gamepad mapping) ----------------------------

XR_BUTTON_TRIGGER = 0
XR_BUTTON_GRIP = 1
XR_BUTTON_STICK = 3
XR_BUTTON_X = 4
XR_BUTTON_Y = 5
TRIGGER_AXIS_INDEX = 4
GRIP_AXIS_INDEX = 5
XR_STALE_TIMEOUT_S = 1.0
# Re-engagement jump guard. Lifting the Quest off and back on interrupts the XR
# pose stream (WebXR standby) or re-initializes controller tracking, which the
# 1s stale timeout can miss (a <1s gap, or an app that keeps streaming a frozen
# pose). If the arm stays engaged across that, it drives to its pre-interruption
# anchor on resume — a dangerous jolt. So treat ANY pose-stream discontinuity as
# lost tracking: disengage and force a fresh grip-release before re-engaging (which
# re-anchors at the current EE). A gap between a controller's consecutive poses, or
# an implausibly large pose jump, both count as a discontinuity. (Because the
# rosbridge link self-heals, a reconnect gap trips this too — teleop won't silently
# resume mid-motion after a backend restart.)
XR_INPUT_GAP_DISENGAGE_S = 0.4   # >0.4s between one controller's poses = interruption
XR_CTRL_JUMP_DISENGAGE_M = 0.20  # >20cm between consecutive poses = tracking re-init/teleport


def _axis_or_button(axes, buttons, btn, axis) -> float:
    if len(axes) > axis and axes[axis] > 0.0:
        return float(axes[axis])
    if len(buttons) > btn:
        return float(buttons[btn])
    return 0.0


@dataclass
class ControllerState:
    position: Optional[np.ndarray] = None
    orientation: Optional[np.ndarray] = None
    trigger: float = 0.0
    grip: float = 0.0
    stick_pressed: bool = False
    x_pressed: bool = False
    y_pressed: bool = False


# -- WebXR (y-up) -> robot (z-up) frame change --------------------------------

R_WEBXR_TO_ROBOT = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=np.float64)


def vr_delta_to_robot_delta(d: np.ndarray) -> np.ndarray:
    return np.array([-d[2], -d[0], d[1]], dtype=np.float64)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1 - xx - yy],
        ]
    )


def matrix_to_quat(m: np.ndarray) -> np.ndarray:
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(m)))
    if i == 0:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s])


# -- rosbridge hub: self-healing controller input + EE anchoring + output -----


class RosBridge:
    CTRL_POSE = ("/controller/left", "/controller/right")
    CTRL_JOY = ("/controller/left/joy", "/controller/right/joy")

    def __init__(self, url: str) -> None:
        self.url = url
        self._ws = None
        self._ws_lock = threading.Lock()      # guards the socket + sends
        self._state_lock = threading.Lock()   # guards controller/ee state
        self.left = ControllerState()
        self.right = ControllerState()
        self.last_msg_time = 0.0
        self.rx_count = 0
        self.rx_parse_errors = 0
        # Per-controller last pose + arrival time, for the discontinuity guard.
        self._pose_time = {"left": 0.0, "right": 0.0}
        self._pose_last = {"left": None, "right": None}
        self.discontinuity = False
        self.discontinuity_reason = ""
        self.ee_pose: dict[str, tuple[np.ndarray, np.ndarray, float]] = {}
        self._ee_subscribed: set[str] = set()
        self._advertised: set[str] = set()
        self.reset_status = "unknown"
        self._call_seq = 0
        self.publish_count = 0
        self.connected = False
        # One background thread owns the connection lifecycle: connect,
        # (re)subscribe, receive, and reconnect-with-backoff on any drop.
        threading.Thread(target=self._conn_loop, daemon=True).start()
        for _ in range(100):  # let the first connection settle for sane startup logs
            if self.connected:
                break
            time.sleep(0.05)

    def _conn_loop(self) -> None:
        backoff = 0.5
        while True:
            try:
                ws = ws_connect(self.url, max_size=None)
            except Exception as e:
                print(f"[relay] rosbridge connect failed ({e}); retry in {backoff:g}s", flush=True)
                time.sleep(backoff)
                backoff = min(backoff * 2, 5.0)
                continue
            with self._ws_lock:
                self._ws = ws
                self._advertised.clear()
                self._ee_subscribed.clear()
            self.connected = True
            backoff = 0.5
            print(f"[relay] rosbridge connected: {self.url}", flush=True)
            self._resubscribe()
            try:
                while True:
                    self._handle(json.loads(ws.recv()))
            except Exception as e:
                print(f"[relay] rosbridge disconnected ({e}); reconnecting…", flush=True)
            self.connected = False
            with self._ws_lock:
                try:
                    ws.close()
                except Exception:
                    pass
                self._ws = None
            time.sleep(0.5)

    def _resubscribe(self) -> None:
        for t in self.CTRL_POSE:
            self._send({"op": "subscribe", "topic": t, "type": "geometry_msgs/msg/PoseStamped", "queue_length": 1})
        for t in self.CTRL_JOY:
            self._send({"op": "subscribe", "topic": t, "type": "sensor_msgs/msg/Joy", "queue_length": 1})
        self._send({"op": "subscribe", "topic": "/arms_resetter/reset_status", "type": "anvil_msgs/msg/ArmsResetStatus", "queue_length": 1})
        print(f"[relay] subscribed to {', '.join(self.CTRL_POSE + self.CTRL_JOY)}", flush=True)

    def _send(self, obj: dict) -> bool:
        with self._ws_lock:
            if self._ws is None:
                return False
            try:
                self._ws.send(json.dumps(obj))
                return True
            except Exception:
                return False

    def _handle(self, msg: dict) -> None:
        op = msg.get("op")
        topic = msg.get("topic", "")
        if op == "publish" and topic in self.CTRL_POSE:
            self._on_pose(topic, msg.get("msg", {}))
        elif op == "publish" and topic in self.CTRL_JOY:
            self._on_joy(topic, msg.get("msg", {}))
        elif op == "publish" and topic.startswith("/ee_pose_"):
            side = topic.removeprefix("/ee_pose_")
            p = msg["msg"]["pose"]["position"]
            q = msg["msg"]["pose"]["orientation"]
            with self._state_lock:
                self.ee_pose[side] = (
                    np.array([p["x"], p["y"], p["z"]]),
                    np.array([q["x"], q["y"], q["z"], q["w"]]),
                    time.time(),
                )
        elif op == "publish" and topic == "/arms_resetter/reset_status":
            self.reset_status = msg["msg"].get("state", "unknown")
        elif op == "service_response" and str(msg.get("id", "")).startswith("rehome-"):
            values = msg.get("values") or {}
            print(f"[relay] rehome response accepted={values.get('accepted')} message={values.get('message','')}", flush=True)
            if not values.get("accepted") and self.reset_status == "requesting":
                self.reset_status = "unknown"

    def _on_pose(self, topic: str, m: dict) -> None:
        try:
            p = m["pose"]["position"]
            q = m["pose"]["orientation"]
            pos = np.array([p["x"], p["y"], p["z"]])
            quat = np.array([q["x"], q["y"], q["z"], q["w"]])
        except (KeyError, TypeError):
            with self._state_lock:
                self.rx_parse_errors += 1
            return
        with self._state_lock:
            now = time.time()
            self.last_msg_time = now
            self.rx_count += 1
            side = "left" if topic.startswith("/controller/left") else "right"
            st = self.left if side == "left" else self.right
            st.position, st.orientation = pos, quat
            prev_t = self._pose_time[side]
            prev_p = self._pose_last[side]
            if prev_t > 0.0 and prev_p is not None:
                gap = now - prev_t
                jump = float(np.linalg.norm(pos - prev_p))
                if gap > XR_INPUT_GAP_DISENGAGE_S:
                    self.discontinuity = True
                    self.discontinuity_reason = f"{side} pose gap {gap * 1000:.0f}ms"
                elif jump > XR_CTRL_JUMP_DISENGAGE_M:
                    self.discontinuity = True
                    self.discontinuity_reason = f"{side} pose jump {jump * 100:.0f}cm in {gap * 1000:.0f}ms"
            self._pose_time[side] = now
            self._pose_last[side] = pos

    def _on_joy(self, topic: str, m: dict) -> None:
        axes = m.get("axes", [])
        buttons = m.get("buttons", [])
        with self._state_lock:
            self.last_msg_time = time.time()
            self.rx_count += 1
            st = self.left if topic.startswith("/controller/left") else self.right
            st.trigger = _axis_or_button(axes, buttons, XR_BUTTON_TRIGGER, TRIGGER_AXIS_INDEX)
            st.grip = _axis_or_button(axes, buttons, XR_BUTTON_GRIP, GRIP_AXIS_INDEX)
            st.stick_pressed = len(buttons) > XR_BUTTON_STICK and bool(buttons[XR_BUTTON_STICK])
            st.x_pressed = len(buttons) > XR_BUTTON_X and bool(buttons[XR_BUTTON_X])
            st.y_pressed = len(buttons) > XR_BUTTON_Y and bool(buttons[XR_BUTTON_Y])

    def snapshot(self):
        with self._state_lock:
            def copy(s: ControllerState) -> ControllerState:
                return ControllerState(
                    None if s.position is None else s.position.copy(),
                    None if s.orientation is None else s.orientation.copy(),
                    s.trigger, s.grip, s.stick_pressed, s.x_pressed, s.y_pressed,
                )
            disc, disc_reason = self.discontinuity, self.discontinuity_reason
            self.discontinuity = False
            return copy(self.left), copy(self.right), self.last_msg_time, self.rx_count, self.rx_parse_errors, disc, disc_reason

    # --- EE anchoring + reset + commanded output ---

    def request_ee_pose(self, side: str, *, refresh: bool = False) -> None:
        with self._state_lock:
            if refresh:
                self.ee_pose.pop(side, None)
        if side in self._ee_subscribed:
            return
        self._send({"op": "subscribe", "topic": f"/ee_pose_{side}", "type": "anvil_msgs/msg/CommandedEEPose", "throttle_rate": 20, "queue_length": 1})
        self._ee_subscribed.add(side)

    def release_ee_pose(self, side: str) -> None:
        if side not in self._ee_subscribed:
            return
        self._send({"op": "unsubscribe", "topic": f"/ee_pose_{side}"})
        self._ee_subscribed.discard(side)

    def publish_commanded_ee(self, topic: str, pos: np.ndarray, quat: np.ndarray, gripper: float) -> None:
        if topic not in self._advertised:
            if self._send({"op": "advertise", "topic": topic, "type": "anvil_msgs/msg/CommandedEEPose"}):
                self._advertised.add(topic)
        now = time.time()
        if self._send({
            "op": "publish", "topic": topic,
            "msg": {
                "header": {"stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)}, "frame_id": "world"},
                "pose": {"position": {"x": pos[0], "y": pos[1], "z": pos[2]},
                         "orientation": {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]}},
                "gripper": gripper,
            },
        }):
            self.publish_count += 1

    def reset_ready(self) -> bool:
        return self.reset_status in ("unknown", "homed", "dehomed", "failed")

    def request_rehome(self) -> bool:
        if not self.reset_ready():
            print(f"[relay] rehome skipped: reset status is {self.reset_status}", flush=True)
            return False
        self._call_seq += 1
        self.reset_status = "requesting"
        self._send({"op": "call_service", "service": "/arms_resetter/reset", "args": {"dehome": False}, "id": f"rehome-{self._call_seq}"})
        print("[relay] left Y pressed - sending rehome request", flush=True)
        return True


# -- per-arm engage/delta state -----------------------------------------------

GRIPPER_PROFILES = {
    "openyam": {"left_open": -0.035, "left_closed": -0.11, "right_open": 0.075, "right_closed": 0.0},
    "openarm": {"left_open": 0.045, "left_closed": 0.0, "right_open": 0.045, "right_closed": 0.0},
    "openarm_v2": {"left_open": 0.045, "left_closed": 0.0, "right_open": 0.045, "right_closed": 0.0},
}
ENGAGE = 0.7
DISENGAGE = 0.3


@dataclass
class Arm:
    side: str
    topic: str
    gripper_open: float
    gripper_closed: float
    engaged: bool = False
    vr_pos0: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vr_rot0: np.ndarray = field(default_factory=lambda: np.eye(3))
    ee_pos0: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ee_rot0: np.ndarray = field(default_factory=lambda: np.eye(3))
    anchor_requested: bool = False
    last_cmd_pos: Optional[np.ndarray] = None
    limited_commands: int = 0

    def disengage(self, bridge: RosBridge) -> None:
        self.engaged = False
        self.last_cmd_pos = None
        if self.anchor_requested:
            self.anchor_requested = False
            bridge.release_ee_pose(self.side)

    def target_pose(self, ctrl: ControllerState, scale: float):
        target_pos = self.ee_pos0 + scale * vr_delta_to_robot_delta(ctrl.position - self.vr_pos0)
        vr_rot = R_WEBXR_TO_ROBOT @ quat_to_matrix(ctrl.orientation)
        target_rot = vr_rot @ self.vr_rot0.T @ self.ee_rot0
        return target_pos, target_rot

    def reanchor_at_current_target(self, ctrl: ControllerState, scale: float) -> bool:
        if not self.engaged or ctrl.position is None or ctrl.orientation is None:
            return False
        target_pos, target_rot = self.target_pose(ctrl, scale)
        self.vr_pos0 = ctrl.position.copy()
        self.vr_rot0 = R_WEBXR_TO_ROBOT @ quat_to_matrix(ctrl.orientation)
        self.ee_pos0 = target_pos.copy()
        self.ee_rot0 = target_rot.copy()
        self.last_cmd_pos = target_pos.copy()
        return True

    def _limit_target_pos(self, target_pos: np.ndarray, max_step: float) -> np.ndarray:
        if max_step <= 0.0 or self.last_cmd_pos is None:
            self.last_cmd_pos = target_pos.copy()
            return target_pos
        delta = target_pos - self.last_cmd_pos
        norm = float(np.linalg.norm(delta))
        if norm > max_step:
            target_pos = self.last_cmd_pos + delta * (max_step / max(norm, 1e-9))
            self.limited_commands += 1
        self.last_cmd_pos = target_pos.copy()
        return target_pos

    def gripper_command(self, ctrl: ControllerState) -> float:
        trigger = max(0.0, min(1.0, ctrl.trigger))
        return self.gripper_open + trigger * (self.gripper_closed - self.gripper_open)

    def tick(self, ctrl: ControllerState, bridge: RosBridge, scale: float, max_ee_step: float, dry: bool) -> Optional[str]:
        if ctrl.position is None or ctrl.orientation is None:
            return None
        if not self.engaged:
            if ctrl.grip >= ENGAGE:
                if not self.anchor_requested:
                    self.anchor_requested = True
                    bridge.request_ee_pose(self.side, refresh=True)
                    return f"{self.side}: requesting /ee_pose_{self.side} anchor"
                anchor = bridge.ee_pose.get(self.side)
                if anchor is None:
                    return None
                self.vr_pos0 = ctrl.position.copy()
                self.vr_rot0 = R_WEBXR_TO_ROBOT @ quat_to_matrix(ctrl.orientation)
                self.ee_pos0 = anchor[0].copy()
                self.ee_rot0 = quat_to_matrix(anchor[1])
                self.engaged = True
                self.last_cmd_pos = self.ee_pos0.copy()
                self.anchor_requested = False
                bridge.release_ee_pose(self.side)
                return f"{self.side}: ENGAGED (anchor ee={np.round(self.ee_pos0, 3).tolist()})"
            if self.anchor_requested:
                self.anchor_requested = False
                bridge.release_ee_pose(self.side)
            return None
        if ctrl.grip <= DISENGAGE:
            self.engaged = False
            self.last_cmd_pos = None
            return f"{self.side}: disengaged"
        target_pos, target_rot = self.target_pose(ctrl, scale)
        target_pos = self._limit_target_pos(target_pos, max_ee_step)
        gripper = self.gripper_command(ctrl)
        if dry:
            return f"{self.side}: cmd pos={np.round(target_pos, 3).tolist()} grip={gripper:.3f}"
        bridge.publish_commanded_ee(self.topic, target_pos, matrix_to_quat(target_rot), gripper)
        return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm-type", choices=tuple(GRIPPER_PROFILES),
                    default=os.environ.get("ANVIL_ARM_TYPE", "openarm_v2"))
    ap.add_argument("--rosbridge", default="ws://localhost:9090")
    ap.add_argument("--left-topic", default="/commanded_ee_left")
    ap.add_argument("--right-topic", default="/commanded_ee_right")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--scale-alt", type=float, default=1.0)
    ap.add_argument("--rate", type=float, default=60.0)
    ap.add_argument("--max-ee-step", type=float, default=0.02)
    ap.add_argument("--dry-run", action="store_true", help="log commands instead of publishing")
    args = ap.parse_args()

    gp = GRIPPER_PROFILES[args.arm_type]
    bridge = RosBridge(args.rosbridge)
    left = Arm("left", args.left_topic, gp["left_open"], gp["left_closed"])
    right = Arm("right", args.right_topic, gp["right_open"], gp["right_closed"])
    arms = (left, right)
    active_scale = args.scale
    scale_alt_active = False

    print(f"[relay] running ({'DRY RUN' if args.dry_run else 'LIVE'}), arm_type={args.arm_type} "
          f"scale={active_scale:g} max_ee_step={args.max_ee_step:g}m (rosbridge only, no adamo/zenoh)",
          flush=True)
    period = 1.0 / args.rate
    last_status = 0.0
    last_rx = 0
    last_pub = 0
    prev_left_y = False
    reset_button_locked = False
    prev_scale_button = False
    grip_release_required = False
    was_stale = True
    while True:
        t0 = time.time()
        l, r, last_msg, rx_count, rx_errors, disc, disc_reason = bridge.snapshot()
        stale = last_msg == 0.0 or t0 - last_msg > XR_STALE_TIMEOUT_S
        if stale and not was_stale:
            for arm in arms:
                arm.disengage(bridge)
            grip_release_required = True
            print(f"[relay] XR input timed out after {XR_STALE_TIMEOUT_S:g}s - disengaged; release grips to re-engage", flush=True)
        was_stale = stale

        # Tracking-discontinuity guard (headset off/on, standby, re-init, rosbridge
        # reconnect): a gap or teleport in the pose stream means the anchor is stale.
        # Disengage and force a fresh grip-release so re-engagement re-anchors at the
        # current EE, never jolting to the pre-interruption target.
        if disc:
            was_engaged = any(arm.engaged for arm in arms)
            for arm in arms:
                arm.disengage(bridge)
            grip_release_required = True
            if was_engaged or not stale:
                print(f"[relay] XR tracking discontinuity ({disc_reason}) - disengaged; release grips to re-engage", flush=True)

        if l.x_pressed and not prev_scale_button:
            old = active_scale
            n = sum(arm.reanchor_at_current_target(ctrl, old) for arm, ctrl in ((left, l), (right, r)))
            scale_alt_active = not scale_alt_active
            active_scale = args.scale_alt if scale_alt_active else args.scale
            print(f"[relay] left X - scale {old:g} -> {active_scale:g} (reanchored {n})", flush=True)
        prev_scale_button = l.x_pressed

        if not l.y_pressed and bridge.reset_ready():
            reset_button_locked = False
        if l.y_pressed and not prev_left_y and not reset_button_locked:
            reset_button_locked = True
            for arm in arms:
                arm.disengage(bridge)
            if args.dry_run:
                print("[relay] left Y - dry-run rehome skipped", flush=True)
            else:
                bridge.request_rehome()
                grip_release_required = True
        prev_left_y = l.y_pressed

        if grip_release_required and l.grip <= DISENGAGE and r.grip <= DISENGAGE:
            grip_release_required = False
            print("[relay] grip release observed - teleop can re-engage", flush=True)

        if not stale and not grip_release_required and bridge.reset_ready():
            for arm, ctrl in ((left, l), (right, r)):
                note = arm.tick(ctrl, bridge, active_scale, args.max_ee_step, args.dry_run)
                if note:
                    print(f"[relay] {note}", flush=True)

        if time.time() - last_status > 5.0:
            now = time.time()
            dt = max(now - last_status, 1e-6) if last_status else 0.0
            rx_hz = ((rx_count - last_rx) / dt) if dt else 0.0
            pub_hz = ((bridge.publish_count - last_pub) / dt) if dt else 0.0
            last_rx, last_pub, last_status = rx_count, bridge.publish_count, now
            state = f"L({'engaged' if left.engaged else 'idle'}) R({'engaged' if right.engaged else 'idle'})"
            xr = "no XR input yet" if last_msg == 0 else ("stale" if stale else "live")
            print(f"[relay] {state} xr={xr} ros={'up' if bridge.connected else 'DOWN'} reset={bridge.reset_status} "
                  f"scale={active_scale:g} ee_poses={sorted(bridge.ee_pose)} "
                  f"trig=({l.trigger:.2f},{r.trigger:.2f}) rx_hz={rx_hz:.1f} pub_hz={pub_hz:.1f} "
                  f"limited={left.limited_commands + right.limited_commands} parse_errors={rx_errors}", flush=True)
        time.sleep(max(0.0, period - (time.time() - t0)))


if __name__ == "__main__":
    main()
