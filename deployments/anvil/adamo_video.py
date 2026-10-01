#!/usr/bin/env python3
"""Adamo publisher for the OpenArm v2 anvil on the canonical adamo SDK, recorded.

Publishes the dual-OAK stereo track (and the wrist cameras when their ROS topics
are live), turns the operator's WebXR frames into the ROS controller topics the
XR relay drives the arms from, and asks the Adamo data service to record every
controlled session together with the arms' joint states.

Replaces the adamo 0.4 version of this file, whose ros2dds bridge forwarded ROS
messages the legacy web app built. app.adamohq.com sends those same messages by
default, as ROS 2 CDR envelopes on control/cdr/xr_tracking, or one XrFrame per
headset frame when built for it, already remapped and rebased in the browser.
Both become /controller/{hand} and /controller/{hand}/tip (PoseStamped in
xr_origin), /controller/{hand}/joy (Joy) and /head_pose.

REQUIRES the OAK->SHM compositor (adamo-oak-stereo.service) and rosbridge on
ws://localhost:9090 (the ROS container), the same one adamo_xr_relay.py uses.

Environment (set by the systemd unit / .env):
  ADAMO_API_KEY             ak_...   org auth (required)
  ADAMO_ROBOT_NAME          openarm  participant name shown in the fleet
  ADAMO_ROS_CONTROL_PREFIX  (empty)  prefix for the controller topics; set it to
                                     /test for a side-by-side trial so the arms
                                     ignore this process
"""

import base64
import json
import os
import struct
import sys
import threading
import time

import adamo
import cv2
import numpy as np
from websockets.sync.client import connect

FPS = 30
ROSBRIDGE = "ws://localhost:9090"
OAK_STEREO_SHM = "camera/oak_stereo_nv12"
OAK_WIDTH, OAK_HEIGHT = 2560, 2880
WRIST_CAMERAS = (
    ("wrist_left", "/cam_wrist_l/image_raw/compressed"),
    ("wrist_right", "/cam_wrist_r/image_raw/compressed"),
)
WRIST_PROBE_SECONDS = 5.0
# Recorded with every sample: what the arms did and what they were told to do.
SAMPLED_TOPICS = (
    ("/joint_states", "sensor_msgs/msg/JointState"),
    ("/commanded_ee_left", "anvil_msgs/msg/CommandedEEPose"),
    ("/commanded_ee_right", "anvil_msgs/msg/CommandedEEPose"),
)

API_KEY = os.environ.get("ADAMO_API_KEY") or sys.exit("Set ADAMO_API_KEY=ak_...")
ROBOT_NAME = os.environ.get("ADAMO_ROBOT_NAME", "openarm")
CONTROL_PREFIX = os.environ.get("ADAMO_ROS_CONTROL_PREFIX", "")


class Rosbridge:
    """One rosbridge websocket, reconnected with backoff on any drop.

    Advertisements and subscriptions are replayed after every reconnect;
    publishes while disconnected are dropped, like a stale control frame.
    """

    def __init__(self, url: str, name: str):
        self.url = url
        self.name = name
        self.send_lock = threading.Lock()
        self.socket = None
        self.setup = []
        self.handlers = {}

    def advertise(self, topic: str, message_type: str) -> None:
        self.setup.append({"op": "advertise", "topic": topic, "type": message_type})

    def subscribe(self, topic: str, message_type: str, handler) -> None:
        self.handlers[topic] = handler
        self.setup.append({"op": "subscribe", "topic": topic, "type": message_type, "queue_length": 1})

    def publish(self, topic: str, message: dict) -> None:
        with self.send_lock:
            if self.socket is None:
                return
            try:
                self.socket.send(json.dumps({"op": "publish", "topic": topic, "msg": message}))
            except Exception:
                self.socket = None

    def run(self) -> None:
        backoff = 0.5
        while True:
            try:
                with connect(self.url, max_size=None, open_timeout=5) as socket:
                    with self.send_lock:
                        for operation in self.setup:
                            socket.send(json.dumps(operation))
                        self.socket = socket
                    print(f"[rosbridge:{self.name}] connected to {self.url}", flush=True)
                    backoff = 0.5
                    for raw in socket:
                        envelope = json.loads(raw)
                        handler = self.handlers.get(envelope.get("topic"))
                        if handler is not None and envelope.get("op") == "publish":
                            handler(envelope["msg"])
            except Exception as error:
                print(f"[rosbridge:{self.name}] {self.url} unavailable ({error}); retrying in {backoff:.1f} s", flush=True)
            with self.send_lock:
                self.socket = None
            time.sleep(backoff)
            backoff = min(backoff * 2, 5.0)


def header(stamp: float, frame_id: str) -> dict:
    seconds = int(stamp)
    return {"stamp": {"sec": seconds, "nanosec": int((stamp - seconds) * 1e9)}, "frame_id": frame_id}


def pose_stamped(pose: dict, stamp: float) -> dict:
    return {"header": header(stamp, "xr_origin"), "pose": {"position": pose["position"], "orientation": pose["orientation"]}}


XR_TOPICS = {"/head_pose": "geometry_msgs/msg/PoseStamped"}
for hand in ("left", "right"):
    XR_TOPICS[f"/controller/{hand}"] = "geometry_msgs/msg/PoseStamped"
    XR_TOPICS[f"/controller/{hand}/tip"] = "geometry_msgs/msg/PoseStamped"
    XR_TOPICS[f"/controller/{hand}/joy"] = "sensor_msgs/msg/Joy"

controls = Rosbridge(ROSBRIDGE, "controls")
for topic, message_type in XR_TOPICS.items():
    controls.advertise(f"{CONTROL_PREFIX}{topic}", message_type)


class CdrReader:
    """Little-endian ROS 2 CDR as the browser writes it: alignment counts from after the 4-byte encapsulation header."""

    def __init__(self, data: bytes):
        self.data = data
        self.offset = 4

    def read(self, layout: str, alignment: int) -> tuple:
        self.offset += -(self.offset - 4) % alignment
        values = struct.unpack_from("<" + layout, self.data, self.offset)
        self.offset += struct.calcsize("<" + layout)
        return values

    def sequence(self, code: str) -> list:
        (count,) = self.read("I", 4)
        return list(self.read(f"{count}{code}", 4))

    def header(self) -> dict:
        sec, nanosec, frame_id_size = self.read("iII", 4)
        frame_id = self.data[self.offset:self.offset + frame_id_size - 1].decode()
        self.offset += frame_id_size
        return {"stamp": {"sec": sec, "nanosec": nanosec}, "frame_id": frame_id}


def pose_stamped_from_cdr(cdr: CdrReader) -> dict:
    header = cdr.header()
    x, y, z, qx, qy, qz, qw = cdr.read("7d", 8)
    return {"header": header, "pose": {"position": {"x": x, "y": y, "z": z}, "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}}}


def joy_from_cdr(cdr: CdrReader) -> dict:
    header = cdr.header()
    return {"header": header, "axes": cdr.sequence("f"), "buttons": cdr.sequence("i")}


CDR_DECODERS = {"geometry_msgs/msg/PoseStamped": pose_stamped_from_cdr, "sensor_msgs/msg/Joy": joy_from_cdr}

sensors = Rosbridge(ROSBRIDGE, "sensors")
latest_lock = threading.Lock()
latest = {}


def remember(topic: str, message: dict) -> None:
    with latest_lock:
        latest[topic] = message


for topic, message_type in SAMPLED_TOPICS:
    sensors.subscribe(topic, message_type, lambda message, topic=topic: remember(topic, message))

wrist_frames = {}
wrist_first = {track: threading.Event() for track, _ in WRIST_CAMERAS}


def on_wrist_image(track: str, message: dict) -> None:
    image = cv2.imdecode(np.frombuffer(base64.b64decode(message["data"]), np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        return
    wrist_frames[track] = image
    wrist_first[track].set()


for track, topic in WRIST_CAMERAS:
    sensors.subscribe(topic, "sensor_msgs/msg/CompressedImage", lambda message, track=track: on_wrist_image(track, message))

threading.Thread(target=controls.run, name="rosbridge-controls", daemon=True).start()
threading.Thread(target=sensors.run, name="rosbridge-sensors", daemon=True).start()


def sample(capture_time_us: int):
    with latest_lock:
        return {topic.lstrip("/"): message for topic, message in latest.items()} or None


robot = adamo.Robot(
    api_key=API_KEY,
    name=ROBOT_NAME,
    protocol="quic",
    record=adamo.Recording(on_sample=sample),
)


@robot.on("json/xr/frame")
def on_xr_frame(control: adamo.ControlSample) -> None:
    frame = control.message
    stamp = frame.get("stamp") or time.time()
    if frame.get("head"):
        controls.publish(f"{CONTROL_PREFIX}/head_pose", pose_stamped(frame["head"], stamp))
    for hand in ("left", "right"):
        controller = frame.get(hand)
        if not controller:
            continue
        topic = f"{CONTROL_PREFIX}/controller/{hand}"
        controls.publish(topic, pose_stamped(controller["pose"], stamp))
        if controller.get("tip"):
            controls.publish(f"{topic}/tip", pose_stamped(controller["tip"], stamp))
        if controller.get("axes") or controller.get("buttons"):
            controls.publish(f"{topic}/joy", {"header": header(stamp, hand), "axes": controller["axes"], "buttons": controller["buttons"]})


def on_cdr_xr(sample: adamo.Sample) -> None:
    envelope = sample.payload
    topic_end = 4 + int.from_bytes(envelope[:4], "big")
    topic = envelope[4:topic_end].decode()
    if topic not in XR_TOPICS:
        return
    payload_start = topic_end + 4 + int.from_bytes(envelope[topic_end:topic_end + 4], "big")
    controls.publish(f"{CONTROL_PREFIX}{topic}", CDR_DECODERS[XR_TOPICS[topic]](CdrReader(envelope[payload_start:])))


# robot.on only delivers JSON and MessagePack, so the CDR envelopes are read raw.
cdr_xr_subscriber = robot.session.subscribe(f"{robot.name}/control/cdr/xr_tracking", callback=on_cdr_xr)


# Wrist cameras only when their ROS topics are live: a caller-fed track needs
# its frame size before the robot runs, and the size comes from the first frame.
wrist_tracks = {}
for track, topic in WRIST_CAMERAS:
    if not wrist_first[track].wait(WRIST_PROBE_SECONDS):
        print(f"[adamo_video] {topic} is not publishing; {track} is left out", flush=True)
        continue
    height, width = wrist_frames[track].shape[:2]
    wrist_tracks[track] = robot.video(
        track,
        pixel_format="BGR",
        width=width,
        height=height,
        fps=FPS,
        encoder="vah264enc",
        bitrate_kbps=1250,
        keyframe_distance=1.0,
    )

# Packed dual-OAK stereo: raw NV12 2560x2880 (two 2560x1440 eyes stacked
# top/bottom) from shared memory, VA-API H.264-encoded by the SDK.
robot.attach_video(
    "oak_stereo",
    shm=OAK_STEREO_SHM,
    pixel_format="NV12",
    width=OAK_WIDTH,
    height=OAK_HEIGHT,
    fps=FPS,
    encoder="vah264enc",
    bitrate_kbps=10000,
    max_bitrate_kbps=15000,
    adaptive_bitrate=True,
    keyframe_distance=1.0,
    stereo=True,
    allow_missing=True,
)


def pump_wrist(track: str) -> None:
    sent = None
    while True:
        image = wrist_frames.get(track)
        if image is not None and image is not sent:
            wrist_tracks[track].send(image)
            sent = image
        time.sleep(1 / (FPS * 2))


for track in wrist_tracks:
    threading.Thread(target=pump_wrist, args=(track,), name=track, daemon=True).start()

print(f"[adamo_video] publishing oak_stereo{''.join(', ' + t for t in wrist_tracks)} as '{ROBOT_NAME}', "
      f"recording on; XR controls to {CONTROL_PREFIX or ''}/controller/*", flush=True)
robot.run()
