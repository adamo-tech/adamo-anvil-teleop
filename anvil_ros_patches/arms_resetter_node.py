#!/usr/bin/env python3

"""
Arms resetter node.

Provides a ~/reset service for managing arm lifecycle with three operations:
1. home   (dehome=false, arms dehomed): full trajectory → homed
2. rehome (dehome=false, arms homed):   final-pose-only trajectory → homed
3. dehome (dehome=true,  arms homed):   reversed trajectory → dehomed

Status is broadcast on ~/reset_status as it progresses.
Runs initial home on startup, then persists to handle runtime requests.
"""

import sys

from anvil_msgs.msg import ArmsResetStatus, CommandedEEPose
from anvil_msgs.srv import ResetArms
from control.controls_owner_manager import ControlsOwnerManager
from control.parallel_arm_homer import ParallelArmHomer
from control.tcp_fk import TcpForwardKinematics
from controller_manager_msgs.srv import SwitchController
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


class ArmsResetterNode(Node):
    """Arms resetter node with ~/reset service and ~/reset_status topic."""

    def __init__(self):
        super().__init__("arms_resetter")

        # Declare parameters
        self.declare_parameter("gains_ramp_duration", 2.0)
        self.declare_parameter("homing_duration", 5.0)
        # Target TCP linear velocity (m/s) for homing. Set to 0 to disable.
        self.declare_parameter("homing_velocity", 0.0)
        self.declare_parameter("home_on_startup", False)
        self.declare_parameter("post_homing_owner_hold_sec", 5.0)
        self.declare_parameter("arm_names", [""])
        self.declare_parameter("controllers_to_activate", [""])

        self._gains_ramp_duration = self.get_parameter("gains_ramp_duration").value
        self._homing_duration = self.get_parameter("homing_duration").value
        self._homing_velocity = self.get_parameter("homing_velocity").value
        self._home_on_startup = bool(self.get_parameter("home_on_startup").value)
        self._post_homing_owner_hold_sec = max(
            0.0, float(self.get_parameter("post_homing_owner_hold_sec").value)
        )
        arm_names = self.get_parameter("arm_names").value
        controllers = self.get_parameter("controllers_to_activate").value

        # Filter out empty values
        self._arm_names = [name for name in arm_names if name]
        self._controllers_to_activate = [c for c in controllers if c]

        # Split dynamics out from other controllers so
        # we can activate grav comp independently
        self._dynamics_controllers = [
            c for c in self._controllers_to_activate if c.endswith("_dynamics")
        ]
        self._post_homing_controllers = [
            c for c in self._controllers_to_activate if not c.endswith("_dynamics")
        ]

        if not self._arm_names:
            raise RuntimeError("No arms configured!")

        if not self._controllers_to_activate:
            raise RuntimeError("No controllers to activate!")

        self.get_logger().info(f"Configured arms: {self._arm_names}")
        self.get_logger().info(
            f"Controllers to activate: {self._controllers_to_activate}"
        )

        # Parse per-arm parameters
        self._arm_configs = {}
        for arm_name in self._arm_names:
            self.declare_parameter(f"{arm_name}.joints", [""])
            self.declare_parameter(f"{arm_name}.home_positions", [0.0])
            self.declare_parameter(f"{arm_name}.gains_controller", "")
            self.declare_parameter(f"{arm_name}.jtc_controller", "")
            self.declare_parameter(f"{arm_name}.tcp_link", "")
            self.declare_parameter(f"{arm_name}.homing_kp", [0.0])
            self.declare_parameter(f"{arm_name}.homing_kd", [0.0])
            self.declare_parameter(f"{arm_name}.teleop_kp", [0.0])
            self.declare_parameter(f"{arm_name}.teleop_kd", [0.0])

            self._arm_configs[arm_name] = {
                "joints": self.get_parameter(f"{arm_name}.joints").value,
                "home_positions": self.get_parameter(f"{arm_name}.home_positions").value,
                "gains_controller": self.get_parameter(
                    f"{arm_name}.gains_controller"
                ).value,
                "jtc_controller": self.get_parameter(
                    f"{arm_name}.jtc_controller"
                ).value,
                "tcp_link": self.get_parameter(f"{arm_name}.tcp_link").value,
                "homing_kp": self.get_parameter(f"{arm_name}.homing_kp").value,
                "homing_kd": self.get_parameter(f"{arm_name}.homing_kd").value,
                "teleop_kp": self.get_parameter(f"{arm_name}.teleop_kp").value,
                "teleop_kd": self.get_parameter(f"{arm_name}.teleop_kd").value,
            }

        # Callback group for async operations
        self._callback_group = ReentrantCallbackGroup()

        self._latest_joint_positions: dict[str, float] = {}
        self._joint_states_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,
            callback_group=self._callback_group,
        )
        self._urdf_string: str | None = None
        self._fk: TcpForwardKinematics | None = None
        self._robot_description_sub = self.create_subscription(
            String,
            "/robot_description",
            self._on_robot_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._callback_group,
        )

        # Build homer configuration
        joint_names = {}
        home_positions = {}
        homing_kp = {}
        homing_kd = {}
        jtc_to_tcp_link = {}

        for arm_name, cfg in self._arm_configs.items():
            jtc = cfg["jtc_controller"]
            gains = cfg["gains_controller"]

            joint_names[jtc] = cfg["joints"]
            home_positions[jtc] = cfg["home_positions"]
            homing_kp[gains] = cfg["homing_kp"]
            homing_kd[gains] = cfg["homing_kd"]
            if cfg["tcp_link"]:
                jtc_to_tcp_link[jtc] = cfg["tcp_link"]

        # Create homer
        self._homer = ParallelArmHomer(
            node=self,
            joint_names=joint_names,
            home_positions=home_positions,
            homing_kp=homing_kp,
            homing_kd=homing_kd,
            gains_ramp_duration=self._gains_ramp_duration,
            homing_duration=self._homing_duration,
            deactivate_jtcs=True,
            jtc_to_tcp_link=jtc_to_tcp_link,
        )

        # Create switch_controller client
        self._switch_controller_client = self.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
            callback_group=self._callback_group,
        )

        self._controls_owner_manager = ControlsOwnerManager(
            self, callback_group=self._callback_group
        )

        self._forward_command_publishers = {}
        self._commanded_ee_publishers = {}
        for arm_name in self._arm_configs:
            controller = f"{arm_name}_forward_position_controller"
            if controller not in self._post_homing_controllers:
                pass
            else:
                self._forward_command_publishers[arm_name] = self.create_publisher(
                    Float64MultiArray,
                    f"/{controller}/commands",
                    10,
                )
            side = self._commanded_ee_side(arm_name)
            if side is not None:
                self._commanded_ee_publishers[arm_name] = self.create_publisher(
                    CommandedEEPose,
                    f"/commanded_ee_{side}",
                    10,
                )

        # Status publisher (latched so late subscribers get current state)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_publisher = self.create_publisher(
            ArmsResetStatus, "~/reset_status", latched_qos
        )

        self.exit_code = 0

        # Arms start in dehomed state
        self._is_dehome: bool = True

        # Publish initial status before creating the service to block external requests
        self._set_status("initializing")

        # Create service server
        self._reset_service = self.create_service(
            ResetArms,
            "~/reset",
            self._reset_service_callback,
            callback_group=self._callback_group,
        )

        if self._home_on_startup:
            # Schedule initial home to run once the executor starts spinning.
            self._startup_timer = self.create_timer(
                0.0, self._initial_reset_callback, callback_group=self._callback_group
            )
        else:
            self._startup_timer = None
            self._set_status("dehomed")
            self.get_logger().warn(
                "Startup homing disabled; waiting for explicit ~/reset request"
            )

        self.get_logger().info("Arms resetter initialized with ~/reset service")

    def _on_joint_states(self, msg: JointState) -> None:
        self._latest_joint_positions = dict(zip(msg.name, msg.position))

    def _on_robot_description(self, msg: String) -> None:
        self._urdf_string = msg.data

    def _set_status(self, state: str) -> None:
        self._status = state
        msg = ArmsResetStatus()
        msg.state = state
        msg.is_dehome = self._is_dehome
        self._status_publisher.publish(msg)
        self.get_logger().info(f"Reset status: {state} (is_dehome={self._is_dehome})")

    @staticmethod
    def _commanded_ee_side(arm_name: str) -> str | None:
        if arm_name.endswith("_l"):
            return "left"
        if arm_name.endswith("_r"):
            return "right"
        return None

    def _ensure_fk(self) -> TcpForwardKinematics | None:
        if self._fk is not None:
            return self._fk
        if self._urdf_string is None:
            # ParallelArmHomer also subscribes to /robot_description and may
            # already have a loaded FK model because homing_velocity uses it.
            homer_fk = getattr(self._homer, "_fk", None)
            if homer_fk is not None:
                return homer_fk
            return None
        try:
            self._fk = TcpForwardKinematics(self._urdf_string)
            self.get_logger().info("Loaded URDF for reset handoff EE seeding FK")
            return self._fk
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF for reset handoff FK: {e}")
            return None

    async def _initial_reset_callback(self):
        """Run the initial home sequence on startup."""
        self._startup_timer.cancel()
        self.get_logger().info("Running initial home...")
        await self._execute_reset(ResetArms.Request())
        if self._status == "failed":
            self.exit_code = 1
            rclpy.shutdown()
            return
        self.get_logger().info("Spinning - use the reset service to home/rehome/dehome")

    def _reset_service_callback(self, request, response):
        """Handle incoming reset service requests."""
        if self._status == "initializing":
            self.get_logger().warn("Reset rejected: initial homing not yet started")
            response.accepted = False
            response.message = "Initial homing not yet started"
            return response

        if self._status not in ("homed", "dehomed", "failed"):
            self.get_logger().warn("Reset rejected: reset in progress")
            response.accepted = False
            response.message = "Reset already in progress"
            return response

        if request.dehome and self._is_dehome:
            self.get_logger().warn("Reset rejected: arms already dehomed")
            response.accepted = False
            response.message = "Cannot dehome: arms are already dehomed"
            return response

        # Silently advance status to block double-accepts before the task starts
        self._status = "deactivating_controllers"
        self.executor.create_task(self._execute_reset(request))
        response.accepted = True
        response.message = "Reset request accepted"
        return response

    async def _deactivate_controllers(self):
        """
        Deactivate runtime controllers before homing. Dynamics is intentionally
        left running so we don't lose grav comp during the homing sequence.
        """
        to_deactivate = [
            c for c in self._controllers_to_activate if not c.endswith("_dynamics")
        ]
        if not to_deactivate:
            return

        if not self._switch_controller_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("timed out waiting for switch_controller service")

        switch_req = SwitchController.Request()
        switch_req.deactivate_controllers = to_deactivate
        switch_req.strictness = SwitchController.Request.BEST_EFFORT
        switch_req.activate_asap = True

        result = await self._switch_controller_client.call_async(switch_req)
        if not result.ok:
            # May fail if controllers weren't active - that's OK
            self.get_logger().warn(
                "Controller deactivation returned not ok (may not have been active)"
            )

    async def _activate_controllers(self, controllers: list[str]):
        """Activate the given controllers via switch_controller."""
        if not controllers:
            return

        if not self._switch_controller_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("timed out waiting for switch_controller service")

        switch_req = SwitchController.Request()
        switch_req.activate_controllers = controllers
        switch_req.strictness = SwitchController.Request.BEST_EFFORT
        switch_req.activate_asap = True

        result = await self._switch_controller_client.call_async(switch_req)
        if not result.ok:
            raise RuntimeError(f"Failed to activate controllers: {controllers}")

    async def _set_runtime_gains(self):
        """Set runtime gains after homing."""
        runtime_gains = {}
        for arm_name, cfg in self._arm_configs.items():
            gains = cfg["gains_controller"]
            runtime_gains[gains] = (cfg["teleop_kp"], cfg["teleop_kd"])

        await self._homer.gains_ramper.ramp_gains(
            runtime_gains,
            duration=self._gains_ramp_duration,
        )

    def _seed_positions_for_arm(
        self,
        arm_name: str,
        *,
        dehome: bool,
        hold_home_target: bool,
    ) -> tuple[list[float], str]:
        cfg = self._arm_configs[arm_name]
        if hold_home_target and not dehome:
            joint_count = len(cfg["joints"])
            positions = [
                float(value) for value in cfg["home_positions"][-joint_count:]
            ]
            return positions, "home_target"

        latest = self._latest_joint_positions
        missing = [joint for joint in cfg["joints"] if joint not in latest]
        if missing:
            raise RuntimeError(
                f"Cannot seed {arm_name} forward-position controller: "
                f"missing joints from /joint_states: {missing}"
            )
        positions = [float(latest[joint]) for joint in cfg["joints"]]
        if not dehome:
            positions[-1] = float(cfg["home_positions"][-1])
        return positions, "joint_states"

    async def _seed_forward_position_commands(
        self,
        *,
        dehome: bool,
        hold_home_target: bool,
        reason: str,
        repeats: int,
        interval_sec: float,
    ) -> None:
        """Prime forward-position controllers so activation cannot replay stale targets."""
        if not self._forward_command_publishers:
            return

        for i in range(repeats):
            sources = []
            for arm_name, publisher in self._forward_command_publishers.items():
                positions, source = self._seed_positions_for_arm(
                    arm_name,
                    dehome=dehome,
                    hold_home_target=hold_home_target,
                )
                msg = Float64MultiArray()
                msg.data = positions
                publisher.publish(msg)
                sources.append(f"{arm_name}:{source}")

            if i == 0:
                self.get_logger().info(
                    f"Seeded forward-position commands ({reason}; {', '.join(sources)})"
                )

            if i + 1 < repeats:
                await self._async_sleep(interval_sec)

    def _current_commanded_ee_for_arm(self, arm_name: str) -> CommandedEEPose:
        cfg = self._arm_configs[arm_name]
        tcp_link = cfg["tcp_link"]
        if not tcp_link:
            raise RuntimeError(f"Cannot seed {arm_name} commanded EE: no tcp_link")

        latest = self._latest_joint_positions
        missing = [joint for joint in cfg["joints"] if joint not in latest]
        if missing:
            raise RuntimeError(
                f"Cannot seed {arm_name} commanded EE: "
                f"missing joints from /joint_states: {missing}"
            )

        fk = self._ensure_fk()
        if fk is None:
            raise RuntimeError(f"Cannot seed {arm_name} commanded EE: FK unavailable")

        if hasattr(fk, "tcp_pose"):
            pos, quat = fk.tcp_pose(latest, tcp_link)
        else:
            # Anvil 1.2.4's TcpForwardKinematics exposes position only. Use
            # that public method to apply the joint state, then read the same
            # pybullet link state for its world-frame orientation. Newer
            # images provide tcp_pose directly and take the branch above.
            import pybullet

            pos = fk.tcp_position(latest, tcp_link)
            link_state = pybullet.getLinkState(
                fk._robot_id,
                fk._link_name_to_index[tcp_link],
                computeForwardKinematics=True,
                physicsClientId=fk._client,
            )
            quat = link_state[5]
        msg = CommandedEEPose()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        msg.gripper = float(cfg["home_positions"][-1])
        return msg

    async def _seed_commanded_ee_targets(
        self,
        *,
        reason: str,
        repeats: int,
        interval_sec: float,
    ) -> None:
        """Prime quest_teleop's commanded-EE cache from current FK before handoff."""
        if not self._commanded_ee_publishers:
            return

        for i in range(repeats):
            seeded = []
            for arm_name, publisher in self._commanded_ee_publishers.items():
                msg = self._current_commanded_ee_for_arm(arm_name)
                publisher.publish(msg)
                p = msg.pose.position
                seeded.append(
                    f"{arm_name}:({p.x:.3f},{p.y:.3f},{p.z:.3f})"
                )

            if i == 0:
                self.get_logger().info(
                    f"Seeded commanded EE targets ({reason}; {', '.join(seeded)})"
                )

            if i + 1 < repeats:
                await self._async_sleep(interval_sec)

    async def _async_sleep(self, seconds: float) -> None:
        future = Future()

        def _on_timeout():
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(
            seconds,
            _on_timeout,
            callback_group=self._callback_group,
        )
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def _execute_reset(self, request: ResetArms.Request) -> None:
        """Execute the full reset sequence, publishing status at each phase."""
        if request.dehome:
            # dehome: reversed trajectory, default duration
            final_pose_only, reverse, duration_override = False, True, 0.0
        elif self._is_dehome:
            # home from dehomed state: full trajectory, default duration
            final_pose_only, reverse, duration_override = False, False, 0.0
        else:
            # rehome: final-pose-only trajectory. The 7.5s override only applies
            # in fixed-duration mode; when homing_velocity > 0 it is ignored and
            # the duration is derived from the gripper's Cartesian travel.
            final_pose_only, reverse, duration_override = True, False, 7.5

        claimed = self._controls_owner_manager.claim("homer")
        if not claimed:
            self.get_logger().warn("Failed to claim controls ownership for homer")

        # Even if we fail to claim, homing should still proceed.
        try:
            self._set_status("deactivating_controllers")
            await self._deactivate_controllers()

            self._set_status("homing_arms")
            await self._homer.home(
                final_pose_only=final_pose_only,
                reverse=reverse,
                homing_duration_override=duration_override,
                homing_velocity=self._homing_velocity,
                activate_after_gains_ramp=self._dynamics_controllers,
            )

            # ParallelArmHomer deactivates the JTCs before returning. Activate
            # the forward controllers immediately with the final home target;
            # otherwise the wrists can fall during the gains ramp and that
            # fallen joint state becomes the new command.
            self._set_status("activating_controllers")
            await self._seed_forward_position_commands(
                dehome=request.dehome,
                hold_home_target=not request.dehome,
                reason="before activation",
                repeats=5,
                interval_sec=0.05,
            )
            await self._activate_controllers(self._post_homing_controllers)
            await self._seed_forward_position_commands(
                dehome=request.dehome,
                hold_home_target=not request.dehome,
                reason="after activation",
                repeats=10,
                interval_sec=0.05,
            )

            self._set_status("setting_runtime_gains")
            await self._set_runtime_gains()
            await self._seed_forward_position_commands(
                dehome=request.dehome,
                hold_home_target=not request.dehome,
                reason="after runtime gains",
                repeats=5,
                interval_sec=0.05,
            )
            if not request.dehome:
                await self._seed_commanded_ee_targets(
                    reason="before homed status",
                    repeats=5,
                    interval_sec=0.05,
                )

            self._is_dehome = request.dehome
            final_status = "dehomed" if request.dehome else "homed"
            self._set_status(final_status)
            if (
                claimed
                and final_status == "homed"
                and self._post_homing_owner_hold_sec > 0.0
            ):
                self.get_logger().info(
                    "Holding homer ownership for "
                    f"{self._post_homing_owner_hold_sec:.2f}s after homed "
                    "for teleop handoff"
                )
                repeats = max(1, int(self._post_homing_owner_hold_sec / 0.05))
                await self._seed_forward_position_commands(
                    dehome=False,
                    hold_home_target=True,
                    reason="post-homed owner hold",
                    repeats=repeats,
                    interval_sec=0.05,
                )
                await self._seed_commanded_ee_targets(
                    reason="post-homed owner hold",
                    repeats=repeats,
                    interval_sec=0.05,
                )
        except Exception as e:
            self.get_logger().error(f"Reset failed: {e}")
            self._set_status("failed")
        finally:
            if claimed:
                self._controls_owner_manager.release("homer")


def main(args=None):
    rclpy.init(args=args)

    node = ArmsResetterNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        # Don't log anything here since ros might already be shutting down.
        pass
    finally:
        node.destroy_node()
        # Only call shutdown if we are not already shutting down.
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
