"""Autonomous ROS 2 controller for the UMD Loop S2 typing challenge."""

import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from autotype_msgs.msg import EpisodeResult, JointVelocityCommand
from my_typist.data import DT, JOINT_NAMES, KEY_LAYOUT, KEY_UNIT, V_MAX
from my_typist.kinematics import ArmKinematics
from my_typist.mission_log import MissionLog
from my_typist.perception import (
    camera_pose_world,
    estimate_board,
    load_keyboard_template,
)
from my_typist.pid import PID
from my_typist.planning import choose_typing_pose
from my_typist.safety import StalenessWatchdog, clamp_velocities
from my_typist.vision_log import VisionLog


QOS_RELIABLE = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)
QOS_COMMAND = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)
QOS_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)
QOS_SENSOR = QoSProfile(
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)


class Typist(Node):
    def __init__(self) -> None:
        super().__init__("typist")

        self.declare_parameter("kp", 2.2)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.08)
        self.declare_parameter("episodes", 1)
        self.declare_parameter("log_path", "/ws/control_log.csv")
        self.declare_parameter("vision_log_path", "/ws/perception_log.csv")
        kp = float(self.get_parameter("kp").value)
        ki = float(self.get_parameter("ki").value)
        kd = float(self.get_parameter("kd").value)
        self.episodes_target = max(1, int(self.get_parameter("episodes").value))

        log_path = str(self.get_parameter("log_path").value)
        log_directory = os.path.dirname(log_path)
        if log_directory:
            os.makedirs(log_directory, exist_ok=True)
        self.mission_log = MissionLog(log_path)
        self.vision_log = VisionLog(str(self.get_parameter("vision_log_path").value))
        self.kinematics = ArmKinematics()
        self.pids = [PID(kp, ki, kd, dt=DT) for _ in JOINT_NAMES]

        self.q = [0.0] * len(JOINT_NAMES)
        self.qd = [0.0] * len(JOINT_NAMES)
        self.have_joint_state = False
        self.camera_matrix = None
        self.distortion = np.zeros(5)
        self.keyboard_template = None
        try:
            self.keyboard_template = load_keyboard_template()
        except Exception as exc:
            self.get_logger().error(f"Cannot load keyboard reference: {exc}")

        self.launch_key = ""
        self.current_key = 0
        self.current_state = "waiting_for_launch_key"
        self.plan = None
        self.board_rotation_world = None
        self.board_translation_world = None
        self.key_targets_world = []
        self.settle_started = None
        self.wait_until = 0.0
        self.last_perception_error = ""
        self.episodes_completed = 0
        self.reset_future = None

        self.joint_state_watchdog = StalenessWatchdog(timeout=0.25)
        self.camera_watchdog = StalenessWatchdog(timeout=0.50)

        self.joint_velocity_publisher = self.create_publisher(
            JointVelocityCommand, "/arm/cmd_joint_velocity", QOS_COMMAND
        )
        self.key_press_publisher = self.create_publisher(
            Empty, "/arm/press", QOS_RELIABLE
        )
        self.done_publisher = self.create_publisher(Empty, "/sim/done", QOS_RELIABLE)
        self.debug_image_publisher = self.create_publisher(
            Image, "/typist/debug_image", QOS_SENSOR
        )

        self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, QOS_RELIABLE
        )
        self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, QOS_SENSOR
        )
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.camera_info_callback, QOS_LATCHED
        )
        self.create_subscription(
            String, "/sim/launch_key", self.launch_key_callback, QOS_LATCHED
        )
        self.create_subscription(
            EpisodeResult, "/sim/result", self.result_callback, QOS_LATCHED
        )
        self.reset_client = self.create_client(Trigger, "/sim/reset")

        self.control_timer = self.create_timer(DT, self.control_loop)
        self.get_logger().info(
            "S2 typist ready; waiting for camera, joints, and launch key"
        )

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def reset_pids(self) -> None:
        for pid in self.pids:
            # Preserve the team's PID module; clear its public controller
            # state here when perception selects a new target.
            pid.prevErr = 0.0
            pid.totalErr = 0.0

    def launch_key_callback(self, msg: String) -> None:
        launch_key = msg.data.strip().upper()
        if not 3 <= len(launch_key) <= 6 or any(
            ch not in KEY_LAYOUT for ch in launch_key
        ):
            self.get_logger().error(f"Invalid launch key: {msg.data!r}")
            return
        self.launch_key = launch_key
        self.current_key = 0
        self.current_state = "waiting_for_perception"
        self.plan = None
        self.board_rotation_world = None
        self.board_translation_world = None
        self.key_targets_world = []
        self.settle_started = None
        self.last_perception_error = ""
        self.reset_pids()
        self.vision_log.write(self.now_seconds(), "episode", launch_key)
        self.get_logger().info(f"Received launch key: {launch_key}")

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(msg.d, dtype=np.float64)

    def joint_state_callback(self, msg: JointState) -> None:
        now = self.now_seconds()
        self.joint_state_watchdog.touch(now)
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        for i, name in enumerate(JOINT_NAMES):
            if name in positions:
                self.q[i] = float(positions[name])
            if name in velocities:
                self.qd[i] = float(velocities[name])
        self.have_joint_state = all(name in positions for name in JOINT_NAMES)

    @staticmethod
    def image_to_numpy(msg: Image) -> np.ndarray:
        if msg.encoding.lower() != "bgr8":
            raise ValueError(f"expected bgr8 camera data, got {msg.encoding!r}")
        packed = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        return packed[:, : msg.width * 3].reshape(msg.height, msg.width, 3).copy()

    def image_callback(self, msg: Image) -> None:
        now = self.now_seconds()
        self.camera_watchdog.touch(now)
        if self.current_state != "waiting_for_perception":
            return
        if (
            self.camera_matrix is None
            or not self.have_joint_state
            or not self.launch_key
            or self.keyboard_template is None
        ):
            return
        if max(abs(v) for v in self.qd) > 0.02:
            return

        try:
            image = self.image_to_numpy(msg)
            detection = estimate_board(
                image, self.camera_matrix, self.distortion, self.keyboard_template
            )
            if detection.reprojection_error_px > 3.0:
                raise ValueError(
                    f"marker reprojection error is {
                        detection.reprojection_error_px:.2f}px"
                )
            if detection.match_score < 0.20:
                raise ValueError(
                    f"keyboard image match is too weak ({detection.match_score:.2f})"
                )

            rotation_world_camera, translation_world_camera = camera_pose_world(self.q)
            self.board_rotation_world = (
                rotation_world_camera @ detection.rotation_camera_board
            )
            self.board_translation_world = (
                rotation_world_camera @ detection.translation_camera_board
                + translation_world_camera
            )
            normal_toward_arm = -self.board_rotation_world[:, 2]

            self.key_targets_world = []
            for key in self.launch_key:
                x, y, width = KEY_LAYOUT[key]
                point_board = np.array(
                    [
                        detection.key_origin_board[0] + (x + width / 2.0) * KEY_UNIT,
                        detection.key_origin_board[1] + (y + 0.5) * KEY_UNIT,
                        0.0,
                    ]
                )
                point_world = (
                    self.board_rotation_world @ point_board
                    + self.board_translation_world
                )
                self.key_targets_world.append(point_world)

            self.plan = choose_typing_pose(self.key_targets_world, normal_toward_arm)
            self.current_state = "moving"
            self.current_key = 0
            self.reset_pids()
            self.publish_debug_image(msg, detection.debug_image)
            values = (
                f"markers={detection.marker_ids}; reprojection_px="
                f"{detection.reprojection_error_px:.3f}; match={
                    detection.match_score:.3f}; "
                f"key_origin={detection.key_origin_board.tolist()}"
            )
            self.vision_log.write(now, "perception", "board_and_keyboard", values)
            self.vision_log.write(
                now,
                "plan",
                "typing_pose",
                f"q_arm={list(self.plan.arm_joints)}; head={
                    list(self.plan.head_position)
                }",
            )
            self.get_logger().info(
                "Perception locked: "
                f"{len(detection.marker_ids)} markers, "
                f"{detection.reprojection_error_px:.2f}px pose error, "
                f"keyboard score {detection.match_score:.2f}"
            )
        except Exception as exc:
            error = str(exc)
            if error != self.last_perception_error:
                self.get_logger().warning(f"Perception not ready: {error}")
                self.last_perception_error = error

    def publish_debug_image(self, source: Image, image: np.ndarray) -> None:
        msg = Image()
        msg.header = source.header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = image.tobytes()
        self.debug_image_publisher.publish(msg)

    def desired_joints(self):
        target = self.key_targets_world[self.current_key]
        pan_tilt = self.kinematics.get_stylus_joint_positions(
            *target, *self.plan.arm_joints
        )
        return list(self.plan.arm_joints) + list(pan_tilt)

    def publish_velocities(self, velocities, state_for_log=None) -> None:
        velocities = clamp_velocities(velocities, V_MAX)
        msg = JointVelocityCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.velocity = list(velocities)
        self.joint_velocity_publisher.publish(msg)
        self.mission_log.log_command(
            self.now_seconds(), state_for_log or self.current_state, velocities
        )

    def control_loop(self) -> None:
        now = self.now_seconds()
        active = self.current_state in {
            "moving",
            "settling",
            "wait_after_press",
            "wait_before_done",
        }
        if active and (
            self.joint_state_watchdog.is_stale(now)
            or self.camera_watchdog.is_stale(now)
        ):
            self.publish_velocities([0.0] * len(JOINT_NAMES), "safe_hold_stale_data")
            self.get_logger().error(
                "Camera or joint data is stale; commanding safe hold",
                throttle_duration_sec=1.0,
            )
            return

        if self.current_state == "moving":
            desired = self.desired_joints()
            errors = [goal - actual for goal, actual in zip(desired, self.q)]
            velocities = [
                pid.update(error, DT) for pid, error in zip(self.pids, errors)
            ]
            if max(abs(error) for error in errors) < 0.0025:
                self.current_state = "settling"
                self.settle_started = None
                velocities = [0.0] * len(JOINT_NAMES)
            self.publish_velocities(velocities)
            return

        if self.current_state == "settling":
            desired = self.desired_joints()
            errors = [goal - actual for goal, actual in zip(desired, self.q)]
            if max(abs(error) for error in errors) > 0.005:
                self.current_state = "moving"
                self.settle_started = None
                self.reset_pids()
                self.publish_velocities([0.0] * len(JOINT_NAMES), "correcting_drift")
                return
            self.publish_velocities([0.0] * len(JOINT_NAMES))
            if max(abs(speed) for speed in self.qd) <= 0.008:
                if self.settle_started is None:
                    self.settle_started = now
                elif now - self.settle_started >= 0.25:
                    key = self.launch_key[self.current_key]
                    self.key_press_publisher.publish(Empty())
                    self.mission_log.log_press(now, key)
                    self.get_logger().info(
                        f"Pressed {key} ({self.current_key + 1}/{len(self.launch_key)})"
                    )
                    self.current_key += 1
                    if self.current_key >= len(self.launch_key):
                        # Reliable delivery is ordered per publisher, not across
                        # two different topics.  Give the final /arm/press time
                        # to arrive before ending the episode on /sim/done.
                        self.wait_until = now + 0.20
                        self.current_state = "wait_before_done"
                    else:
                        self.wait_until = now + 0.20
                        self.current_state = "wait_after_press"
                        self.settle_started = None
                        self.reset_pids()
            else:
                self.settle_started = None
            return

        if self.current_state == "wait_after_press":
            self.publish_velocities([0.0] * len(JOINT_NAMES))
            if now >= self.wait_until:
                self.current_state = "moving"
                self.reset_pids()
            return

        if self.current_state == "wait_before_done":
            self.publish_velocities([0.0] * len(JOINT_NAMES))
            if now >= self.wait_until:
                self.done_publisher.publish(Empty())
                self.current_state = "finished"
                self.get_logger().info("Launch key complete; published /sim/done")
            return

        if self.current_state == "finished":
            self.publish_velocities([0.0] * len(JOINT_NAMES))

    def result_callback(self, msg: EpisodeResult) -> None:
        if not self.launch_key or self.current_state != "finished":
            return
        self.episodes_completed += 1
        summary = (
            f"target={msg.target}; typed={msg.typed}; exact={msg.exact_match}; "
            f"attempted={msg.presses_attempted}; accepted={msg.presses_accepted}; "
            f"elapsed={msg.elapsed:.3f}"
        )
        self.vision_log.write(self.now_seconds(), "result", "episode", summary)
        if msg.exact_match:
            self.get_logger().info(
                f"SUCCESS: typed {msg.typed!r} in {msg.elapsed:.2f}s"
            )
        else:
            self.get_logger().error(
                f"FAILED: target={msg.target!r}, typed={msg.typed!r}, "
                f"edit distance={msg.edit_distance}"
            )

        if self.episodes_completed < self.episodes_target:
            if self.reset_client.service_is_ready():
                self.get_logger().info(
                    f"Requesting next seed ({self.episodes_completed}/{
                        self.episodes_target
                    })"
                )
                self.reset_future = self.reset_client.call_async(Trigger.Request())
            else:
                self.get_logger().error(
                    "/sim/reset is unavailable; cannot run next episode"
                )

    def destroy_node(self):
        self.mission_log.close()
        self.vision_log.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    typist = Typist()
    try:
        rclpy.spin(typist)
    except KeyboardInterrupt:
        pass
    finally:
        typist.publish_velocities([0.0] * len(JOINT_NAMES), "shutdown")
        typist.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
