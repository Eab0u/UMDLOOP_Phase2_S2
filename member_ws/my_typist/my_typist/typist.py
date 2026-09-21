import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
from autotype_msgs.msg import JointVelocityCommand
from my_typist.kinematics import ArmKinematics
from my_typist.pid import PID
from my_typist.data import (
    BASE_HEIGHT,
    DISTANCE_FROM_KEYBOARD,
    DT,
    FOREARM,
    JOINT_NAMES,
    KEY_LAYOUT,
    KEY_UNIT,
    KEYBOARD_CENTER_OFFSET,
    KEYBOARD_VIEW_OFFSET,
    STYLUS_MAX_REACH,
    STYLUS_MIN_REACH,
    UPPER_ARM,
    cross,
    dist,
)


KEY = "P"
KEYBOARD_CORNERS = [
    (1.07599662905791, 0.200783625138841, 0.374876995935496),
    (1.03800682995812, -0.138681282925482, 0.421226268874437),
    (1.02851756549849, -0.150568592180247, 0.305319834998152),
    (1.06678379540004, 0.188223530253409, 0.257635975726943),
]


class Typist(Node):
    def __init__(self) -> None:
        super().__init__("typist")

        self.declare_parameter("kp", 2.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.1)

        self.arm_target = (0, 0, 0)
        self.stylus_target = (0, 0, 0)
        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kd = self.get_parameter("kd").value

        self.kinematics = ArmKinematics()
        self.pids = [PID(kp, ki, kd, dt=DT) for _ in range(len(JOINT_NAMES))]
        self.q = [0.0] * len(JOINT_NAMES)
        self.launch_key = ""
        self.current_key = 0
        self.current_state = "idle"
        self.idle_time = 0

        self.joint_velocity_publisher = self.create_publisher(
            JointVelocityCommand,
            "/arm/cmd_joint_velocity",
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.key_press_publisher = self.create_publisher(
            Empty,
            "/arm/press",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.done_publisher = self.create_publisher(
            Empty,
            "/sim/done",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self.launch_key = self.create_subscription(
            String,
            "/sim/launch_key",
            self.launch_key_callback,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        self.get_arm_target(*KEYBOARD_CORNERS)

    def launch_key_callback(self, msg: String) -> None:
        self.launch_key = msg.data
        if len(self.launch_key) == 0:
            self.get_logger().error(f"Invalid Launch Key {self.launch_key}")

        self.get_logger().info(f"Recived Launch Key: {self.launch_key}")
        self.current_state = "get_target"
        self.current_key = 0
        self.create_timer(DT, self.control_loop)

    def joint_state_callback(self, msg: JointState) -> None:
        state = dict(zip(msg.name, msg.position))
        for i, name in enumerate(JOINT_NAMES):
            if name in state:
                self.q[i] = float(state[name])

    def get_arm_target(self, kb_tl, kb_tr, kb_br, kb_bl):
        corners = [kb_tl, kb_tr, kb_br, kb_bl]
        keyboard_center = tuple(
            (tl + tr + br + bl) / 4
            for tl, tr, br, bl in zip(kb_tl, kb_tr, kb_br, kb_bl)
        )
        row = [a - b for a, b in zip(kb_tl, kb_tr)]
        row_len = math.sqrt(sum(a * a for a in row))
        if row_len < 1e-9:
            self.get_logger().error("bad keyboard corners, no left axis")
            return
        left = [a / row_len for a in row]
        keyboard_center = [
            a + b * KEYBOARD_CENTER_OFFSET for a, b in zip(keyboard_center, left)
        ]
        keyboard_center[2] += KEYBOARD_VIEW_OFFSET
        self.get_logger().info(f"Keyboard Center: {keyboard_center}")
        edge1 = [a - b for a, b in zip(kb_tr, kb_tl)]
        edge2 = [a - b for a, b in zip(kb_bl, kb_tl)]
        normal = list(cross(edge1, edge2))
        norm_len = math.sqrt(sum(a * a for a in normal))
        if norm_len < 1e-9:
            self.get_logger().error("bad keyboard corners, no plane normal")
            return
        normal = [a / norm_len for a in normal]

        shoulder = (0.0, 0.0, BASE_HEIGHT)
        to_shoulder = [a - b for a, b in zip(shoulder, keyboard_center)]
        if sum(a * b for a, b in zip(normal, to_shoulder)) < 0.0:
            normal = [-a for a in normal]

        arm_reach = FOREARM + UPPER_ARM
        arm_margin = 0.02

        candidate = [
            a + b * DISTANCE_FROM_KEYBOARD for a, b in zip(keyboard_center, normal)
        ]
        over = dist(candidate, shoulder) - (arm_reach - arm_margin)
        if over > 0.0:
            direction = [
                (a - b) / dist(candidate, shoulder) for a, b in zip(candidate, shoulder)
            ]
            candidate = [a - b * over for a, b in zip(candidate, direction)]
        self.arm_target = candidate

        if not all(
            STYLUS_MIN_REACH <= dist(corner, self.arm_target) <= STYLUS_MAX_REACH
            for corner in corners
        ):
            self.get_logger().error("not all corners are within stylus range")

        self.get_logger().info(f"Arm Target: {self.arm_target}")

        if not all(
            STYLUS_MIN_REACH < dist(corner, self.arm_target) < STYLUS_MAX_REACH
            for corner in corners
        ):
            self.get_logger().error("not all corners are reachable")

    def get_stylus_target(self, key, kb_tl, kb_tr, kb_br, kb_bl):
        key = key.upper()

        if key not in KEY_LAYOUT:
            self.get_logger().error(f"Unknown key: {key}")
            return

        x, y, width = KEY_LAYOUT[key]

        u = (x + width / 2.0) * KEY_UNIT
        v = (y + 0.5) * KEY_UNIT

        keyboard_width = 18.25 * KEY_UNIT
        keyboard_height = 6.25 * KEY_UNIT

        s = u / keyboard_width
        t = v / keyboard_height

        # Bilinear interpolation
        self.stylus_target = (
            (1 - s) * (1 - t) * kb_tl[0]
            + s * (1 - t) * kb_tr[0]
            + s * t * kb_br[0]
            + (1 - s) * t * kb_bl[0],
            (1 - s) * (1 - t) * kb_tl[1]
            + s * (1 - t) * kb_tr[1]
            + s * t * kb_br[1]
            + (1 - s) * t * kb_bl[1],
            (1 - s) * (1 - t) * kb_tl[2]
            + s * (1 - t) * kb_tr[2]
            + s * t * kb_br[2]
            + (1 - s) * t * kb_bl[2],
        )

    def control_loop(self) -> None:
        velocities = [0.0] * len(JOINT_NAMES)
        if self.current_state == "get_target":
            self.get_stylus_target(self.launch_key[self.current_key], *KEYBOARD_CORNERS)
            self.get_logger().info(
                f"Calculated Stylus Target for key {
                    self.launch_key[self.current_key]
                }: {self.stylus_target}"
            )
            self.current_state = "moving"
        elif self.current_state == "next":
            if self.idle_time > 0:
                self.idle_time -= DT
            else:
                self.get_logger().info(
                    f"Pressing Key {self.launch_key[self.current_key]}"
                )
                self.key_press_publisher.publish(Empty())
                self.current_key += 1
                if self.current_key >= len(self.launch_key):
                    self.get_logger().info(
                        f"Finished Launch Key: {self.launch_key}, Sending Done"
                    )
                    self.done_publisher.publish(Empty())
                    self.current_state = "idle"
                    return
                else:
                    self.current_state = "get_target"
        elif self.current_state == "moving":
            q_arm = self.kinematics.get_arm_joint_positions(*self.arm_target)
            if q_arm is None:
                self.get_logger().warning(
                    "target out of reach", throttle_duration_sec=1.0
                )
                return

            q_stylus = self.kinematics.get_stylus_joint_positions(
                *self.stylus_target, *self.q[:3]
            )
            if q_stylus is None:
                self.get_logger().warning(
                    "stylus cannot reach from desired arm pose",
                    throttle_duration_sec=1.0,
                )
                return

            q_des = [i for i in q_arm] + [i for i in q_stylus]

            for i, pid in enumerate(self.pids):
                if i >= len(q_des) or q_des[i] is None:
                    continue
                velocities[i] = pid.update(q_des[i] - self.q[i], DT)

            if all(abs(v) < 1e-2 for v in velocities):
                self.idle_time = 0.5
                self.current_state = "next"

        msg = JointVelocityCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.velocity = velocities
        self.joint_velocity_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    typist = Typist()
    rclpy.spin(typist)
    typist.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
