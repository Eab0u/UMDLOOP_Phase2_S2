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

from autotype_msgs.msg import JointVelocityCommand
from my_typist.kinematics import ArmKinematics
from my_typist.pid import PID
from my_typist.data import (
    BASE_HEIGHT,
    DT,
    FOREARM,
    JOINT_NAMES,
    STYLUS_MAX_REACH,
    STYLUS_MIN_REACH,
    UPPER_ARM,
    cross,
)


KEYBOARD_CORNERS = [
    (0.97, 0.17, 0.39),
    (0.97, -0.14, 0.43),
    (0.97, -0.15, 0.32),
    (0.93, 0.16, 0.32),
]


class Typist(Node):
    def __init__(self) -> None:
        super().__init__("typist")

        self.declare_parameter("kp", 2.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.1)

        self.arm_target = (0, 0, 0)
        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kd = self.get_parameter("kd").value

        self.kinematics = ArmKinematics()
        self.pids = [PID(kp, ki, kd, dt=DT) for _ in range(len(JOINT_NAMES))]
        self.q = [0.0] * len(JOINT_NAMES)

        self.joint_velocity_publisher = self.create_publisher(
            JointVelocityCommand,
            "/arm/cmd_joint_velocity",
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
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

        self.setup()

    def setup(self):
        self.get_arm_target(*KEYBOARD_CORNERS)

        q_des = self.kinematics.get_arm_joint_positions(*self.arm_target) or []
        print(f"Arm Angles: {[math.degrees(x) for x in q_des]}")

        self.create_timer(DT, self.control_loop)

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
        keyboard_center = tuple(a + b * 0.025 for a, b in zip(keyboard_center, left))
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
        stylus_lo = STYLUS_MIN_REACH + 0.01
        stylus_hi = STYLUS_MAX_REACH - 0.01

        def dist(a, b):
            return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

        def is_feasible(p):
            if dist(p, shoulder) >= arm_reach - arm_margin:
                return False
            if self.kinematics.get_arm_joint_positions(*p) is None:
                return False
            return all(stylus_lo < dist(c, p) < stylus_hi for c in corners)

        self.arm_target = None
        standoff = 0.05
        while standoff <= 0.40:
            candidate = [a + b * standoff for a, b in zip(keyboard_center, normal)]
            if is_feasible(candidate):
                self.arm_target = candidate
                break
            standoff += 0.005

        if self.arm_target is None:
            candidate = [a + b * 0.05 for a, b in zip(keyboard_center, normal)]
            over = dist(candidate, shoulder) - (arm_reach - arm_margin)
            if over > 0.0:
                direction = [
                    (a - b) / dist(candidate, shoulder)
                    for a, b in zip(candidate, shoulder)
                ]
                candidate = [a - b * over for a, b in zip(candidate, direction)]
            self.arm_target = candidate
            self.get_logger().error("not all corners are reachable")

        self.get_logger().info(f"Arm Target: {self.arm_target}")

        if not all(
            STYLUS_MIN_REACH < dist(corner, self.arm_target) < STYLUS_MAX_REACH
            for corner in corners
        ):
            self.get_logger().error("not all corners are reachable")

    def joint_state_callback(self, msg: JointState) -> None:
        state = dict(zip(msg.name, msg.position))
        for i, name in enumerate(JOINT_NAMES):
            if name in state:
                self.q[i] = float(state[name])
        # q_des = self.kinematics.get_arm_joint_positions(*self.arm_target)
        # print(f"Arm Position: {q_des}")

    def control_loop(self) -> None:
        velocities = [0.0] * len(JOINT_NAMES)
        q_des = self.kinematics.get_arm_joint_positions(*self.arm_target)
        if q_des is None:
            self.get_logger().warning("target out of reach", throttle_duration_sec=1.0)
        else:
            for i, pid in enumerate(self.pids):
                if i >= len(q_des):
                    continue
                velocities[i] = pid.update(q_des[i] - self.q[i], DT)

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
