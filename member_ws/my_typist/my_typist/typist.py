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

JOINT_NAMES = [
    "base_yaw",
    "shoulder_pitch",
    "elbow_pitch",
    "head_pan",
    "head_tilt",
]

RATE = 50.0
DT = 1.0 / RATE

UPPER_ARM = 0.6
FOREARM = 0.4
BASE_HEIGHT = 0.3

HEAD_PAN_LIMIT = math.radians(45.0)
HEAD_TILT_LIMIT = math.radians(35.0)


class Typist(Node):
    def __init__(self) -> None:
        super().__init__("typist")

        self.declare_parameter("target_x", 0.85)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 0.62)
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.1)

        self.target = (
            self.get_parameter("target_x").value,
            self.get_parameter("target_y").value,
            self.get_parameter("target_z").value,
        )
        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kd = self.get_parameter("kd").value

        self.kinematics = ArmKinematics()
        self.pids = [PID(kp, ki, kd, dt=DT) for _ in range(len(JOINT_NAMES))]
        self.q = [0.0] * len(JOINT_NAMES)

        q_des = self.kinematics.get_arm_joint_positions(*self.target) or []
        print(*[math.degrees(x) for x in q_des])

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
        self.create_timer(DT, self.control_loop)

    def joint_state_callback(self, msg: JointState) -> None:
        state = dict(zip(msg.name, msg.position))
        for i, name in enumerate(JOINT_NAMES):
            if name in state:
                self.q[i] = float(state[name])

    def control_loop(self) -> None:
        velocities = [0.0] * len(JOINT_NAMES)
        q_des = self.kinematics.get_arm_joint_positions(*self.target)
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
