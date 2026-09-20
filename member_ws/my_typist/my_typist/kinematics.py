import math
from my_typist.data import (
    BASE_HEIGHT,
    UPPER_ARM,
    FOREARM,
    HEAD_PAN_LIMIT,
    HEAD_TILT_LIMIT,
)


class ArmKinematics:
    def __init__(self):
        pass

    def get_arm_joint_positions(self, x, y, z):
        z -= BASE_HEIGHT
        d = math.sqrt(x * x + y * y + z * z)
        if d >= UPPER_ARM + FOREARM or d <= abs(UPPER_ARM - FOREARM):
            return None
        if d < 1e-9:
            return None
        cos_alpha = (UPPER_ARM**2 + d**2 - FOREARM**2) / (2 * UPPER_ARM * d)
        cos_beta = (d**2 - UPPER_ARM**2 - FOREARM**2) / (2 * UPPER_ARM * FOREARM)
        cos_alpha = max(-1.0, min(1.0, cos_alpha))
        cos_beta = max(-1.0, min(1.0, cos_beta))
        theta0 = math.atan2(y, x)
        theta1 = math.atan2(z, math.sqrt(x * x + y * y)) + math.acos(cos_alpha)
        theta2 = -math.acos(cos_beta)

        if not (
            math.radians(-120) <= theta0 <= math.radians(120)
            and math.radians(-30) <= theta1 <= math.radians(100)
            and math.radians(-140) <= theta2 <= math.radians(0)
        ):
            return None

        return (theta0, theta1, theta2)

    def get_arm_location(self, theta0, theta1, theta2):
        pr = UPPER_ARM * math.cos(theta1) + FOREARM * math.cos(theta1 + theta2)
        x = pr * math.cos(theta0)
        y = pr * math.sin(theta0)
        z = (
            UPPER_ARM * math.sin(theta1)
            + FOREARM * math.sin(theta1 + theta2)
            + BASE_HEIGHT
        )
        return (x, y, z)

    def get_stylus_joint_positions(self, x, y, z, q0, q1, q2):
        tx, ty, tz = x, y, z
        c0, s0 = math.cos(q0), math.sin(q0)
        c1, s1 = math.cos(q1), math.sin(q1)

        px, py, pz = self.get_arm_location(q0, q1, q2)
        ex = UPPER_ARM * c1 * c0
        ey = UPPER_ARM * c1 * s0
        ez = UPPER_ARM * s1 + BASE_HEIGHT

        x_head = (
            (px - ex) / FOREARM,
            (py - ey) / FOREARM,
            (pz - ez) / FOREARM,
        )
        y_head = (-s0, c0, 0.0)
        z_head = (
            y_head[1] * x_head[2] - y_head[2] * x_head[1],
            y_head[2] * x_head[0] - y_head[0] * x_head[2],
            y_head[0] * x_head[1] - y_head[1] * x_head[0],
        )

        v = (tx - px, ty - py, tz - pz)
        vx = v[0] * x_head[0] + v[1] * x_head[1] + v[2] * x_head[2]
        vy = v[0] * y_head[0] + v[1] * y_head[1] + v[2] * y_head[2]
        vz = v[0] * z_head[0] + v[1] * z_head[1] + v[2] * z_head[2]

        pan = max(-HEAD_PAN_LIMIT, min(HEAD_PAN_LIMIT, math.atan2(vy, vx)))
        tilt = max(
            -HEAD_TILT_LIMIT,
            min(HEAD_TILT_LIMIT, math.atan2(vz, math.hypot(vx, vy))),
        )
        return pan, tilt
