import math
from my_typist.data import (
    BASE_HEIGHT,
    UPPER_ARM,
    FOREARM,
    HEAD_PAN_LIMIT,
    HEAD_TILT_LIMIT,
    normalized,
    solve,
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

        return [theta0, theta1, theta2]

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

    def get_stylus_aim(self, theta0, theta1, theta2, theta3, theta4):
        c0, s0 = math.cos(theta0), math.sin(theta0)
        phi = theta1 + theta2
        x_head = (math.cos(phi) * c0, math.cos(phi) * s0, math.sin(phi))
        y_head = (-s0, c0, 0)
        z_head = (-math.sin(phi) * c0, -math.sin(phi) * s0, math.cos(phi))

        u_head = (
            math.cos(theta4) * math.cos(theta3),
            math.cos(theta4) * math.sin(theta3),
            math.sin(theta4),
        )

        aim_world = tuple(
            x * u_head[0] + y * u_head[1] + z * u_head[2]
            for x, y, z in zip(x_head, y_head, z_head)
        )

        return aim_world

    def get_stylus_joint_positions(self, x, y, z, theta0, theta1, theta2):
        p2 = self.get_arm_location(theta0, theta1, theta2)
        p3 = (x, y, z)
        c0, s0 = math.cos(theta0), math.sin(theta0)
        phi = theta1 + theta2
        uw = normalized([a - b for a, b in zip(p3, p2)])
        R = [
            [math.cos(phi) * c0, math.cos(phi) * s0, math.sin(phi)],
            [-s0, c0, 0],
            [-math.sin(phi) * c0, -math.sin(phi) * s0, math.cos(phi)],
        ]
        us = solve(R, uw)

        pan = math.atan2(us[1], us[0])
        tilt = math.asin(us[2])
        return [pan, tilt]
