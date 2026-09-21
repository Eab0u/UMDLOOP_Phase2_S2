import math
from my_typist.data import (
    BASE_HEIGHT,
    UPPER_ARM,
    FOREARM,
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
        px, py, pz = self.get_arm_location(theta0, theta1, theta2)
        vx, vy, vz = x - px, y - py, z - pz
        n = math.sqrt(vx * vx + vy * vy + vz * vz)
        if n < 1e-12:
            return [0.0, 0.0]
        ux, uy, uz = vx / n, vy / n, vz / n
        c0, s0 = math.cos(theta0), math.sin(theta0)
        phi = theta1 + theta2
        x_head = (math.cos(phi) * c0, math.cos(phi) * s0, math.sin(phi))
        y_head = (-s0, c0, 0.0)
        z_head = (
            -math.sin(phi) * c0,
            -math.sin(phi) * s0,
            math.cos(phi),
        )
        us0 = ux * x_head[0] + uy * x_head[1] + uz * x_head[2]
        us1 = ux * y_head[0] + uy * y_head[1] + uz * y_head[2]
        us2 = ux * z_head[0] + uy * z_head[1] + uz * z_head[2]

        pan = math.atan2(us1, us0)
        tilt = math.atan2(us2, math.hypot(us0, us1))
        return [pan, tilt]
