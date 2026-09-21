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

    def get_stylus_joint_positions(self, x, y, z, q0, q1, q2):
        px, py, pz = self.get_arm_location(q0, q1, q2)

        # Direction from the stylus/head to the target.
        vx = x - px
        vy = y - py
        vz = z - pz

        length = math.sqrt(vx * vx + vy * vy + vz * vz)
        if length < 1e-9:
            return None

        vx /= length
        vy /= length
        vz /= length

        # Direction the head is pointing before applying head pan/tilt.
        phi = q1 + q2
        c0 = math.cos(q0)
        s0 = math.sin(q0)

        forward = (
            math.cos(phi) * c0,
            math.cos(phi) * s0,
            math.sin(phi),
        )

        right = (
            -s0,
            c0,
            0.0,
        )

        up = (
            -math.sin(phi) * c0,
            -math.sin(phi) * s0,
            math.cos(phi),
        )

        # Express target direction in the head's coordinate frame.
        fx = vx * forward[0] + vy * forward[1] + vz * forward[2]
        fy = vx * right[0] + vy * right[1] + vz * right[2]
        fz = vx * up[0] + vy * up[1] + vz * up[2]

        pan = math.atan2(fy, fx)
        tilt = math.atan2(fz, math.hypot(fx, fy))

        # if not -HEAD_PAN_LIMIT <= pan <= HEAD_PAN_LIMIT:
        #     return None
        #
        # if not -HEAD_TILT_LIMIT <= tilt <= HEAD_TILT_LIMIT:
        #     return None

        return [pan, tilt]
