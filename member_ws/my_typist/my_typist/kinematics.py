import math


class ArmKinematics:
    def __init__(self):
        self.L0 = 0.3
        self.L1 = 0.6
        self.L2 = 0.4

    def get_arm_joint_positions(self, x, y, z):
        z -= self.L0
        d = math.sqrt(x * x + y * y + z * z)
        theta0 = math.atan2(y, x)
        theta1 = math.atan2(z, math.sqrt(x * x + y * y)) + math.acos(
            (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        )
        theta2 = -math.acos((d**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2))

        if (
            math.radians(-120) > theta0 > math.radians(120)
            or math.radians(-30) > theta1 > math.radians(100)
            or math.radians(-140) > theta2 > math.radians(0)
        ):
            return None

        return (theta0, theta1, theta2)

    def get_arm_location(self, theta0, theta1, theta2):
        pr = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        x = pr * math.cos(theta0)
        y = pr * math.sin(theta0)
        z = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2) + self.L0
        return (x, y, z)
