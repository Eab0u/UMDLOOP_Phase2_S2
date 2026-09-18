class PID:
    def __init__(self, kp, ki, kd, dt=None) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt or 0.1
        self.prevErr = 0
        self.totalErr = 0

    def update(self, err: float, dt: float | None = None) -> float:
        dt = dt or self.dt
        self.totalErr += err * dt
        derivative = (err - self.prevErr) / dt
        self.prevErr = err
        return self.kp * err + self.kd * derivative + self.ki * self.totalErr
