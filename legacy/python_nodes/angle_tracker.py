"""单轴角度恒速跟踪；单位：度、秒。"""
import math


class AngleTracker:
    def __init__(self, max_gap=0.3, jump_threshold=5.0, alpha=0.5):
        values = (max_gap, jump_threshold, alpha)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Parameters must be finite")
        if max_gap <= 0 or jump_threshold <= 0 or not 0 < alpha <= 1:
            raise ValueError("Invalid tracker parameters")
        self.max_gap = max_gap
        self.jump_threshold = jump_threshold
        self.alpha = alpha
        self.reset()

    def reset(self):
        self.angle = None
        self.stamp = None
        self.velocity = 0.0
        self.ready = False

    def update(self, angle, stamp):
        angle, stamp = float(angle), float(stamp)
        if not math.isfinite(angle) or not math.isfinite(stamp):
            self.reset()
            return False

        if self.stamp is None:
            self.angle, self.stamp = angle, stamp
            return False

        dt = stamp - self.stamp
        # 拒绝重复或乱序数据，不覆盖已有状态。
        if dt <= 0:
            return False

        # 最短角差，处理跨越 ±180°。
        delta = (angle - self.angle + 180.0) % 360.0 - 180.0
        if dt > self.max_gap or abs(delta) > self.jump_threshold:
            self.reset()
            self.angle, self.stamp = angle, stamp
            return False

        measured_velocity = delta / dt
        self.velocity = (
            self.alpha * measured_velocity
            + (1.0 - self.alpha) * self.velocity
            if self.ready else measured_velocity
        )
        self.angle += delta
        self.stamp = stamp
        self.ready = True
        return True

    def predict(self, horizon):
        horizon = float(horizon)
        if not math.isfinite(horizon) or not 0 <= horizon <= 0.5:
            raise ValueError("Prediction horizon must be within 0–0.5 s")
        if not self.ready:
            return None
        return self.angle + self.velocity * horizon
