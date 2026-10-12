"""Frame and motor-model helpers for SITL calibration tests."""

import math
from dataclasses import dataclass
import numpy as np


class VerticalEstimator:
    """Fuse FC world-vertical inertial acceleration with noisy barometric altitude."""

    def __init__(self):
        self.state = np.zeros(2)
        self.covariance = np.diag([0.25, 0.25])
        self.initialized = False

    @property
    def height(self) -> float:
        return float(self.state[0])

    @property
    def velocity(self) -> float:
        return float(self.state[1])

    def predict(self, acceleration: float, dt: float):
        if not self.initialized or not math.isfinite(acceleration) or not 0 < dt <= 0.2:
            raise ValueError("invalid vertical estimator prediction")
        transition = np.array([[1.0, dt], [0.0, 1.0]])
        gain = np.array([0.5 * dt * dt, dt])
        self.state = transition @ self.state + gain * acceleration
        self.covariance = transition @ self.covariance @ transition.T + np.outer(gain, gain)

    def update(self, altitude: float):
        if not math.isfinite(altitude):
            raise ValueError("non-finite barometric altitude")
        if not self.initialized:
            self.state[0] = altitude
            self.initialized = True
            return
        variance = 0.25
        gain = self.covariance[:, 0] / (self.covariance[0, 0] + variance)
        self.state += gain * (altitude - self.state[0])
        correction = np.eye(2) - np.outer(gain, np.array([1.0, 0.0]))
        self.covariance = correction @ self.covariance @ correction.T + variance * np.outer(gain, gain)


@dataclass(frozen=True)
class MotorModel:
    A: float
    B: float
    C: float
    D: float
    n: int = 1
    thrust_limit: float = 0.5
    spin_k: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in
                   (self.A, self.B, self.C, self.D, self.thrust_limit, self.spin_k)):
            raise ValueError("motor parameters must be finite")
        if self.n < 1 or not 0 < self.thrust_limit <= 1 or not 0 <= self.spin_k <= 1:
            raise ValueError("invalid motor count, thrust limit or spin_k")
        if self.C <= 0:
            raise ValueError("motor_parameters.C must be positive")

    def throttle(self, force: float) -> float:
        if not math.isfinite(force):
            raise ValueError("collective force must be finite")
        # Same per-motor force -> throttle polynomial as quadratic_thrust_model.h.
        x = max(0.0, force) / self.n
        t = ((self.A * x + self.B) * x + self.C) * x + self.D
        t = (1 - self.spin_k) * t + self.spin_k * t * t
        return max(0.0, min(self.thrust_limit, t))


def quaternion(q) -> tuple[float, float, float, float]:
    values = (float(q.x_val), float(q.y_val), float(q.z_val), float(q.w_val))
    norm = math.sqrt(sum(v * v for v in values))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("invalid AirSim quaternion")
    return tuple(v / norm for v in values)


def multiply(a, b) -> tuple[float, float, float, float]:
    x, y, z, w = a
    X, Y, Z, W = b
    return (w * X + x * W + y * Z - z * Y,
            w * Y - x * Z + y * W + z * X,
            w * Z + x * Y - y * X + z * W,
            w * W - x * X - y * Y - z * Z)


def conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def enu_flu(q) -> tuple[float, float, float, float]:
    """R_ENU_FLU = R_ENU_NED * R_NED_FRD * R_FRD_FLU."""
    h = math.sqrt(0.5)
    return multiply(multiply((h, h, 0.0, 0.0), quaternion(q)),
                    (1.0, 0.0, 0.0, 0.0))


def euler(q) -> tuple[float, float, float]:
    x, y, z, w = q
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def specific_force(kinematics, gravity: float) -> tuple[float, float, float]:
    """Ground-truth NED inertial acceleration -> FLU accelerometer reading."""
    a = kinematics.linear_acceleration
    rotated = multiply(multiply(conjugate(quaternion(kinematics.orientation)),
                                (a.x_val, a.y_val, a.z_val - gravity, 0.0)),
                       quaternion(kinematics.orientation))
    return (rotated[0], -rotated[1], -rotated[2])


def projected_force(kinematics, mass: float, gravity: float) -> float:
    return mass * specific_force(kinematics, gravity)[2]
