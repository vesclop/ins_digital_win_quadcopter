"""Кватернионная механизация БИНС и 15-компонентный фильтр ошибок."""

from dataclasses import dataclass
from math import atan2, cos, sin

import numpy as np

GRAVITY_ENU = np.array([0.0, 0.0, -9.80665])


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = left
    w2, x2, y2, z2 = right
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def rotation_vector_from_quaternion(q: np.ndarray) -> np.ndarray:
    q = normalize_quaternion(q)
    if q[0] < 0.0:
        q = -q
    vector_norm = float(np.linalg.norm(q[1:]))
    if vector_norm < 1e-12:
        return 2.0 * q[1:]
    angle = 2.0 * np.arctan2(vector_norm, q[0])
    return q[1:] / vector_norm * angle


def quaternion_from_rotation_vector(vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return normalize_quaternion(np.concatenate(([1.0], vector * 0.5)))
    return np.concatenate(([cos(angle / 2.0)], vector / angle * sin(angle / 2.0)))


def rotation_matrix_from_quaternion(q: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quaternion(q)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ])


def quaternion_from_rotation_matrix(matrix: np.ndarray) -> np.ndarray:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        q = np.array([
            0.25 * scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        ])
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        q = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            0.25 * scale,
            (matrix[0, 1] + matrix[1, 0]) / scale,
            (matrix[0, 2] + matrix[2, 0]) / scale,
        ])
    elif matrix[1, 1] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        q = np.array([
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[0, 1] + matrix[1, 0]) / scale,
            0.25 * scale,
            (matrix[1, 2] + matrix[2, 1]) / scale,
        ])
    else:
        scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        q = np.array([
            (matrix[1, 0] - matrix[0, 1]) / scale,
            (matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[1, 2] + matrix[2, 1]) / scale,
            0.25 * scale,
        ])
    return normalize_quaternion(q)


def rotation_from_specific_force(specific_force_enu: np.ndarray, heading_deg: float) -> np.ndarray:
    heading = np.deg2rad(heading_deg)
    forward = np.array([sin(heading), cos(heading), 0.0])
    body_z = -specific_force_enu / np.linalg.norm(specific_force_enu)
    body_y = np.cross(body_z, forward)
    body_y /= np.linalg.norm(body_y)
    body_x = np.cross(body_y, body_z)
    return np.column_stack((body_x, body_y, body_z))


def heading_from_quaternion(q: np.ndarray) -> float:
    matrix = rotation_matrix_from_quaternion(q)
    return np.rad2deg(atan2(matrix[0, 0], matrix[1, 0])) % 360.0


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


@dataclass
class NavigationSolution:
    position: np.ndarray
    velocity: np.ndarray
    attitude: np.ndarray
    gyro_bias: np.ndarray
    accelerometer_bias: np.ndarray
    covariance: np.ndarray

    @property
    def heading_deg(self) -> float:
        return heading_from_quaternion(self.attitude)


class ErrorStateINS:
    """Ошибки: позиция, скорость, ориентация, смещения гироскопа и акселерометра."""

    def __init__(self, position: np.ndarray, attitude: np.ndarray):
        self.solution = NavigationSolution(
            position.copy(),
            np.zeros(3),
            normalize_quaternion(attitude.copy()),
            np.zeros(3),
            np.zeros(3),
            np.eye(15),
        )
        covariance = self.solution.covariance
        covariance[0:3, 0:3] = np.eye(3) * 1.0**2
        covariance[3:6, 3:6] = np.eye(3) * 0.3**2
        covariance[6:9, 6:9] = np.diag(np.deg2rad([1.0, 1.0, 3.0]) ** 2)
        covariance[9:12, 9:12] = np.eye(3) * np.deg2rad(0.1) ** 2
        covariance[12:15, 12:15] = np.eye(3) * 0.15**2

    def predict(self, gyro_measurement: np.ndarray, accel_measurement: np.ndarray, dt: float) -> None:
        state = self.solution
        matrix = rotation_matrix_from_quaternion(state.attitude)
        omega = gyro_measurement - state.gyro_bias
        specific_force_body = accel_measurement - state.accelerometer_bias
        specific_force_enu = matrix @ specific_force_body
        acceleration_enu = specific_force_enu + GRAVITY_ENU

        state.position += state.velocity * dt + 0.5 * acceleration_enu * dt * dt
        state.velocity += acceleration_enu * dt
        state.attitude = normalize_quaternion(quaternion_multiply(
            state.attitude,
            quaternion_from_rotation_vector(omega * dt),
        ))

        transition = np.eye(15)
        transition[0:3, 3:6] = np.eye(3) * dt
        transition[3:6, 6:9] = skew(specific_force_enu) * dt
        transition[3:6, 12:15] = -matrix * dt
        transition[6:9, 9:12] = matrix * dt
        process_noise = np.zeros((15, 15))
        process_noise[3:6, 3:6] = np.eye(3) * (0.08**2 * dt)
        process_noise[6:9, 6:9] = np.eye(3) * (np.deg2rad(0.003)**2 * dt)
        process_noise[9:12, 9:12] = np.eye(3) * (np.deg2rad(0.00005)**2 * dt)
        process_noise[12:15, 12:15] = np.eye(3) * (0.00008**2 * dt)
        covariance = transition @ state.covariance @ transition.T + process_noise
        state.covariance = (covariance + covariance.T) * 0.5

    def _update(
        self,
        residual: np.ndarray,
        measurement_matrix: np.ndarray,
        noise: np.ndarray,
        apply_gate: bool = True,
    ) -> bool:
        state = self.solution
        covariance = state.covariance
        innovation_covariance = measurement_matrix @ covariance @ measurement_matrix.T + noise
        normalized = float(residual.T @ np.linalg.solve(innovation_covariance, residual))
        gate = 9.0 if residual.size == 1 else 16.27
        if not np.isfinite(normalized) or (apply_gate and normalized > gate):
            return False
        gain = covariance @ measurement_matrix.T @ np.linalg.inv(innovation_covariance)
        correction = gain @ residual
        identity = np.eye(15)
        residual_matrix = identity - gain @ measurement_matrix
        state.covariance = (
            residual_matrix @ covariance @ residual_matrix.T
            + gain @ noise @ gain.T
        )
        state.covariance = (state.covariance + state.covariance.T) * 0.5

        state.position += correction[0:3]
        state.velocity += correction[3:6]
        state.attitude = normalize_quaternion(quaternion_multiply(
            quaternion_from_rotation_vector(-correction[6:9]),
            state.attitude,
        ))
        state.gyro_bias += correction[9:12]
        state.accelerometer_bias += correction[12:15]
        return True

    def update_position(self, measured_position: np.ndarray, sigma_m: np.ndarray) -> bool:
        matrix = np.zeros((3, 15))
        matrix[0:3, 0:3] = np.eye(3)
        return self._update(
            measured_position - self.solution.position,
            matrix,
            np.diag(sigma_m**2),
        )

    def update_horizontal_position(self, measured_position: np.ndarray, sigma_m: float) -> bool:
        matrix = np.zeros((2, 15))
        matrix[0:2, 0:2] = np.eye(2)
        return self._update(
            measured_position - self.solution.position[:2],
            matrix,
            np.eye(2) * sigma_m**2,
        )

    def update_velocity(self, measured_velocity: np.ndarray, sigma_m_s: np.ndarray) -> bool:
        matrix = np.zeros((3, 15))
        matrix[0:3, 3:6] = np.eye(3)
        return self._update(
            measured_velocity - self.solution.velocity,
            matrix,
            np.diag(sigma_m_s**2),
            apply_gate=False,
        )

    def update_altitude(self, measured_altitude: float, sigma_m: float) -> bool:
        matrix = np.zeros((1, 15))
        matrix[0, 2] = 1.0
        return self._update(
            np.array([measured_altitude - self.solution.position[2]]),
            matrix,
            np.array([[sigma_m**2]]),
        )

    def update_heading(
        self,
        measured_heading_deg: float,
        sigma_deg: float,
        apply_gate: bool = True,
    ) -> bool:
        matrix = np.zeros((1, 15))
        matrix[0, 8] = 1.0
        residual_deg = (measured_heading_deg - self.solution.heading_deg + 180.0) % 360.0 - 180.0
        return self._update(
            np.array([np.deg2rad(residual_deg)]),
            matrix,
            np.array([[np.deg2rad(sigma_deg)**2]]),
            apply_gate=apply_gate,
        )
