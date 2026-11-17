"""Модели барометра BMP390 и магнитометра IST8310."""

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, sin

import numpy as np

SEA_LEVEL_PRESSURE_PA = 101_325.0
SEA_LEVEL_TEMPERATURE_K = 288.15
TEMPERATURE_LAPSE_K_M = 0.0065
BAROMETRIC_EXPONENT = 5.25588


def pressure_from_altitude(altitude_m: float) -> float:
    return SEA_LEVEL_PRESSURE_PA * (
        1.0 - TEMPERATURE_LAPSE_K_M * altitude_m / SEA_LEVEL_TEMPERATURE_K
    ) ** BAROMETRIC_EXPONENT


def altitude_from_pressure(pressure_pa: float) -> float:
    return SEA_LEVEL_TEMPERATURE_K / TEMPERATURE_LAPSE_K_M * (
        1.0 - (pressure_pa / SEA_LEVEL_PRESSURE_PA) ** (1.0 / BAROMETRIC_EXPONENT)
    )


@dataclass(frozen=True)
class BarometerReading:
    pressure_pa: float
    altitude_m: float
    temperature_c: float


class BMP390Barometer:
    """Относительная барометрическая высота с шумом и температурным дрейфом."""

    sample_rate_hz = 25.0

    def __init__(self, reference_altitude_m: float, rng: np.random.Generator):
        self.reference_altitude_m = reference_altitude_m
        self.reference_pressure_pa = pressure_from_altitude(reference_altitude_m)
        self.rng = rng
        self.noise_pa = 0.9
        self.rotor_noise_pa = 1.2
        self.weather_drift_pa_s = 0.003
        self.temperature_coefficient_pa_k = 0.6
        self.reference_temperature_c = 15.0

    @property
    def period_s(self) -> float:
        return 1.0 / self.sample_rate_hz

    def read(self, true_altitude_m: float, temperature_c: float, time_s: float) -> BarometerReading:
        true_pressure = pressure_from_altitude(true_altitude_m)
        measured_pressure = (
            true_pressure
            + self.rng.normal(0.0, self.noise_pa)
            + self.rng.normal(0.0, self.rotor_noise_pa)
            + self.weather_drift_pa_s * time_s
            + self.temperature_coefficient_pa_k
            * (temperature_c - self.reference_temperature_c)
        )
        measured_altitude = altitude_from_pressure(measured_pressure)
        return BarometerReading(measured_pressure, measured_altitude, temperature_c)


@dataclass(frozen=True)
class MagnetometerReading:
    field_body_ut: np.ndarray
    heading_deg: float
    valid: bool
    field_magnitude_ut: float


class IST8310Magnetometer:
    """Трёхосный AMR-магнитометр; курс отсчитывается по часовой от севера."""

    sample_rate_hz = 50.0

    def __init__(self, rng: np.random.Generator):
        self.rng = rng
        self.noise_ut = 0.12
        self.quantization_ut = 100.0 / 1320.0
        self.offset_drift_ut_c = 0.024
        self.reference_temperature_c = 25.0
        self.hard_iron_bias_ut = np.array([0.35, -0.25, 0.2])
        self.earth_field_enu_ut = np.array([0.0, 20.0, -45.0])
        self.valid_field_tolerance_ut = 12.0

    @property
    def period_s(self) -> float:
        return 1.0 / self.sample_rate_hz

    def read(
        self,
        true_heading_deg: float,
        disturbance_enu_ut: np.ndarray,
        temperature_c: float,
    ) -> MagnetometerReading:
        heading = np.deg2rad(true_heading_deg)
        field_enu = self.earth_field_enu_ut + disturbance_enu_ut
        east, north, up = field_enu
        field_body = np.array([
            north * cos(heading) + east * sin(heading),
            -north * sin(heading) + east * cos(heading),
            -up,
        ])
        thermal_offset = self.offset_drift_ut_c * (
            temperature_c - self.reference_temperature_c
        )
        measured = (
            field_body
            + self.hard_iron_bias_ut
            + thermal_offset
            + self.rng.normal(0.0, self.noise_ut, 3)
        )
        measured = np.round(measured / self.quantization_ut) * self.quantization_ut
        magnitude = float(np.linalg.norm(measured))
        expected_magnitude = float(np.linalg.norm(self.earth_field_enu_ut))
        valid = abs(magnitude - expected_magnitude) <= self.valid_field_tolerance_ut
        heading_deg = degrees(atan2(-measured[1], measured[0])) % 360.0
        return MagnetometerReading(measured, heading_deg, valid, magnitude)


def angle_error_deg(estimated_deg: float, true_deg: float) -> float:
    return (estimated_deg - true_deg + 180.0) % 360.0 - 180.0


def distance_2d(x: float, y: float, center_x: float, center_y: float) -> float:
    return hypot(x - center_x, y - center_y)


@dataclass(frozen=True)
class IMUReading:
    gyro_rad_s: np.ndarray
    specific_force_m_s2: np.ndarray


class ICM42688P:
    """ICM-42688-P: калиброванные остаточные смещения, шум, дрейф и насыщение."""

    sample_rate_hz = 200.0

    def __init__(self, rng: np.random.Generator):
        self.rng = rng
        self.gyro_bias_rad_s = rng.normal(0.0, np.deg2rad(0.03), 3)
        self.accel_bias_m_s2 = rng.normal(0.0, 0.04, 3)
        self.gyro_noise_density = np.deg2rad(0.0028)
        self.accel_noise_density = 70e-6 * 9.80665
        self.gyro_scale = np.diag(1.0 + rng.normal(0.0, 0.0025, 3))
        self.accel_scale = np.diag(1.0 + rng.normal(0.0, 0.0025, 3))
        self.gyro_cross_axis = self._cross_axis_matrix(rng, 0.004)
        self.accel_cross_axis = self._cross_axis_matrix(rng, 0.003)
        self.reference_temperature_c = 25.0

    @staticmethod
    def _cross_axis_matrix(rng: np.random.Generator, sigma: float) -> np.ndarray:
        matrix = rng.normal(0.0, sigma, (3, 3))
        np.fill_diagonal(matrix, 0.0)
        return matrix

    def read(
        self,
        true_omega_rad_s: np.ndarray,
        true_specific_force_m_s2: np.ndarray,
        temperature_c: float,
    ) -> IMUReading:
        dt = 1.0 / self.sample_rate_hz
        self.gyro_bias_rad_s += self.rng.normal(0.0, np.deg2rad(0.00005) * np.sqrt(dt), 3)
        self.accel_bias_m_s2 += self.rng.normal(0.0, 0.00008 * np.sqrt(dt), 3)
        delta_temperature = temperature_c - self.reference_temperature_c
        gyro_thermal_bias = np.deg2rad(0.005) * delta_temperature
        accel_thermal_bias = 0.15e-3 * 9.80665 * delta_temperature
        gyro_noise = self.rng.normal(
            0.0, self.gyro_noise_density * np.sqrt(self.sample_rate_hz / 2.0), 3
        )
        accel_noise = self.rng.normal(
            0.0, self.accel_noise_density * np.sqrt(self.sample_rate_hz / 2.0), 3
        )
        gyro = (
            (self.gyro_scale + self.gyro_cross_axis) @ true_omega_rad_s
            + self.gyro_bias_rad_s
            + gyro_thermal_bias
            + gyro_noise
        )
        accel = (
            (self.accel_scale + self.accel_cross_axis) @ true_specific_force_m_s2
            + self.accel_bias_m_s2
            + accel_thermal_bias
            + accel_noise
        )
        return IMUReading(
            np.clip(gyro, -np.deg2rad(2000.0), np.deg2rad(2000.0)),
            np.clip(accel, -16.0 * 9.80665, 16.0 * 9.80665),
        )


@dataclass(frozen=True)
class VelocityReading:
    velocity_enu_m_s: np.ndarray
    sigma_m_s: np.ndarray
    radial_speeds_m_s: np.ndarray | None = None
    doppler_frequencies_hz: np.ndarray | None = None


class VisualOdometry:
    sample_rate_hz = 20.0

    def __init__(self, rng: np.random.Generator):
        self.rng = rng
        self.scale_error = rng.normal(0.0, 0.008)
        self.bias_enu_m_s = rng.normal(0.0, 0.025, 3)

    def read(self, ground_velocity_enu: np.ndarray, available: bool) -> VelocityReading | None:
        if not available:
            return None
        self.bias_enu_m_s += self.rng.normal(0.0, 0.00035, 3)
        self.bias_enu_m_s[2] *= 0.3
        measured = (
            ground_velocity_enu * (1.0 + self.scale_error)
            + self.bias_enu_m_s
            + self.rng.normal(0.0, 0.12, 3)
        )
        return VelocityReading(measured, np.array([0.16, 0.16, 0.22]))


class IWR6843AOPDopplerRadar:
    """Трёхлучевой FMCW-радар 62 ГГц с квантованием доплеровской скорости."""

    sample_rate_hz = 20.0
    carrier_frequency_hz = 62e9
    delay_s = 0.06

    def __init__(self, rng: np.random.Generator):
        self.rng = rng
        self.scale_error = rng.normal(0.0, 0.003)
        self.maximum_radial_speed_m_s = 30.0
        self.doppler_bins = 256
        self.velocity_quantum_m_s = (
            2.0 * self.maximum_radial_speed_m_s / self.doppler_bins
        )
        azimuths = np.deg2rad([0.0, 120.0, 240.0])
        elevation = np.deg2rad(45.0)
        self.beams_body = np.array([
            [np.sin(elevation) * np.cos(azimuth),
             np.sin(elevation) * np.sin(azimuth),
             -np.cos(elevation)]
            for azimuth in azimuths
        ])

    def read(
        self,
        ground_velocity_enu: np.ndarray,
        rotation_body_to_enu: np.ndarray,
        over_water: bool,
    ) -> VelocityReading:
        velocity_body = rotation_body_to_enu.T @ ground_velocity_enu
        radial_true = self.beams_body @ velocity_body
        surface_noise = 0.25 if over_water else 0.08
        radial_measured = (
            radial_true * (1.0 + self.scale_error)
            + self.rng.normal(0.0, surface_noise, 3)
        )
        radial_measured = np.clip(
            radial_measured,
            -self.maximum_radial_speed_m_s,
            self.maximum_radial_speed_m_s,
        )
        radial_quantized = np.round(
            radial_measured / self.velocity_quantum_m_s
        ) * self.velocity_quantum_m_s
        velocity_body_measured = np.linalg.lstsq(
            self.beams_body, radial_quantized, rcond=None
        )[0]
        velocity_enu = rotation_body_to_enu @ velocity_body_measured
        wavelength_m = 299_792_458.0 / self.carrier_frequency_hz
        doppler_frequencies = 2.0 * radial_quantized / wavelength_m
        sigma = np.sqrt(surface_noise**2 + (self.velocity_quantum_m_s**2 / 12.0))
        return VelocityReading(
            velocity_enu,
            np.full(3, sigma),
            radial_quantized,
            doppler_frequencies,
        )


class RadarAltimeter:
    sample_rate_hz = 50.0

    def __init__(self, rng: np.random.Generator):
        self.rng = rng

    def read(self, true_agl_m: float, over_water: bool) -> float:
        sigma = 0.3 if over_water else 0.16
        return true_agl_m + self.rng.normal(0.0, sigma)
