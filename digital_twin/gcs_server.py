"""Локальный сервер 3D-станции визуализации цифрового двойника БПЛА в реальном времени."""

from __future__ import annotations

import json
import math
import sys
import threading
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np

BASE_DIR = Path(__file__).resolve().parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

from configuration import SimulationFeatures
from mission import Mission
from navigation import rotation_matrix_from_quaternion
from sensors import angle_error_deg
from uav import FlightSimulation


def safe_float(value: Any) -> float | None:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(f) or math.isinf(f):
        return None
    return round(f, 5)


def euler_from_quaternion_frd_to_enu(q: np.ndarray) -> tuple[float, float, float]:
    """Извлекает (roll_deg, pitch_deg, yaw_deg) из матрицы Body(FRD) -> ENU."""
    R = rotation_matrix_from_quaternion(q)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(R[2, 0])))))
    roll = math.degrees(math.atan2(float(-R[2, 1]), float(-R[2, 2])))
    yaw = math.degrees(math.atan2(float(R[0, 0]), float(R[1, 0]))) % 360.0
    return roll, pitch, yaw


class BatteryModel:
    """Физическая модель 4S LiPo АКБ (5000 мА·ч, 77 Вт·ч, 15.4 В номинал, класс Mavic 3 Pro)."""

    def __init__(self) -> None:
        self.capacity_mah = 5000.0
        self.nominal_voltage_v = 15.4
        self.energy_wh_total = 77.0
        self.energy_j_total = self.energy_wh_total * 3600.0
        self.energy_j_remaining = self.energy_j_total
        self.used_mah = 0.0
        self.mass_kg = 0.958
        self.gravity = 9.80665
        self.hover_power_w = 98.0
        self.avionics_power_w = 14.0  # БИНС + радар IWR6843AOP + вычислитель
        self.internal_resistance_ohm = 0.045
        self.temp_c = 22.0
        self.power_w = self.hover_power_w + self.avionics_power_w
        self.voltage_v = 16.65
        self.current_a = self.power_w / self.voltage_v
        self.avg_power_w = self.power_w
        self.prev_velocity = np.zeros(3)

    @staticmethod
    def _ocv_from_soc(soc: float) -> float:
        """Кривая ЭДС 4S LiPo (от 16.8 В при 100% до 13.2 В при 0%)."""
        s = max(0.0, min(1.0, soc))
        # Характерное плато LiPo в диапазоне 20–85% + спад на краях
        return (
            13.2
            + 2.25 * (s ** 0.35)
            + 0.75 * s
            + 0.60 * (s ** 4)
        )

    def update(
        self,
        ground_velocity: np.ndarray,
        wind_velocity: np.ndarray,
        dt: float,
        ambient_temp_c: float,
    ) -> None:
        airspeed_vec = ground_velocity - wind_velocity
        airspeed = float(np.linalg.norm(airspeed_vec))
        vz = float(ground_velocity[2])
        accel = float(np.linalg.norm((ground_velocity - self.prev_velocity) / max(dt, 1e-4)))
        self.prev_velocity = ground_velocity.copy()

        # Мощность несущих винтов: висение + профиль/сопротивление + набор высоты + манёвр
        drag_power = 0.032 * (airspeed ** 3) + 0.45 * airspeed
        if vz >= 0.0:
            climb_power = self.mass_kg * self.gravity * vz / 0.58
        else:
            climb_power = max(-0.32 * self.hover_power_w, 0.35 * self.mass_kg * self.gravity * vz)
        maneuver_power = min(35.0, 1.8 * (accel ** 2))

        rotor_power = max(0.65 * self.hover_power_w, self.hover_power_w + drag_power + climb_power + maneuver_power)
        self.power_w = rotor_power + self.avionics_power_w

        soc = self.energy_j_remaining / self.energy_j_total
        v_ocv = self._ocv_from_soc(soc)
        disc = max(0.0, v_ocv * v_ocv - 4.0 * self.internal_resistance_ohm * self.power_w)
        self.current_a = (v_ocv - math.sqrt(disc)) / (2.0 * self.internal_resistance_ohm)
        self.voltage_v = max(12.4, v_ocv - self.current_a * self.internal_resistance_ohm)

        # Расход энергии и заряда
        joule_loss_w = (self.current_a ** 2) * self.internal_resistance_ohm
        self.energy_j_remaining = max(0.0, self.energy_j_remaining - (self.power_w + joule_loss_w) * dt)
        self.used_mah += self.current_a * (dt / 3.6)

        # Плавное сглаживание средней мощности для прогноза остатка полёта
        alpha = min(1.0, dt / 4.0)
        self.avg_power_w += alpha * (self.power_w - self.avg_power_w)

        # Тепловая модель АКБ
        target_temp = ambient_temp_c + 4.5 + 2.4 * joule_loss_w
        self.temp_c += (target_temp - self.temp_c) * (dt / 55.0)

    def as_dict(self) -> dict[str, float]:
        soc_pct = 100.0 * self.energy_j_remaining / self.energy_j_total
        rem_min = (self.energy_j_remaining / max(self.avg_power_w, 1.0)) / 60.0
        return {
            "soc_pct": round(soc_pct, 2),
            "voltage_v": round(self.voltage_v, 2),
            "current_a": round(self.current_a, 2),
            "power_w": round(self.power_w, 1),
            "used_mah": round(self.used_mah, 1),
            "capacity_mah": self.capacity_mah,
            "temp_c": round(self.temp_c, 1),
            "remaining_min": round(rem_min, 1),
        }


class LiveTwinSimulation(FlightSimulation):
    """Расширение FlightSimulation для потоковой 3D-визуализации без изменения математики."""

    def __init__(
        self,
        seed: int = 42,
        dt: float = 0.005,
        features: SimulationFeatures | None = None,
    ):
        self.last_imu_reading = None
        self.battery = BatteryModel()
        self.pending_events: list[dict[str, Any]] = []

        # Накопительные метрики в реальном времени
        self._err_sq_sum = 0.0
        self._route_err_sq_sum = 0.0
        self._err_samples = 0
        self.nav_err_max_m = 0.0
        self.route_err_max_m = 0.0
        self.actual_distance_m = 0.0
        self.no_fly_violations = 0
        self.min_en_route_agl = float("inf")
        self.bridge_crossed = False
        self.bridge_clearance_m: float | None = None

        self._prev_label = ""
        self._prev_mag_valid = True
        self._prev_vio_avail = True
        self._prev_fixes = 0
        self._prev_failed_fixes = 0
        self._prev_pos = None

        super().__init__(seed=seed, dt=dt, features=features)
        self._prev_pos = self.true_position.copy()
        self._prev_label = self.mission.waypoints[self.target_index].label

        orig_imu_read = self.imu.read

        def _wrapped_imu(true_omega, true_sf, temp_c):
            reading = orig_imu_read(true_omega, true_sf, temp_c)
            self.last_imu_reading = reading
            return reading

        self.imu.read = _wrapped_imu
        self._push_event("info", "Система БИНС/EKF15 инициализирована. ГНСС отключена. Взлёт!")

    def _push_event(self, level: str, text: str) -> None:
        self.pending_events.append({
            "t": round(self.time_s, 1),
            "level": level,
            "text": text,
        })

    def _point_route_error(self, pos: np.ndarray) -> float:
        route = self.mission.route_array
        best = float("inf")
        for start, end in zip(route[:-1], route[1:]):
            seg = end - start
            denom = float(seg @ seg)
            if denom == 0.0:
                continue
            frac = max(0.0, min(1.0, float((pos - start) @ seg) / denom))
            d = float(np.linalg.norm(pos - (start + frac * seg)))
            if d < best:
                best = d
        return best

    def step(self) -> bool:
        if self.completed:
            return False

        prev_x = float(self.true_position[0])
        prev_pos = self.true_position.copy()
        alive = super().step()

        # Обновление аккумулятора и пройденной дистанции
        total_wind = self.wind_enu_m_s + self.gust_enu_m_s
        temp_c = 15.0 + 4.0 * (1.0 - math.exp(-self.time_s / 300.0))
        self.battery.update(self.true_ground_velocity, total_wind, self.dt, temp_c)
        self.actual_distance_m += float(np.linalg.norm(self.true_position - prev_pos))

        # Проверка событий в реальном времени
        cur_x = float(self.true_position[0])
        if not self.bridge_crossed and prev_x < 4800.0 <= cur_x:
            self.bridge_crossed = True
            self.bridge_clearance_m = float(self.mission.bridge_deck_m - self.true_position[2])
            lvl = "ok" if self.bridge_clearance_m > 0 else "err"
            self._push_event(
                lvl,
                f"Проход под стальным мостом! Запас до пролёта: {self.bridge_clearance_m:.2f} м",
            )

        wp_idx = min(self.target_index, len(self.mission.waypoints) - 1)
        cur_label = self.mission.waypoints[wp_idx].label
        if cur_label != self._prev_label:
            self._push_event("info", f"Новый этап маршрута → {cur_label}")
            self._prev_label = cur_label

        mag_valid = bool(self.last_magnetometer.valid)
        if mag_valid != self._prev_mag_valid:
            if not mag_valid:
                self._push_event(
                    "warn",
                    f"IST8310 отбракован: магнитная аномалия |B|={self.last_magnetometer.field_magnitude_ut:.1f} мкТл (курс ведёт гироскоп)",
                )
            else:
                self._push_event("ok", "Магнитное поле в норме — коррекция курса IST8310 восстановлена")
            self._prev_mag_valid = mag_valid

        if self.features.vio and self.vio_available != self._prev_vio_avail:
            if not self.vio_available:
                self._push_event("warn", "VIO: потеря визуальной текстуры / тень моста")
            else:
                self._push_event("ok", "VIO: визуальный трекинг восстановлен")
            self._prev_vio_avail = self.vio_available

        if self.successful_fixes > self._prev_fixes:
            self._prev_fixes = self.successful_fixes
            self._push_event("ok", f"Ориентир распознан: выполнена горизонтальная коррекция EKF15 (#{self.successful_fixes})")
        if self.failed_fixes > self._prev_failed_fixes:
            self._prev_failed_fixes = self.failed_fixes
            self._push_event("warn", "Попытка привязки по ориентиру не прошла валидацию")

        if self.completed:
            self._push_event("ok", f"Миссия успешно завершена за {self.time_s / 60.0:.2f} мин!")

        return alive

    def update_live_stats(self) -> tuple[float, float]:
        nav_err = float(np.linalg.norm(self.ins.solution.position - self.true_position))
        route_err = self._point_route_error(self.true_position)
        self._err_sq_sum += nav_err * nav_err
        self._route_err_sq_sum += route_err * route_err
        self._err_samples += 1
        if nav_err > self.nav_err_max_m:
            self.nav_err_max_m = nav_err
        if route_err > self.route_err_max_m:
            self.route_err_max_m = route_err
        if self.mission.is_in_no_fly_zone(float(self.true_position[0]), float(self.true_position[1])):
            self.no_fly_violations += 1
        true_agl = float(self.true_position[2] - self.mission.terrain.height(self.true_position[0], self.true_position[1]))
        if 100.0 <= self.true_position[0] <= 5350.0 and true_agl < self.min_en_route_agl:
            self.min_en_route_agl = true_agl
        return nav_err, route_err

    def set_features_live(self, features: SimulationFeatures) -> None:
        old = self.features
        self.features = features
        # Синхронизация таймеров при включении на лету
        if features.vio and not old.vio:
            self.next_vio_s = self.time_s
        if features.radar_altimeter and not old.radar_altimeter:
            self.next_radar_altimeter_s = self.time_s
        self._push_event(
            "info",
            f"Каналы обновлены: VIO={'ВКЛ' if features.vio else 'ОТКЛ'}, "
            f"РВ={'ВКЛ' if features.radar_altimeter else 'ОТКЛ'}, "
            f"Ориентиры={'ВКЛ' if features.landmark_correction else 'ОТКЛ'}",
        )

    def telemetry_packet(self) -> dict[str, Any]:
        nav_err, route_err = self.update_live_stats()
        state = self.ins.solution
        R_true = rotation_matrix_from_quaternion(self.true_attitude)
        roll, pitch, yaw = euler_from_quaternion_frd_to_enu(self.true_attitude)
        est_roll, est_pitch, est_yaw = euler_from_quaternion_frd_to_enu(state.attitude)

        true_agl = float(
            self.true_position[2]
            - self.mission.terrain.height(self.true_position[0], self.true_position[1])
        )
        over_water = bool(abs(self.true_position[0] - 4800.0) < 140.0)
        wp_idx = min(self.target_index, len(self.mission.waypoints) - 1)
        wp = self.mission.waypoints[wp_idx]

        # Ковариация EKF15 (1-сигма)
        sig = np.sqrt(np.maximum(np.diag(state.covariance), 0.0))

        # Сырые датчики
        if self.last_imu_reading is not None:
            gyro_deg = np.rad2deg(self.last_imu_reading.gyro_rad_s)
            sf = self.last_imu_reading.specific_force_m_s2
        else:
            gyro_deg = np.zeros(3)
            sf = np.array([0.0, 0.0, -9.80665])

        if self.last_doppler_reading is not None:
            dv = self.last_doppler_reading.velocity_enu_m_s
            d_spd = float(np.linalg.norm(dv))
            beams = (
                self.last_doppler_reading.radial_speeds_m_s.tolist()
                if self.last_doppler_reading.radial_speeds_m_s is not None
                else [0.0, 0.0, 0.0]
            )
            freqs = (
                self.last_doppler_reading.doppler_frequencies_hz.tolist()
                if self.last_doppler_reading.doppler_frequencies_hz is not None
                else [0.0, 0.0, 0.0]
            )
            mean_f = float(np.mean(np.abs(freqs)))
        else:
            dv = np.zeros(3)
            d_spd = 0.0
            beams = [0.0, 0.0, 0.0]
            freqs = [0.0, 0.0, 0.0]
            mean_f = 0.0

        total_wind = self.wind_enu_m_s + self.gust_enu_m_s
        events = list(self.pending_events)
        self.pending_events.clear()

        rms_nav = math.sqrt(self._err_sq_sum / max(self._err_samples, 1))
        rms_route = math.sqrt(self._route_err_sq_sum / max(self._err_samples, 1))

        return {
            "t": round(self.time_s, 3),
            "completed": self.completed,
            "stage": wp.label,
            "wp_index": wp_idx,
            "wp_total": len(self.mission.waypoints),
            "features": self.features.as_dict(),
            "true": {
                "x": safe_float(self.true_position[0]),
                "y": safe_float(self.true_position[1]),
                "z": safe_float(self.true_position[2]),
                "agl": safe_float(true_agl),
                "vx": safe_float(self.true_ground_velocity[0]),
                "vy": safe_float(self.true_ground_velocity[1]),
                "vz": safe_float(self.true_ground_velocity[2]),
                "speed": safe_float(np.linalg.norm(self.true_ground_velocity)),
                "roll": safe_float(roll),
                "pitch": safe_float(pitch),
                "yaw": safe_float(yaw),
                "R": [[round(float(v), 5) for v in row] for row in R_true],
            },
            "est": {
                "x": safe_float(state.position[0]),
                "y": safe_float(state.position[1]),
                "z": safe_float(state.position[2]),
                "vx": safe_float(state.velocity[0]),
                "vy": safe_float(state.velocity[1]),
                "vz": safe_float(state.velocity[2]),
                "roll": safe_float(est_roll),
                "pitch": safe_float(est_pitch),
                "yaw": safe_float(est_yaw),
                "sigma_x": safe_float(sig[0]),
                "sigma_y": safe_float(sig[1]),
                "sigma_z": safe_float(sig[2]),
                "sigma_yaw_deg": safe_float(math.degrees(sig[8])),
                "gyro_bias_deg_s": safe_float(np.linalg.norm(np.rad2deg(state.gyro_bias))),
                "accel_bias_m_s2": safe_float(np.linalg.norm(state.accelerometer_bias)),
            },
            "errors": {
                "nav_3d": safe_float(nav_err),
                "nav_x": safe_float(state.position[0] - self.true_position[0]),
                "nav_y": safe_float(state.position[1] - self.true_position[1]),
                "nav_z": safe_float(state.position[2] - self.true_position[2]),
                "nav_rms": safe_float(rms_nav),
                "nav_max": safe_float(self.nav_err_max_m),
                "route_3d": safe_float(route_err),
                "route_rms": safe_float(rms_route),
                "route_max": safe_float(self.route_err_max_m),
                "heading_deg": safe_float(angle_error_deg(est_yaw, yaw)),
            },
            "sensors": {
                "imu": {
                    "gx": safe_float(gyro_deg[0]),
                    "gy": safe_float(gyro_deg[1]),
                    "gz": safe_float(gyro_deg[2]),
                    "fx": safe_float(sf[0]),
                    "fy": safe_float(sf[1]),
                    "fz": safe_float(sf[2]),
                },
                "baro": {
                    "alt_m": safe_float(self.last_barometer.altitude_m),
                    "err_m": safe_float(self.last_barometer.altitude_m - self.true_position[2]),
                    "pressure_pa": safe_float(self.last_barometer.pressure_pa),
                    "temp_c": safe_float(self.last_barometer.temperature_c),
                },
                "mag": {
                    "heading_deg": safe_float(self.last_magnetometer.heading_deg),
                    "field_ut": safe_float(self.last_magnetometer.field_magnitude_ut),
                    "bx": safe_float(self.last_magnetometer.field_body_ut[0]),
                    "by": safe_float(self.last_magnetometer.field_body_ut[1]),
                    "bz": safe_float(self.last_magnetometer.field_body_ut[2]),
                    "valid": bool(self.last_magnetometer.valid),
                    "accepted": bool(self.magnetic_accepted),
                    "rejections": int(self.magnetic_rejections),
                },
                "radar": {
                    "speed_m_s": safe_float(d_spd),
                    "vx": safe_float(dv[0]),
                    "vy": safe_float(dv[1]),
                    "vz": safe_float(dv[2]),
                    "beams": [safe_float(v) for v in beams],
                    "freqs": [safe_float(v) for v in freqs],
                    "mean_freq_hz": safe_float(mean_f),
                    "over_water": over_water,
                    "updates": int(self.velocity_updates),
                },
                "opt": {
                    "vio_available": bool(self.vio_available),
                    "vio_outage_s": safe_float(self.vio_outage_s),
                    "radar_agl_m": safe_float(self.last_radar_agl_m),
                    "fixes_ok": int(self.successful_fixes),
                    "fixes_fail": int(self.failed_fixes),
                },
            },
            "env": {
                "wind_x": safe_float(total_wind[0]),
                "wind_y": safe_float(total_wind[1]),
                "wind_z": safe_float(total_wind[2]),
                "wind_speed": safe_float(np.linalg.norm(total_wind)),
                "over_water": over_water,
            },
            "battery": self.battery.as_dict(),
            "mission_stats": {
                "distance_m": round(self.actual_distance_m, 1),
                "no_fly_violations": self.no_fly_violations,
                "min_agl": safe_float(self.min_en_route_agl) if math.isfinite(self.min_en_route_agl) else safe_float(true_agl),
                "bridge_clearance_m": safe_float(self.bridge_clearance_m),
            },
            "events": events,
        }


class SimulationManager:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.seed = 42
        self.features = SimulationFeatures()
        self.sim = LiveTwinSimulation(seed=self.seed, features=self.features)
        self._terrain_cache = self._build_scene_payload(self.sim.mission)

    @staticmethod
    def _build_scene_payload(mission: Mission) -> dict[str, Any]:
        nx, ny = 220, 44
        x_vals = np.linspace(-150.0, 5600.0, nx)
        y_vals = np.linspace(-420.0, 420.0, ny)
        xx, yy = np.meshgrid(x_vals, y_vals)
        zz = mission.terrain.height(xx, yy)

        return {
            "terrain": {
                "nx": nx,
                "ny": ny,
                "x": [round(float(v), 2) for v in x_vals],
                "y": [round(float(v), 2) for v in y_vals],
                "z": [round(float(v), 2) for v in zz.ravel()],
            },
            "route": [
                {
                    "x": round(p.x, 2),
                    "y": round(p.y, 2),
                    "z": round(p.z, 2),
                    "speed": p.speed_m_s,
                    "label": p.label,
                    "obstacle": p.obstacle_number,
                }
                for p in mission.waypoints
            ],
            "obstacles": [
                {
                    "number": o.number,
                    "name": o.name,
                    "kind": o.kind,
                    "x": o.x,
                    "y": o.y,
                    "radius_m": o.radius_m,
                    "terrain_z": round(float(mission.terrain.height(o.x, o.y)), 2),
                    "extra_pct": round(o.extra_path_fraction * 100.0, 1),
                }
                for o in mission.obstacles
            ],
            "bridge_deck_m": round(mission.bridge_deck_m, 2),
            "bridge_terrain_z": round(float(mission.terrain.height(4800.0, 0.0)), 2),
            "ideal_length_m": round(mission.ideal_length_m, 1),
        }

    def get_init(self) -> dict[str, Any]:
        with self.lock:
            return {
                "scene": self._terrain_cache,
                "seed": self.seed,
                "telemetry": self.sim.telemetry_packet(),
            }

    def step(self, steps: int) -> dict[str, Any]:
        steps = max(1, min(int(steps), 600))
        with self.lock:
            trail_true = []
            trail_est = []
            stride = max(1, steps // 6)
            for i in range(steps):
                if self.sim.completed:
                    break
                self.sim.step()
                if (i + 1) % stride == 0 or i == steps - 1 or self.sim.completed:
                    p = self.sim.true_position
                    e = self.sim.ins.solution.position
                    trail_true.append([round(float(p[0]), 2), round(float(p[1]), 2), round(float(p[2]), 2)])
                    trail_est.append([round(float(e[0]), 2), round(float(e[1]), 2), round(float(e[2]), 2)])
            pkt = self.sim.telemetry_packet()
            pkt["trail_true"] = trail_true
            pkt["trail_est"] = trail_est
            return pkt

    def reset(self, seed: int, features: SimulationFeatures) -> dict[str, Any]:
        with self.lock:
            self.seed = int(seed)
            self.features = features
            self.sim = LiveTwinSimulation(seed=self.seed, features=self.features)
            return self.sim.telemetry_packet()

    def set_features(self, features: SimulationFeatures) -> dict[str, Any]:
        with self.lock:
            self.features = features
            self.sim.set_features_live(features)
            return self.sim.telemetry_packet()

    def jump_to_obstacle(self, obstacle_number: int) -> dict[str, Any]:
        with self.lock:
            obs = next((o for o in self.sim.mission.obstacles if o.number == obstacle_number), None)
            if obs is None:
                return self.sim.telemetry_packet()
            target_x = max(0.0, obs.x - 190.0)
            if self.sim.true_position[0] > target_x + 30.0 or self.sim.completed:
                self.sim = LiveTwinSimulation(seed=self.seed, features=self.features)

            guard = 0
            while not self.sim.completed and self.sim.true_position[0] < target_x and guard < 250000:
                self.sim.step()
                guard += 1

            # Собираем прореженный след от старта до текущей точки для бесшовной 3D-трассы
            logs = self.sim.logs
            stride = max(1, len(logs) // 700)
            full_true = [
                [round(r["true_x_m"], 2), round(r["true_y_m"], 2), round(r["true_z_m"], 2)]
                for r in logs[::stride]
            ]
            full_est = [
                [round(r["estimated_x_m"], 2), round(r["estimated_y_m"], 2), round(r["estimated_z_m"], 2)]
                for r in logs[::stride]
            ]
            self.sim._push_event("info", f"Быстрый переход к препятствию #{obs.number}: {obs.name}")
            pkt = self.sim.telemetry_packet()
            pkt["full_trail_true"] = full_true
            pkt["full_trail_est"] = full_est
            return pkt


MANAGER = SimulationManager()


class GCSHandler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args: Any) -> None:
        return  # Не засоряем консоль частыми тиками телеметрии

    def _send_json(self, data: dict[str, Any], status: int = HTTPStatus.OK) -> None:
        raw = json.dumps(data, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def _send_file(self, path: Path, content_type: str) -> None:
        if not path.exists():
            self.send_error(HTTPStatus.NOT_FOUND, f"File not found: {path.name}")
            return
        raw = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _read_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            self._send_file(BASE_DIR / "gcs_ui.html", "text/html; charset=utf-8")
        elif self.path == "/three.min.js":
            self._send_file(BASE_DIR / "three.min.js", "application/javascript; charset=utf-8")
        elif self.path == "/api/init":
            self._send_json(MANAGER.get_init())
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        try:
            body = self._read_body()
            if self.path == "/api/step":
                self._send_json(MANAGER.step(body.get("steps", 10)))
            elif self.path == "/api/reset":
                feats = SimulationFeatures(
                    vio=bool(body.get("vio", False)),
                    radar_altimeter=bool(body.get("radar_altimeter", False)),
                    landmark_correction=bool(body.get("landmark_correction", False)),
                )
                self._send_json(MANAGER.reset(body.get("seed", 42), feats))
            elif self.path == "/api/features":
                feats = SimulationFeatures(
                    vio=bool(body.get("vio", False)),
                    radar_altimeter=bool(body.get("radar_altimeter", False)),
                    landmark_correction=bool(body.get("landmark_correction", False)),
                )
                self._send_json(MANAGER.set_features(feats))
            elif self.path == "/api/jump":
                self._send_json(MANAGER.jump_to_obstacle(int(body.get("obstacle_number", 1))))
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.INTERNAL_SERVER_ERROR)


def main() -> None:
    host = "127.0.0.1"
    port = 8080
    server = ThreadingHTTPServer((host, port), GCSHandler)
    url = f"http://{host}:{port}"
    print("=" * 62)
    print(f"  3D-станция цифрового двойника БПЛА запущена: {url}")
    print("  Для остановки нажмите Ctrl+C в этом окне.")
    print("=" * 62)
    threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nСервер остановлен.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()