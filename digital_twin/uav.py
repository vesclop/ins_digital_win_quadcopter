"""Симуляция квадрокоптера с кватернионной БИНС и комплексированием без ГНСС."""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

from configuration import SimulationFeatures
from mission import Mission
from navigation import (
    GRAVITY_ENU,
    ErrorStateINS,
    quaternion_conjugate,
    quaternion_from_rotation_matrix,
    quaternion_from_rotation_vector,
    quaternion_multiply,
    rotation_from_specific_force,
    rotation_matrix_from_quaternion,
    rotation_vector_from_quaternion,
)
from sensors import (
    BMP390Barometer,
    ICM42688P,
    IWR6843AOPDopplerRadar,
    IST8310Magnetometer,
    RadarAltimeter,
    VisualOdometry,
    angle_error_deg,
)


class UAV:
    def __init__(self):
        self.commanded_velocity = np.zeros(3)
        self.inertia_s = 0.85
        self.kp = 1.1

    def compute_velocity(
        self,
        estimated_position: np.ndarray,
        target: np.ndarray,
        speed_limit: float,
        dt: float,
    ) -> np.ndarray:
        desired = (target - estimated_position) * self.kp
        speed = float(np.linalg.norm(desired))
        if speed > speed_limit:
            desired *= speed_limit / speed
        alpha = 1.0 - math.exp(-dt / self.inertia_s)
        self.commanded_velocity += alpha * (desired - self.commanded_velocity)
        return self.commanded_velocity


class FlightSimulation:
    def __init__(
        self,
        seed: int = 42,
        dt: float = 0.005,
        features: SimulationFeatures | None = None,
    ):
        self.mission = Mission()
        self.features = features or SimulationFeatures()
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        self.drone = UAV()
        self.true_position = self.mission.route_array[0].copy()
        self.true_heading_deg = 90.0
        self.true_ground_velocity = np.zeros(3)
        initial_specific_force = -GRAVITY_ENU
        initial_rotation = rotation_from_specific_force(initial_specific_force, 90.0)
        self.true_attitude = quaternion_from_rotation_matrix(initial_rotation)
        self.true_omega_rad_s = np.zeros(3)
        self.true_specific_force_body = initial_rotation.T @ initial_specific_force

        alignment_error = np.deg2rad(self.rng.normal(0.0, [0.3, 0.3, 1.0]))
        initial_attitude = quaternion_multiply(
            quaternion_from_rotation_vector(alignment_error), self.true_attitude
        )
        self.ins = ErrorStateINS(
            self.true_position + self.rng.normal(0.0, 0.3, 3), initial_attitude
        )
        self.imu = ICM42688P(self.rng)
        self.barometer = BMP390Barometer(self.true_position[2], self.rng)
        self.magnetometer = IST8310Magnetometer(self.rng)
        self.vio = VisualOdometry(self.rng)
        self.doppler_radar = IWR6843AOPDopplerRadar(self.rng)
        self.radar_altimeter = RadarAltimeter(self.rng)

        self.wind_enu_m_s = np.array([0.45, -0.3, 0.0])
        self.gust_enu_m_s = np.zeros(3)
        self.time_s = 0.0
        self.target_index = 1
        self.next_barometer_s = 0.0
        self.next_magnetometer_s = 0.0
        self.next_vio_s = 0.0
        self.next_doppler_s = 0.0
        self.next_radar_altimeter_s = 0.0
        self.next_log_s = 0.0
        self.pending_velocity_updates: list[tuple[float, object]] = []
        self.last_barometer = self.barometer.read(self.true_position[2], 15.0, 0.0)
        self.last_magnetometer = self.magnetometer.read(90.0, np.zeros(3), 15.0)
        self.magnetic_accepted = False
        self.last_radar_agl_m = 2.0 if self.features.radar_altimeter else float("nan")
        self.last_doppler_reading = None
        self.vio_available = False
        self.last_fix_accepted = False
        self.attempted_obstacles: set[int] = set()
        self.successful_fixes = 0
        self.failed_fixes = 0
        self.magnetic_rejections = 0
        self.velocity_updates = 0
        self.vio_outage_s = 0.0
        self.logs: list[dict] = []
        self.completed = False
        self._record_log()

    @property
    def estimated_heading_deg(self) -> float:
        return self.ins.solution.heading_deg

    def _update_disturbance(self) -> None:
        tau_s = 2.5
        diffusion = 0.25 * math.sqrt(2.0 * self.dt / tau_s)
        self.gust_enu_m_s += (
            -self.gust_enu_m_s * self.dt / tau_s
            + self.rng.normal(0.0, diffusion, 3)
        )
        self.gust_enu_m_s[2] *= 0.2

    def _update_navigation(self, temperature_c: float) -> None:
        imu_reading = self.imu.read(
            self.true_omega_rad_s,
            self.true_specific_force_body,
            temperature_c,
        )
        self.ins.predict(
            imu_reading.gyro_rad_s,
            imu_reading.specific_force_m_s2,
            self.dt,
        )

        over_water = abs(self.true_position[0] - 4800.0) < 140.0
        if self.features.vio:
            self.vio_available = self.mission.terrain.vio_available(
                self.true_position[0], self.true_position[1]
            )
            if not self.vio_available:
                self.vio_outage_s += self.dt

            if self.time_s + 1e-9 >= self.next_vio_s:
                reading = self.vio.read(self.true_ground_velocity, self.vio_available)
                self.next_vio_s += 1.0 / self.vio.sample_rate_hz
                if reading is not None:
                    self.pending_velocity_updates.append((self.time_s + 0.12, reading))

        if self.time_s + 1e-9 >= self.next_doppler_s:
            reading = self.doppler_radar.read(
                self.true_ground_velocity,
                rotation_matrix_from_quaternion(self.true_attitude),
                over_water,
            )
            self.last_doppler_reading = reading
            self.next_doppler_s += 1.0 / self.doppler_radar.sample_rate_hz
            self.pending_velocity_updates.append(
                (self.time_s + self.doppler_radar.delay_s, reading)
            )

        ready = [item for item in self.pending_velocity_updates if item[0] <= self.time_s]
        self.pending_velocity_updates = [
            item for item in self.pending_velocity_updates if item[0] > self.time_s
        ]
        for _, reading in ready:
            effective_sigma = np.maximum(reading.sigma_m_s, 0.25)
            if self.ins.update_velocity(reading.velocity_enu_m_s, effective_sigma):
                self.velocity_updates += 1

        if self.time_s + 1e-9 >= self.next_barometer_s:
            self.last_barometer = self.barometer.read(
                self.true_position[2], temperature_c, self.time_s
            )
            self.next_barometer_s += self.barometer.period_s
            self.ins.update_altitude(self.last_barometer.altitude_m, 0.45)

        true_agl = self.true_position[2] - self.mission.terrain.height(
            self.true_position[0], self.true_position[1]
        )
        if (
            self.features.radar_altimeter
            and self.time_s + 1e-9 >= self.next_radar_altimeter_s
        ):
            self.last_radar_agl_m = self.radar_altimeter.read(true_agl, over_water)
            self.next_radar_altimeter_s += 1.0 / self.radar_altimeter.sample_rate_hz
            radar_absolute_z = self.mission.terrain.map_height(
                self.ins.solution.position[0], self.ins.solution.position[1]
            ) + self.last_radar_agl_m
            self.ins.update_altitude(radar_absolute_z, 1.8)

        if self.time_s + 1e-9 >= self.next_magnetometer_s:
            disturbance = self.mission.terrain.magnetic_disturbance_enu_ut(
                self.true_position[0], self.true_position[1]
            )
            self.last_magnetometer = self.magnetometer.read(
                self.true_heading_deg, disturbance, temperature_c
            )
            self.next_magnetometer_s += self.magnetometer.period_s
            self.magnetic_accepted = False
            if self.last_magnetometer.valid:
                self.magnetic_accepted = self.ins.update_heading(
                    self.last_magnetometer.heading_deg, 1.2, apply_gate=False
                )
            if not self.magnetic_accepted:
                self.magnetic_rejections += 1

        self.last_fix_accepted = False
        if self.features.landmark_correction:
            for obstacle in self.mission.obstacles:
                if obstacle.number in self.attempted_obstacles:
                    continue
                distance = np.linalg.norm(
                    self.true_position[:2] - np.array([obstacle.x, obstacle.y])
                )
                if distance <= 120.0:
                    self.attempted_obstacles.add(obstacle.number)
                    if self.rng.random() < 0.70:
                        measured_xy = self.true_position[:2] + self.rng.normal(0.0, 2.0, 2)
                        self.last_fix_accepted = self.ins.update_horizontal_position(
                            measured_xy, 2.5
                        )
                        if self.last_fix_accepted:
                            self.successful_fixes += 1
                        else:
                            self.failed_fixes += 1
                    else:
                        self.failed_fixes += 1

    def _update_true_attitude(self, commanded_acceleration: np.ndarray) -> None:
        specific_force_enu = commanded_acceleration - GRAVITY_ENU
        rotation = rotation_from_specific_force(specific_force_enu, self.true_heading_deg)
        new_attitude = quaternion_from_rotation_matrix(rotation)
        relative = quaternion_multiply(
            quaternion_conjugate(self.true_attitude), new_attitude
        )
        self.true_omega_rad_s = rotation_vector_from_quaternion(relative) / self.dt
        self.true_attitude = new_attitude
        self.true_specific_force_body = rotation.T @ specific_force_enu

    def _record_log(self) -> None:
        target = self.mission.route_array[min(self.target_index, len(self.mission.waypoints) - 1)]
        true_agl = self.true_position[2] - self.mission.terrain.height(
            self.true_position[0], self.true_position[1]
        )
        state = self.ins.solution
        doppler_velocity = (
            self.last_doppler_reading.velocity_enu_m_s
            if self.last_doppler_reading is not None
            else np.full(3, np.nan)
        )
        mean_doppler_frequency = (
            float(np.mean(np.abs(self.last_doppler_reading.doppler_frequencies_hz)))
            if self.last_doppler_reading is not None
            else float("nan")
        )
        self.logs.append({
            "time_s": self.time_s,
            "true_x_m": self.true_position[0],
            "true_y_m": self.true_position[1],
            "true_z_m": self.true_position[2],
            "estimated_x_m": state.position[0],
            "estimated_y_m": state.position[1],
            "estimated_z_m": state.position[2],
            "estimated_vx_m_s": state.velocity[0],
            "estimated_vy_m_s": state.velocity[1],
            "estimated_vz_m_s": state.velocity[2],
            "target_x_m": target[0],
            "target_y_m": target[1],
            "target_z_m": target[2],
            "ground_speed_m_s": float(np.linalg.norm(self.true_ground_velocity)),
            "doppler_vx_m_s": doppler_velocity[0],
            "doppler_vy_m_s": doppler_velocity[1],
            "doppler_vz_m_s": doppler_velocity[2],
            "doppler_mean_abs_frequency_hz": mean_doppler_frequency,
            "true_agl_m": true_agl,
            "radar_agl_m": self.last_radar_agl_m,
            "barometer_altitude_m": self.last_barometer.altitude_m,
            "pressure_pa": self.last_barometer.pressure_pa,
            "temperature_c": self.last_barometer.temperature_c,
            "true_heading_deg": self.true_heading_deg,
            "magnetic_heading_deg": self.last_magnetometer.heading_deg,
            "estimated_heading_deg": self.estimated_heading_deg,
            "magnetic_valid": int(self.last_magnetometer.valid),
            "magnetic_accepted": int(self.magnetic_accepted),
            "magnetic_field_ut": self.last_magnetometer.field_magnitude_ut,
            "vio_available": int(self.vio_available),
            "position_fix_accepted": int(self.last_fix_accepted),
            "gyro_bias_estimate_deg_s": float(np.linalg.norm(np.rad2deg(state.gyro_bias))),
            "accel_bias_estimate_m_s2": float(np.linalg.norm(state.accelerometer_bias)),
            "target_label": self.mission.waypoints[min(
                self.target_index, len(self.mission.waypoints) - 1
            )].label,
        })

    def step(self) -> bool:
        if self.completed:
            return False
        target_waypoint = self.mission.waypoints[self.target_index]
        target = self.mission.route_array[self.target_index]
        self._update_disturbance()
        temperature_c = 15.0 + 4.0 * (1.0 - math.exp(-self.time_s / 300.0))
        self._update_navigation(temperature_c)

        previous_command = self.drone.commanded_velocity.copy()
        self.drone.compute_velocity(
            self.ins.solution.position, target, target_waypoint.speed_m_s, self.dt
        )
        commanded_acceleration = (self.drone.commanded_velocity - previous_command) / self.dt
        self.true_ground_velocity = (
            self.drone.commanded_velocity + self.wind_enu_m_s + self.gust_enu_m_s
        )
        self.true_position += self.true_ground_velocity * self.dt

        horizontal_speed = float(np.linalg.norm(self.true_ground_velocity[:2]))
        if horizontal_speed > 0.7:
            desired_heading = math.degrees(math.atan2(
                self.true_ground_velocity[0], self.true_ground_velocity[1]
            )) % 360.0
            heading_change = angle_error_deg(desired_heading, self.true_heading_deg)
            maximum_change = 120.0 * self.dt
            heading_change = float(np.clip(heading_change, -maximum_change, maximum_change))
            self.true_heading_deg = (self.true_heading_deg + heading_change) % 360.0
        self._update_true_attitude(commanded_acceleration)

        self.time_s += self.dt
        distance_to_target = float(np.linalg.norm(target - self.ins.solution.position))
        reach_radius = 1.5 if self.target_index == len(self.mission.waypoints) - 1 else 4.0
        if distance_to_target < reach_radius:
            self.target_index += 1
            if self.target_index >= len(self.mission.waypoints):
                self.completed = True

        if self.time_s + 1e-9 >= self.next_log_s or self.completed:
            self._record_log()
            self.next_log_s += 0.1
        return not self.completed

    def run(self, maximum_time_s: float = 1800.0) -> None:
        while not self.completed and self.time_s < maximum_time_s:
            self.step()
        if not self.completed:
            raise RuntimeError("Маршрут не завершён за отведённое время")
        self._record_log()

    def _distance_to_ideal_route(self, positions: np.ndarray, dimensions: int) -> np.ndarray:
        route = self.mission.route_array[:, :dimensions]
        points = positions[:, :dimensions]
        minimum = np.full(len(points), np.inf)
        for start, end in zip(route[:-1], route[1:]):
            segment = end - start
            denominator = float(segment @ segment)
            if denominator == 0.0:
                continue
            fraction = np.clip(((points - start) @ segment) / denominator, 0.0, 1.0)
            closest = start + fraction[:, None] * segment
            minimum = np.minimum(minimum, np.linalg.norm(points - closest, axis=1))
        return minimum

    def statistics(self) -> dict:
        true_positions = np.array([[row["true_x_m"], row["true_y_m"], row["true_z_m"]] for row in self.logs])
        estimated_positions = np.array([
            [row["estimated_x_m"], row["estimated_y_m"], row["estimated_z_m"]]
            for row in self.logs
        ])
        route_error = self._distance_to_ideal_route(true_positions, 3)
        horizontal_route_error = self._distance_to_ideal_route(true_positions, 2)
        navigation_error = np.linalg.norm(true_positions - estimated_positions, axis=1)
        speeds = np.array([row["ground_speed_m_s"] for row in self.logs])
        barometer_error = np.array([
            row["barometer_altitude_m"] - row["true_z_m"] for row in self.logs
        ])
        heading_error = np.array([
            angle_error_deg(row["estimated_heading_deg"], row["true_heading_deg"])
            for row in self.logs
        ])
        agl = np.array([row["true_agl_m"] for row in self.logs])
        cruise_agl = np.array([
            row["true_agl_m"] for row in self.logs
            if 100.0 <= row["true_x_m"] <= 5350.0
        ])
        no_fly_violations = sum(
            self.mission.is_in_no_fly_zone(row["true_x_m"], row["true_y_m"])
            for row in self.logs
        )
        bridge_index = min(
            range(len(self.logs)),
            key=lambda i: abs(self.logs[i]["true_x_m"] - 4800.0),
        )
        bridge_clearance = self.mission.bridge_deck_m - self.logs[bridge_index]["true_z_m"]
        event_statistics = [
            {
                "number": event.obstacle.number,
                "name": event.obstacle.name,
                "required_extra_percent": event.obstacle.extra_path_fraction * 100.0,
                "route_extra_percent": event.actual_extra_fraction * 100.0,
                "direct_length_m": event.direct_length_m,
                "detour_length_m": event.detour_length_m,
            }
            for event in self.mission.events
        ]
        return {
            "completed": self.completed,
            "enabled_features": self.features.as_dict(),
            "flight_time_s": self.time_s,
            "ideal_route_length_m": self.mission.ideal_length_m,
            "actual_route_length_m": float(np.linalg.norm(np.diff(true_positions, axis=0), axis=1).sum()),
            "mean_ground_speed_m_s": float(np.mean(speeds)),
            "max_ground_speed_m_s": float(np.max(speeds)),
            "route_error_rms_m": float(np.sqrt(np.mean(route_error**2))),
            "route_error_max_m": float(np.max(route_error)),
            "horizontal_route_error_rms_m": float(np.sqrt(np.mean(horizontal_route_error**2))),
            "navigation_error_rms_m": float(np.sqrt(np.mean(navigation_error**2))),
            "navigation_error_max_m": float(np.max(navigation_error)),
            "final_navigation_error_m": float(navigation_error[-1]),
            "barometer_error_rms_m": float(np.sqrt(np.mean(barometer_error**2))),
            "heading_error_rms_deg": float(np.sqrt(np.mean(heading_error**2))),
            "heading_error_max_abs_deg": float(np.max(np.abs(heading_error))),
            "minimum_agl_m": float(np.min(agl)),
            "minimum_en_route_agl_m": float(np.min(cruise_agl)),
            "map_fix_attempts": len(self.attempted_obstacles),
            "map_corrections": self.successful_fixes,
            "failed_map_fixes": self.failed_fixes,
            "magnetic_rejections": self.magnetic_rejections,
            "velocity_updates": self.velocity_updates,
            "vio_outage_s": self.vio_outage_s,
            "final_gyro_bias_estimate_deg_s": float(np.linalg.norm(np.rad2deg(self.ins.solution.gyro_bias))),
            "final_accel_bias_estimate_m_s2": float(np.linalg.norm(self.ins.solution.accelerometer_bias)),
            "no_fly_zone_violations": no_fly_violations,
            "bridge_deck_clearance_m": bridge_clearance,
            "bridge_passed_under_deck": bool(bridge_clearance > 0.0),
            "events": event_statistics,
        }

    def write_outputs(self, output_dir: Path, with_figure: bool = True) -> dict:
        output_dir.mkdir(parents=True, exist_ok=True)
        with (output_dir / "route.csv").open("w", newline="", encoding="utf-8-sig") as file:
            writer = csv.writer(file)
            writer.writerow(["number", "label", "x_m", "y_m", "z_m", "speed_m_s", "obstacle"])
            for index, point in enumerate(self.mission.waypoints):
                writer.writerow([
                    index, point.label, point.x, point.y, point.z,
                    point.speed_m_s, point.obstacle_number,
                ])

        with (output_dir / "flight_log.csv").open("w", newline="", encoding="utf-8-sig") as file:
            writer = csv.DictWriter(file, fieldnames=list(self.logs[0]))
            writer.writeheader()
            writer.writerows(self.logs)

        statistics = self.statistics()
        with (output_dir / "summary.json").open("w", encoding="utf-8") as file:
            json.dump(statistics, file, ensure_ascii=False, indent=2)

        if with_figure:
            from visualization import plot_results

            plot_results(self.mission, self.logs, statistics, output_dir / "simulation_result.png")
        return statistics


def _print_summary(statistics: dict) -> None:
    features = statistics["enabled_features"]
    print("Дополнительные модули: VIO={0}, радиовысотомер={1}, ориентиры={2}".format(
        features["vio"], features["radar_altimeter"], features["landmark_correction"]
    ))
    print(f"Полёт завершён за {statistics['flight_time_s'] / 60.0:.2f} мин")
    print(f"Идеальный / фактический путь: {statistics['ideal_route_length_m']:.1f} / "
          f"{statistics['actual_route_length_m']:.1f} м")
    print(f"СКО / максимум ошибки навигации: {statistics['navigation_error_rms_m']:.2f} / "
          f"{statistics['navigation_error_max_m']:.2f} м")
    print(f"СКО отклонения от маршрута: {statistics['route_error_rms_m']:.2f} м")
    if features["landmark_correction"]:
        print(f"Привязки по карте: {statistics['map_corrections']} успешных из "
              f"{statistics['map_fix_attempts']}")
    else:
        print("Привязка по ориентирам отключена")
    if features["vio"]:
        print(f"VIO недоступна {statistics['vio_outage_s']:.1f} с")
    else:
        print("VIO отключена")
    print(f"Минимальная высота на маршруте: {statistics['minimum_en_route_agl_m']:.2f} м")
    print(f"Нарушений запретной зоны: {statistics['no_fly_zone_violations']}")
    print(f"Запас под мостом: {statistics['bridge_deck_clearance_m']:.2f} м")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--monte-carlo", type=int, default=1, metavar="N")
    parser.add_argument("--enable-vio", action="store_true")
    parser.add_argument("--enable-radar-altimeter", action="store_true")
    parser.add_argument("--enable-landmarks", action="store_true")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "results",
    )
    args = parser.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if args.monte_carlo < 1:
        parser.error("--monte-carlo должен быть не меньше 1")
    features = SimulationFeatures(
        vio=args.enable_vio,
        radar_altimeter=args.enable_radar_altimeter,
        landmark_correction=args.enable_landmarks,
    )

    all_statistics = []
    for run_index in range(args.monte_carlo):
        simulation = FlightSimulation(seed=args.seed + run_index, features=features)
        simulation.run()
        run_directory = args.output_dir if run_index == 0 else args.output_dir / f"run_{run_index + 1:03d}"
        statistics = simulation.write_outputs(run_directory, with_figure=run_index == 0)
        all_statistics.append(statistics)
        if args.monte_carlo > 1:
            print(f"Прогон {run_index + 1}/{args.monte_carlo}: "
                  f"СКО {statistics['navigation_error_rms_m']:.2f} м, "
                  f"максимум {statistics['navigation_error_max_m']:.2f} м")

    _print_summary(all_statistics[0])
    if args.monte_carlo > 1:
        rms_values = np.array([item["navigation_error_rms_m"] for item in all_statistics])
        max_values = np.array([item["navigation_error_max_m"] for item in all_statistics])
        aggregate = {
            "runs": args.monte_carlo,
            "navigation_error_rms_mean_m": float(np.mean(rms_values)),
            "navigation_error_rms_p95_m": float(np.percentile(rms_values, 95)),
            "navigation_error_max_p95_m": float(np.percentile(max_values, 95)),
            "completed_runs": sum(item["completed"] for item in all_statistics),
            "no_fly_violation_runs": sum(item["no_fly_zone_violations"] > 0 for item in all_statistics),
            "runs_detail": all_statistics,
        }
        with (args.output_dir / "monte_carlo.json").open("w", encoding="utf-8") as file:
            json.dump(aggregate, file, ensure_ascii=False, indent=2)
        print(f"Монте-Карло: среднее СКО {np.mean(rms_values):.2f} м, "
              f"95-й процентиль максимальной ошибки {np.percentile(max_values, 95):.2f} м")
    print(f"Результаты: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
