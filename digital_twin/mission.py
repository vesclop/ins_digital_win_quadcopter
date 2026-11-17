"""Синтетическая местность и маршрут с семью препятствиями и мостом."""

from dataclasses import dataclass
from math import hypot, sqrt

import numpy as np

from sensors import distance_2d


@dataclass(frozen=True)
class Obstacle:
    number: int
    name: str
    kind: str
    x: float
    y: float
    extra_path_fraction: float
    radius_m: float = 0.0


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    speed_m_s: float
    label: str
    obstacle_number: int = 0


@dataclass(frozen=True)
class RouteEvent:
    obstacle: Obstacle
    direct_length_m: float
    detour_length_m: float
    waypoint_indices: tuple[int, int, int]

    @property
    def actual_extra_fraction(self) -> float:
        return self.detour_length_m / self.direct_length_m - 1.0


class Terrain:
    """Высотная модель в метрах; оси X и Y направлены на восток и север."""

    def height(self, x, y):
        x_array = np.asarray(x)
        y_array = np.asarray(y)
        height = (
            105.0
            + 10.0 * np.sin(x_array / 450.0)
            + 6.0 * np.cos(y_array / 180.0)
            + 0.002 * x_array
            + 16.0 * np.exp(-((x_array - 1200.0) / 120.0) ** 2 - (y_array / 150.0) ** 2)
            + 20.0 * np.exp(-((x_array - 1800.0) / 130.0) ** 2 - (y_array / 170.0) ** 2)
            + 13.0 * np.exp(-((x_array - 2400.0) / 110.0) ** 2 - (y_array / 140.0) ** 2)
            - 28.0 * np.exp(-((x_array - 4800.0) / 105.0) ** 2 - (y_array / 850.0) ** 2)
        )
        return float(height) if height.ndim == 0 else height

    def map_height(self, x, y):
        x_array = np.asarray(x)
        y_array = np.asarray(y)
        error = 1.4 * np.sin(x_array / 370.0) + 0.8 * np.cos(y_array / 120.0)
        return self.height(x, y) + error

    def vio_available(self, x: float, y: float) -> bool:
        low_texture_field = 2550.0 < x < 2750.0
        bridge_shadow = 4640.0 < x < 4930.0
        return not (low_texture_field or bridge_shadow)

    def magnetic_disturbance_enu_ut(self, x: float, y: float) -> np.ndarray:
        distance = distance_2d(x, y, 4800.0, 0.0)
        amplitude = 42.0 * np.exp(-0.5 * (distance / 70.0) ** 2)
        return np.array([amplitude, -0.25 * amplitude, 0.0])


class Mission:
    def __init__(self):
        self.terrain = Terrain()
        self.cruise_agl_m = 60.0
        self.bridge_agl_m = 25.0
        self.bridge_deck_agl_m = 35.0
        self.obstacles = [
            Obstacle(1, "Запретная зона", "no_fly", 600.0, 0.0, 0.20, 65.0),
            Obstacle(2, "Вертикальный облёт 1", "vertical", 1200.0, 0.0, 0.04, 35.0),
            Obstacle(3, "Вертикальный облёт 2", "vertical", 1800.0, 0.0, 0.04, 35.0),
            Obstacle(4, "Вертикальный облёт 3", "vertical", 2400.0, 0.0, 0.04, 35.0),
            Obstacle(5, "Поворот на 4 %", "horizontal", 3000.0, 0.0, 0.04, 30.0),
            Obstacle(6, "Поворот на 5 %", "horizontal", 3600.0, 0.0, 0.05, 30.0),
            Obstacle(7, "Поворот на 6 %", "horizontal", 4200.0, 0.0, 0.06, 30.0),
            Obstacle(8, "Стальной мост", "bridge", 4800.0, 0.0, 0.04, 45.0),
        ]
        self.waypoints, self.events = self._build_route()
        self.route_array = np.array([[p.x, p.y, p.z] for p in self.waypoints])

    @property
    def bridge_deck_m(self) -> float:
        return self.terrain.height(4800.0, 0.0) + self.bridge_deck_agl_m

    @property
    def ideal_length_m(self) -> float:
        return float(np.linalg.norm(np.diff(self.route_array, axis=0), axis=1).sum())

    def _build_route(self) -> tuple[list[Waypoint], list[RouteEvent]]:
        start_z = self.terrain.height(0.0, 0.0) + 2.0
        waypoints = [Waypoint(0.0, 0.0, start_z, 0.0, "Старт")]
        waypoints.append(Waypoint(
            100.0,
            0.0,
            self.terrain.height(100.0, 0.0) + self.cruise_agl_m,
            12.0,
            "Набор высоты",
        ))
        events: list[RouteEvent] = []

        for obstacle in self.obstacles:
            if obstacle.kind == "vertical":
                height_change = self.cruise_agl_m * 0.5
                direct_length = 2.0 * height_change / sqrt(
                    (1.0 + obstacle.extra_path_fraction) ** 2 - 1.0
                )
            elif obstacle.kind == "bridge":
                height_change = self.cruise_agl_m - self.bridge_agl_m
                direct_length = 2.0 * height_change / sqrt(
                    (1.0 + obstacle.extra_path_fraction) ** 2 - 1.0
                )
            else:
                direct_length = 300.0

            half_length = direct_length / 2.0
            base_z = self.terrain.height(obstacle.x, obstacle.y) + self.cruise_agl_m
            entry = Waypoint(
                obstacle.x - half_length,
                0.0,
                base_z,
                7.0 if obstacle.kind != "bridge" else 5.0,
                f"Вход: {obstacle.name}",
                obstacle.number,
            )

            if obstacle.kind in {"vertical", "bridge"}:
                lateral_offset = 0.0
            else:
                lateral_offset = half_length * sqrt(
                    (1.0 + obstacle.extra_path_fraction) ** 2 - 1.0
                )
                if obstacle.number in {6, 7}:
                    lateral_offset *= -1.0

            if obstacle.kind == "vertical":
                middle_z = base_z + self.cruise_agl_m * 0.5
                speed = 5.0
            elif obstacle.kind == "bridge":
                middle_z = base_z - (self.cruise_agl_m - self.bridge_agl_m)
                speed = 4.0
            else:
                middle_z = base_z
                speed = 6.0 if obstacle.kind == "no_fly" else 7.0

            middle = Waypoint(
                obstacle.x,
                lateral_offset,
                middle_z,
                speed,
                f"Манёвр: {obstacle.name}",
                obstacle.number,
            )
            exit_point = Waypoint(
                obstacle.x + half_length,
                0.0,
                base_z,
                12.0,
                f"Выход: {obstacle.name}",
                obstacle.number,
            )
            first_index = len(waypoints)
            waypoints.extend([entry, middle, exit_point])
            detour_length = hypot(half_length, lateral_offset)
            if obstacle.kind in {"vertical", "bridge"}:
                detour_length = hypot(detour_length, abs(middle_z - base_z))
            events.append(RouteEvent(
                obstacle,
                direct_length,
                2.0 * detour_length,
                (first_index, first_index + 1, first_index + 2),
            ))

        waypoints.append(Waypoint(
            5350.0,
            0.0,
            self.terrain.height(5350.0, 0.0) + self.cruise_agl_m,
            12.0,
            "Точка начала посадки",
        ))
        waypoints.append(Waypoint(
            5450.0,
            0.0,
            self.terrain.height(5450.0, 0.0) + 2.0,
            2.5,
            "Посадка",
        ))
        return waypoints, events

    def is_in_no_fly_zone(self, x: float, y: float) -> bool:
        return distance_2d(x, y, 600.0, 0.0) < 65.0
