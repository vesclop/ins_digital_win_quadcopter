"""Построение карты местности, телеметрии и статистики миссии."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

from sensors import angle_error_deg


def _route_error(mission, positions: np.ndarray) -> np.ndarray:
    route = mission.route_array
    result = np.full(len(positions), np.inf)
    for start, end in zip(route[:-1], route[1:]):
        segment = end - start
        fraction = np.clip(
            ((positions - start) @ segment) / float(segment @ segment),
            0.0,
            1.0,
        )
        closest = start + fraction[:, None] * segment
        result = np.minimum(result, np.linalg.norm(positions - closest, axis=1))
    return result


def plot_results(mission, logs: list[dict], statistics: dict, output_path: Path) -> None:
    time_min = np.array([row["time_s"] for row in logs]) / 60.0
    true = np.array([[row["true_x_m"], row["true_y_m"], row["true_z_m"]] for row in logs])
    estimated = np.array([
        [row["estimated_x_m"], row["estimated_y_m"], row["estimated_z_m"]]
        for row in logs
    ])
    speeds = np.array([row["ground_speed_m_s"] for row in logs])
    doppler_speeds = np.sqrt(np.array([
        row["doppler_vx_m_s"]**2 + row["doppler_vy_m_s"]**2 + row["doppler_vz_m_s"]**2
        for row in logs
    ]))
    barometer = np.array([row["barometer_altitude_m"] for row in logs])
    heading_error = np.array([
        angle_error_deg(row["estimated_heading_deg"], row["true_heading_deg"])
        for row in logs
    ])
    magnetic_valid = np.array([row["magnetic_accepted"] for row in logs], dtype=bool)
    route_error = _route_error(mission, true)
    navigation_error = np.linalg.norm(true - estimated, axis=1)

    figure = plt.figure(figsize=(18, 18), constrained_layout=True)
    grid = figure.add_gridspec(4, 2, height_ratios=[1.45, 0.85, 0.85, 0.65])
    map_axis = figure.add_subplot(grid[0, :])
    altitude_axis = figure.add_subplot(grid[1, 0])
    error_axis = figure.add_subplot(grid[1, 1])
    speed_axis = figure.add_subplot(grid[2, 0])
    heading_axis = figure.add_subplot(grid[2, 1])
    text_axis = figure.add_subplot(grid[3, :])
    text_axis.axis("off")

    x_grid = np.linspace(-100.0, 5550.0, 320)
    y_grid = np.linspace(-450.0, 450.0, 150)
    xx, yy = np.meshgrid(x_grid, y_grid)
    terrain = mission.terrain.height(xx, yy)
    contour = map_axis.contourf(xx, yy, terrain, levels=28, cmap="terrain", alpha=0.9)
    colorbar = figure.colorbar(contour, ax=map_axis, pad=0.01, fraction=0.025)
    colorbar.set_label("Высота рельефа, м")

    map_axis.add_patch(Circle((600.0, 0.0), 65.0, color="red", alpha=0.3, label="Запретная зона"))
    map_axis.add_patch(Circle((600.0, 0.0), 65.0, fill=False, edgecolor="red", linewidth=2.0))
    map_axis.add_patch(Rectangle(
        (4765.0, -140.0), 70.0, 280.0,
        facecolor="dimgray", edgecolor="white", alpha=0.75, label="Мост",
    ))
    for obstacle in mission.obstacles[1:7]:
        marker = "^" if obstacle.kind == "vertical" else "s"
        map_axis.scatter(obstacle.x, obstacle.y, marker=marker, s=70, c="black", zorder=6)
    for obstacle in mission.obstacles:
        map_axis.annotate(
            str(obstacle.number),
            (obstacle.x, obstacle.y),
            xytext=(0, 13),
            textcoords="offset points",
            ha="center",
            fontsize=10,
            fontweight="bold",
            color="black",
            bbox={"boxstyle": "round,pad=0.2", "facecolor": "white", "alpha": 0.8},
        )

    map_axis.plot(
        mission.route_array[:, 0], mission.route_array[:, 1],
        color="white", linewidth=3.2, alpha=0.9,
    )
    map_axis.plot(
        mission.route_array[:, 0], mission.route_array[:, 1],
        "--", color="navy", linewidth=1.8, label="Идеальный маршрут",
    )
    map_axis.plot(true[:, 0], true[:, 1], color="red", linewidth=1.25, label="Фактический полёт")
    map_axis.plot(
        estimated[:, 0], estimated[:, 1],
        color="cyan", linewidth=0.7, alpha=0.75, label="Оценка БИНС без ГНСС",
    )
    map_axis.scatter(true[0, 0], true[0, 1], marker="o", s=100, c="lime", edgecolors="black", label="Старт", zorder=8)
    map_axis.scatter(true[-1, 0], true[-1, 1], marker="X", s=120, c="yellow", edgecolors="black", label="Посадка", zorder=8)
    map_axis.set_title("Цифровой двойник миссии: рельеф, идеальный и фактический маршруты")
    map_axis.set_xlabel("X, восток, м")
    map_axis.set_ylabel("Y, север, м")
    map_axis.set_xlim(-100.0, 5550.0)
    map_axis.set_ylim(-450.0, 450.0)
    map_axis.set_aspect("equal")
    map_axis.legend(loc="upper right", ncol=3, fontsize=9)

    ground_profile = mission.terrain.height(true[:, 0], true[:, 1])
    altitude_axis.fill_between(true[:, 0], 0.0, ground_profile, color="saddlebrown", alpha=0.35, label="Рельеф")
    altitude_axis.plot(mission.route_array[:, 0], mission.route_array[:, 2], "--", color="navy", label="Идеальная высота")
    altitude_axis.plot(true[:, 0], true[:, 2], color="red", linewidth=1.3, label="Фактическая высота")
    altitude_axis.plot(true[::5, 0], barometer[::5], color="purple", linewidth=0.6, alpha=0.55, label="BMP390")
    altitude_axis.axhline(mission.bridge_deck_m, xmin=0.84, xmax=0.90, color="dimgray", linewidth=5.0, label="Пролёт моста")
    altitude_axis.set_title("Профиль высоты")
    altitude_axis.set_xlabel("X, м")
    altitude_axis.set_ylabel("Абсолютная высота, м")
    altitude_axis.grid(alpha=0.25)
    altitude_axis.legend(fontsize=8, ncol=2)

    error_axis.plot(time_min, route_error, label="Отклонение от идеального маршрута", color="red")
    error_axis.plot(time_min, navigation_error, label="Ошибка навигационной оценки", color="darkorange")
    error_axis.set_title("Ошибки полёта и навигации")
    error_axis.set_xlabel("Время, мин")
    error_axis.set_ylabel("Ошибка, м")
    error_axis.grid(alpha=0.25)
    error_axis.legend(fontsize=9)

    speed_axis.plot(time_min, speeds, color="teal", linewidth=1.2, label="Истинная скорость")
    speed_axis.plot(
        time_min[::2], doppler_speeds[::2],
        color="darkorange", linewidth=0.7, alpha=0.75, label="IWR6843AOP",
    )
    speed_axis.set_title("Путевая скорость и измерение доплеровского радара")
    speed_axis.set_xlabel("Время, мин")
    speed_axis.set_ylabel("Скорость, м/с")
    speed_axis.set_ylim(bottom=0.0)
    speed_axis.grid(alpha=0.25)
    speed_axis.legend(fontsize=9)

    heading_axis.plot(time_min, heading_error, color="darkblue", linewidth=1.0, label="Ошибка курса БИНС")
    rejected = ~magnetic_valid
    if np.any(rejected):
        heading_axis.scatter(
            time_min[rejected], heading_error[rejected],
            s=8, color="red", label="Компас отбракован у моста", zorder=3,
        )
    heading_axis.axhline(0.0, color="black", linewidth=0.6)
    heading_axis.set_title("Курс: IST8310, гироскоп и отбраковка магнитных помех")
    heading_axis.set_xlabel("Время, мин")
    heading_axis.set_ylabel("Ошибка курса, °")
    heading_axis.grid(alpha=0.25)
    heading_axis.legend(fontsize=9)

    features = statistics["enabled_features"]
    fixes_text = (
        f"Привязки по карте: {statistics['map_corrections']} успешно, "
        f"{statistics['failed_map_fixes']} не выполнено"
        if features["landmark_correction"]
        else "Привязка по известным ориентирам: отключена"
    )
    vio_text = (
        f"Потеря VIO: {statistics['vio_outage_s']:.1f} с"
        if features["vio"]
        else "VIO: отключена пользователем"
    )
    altimeter_text = (
        "Радиовысотомер: включён"
        if features["radar_altimeter"]
        else "Радиовысотомер: отключён"
    )
    left_text = "\n".join([
        "ИТОГИ МИССИИ",
        f"Время полёта: {statistics['flight_time_s'] / 60.0:.2f} мин",
        f"Длина идеального маршрута: {statistics['ideal_route_length_m'] / 1000.0:.3f} км",
        f"Фактически пройдено: {statistics['actual_route_length_m'] / 1000.0:.3f} км",
        f"Средняя / максимальная скорость: {statistics['mean_ground_speed_m_s']:.2f} / {statistics['max_ground_speed_m_s']:.2f} м/с",
        f"СКО / максимум отклонения от маршрута: {statistics['route_error_rms_m']:.2f} / {statistics['route_error_max_m']:.2f} м",
        f"СКО / максимум ошибки навигации: {statistics['navigation_error_rms_m']:.2f} / {statistics['navigation_error_max_m']:.2f} м",
        f"СКО ошибки BMP390: {statistics['barometer_error_rms_m']:.2f} м",
        f"СКО ошибки курса: {statistics['heading_error_rms_deg']:.2f}°",
        f"Минимальная высота на маршруте: {statistics['minimum_en_route_agl_m']:.2f} м",
        fixes_text,
        vio_text,
        altimeter_text,
        f"Обновлений скорости в EKF: {statistics['velocity_updates']}",
        f"Отбраковано измерений IST8310: {statistics['magnetic_rejections']}",
        f"Входов в запретную зону: {statistics['no_fly_zone_violations']}",
        f"Запас высоты под мостом: {statistics['bridge_deck_clearance_m']:.2f} м",
        "Ветер: постоянный (0,45; −0,30) м/с + случайные порывы",
        "ICM-42688-P + БИНС/EKF15 + IWR6843AOP; ГНСС отсутствует",
    ])
    right_lines = ["СООТВЕТСТВИЕ УДЛИНЕНИЯ ПУТИ ТЗ"]
    for event in statistics["events"]:
        right_lines.append(
            f"{event['number']}. {event['name']}: {event['route_extra_percent']:.3f} % "
            f"(требуется {event['required_extra_percent']:.0f} %), "
            f"{event['detour_length_m']:.1f} м вместо {event['direct_length_m']:.1f} м"
        )
    right_lines.extend([
        "",
        "Вертикальные облёты: подъём с 60 до 90 м над исходной поверхностью.",
        "Пролёт под мостом: снижение с 60 до 25 м; нижняя плоскость моста — 35 м.",
        "Удлинение рассчитано отдельно для локального участка каждого препятствия.",
    ])
    text_axis.text(0.01, 0.97, left_text, va="top", ha="left", fontsize=10.5, family="monospace")
    text_axis.text(0.52, 0.97, "\n".join(right_lines), va="top", ha="left", fontsize=10.5, family="monospace")

    figure.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(figure)
