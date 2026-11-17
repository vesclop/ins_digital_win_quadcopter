import numpy as np
import pandas as pd
from scipy.interpolate import PchipInterpolator
import matplotlib.pyplot as plt


class RouteBuilder:
    """Генератор сглаженного 3D-маршрута БПЛА по опорным точкам местности."""

    def __init__(
        self,
        input_csv: str,
        v_cruise: float = 9.0,  # крейсерная скорость
        v_maneuver: float = 5.0, # пониженная скорость для выполнения маневров 
        cruise_altitude: float = 50.0,  # высота полета
        transition_dist: float = 150.0,  # дистанция плавного торможения
    ):

        self.input_csv = input_csv
        self.v_cruise = v_cruise
        self.v_maneuver = v_maneuver
        self.cruise_altitude = cruise_altitude
        self.transition_dist = transition_dist

        self.total_length: float = 0.0  # общая длина пути
        self.total_time: float = 0.0  # общее время пути
        self._arc_len: np.ndarray | None = None  # длина траектории
        self._xf: np.ndarray | None = None  
        self._yf: np.ndarray | None = None
        self._zf: np.ndarray | None = None
        self._t_fine: np.ndarray | None = None  # время

        self._build()  

    def _build(self) -> None:
        """Выполняет чтение CSV, сглаживание траектории и расчёт профиля скорости."""
        df_waypoints = pd.read_csv(self.input_csv)

        # хордовая параметризация по 3D-расстоянию между опорными узлами
        dx = np.diff(df_waypoints["x"], prepend=df_waypoints["x"].iloc[0])
        dy = np.diff(df_waypoints["y"], prepend=df_waypoints["y"].iloc[0])
        dz = np.diff(df_waypoints["z"], prepend=df_waypoints["z"].iloc[0])
        s_chord = np.cumsum(np.sqrt(dx**2 + dy**2 + dz**2))

        # сглаживание траектории с сохранением монотонности
        interp_x = PchipInterpolator(s_chord, df_waypoints["x"])
        interp_y = PchipInterpolator(s_chord, df_waypoints["y"])
        interp_z = PchipInterpolator(s_chord, df_waypoints["z"])

        # перепараметризация по точной длине дуги
        s_fine = np.linspace(0.0, s_chord[-1], 200_000)
        self._xf = interp_x(s_fine)
        self._yf = interp_y(s_fine)
        self._zf = interp_z(s_fine)

        ds_fine = np.sqrt(
            np.diff(self._xf) ** 2 + np.diff(self._yf) ** 2 + np.diff(self._zf) ** 2
        )
        self._arc_len = np.concatenate([[0.0], np.cumsum(ds_fine)])
        self.total_length = float(self._arc_len[-1])

        # скоростной профиль DJI Mavic 3 Pro (снижение скорости на маневрах)
        in_maneuver = (np.abs(self._yf) > 1.0) | (
            np.abs(self._zf - self.cruise_altitude) > 0.5
        )
        maneuver_s = self._arc_len[in_maneuver]

        if len(maneuver_s) > 0:
            idx = np.searchsorted(maneuver_s, self._arc_len)
            idx_l = np.clip(idx - 1, 0, len(maneuver_s) - 1)
            idx_r = np.clip(idx, 0, len(maneuver_s) - 1)
            dist = np.minimum(
                np.abs(self._arc_len - maneuver_s[idx_l]),
                np.abs(self._arc_len - maneuver_s[idx_r]),
            )
            blend = np.clip(dist / self.transition_dist, 0.0, 1.0)
            v_profile = self.v_maneuver + (self.v_cruise - self.v_maneuver) * blend
        else:
            v_profile = np.full_like(self._arc_len, self.v_cruise)

        dt_fine = ds_fine / (0.5 * (v_profile[:-1] + v_profile[1:]))
        self._t_fine = np.concatenate([[0.0], np.cumsum(dt_fine)])
        self.total_time = float(self._t_fine[-1])

    def get_route(self, step_meters: float = 18.0) -> list[tuple[float, float, float]]:
        """Возвращает маршрут в виде списка кортежей [(x, y, z), ...] с заданным шагом."""
        s_grid = np.arange(0.0, self.total_length, step_meters)
        if len(s_grid) == 0 or s_grid[-1] < self.total_length:
            s_grid = np.append(s_grid, self.total_length)

        x_route = np.interp(s_grid, self._arc_len, self._xf)
        y_route = np.interp(s_grid, self._arc_len, self._yf)
        z_route = np.interp(s_grid, self._arc_len, self._zf)

        return [
            (round(float(x), 4), round(float(y), 4), round(float(z), 4))
            for x, y, z in zip(x_route, y_route, z_route)
        ]

    def export_reports(
        self,
        output_csv: str,
        output_png: str,
        dt: float = 0.1,
    ) -> None:
        """Сохраняет таблицу координат по времени и график x(t), y(t), z(t)."""
        t_grid = np.arange(0.0, self.total_time + dt / 2, dt)
        t_grid = t_grid[t_grid <= self.total_time]

        x_out = np.interp(t_grid, self._t_fine, self._xf)
        y_out = np.interp(t_grid, self._t_fine, self._yf)
        z_out = np.interp(t_grid, self._t_fine, self._zf)

        pd.DataFrame({
            "t": np.round(t_grid, 4),
            "x": np.round(x_out, 6),
            "y": np.round(y_out, 6),
            "z": np.round(z_out, 6),
        }).to_csv(output_csv, index=False)

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        axes[0].plot(t_grid, x_out, color="#1f77b4", linewidth=1.8)
        axes[0].set_ylabel("Координата X, м", fontsize=11)
        axes[0].set_title(
            f"Зависимость координат DJI Mavic 3 Pro от времени полёта (v_крейс = {self.v_cruise} м/с, шаг {dt} с)",
            fontsize=13,
        )
        axes[0].grid(True, linestyle="--", alpha=0.6)

        axes[1].plot(t_grid, y_out, color="#2ca02c", linewidth=1.8)
        axes[1].set_ylabel("Координата Y, м", fontsize=11)
        axes[1].grid(True, linestyle="--", alpha=0.6)

        axes[2].plot(t_grid, z_out, color="#d62728", linewidth=1.8)
        axes[2].set_ylabel("Высота Z, м", fontsize=11)
        axes[2].set_xlabel("Время полёта t, с", fontsize=11)
        axes[2].grid(True, linestyle="--", alpha=0.6)

        plt.tight_layout()
        plt.savefig(output_png, dpi=200)
        plt.close(fig)


def main():
    input_csv = "Маршрут ЛА.csv"
    output_csv = "trajectory_mavic3pro.csv"
    output_png = "coordinates_vs_time.png"

    builder = RouteBuilder(input_csv=input_csv)
    builder.export_reports(output_csv=output_csv, output_png=output_png, dt=0.1)

    route = builder.get_route(step_meters=18.0)

    print(f"Длина траектории: {builder.total_length:.2f} м")
    print(f"Время полёта:     {builder.total_time:.2f} с ({builder.total_time / 60:.2f} мин)")
    print(f"Точек в списке:   {len(route)}")
    print(f"Первая точка:     {route[0]}")
    print(f"Последняя точка:  {route[-1]}")


if __name__ == "__main__":
    main()