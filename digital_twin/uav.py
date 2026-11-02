"""Абстрактная модель цифрового двойника квадрокоптера.

Состав модуля:
    Position   — точка в пространстве (также используется для скорости и ветра).
    Sensor     — базовый датчик с шумом измерения и весом доверия.
    IMUSensor  — инерциальный датчик с накапливающимся дрейфом.
    UAV        — дрон: оценивает своё положение по датчикам и вычисляет скорость.
    Simulation — мир: истинное положение, идеальный маршрут, внешние воздействия.

Ключевая идея: дрон НЕ знает истинного положения. Он работает только
с оценкой, полученной взвешенным объединением показаний датчиков.
Истинное положение принадлежит симуляции и передаётся датчикам,
которые искажают его своими ошибками.
"""

import random
from dataclasses import dataclass, field
from math import hypot


@dataclass
class Position:
    """Точка в пространстве, м.

    Один класс используется для трёх разных сущностей:
    - позиция (x, y, z) — координаты в горизонтальной плоскости и высота;
    - скорость (x, y, z) — составляющие скорости по осям, м/с;
    - ветер (x, y, z) — скорость сноса по осям, м/с.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Sensor:
    """Базовый датчик: по истинной позиции выдаёт измерение со своей ошибкой.

    Атрибуты:
        name   — имя датчика, по нему работает UAV.remove_sensor().
        weight — вес доверия (0..1). Абсолютное значение не важно:
                 при объединении веса нормализуются, играет роль
                 только соотношение весов между датчиками.
        noise  — стандартное отклонение гауссова шума измерения, м.
                 0 означает идеальный датчик.
    """

    def __init__(self, name: str, weight: float, noise: float = 0.0):
        self.name = name
        self.weight = weight
        self.noise = noise

    def read(self, true_position: Position) -> Position:
        """Снять показание: истинная позиция + независимый гауссов шум по осям.

        Истинная позиция передаётся симуляцией — дрон сам её не знает,
        доступ к ней есть только у датчиков в момент измерения.
        """
        return Position(
            true_position.x + random.gauss(0, self.noise),
            true_position.y + random.gauss(0, self.noise),
            true_position.z + random.gauss(0, self.noise),
        )


class IMUSensor(Sensor):
    """Инерциальный датчик (акселерометр + гироскоп) с дрейфом.

    Главная особенность IMU — ошибка не просто шумит, а НАКАПЛИВАЕТСЯ:
    смещение (bias) совершает случайное блуждание и не исчезает само.
    Поэтому без коррекции по GPS оценка по IMU со временем "уплывает".
    """

    def __init__(self, name: str = "imu", weight: float = 0.2, drift: float = 0.05):
        # Собственный шум измерения у IMU небольшой — основная проблема дрейф
        super().__init__(name, weight, noise=0.1)
        self.drift = drift  # скорость случайного блуждания смещения, м за шаг
        self.bias = Position()  # накопленное смещение по каждой оси

    def read(self, true_position: Position) -> Position:
        """Снять показание: шум базового датчика + накопленный дрейф."""
        # Случайное блуждание: на каждом шаге смещение немного меняется
        self.bias.x += random.gauss(0, self.drift)
        self.bias.y += random.gauss(0, self.drift)
        self.bias.z += random.gauss(0, self.drift)
        reading = super().read(true_position)
        reading.x += self.bias.x
        reading.y += self.bias.y
        reading.z += self.bias.z
        return reading


class UAV:
    """Дрон: предполагаемая позиция, датчики, скорость.

    Атрибуты:
        position  — предполагаемая позиция (оценка по датчикам), НЕ истина.
        velocity  — текущая скорость, м/с.
        sensors   — список датчиков; добавление/удаление через методы.
        max_speed — ограничение скорости, м/с.
        inertia   — постоянная времени разгона, с: чем больше, тем "тяжелее"
                    дрон и тем медленнее скорость догоняет желаемую.
        kp        — коэффициент P-регулятора: желаемая скорость пропорциональна
                    отклонению от цели (1/с).
    """

    def __init__(self, max_speed: float = 10.0, inertia: float = 0.5):
        self.position = Position()
        self.velocity = Position()
        self.sensors: list[Sensor] = []
        self.max_speed = max_speed
        self.inertia = inertia
        self.kp = 1.0

    def add_sensor(self, sensor: Sensor) -> None:
        """Добавить датчик в список — он сразу участвует в оценке позиции."""
        self.sensors.append(sensor)

    def remove_sensor(self, name: str) -> None:
        """Убрать датчик по имени (например, при отказе или потере сигнала)."""
        self.sensors = [s for s in self.sensors if s.name != name]

    def estimate_position(self, true_position: Position) -> Position:
        """Главный метод: оценка положения по всем датчикам и их весам.

        Опрашивает каждый датчик из списка и считает взвешенное среднее:
            position = сумма(w_i * p_i) / сумма(w_i)
        Нормализация позволяет задавать веса в любой шкале и свободно
        добавлять/удалять датчики — сумма весов не обязана быть равна 1.
        """
        readings = [(s.read(true_position), s.weight) for s in self.sensors]
        total_weight = sum(weight for _, weight in readings)
        if total_weight == 0:
            # Датчиков нет или всем обнулено доверие — остаёмся на старой оценке
            return self.position

        self.position = Position()
        for reading, weight in readings:
            w = weight / total_weight
            self.position.x += reading.x * w
            self.position.y += reading.y * w
            self.position.z += reading.z * w
        return self.position

    def compute_velocity(self, target: Position, dt: float) -> Position:
        """Вычислить скорость движения к цели с ограничением и инерцией.

        Работает в два этапа:
        1. P-регулятор: желаемая скорость пропорциональна вектору
           (цель - оценка позиции), затем ограничивается max_speed.
        2. Инерция: реальная скорость не меняется мгновенно, а стремится
           к желаемой — за каждый шаг сокращается доля dt/inertia
           отставания. Из-за этого появляются плавные разгоны и перелёты
           мимо точек при резких манёврах.
        """
        # Этап 1: желаемая скорость по отклонению от цели
        desired = Position(
            (target.x - self.position.x) * self.kp,
            (target.y - self.position.y) * self.kp,
            (target.z - self.position.z) * self.kp,
        )
        # Ограничение по модулю вектора скорости, чтобы не превысить max_speed
        speed = hypot(hypot(desired.x, desired.y), desired.z)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            desired.x *= scale
            desired.y *= scale
            desired.z *= scale

        # Этап 2: инерция — плавное приближение скорости к желаемой
        alpha = min(dt / self.inertia, 1.0)  # доля сближения за шаг, <= 1
        self.velocity.x += (desired.x - self.velocity.x) * alpha
        self.velocity.y += (desired.y - self.velocity.y) * alpha
        self.velocity.z += (desired.z - self.velocity.z) * alpha
        return self.velocity


class Simulation:
    """Мир симуляции: истина, маршрут и внешние воздействия.

    Атрибуты:
        drone         — объект UAV (его истинное положение хранится здесь,
                        а не в дроне: дрон знает только свою оценку).
        true_position — настоящая позиция дрона, м.
        route         — идеальный маршрут: список Position, облёт по порядку.
        target_index  — индекс текущей целевой точки маршрута.
        wind          — постоянная составляющая ветра, м/с.
        gust_noise    — случайные порывы ветра (стандартное отклонение), м/с.
        reach_radius  — расстояние до цели, на котором точка считается
                        достигнутой, м.
        history       — журнал шагов: пары (истина, оценка) для визуализации.
    """

    def __init__(self, drone: UAV, route: list[Position], wind: Position | None = None):
        self.drone = drone
        self.true_position = Position()
        self.route = route
        self.target_index = 0
        self.wind = wind or Position()
        self.gust_noise = 0.5
        self.reach_radius = 2.0
        self.history: list[tuple[Position, Position]] = []  # (истина, оценка)

    def target(self) -> Position | None:
        """Текущая целевая точка маршрута или None, если маршрут пройден."""
        if self.target_index < len(self.route):
            return self.route[self.target_index]
        return None

    def apply_disturbances(self, dt: float) -> None:
        """Применить внешние воздействия к истинному положению дрона.

        Ветер (постоянный + случайные порывы) сносит дрон независимо от
        его команд — именно поэтому контроллеру есть что корректировать.
        Сюда же позже можно добавить другие воздействия (например,
        изменение давления для барометра или помехи GPS).
        """
        self.true_position.x += (self.wind.x + random.gauss(0, self.gust_noise)) * dt
        self.true_position.y += (self.wind.y + random.gauss(0, self.gust_noise)) * dt
        self.true_position.z += (self.wind.z + random.gauss(0, self.gust_noise)) * dt

    def step(self, dt: float) -> bool:
        """Один шаг симуляции. Возвращает False, когда маршрут пройден.

        Порядок действий на шаге:
        1. Дрон оценивает положение по датчикам (видит только искажённые
           измерения истины).
        2. Дрон вычисляет скорость к текущей точке маршрута.
        3. Симуляция перемещает ИСТИННОЕ положение по этой скорости.
        4. Внешние воздействия дополнительно сносят истинное положение.
        5. Шаг записывается в историю; если цель достигнута (по оценке
           дрона — как в реальности), цель переключается на следующую.
        """
        target = self.target()
        if target is None:
            return False

        # 1–2: оценка положения и скорость
        estimated = self.drone.estimate_position(self.true_position)
        velocity = self.drone.compute_velocity(target, dt)

        # 3: движение истинного положения по команде дрона
        self.true_position.x += velocity.x * dt
        self.true_position.y += velocity.y * dt
        self.true_position.z += velocity.z * dt

        # 4: внешние воздействия
        self.apply_disturbances(dt)

        # 5: журнал и проверка достижения цели (копии, чтобы история
        # не менялась задним числом при обновлении позиций)
        self.history.append((
            Position(self.true_position.x, self.true_position.y, self.true_position.z),
            Position(estimated.x, estimated.y, estimated.z),
        ))

        distance = hypot(
            hypot(target.x - estimated.x, target.y - estimated.y),
            target.z - estimated.z,
        )
        if distance < self.reach_radius:
            self.target_index += 1
        return True


if __name__ == "__main__":
    import sys

    # Корректный вывод русского текста в консоли Windows
    sys.stdout.reconfigure(encoding="utf-8")
    random.seed(42)  # фиксируем случайность для воспроизводимого результата

    # Собираем дрона: GPS точнее всего — ему самый большой вес,
    # IMU дрейфует, барометр помогает по высоте
    drone = UAV(max_speed=10.0, inertia=0.5)
    drone.add_sensor(Sensor("gps", weight=0.6, noise=1.5))
    drone.add_sensor(IMUSensor(weight=0.2, drift=0.05))
    drone.add_sensor(Sensor("barometer", weight=0.2, noise=0.3))

    # Идеальный маршрут из трёх точек и боковой ветер 1 м/с по x, 0.5 по y
    route = [Position(0, 0, 50), Position(100, 0, 50), Position(100, 100, 60)]
    sim = Simulation(drone, route, wind=Position(1.0, 0.5, 0.0))

    # Крутим симуляцию с шагом 0.1 с, пока маршрут не будет пройден
    for step in range(5000):
        if not sim.step(dt=0.1):
            break

    print(f"Шагов: {step}, конечная точка маршрута: {route[-1]}")
    print(f"Истинное положение: {sim.true_position}")
    print(f"Оценка дрона:       {drone.position}")
