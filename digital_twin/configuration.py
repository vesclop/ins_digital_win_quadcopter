"""Включение дополнительных каналов коррекции без удаления их реализации."""

from dataclasses import dataclass


@dataclass(frozen=True)
class SimulationFeatures:
    vio: bool = False
    radar_altimeter: bool = False
    landmark_correction: bool = False

    def as_dict(self) -> dict[str, bool]:
        return {
            "vio": self.vio,
            "radar_altimeter": self.radar_altimeter,
            "landmark_correction": self.landmark_correction,
        }
