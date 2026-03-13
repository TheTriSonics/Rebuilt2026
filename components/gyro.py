from magicbot import feedback
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Rotation2d

from ids import CanId


class GyroComponent:
    def __init__(self):
        self.pigeon = Pigeon2(CanId.PIGEON.id, CanId.PIGEON.bus)
        self.offset = 0.0

    def reset_heading(self, heading: float = 0.0) -> None:
        self.offset = heading - self.get_heading()


    @feedback
    def get_heading(self) -> float:
        return self.pigeon.get_yaw().value + self.offset

    def get_Rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_heading())

    def execute(self) -> None:
        pass
