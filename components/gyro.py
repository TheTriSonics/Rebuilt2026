from magicbot import feedback
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Rotation2d

from ids import CanId


class GyroComponent:
    def __init__(self):
        self.pigeon = Pigeon2(CanId.PIGEON.id, CanId.PIGEON.bus)

    def reset_heading(self, heading: float = 0.0) -> None:
        self.pigeon.set_yaw(heading)

    @feedback
    def get_heading(self) -> float:
        return self.pigeon.get_yaw().value

    def get_Rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_heading())

    def execute(self) -> None:
        pass
