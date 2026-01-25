from magicbot import feedback
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Rotation2d

from generated.tuner_constants_swerve import TunerConstants


class GyroComponent:
    def __init__(self):
        self.pigeon = Pigeon2(TunerConstants._pigeon_id, TunerConstants.canbus.name)

    # TODO: Set this to heading=None
    # then use:
    # heading = 180 if is_red() else 0
    def reset_heading(self, heading: float = 0.0) -> None:
        self.pigeon.set_yaw(heading)

    @feedback
    def get_heading(self) -> float:
        return self.pigeon.get_yaw().value

    def get_Rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_heading())

    def execute(self) -> None:
        pass
