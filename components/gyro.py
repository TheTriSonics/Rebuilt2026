from magicbot import feedback, tunable
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Rotation2d

from ids import CanId
from utilities.scalers import clamp_degrees



class GyroComponent:
    bump_roll_threshold = tunable(5.0)    # degrees
    bump_pitch_threshold = tunable(5.0)   # degrees
    bump_z_accel_threshold = tunable(0.7) # g's — below this, wheels likely airborne

    def __init__(self):
        self.pigeon = Pigeon2(CanId.PIGEON.id, CanId.PIGEON.bus)
        self.offset = 0.0

    def reset_heading(self, heading: float = 0.0) -> None:
        self.offset = heading - self.get_heading()

    @feedback
    def get_heading(self) -> float:
        return clamp_degrees(self.pigeon.get_yaw().value + self.offset)

    def get_Rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_heading())

    @feedback
    def get_roll(self) -> float:
        return self.pigeon.get_roll().value

    @feedback
    def get_pitch(self) -> float:
        return self.pigeon.get_pitch().value

    @feedback
    def get_z_accel(self) -> float:
        return self.pigeon.get_acceleration_z().value

    @feedback
    def is_on_bump(self) -> bool:
        return (
            abs(self.get_roll()) > self.bump_roll_threshold
            or abs(self.get_pitch()) > self.bump_pitch_threshold
            or self.get_z_accel() < self.bump_z_accel_threshold
        )

    def execute(self) -> None:
        pass
