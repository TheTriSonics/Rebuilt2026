from magicbot import feedback
from phoenix6.configs import Pigeon2Configuration, MountPoseConfigs
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Rotation2d

from ids import CanId


class GyroComponent:
    def __init__(self):
        self.pigeon = Pigeon2(CanId.PIGEON.id, CanId.PIGEON.bus)
        # TODO: Revert this if we find our robot's heading is positive in
        # counter-clockwise rotation. If we find that the robot's heading is
        # negative in the counter-clockwise rotation then this is our fix.
        pigeon_config = Pigeon2Configuration().with_mount_pose(
            MountPoseConfigs().with_mount_pose_yaw(180)
        )
        self.pigeon.configurator.apply(pigeon_config)

    def reset_heading(self, heading: float = 0.0) -> None:
        self.pigeon.set_yaw(heading)

    @feedback
    def get_heading(self) -> float:
        return self.pigeon.get_yaw().value

    def get_Rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_heading())

    def execute(self) -> None:
        pass
