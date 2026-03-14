import ntcore
from wpimath import units
from magicbot import tunable, feedback
from wpimath.geometry import Pose3d, Rotation3d, Translation3d, Rotation2d, Translation2d, Transform2d


from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from utilities.game import is_red
from math import atan2, sqrt

_shooter_height = 0.15  # meters
_goal_height = 2.00  # meters

# Fixed field targets derived from AprilTag positions (k2026RebuiltWelded, meters)
_BLUE_HUB    = Translation3d(4.626, 4.035, _goal_height)
_RED_HUB     = Translation3d(11.916, 4.035, _goal_height)
_FIELD_LENGTH = 16.541
_FIELD_WIDTH  = 8.069

class ShotCalculatorComponent:
    gyro: GyroComponent
    drivetrain: DrivetrainComponent

    flight_time = tunable(1.0)
    # Lob target offsets from corner of own alliance zone (meters, ~4 ft default)
    lob_alliance_wall_offset = tunable(1.219)  # distance inward from end wall (X axis)
    lob_side_wall_offset     = tunable(1.219)  # distance inward from side wall (Y axis)

    def __init__(self):
        self.shot_dx = 0.0
        self.shot_dy = 0.0
        self.field_angle = 0.0
        self.field_distance = 0.0
        self.goal_pose = Pose3d()
        self.active_target = Pose3d()
        self.shooter_offset = Transform2d(
            Translation2d(units.inchesToMeters(-9.5), 0),
            Rotation2d.fromDegrees(0)
        )
        self.shooter_pose = Pose3d()

        # We'll use this to debug targetting. Right now it shows where the
        # robot's turret should be aiming at if it were to launch a fuel cell
        self.targets = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic('/components/shot_calc/fuel_target', Pose3d)
            .publish()
        )

        self.position = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic('/components/shot_calc/position', Pose3d)
            .publish()
        )


    def setup(self):
        self.set_target("hub")


    def set_target(self, name: str) -> None:
        """Set the active aim target. name: 'hub', 'left', or 'right'.
        Left/right are from the driver's perspective (blue faces high-X:
        left=low-Y corner, right=high-Y corner of own alliance zone).
        """
        ax = self.lob_alliance_wall_offset
        sy = self.lob_side_wall_offset
        if is_red():
            lookup = {
                "hub":   _RED_HUB,
                "left":  Translation3d(_FIELD_LENGTH - ax, sy, 0),
                "right": Translation3d(_FIELD_LENGTH - ax, _FIELD_WIDTH - sy, 0),
            }
        else:
            lookup = {
                "hub":   _BLUE_HUB,
                "left":  Translation3d(ax, _FIELD_WIDTH - sy, 0),
                "right": Translation3d(ax, sy, 0),
            }
        if name not in lookup:
            raise ValueError(f"Unknown turret target {name!r}. Valid: {list(lookup)}")
        self.active_target = Pose3d(lookup[name], Rotation3d(0, 0, 0))
        self.targets.set(self.active_target)


    @feedback
    def get_field_shot_angle(self) -> float:
        return self.field_angle

    @feedback
    def get_field_shot_distance(self) -> float:
        return self.field_distance


    def execute(self) -> None:
        # Auto-tracking
        curr_pose = self.drivetrain.get_pose()

        # Take the robot's current pose, go back X inches to get to where the
        # turret is and use that to calculate our angle on the field to the
        # desired target
        self.shooter_pose = curr_pose.transformBy(self.shooter_offset)
        # Get x and y speeds from the drivetrain; it can average things out over
        # time to give a smooth reading.
        robotvx = self.drivetrain.vx
        robotvy = self.drivetrain.vy
        # Calculate where the target is relative to our velocites, this is our
        # actual aiming point.
        futurex: float = self.active_target.translation().x - robotvx * self.flight_time
        futurey: float = self.active_target.translation().y - robotvy * self.flight_time

        self.goal_pose = Pose3d(
            Translation3d(futurex, futurey, self.active_target.translation().z),
            Rotation3d(0, 0, 0),
        )
        # TODO: Turn this off if an FMS is connected. Debug only.
        self.targets.set(self.goal_pose)

        # Now find the difference in x and y coordinates between the turret and
        # the future target position.
        shot_dx = futurex - self.shooter_pose.translation().x
        shot_dy = futurey - self.shooter_pose.translation().y
        # field_angle = self.get_field_shot_angle()
        self.field_angle = atan2(shot_dy, shot_dx)
        self.field_distance = sqrt(shot_dx**2 + shot_dy**2)
