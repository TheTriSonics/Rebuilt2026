import ntcore
from wpimath import units
from magicbot import tunable, feedback
from wpimath.geometry import (
    Pose3d,
    Rotation3d,
    Translation3d,
    Rotation2d,
    Translation2d,
    Transform2d,
)

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from utilities.game import is_red
from utilities.shot_tables import HOOD_RPS_TABLE, FLYWHEEL_RPS_TABLE, FLIGHT_TIME_TABLE
from math import atan2, sqrt, cos, sin

_shooter_height = 0.15  # meters
_goal_height = 2.00  # meters

# Fixed field targets derived from AprilTag positions (k2026RebuiltWelded, meters)
_BLUE_HUB = Translation3d(4.626, 4.035, _goal_height)
_RED_HUB = Translation3d(11.916, 4.035, _goal_height)
_FIELD_LENGTH = 16.541
_FIELD_WIDTH = 8.069


class ShotCalculatorComponent:
    gyro: GyroComponent
    drivetrain: DrivetrainComponent

    # Lob target offsets from corner of own alliance zone (meters, ~4 ft default)
    lob_alliance_wall_offset = tunable(1.219)  # distance inward from end wall (X axis)
    lob_side_wall_offset = tunable(1.219)  # distance inward from side wall (Y axis)

    # Number of iterations for TOF convergence (3-5 is plenty)
    tof_iterations = tunable(5)

    # Phase delay to compensate for system latency (sensor -> actuator, seconds)
    phase_delay_s = tunable(0.04)

    # Enable/disable shoot-on-the-fly compensation
    sotf_enabled = tunable(True)

    def __init__(self):
        self.shot_dx = 0.0
        self.shot_dy = 0.0
        self.field_angle = 0.0
        self.field_distance = 0.0
        self.effective_distance = 0.0
        self.flight_time = 0.0
        self.radial_velocity = 0.0
        self.tangential_velocity = 0.0
        self.robot_speed = 0.0
        self.goal_pose = Pose3d()
        self.active_target = Pose3d()
        self.shooter_offset = Transform2d(
            Translation2d(units.inchesToMeters(-9.5), 0),
            Rotation2d.fromDegrees(0),
        )
        self.shooter_pose = Pose3d()

        # NetworkTables publishers for debug visualization
        self.targets = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/shot_calc/fuel_target", Pose3d)
            .publish()
        )
        self.position = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/shot_calc/position", Pose3d)
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
                "hub": _RED_HUB,
                "left": Translation3d(_FIELD_LENGTH - ax, sy, 0),
                "right": Translation3d(_FIELD_LENGTH - ax, _FIELD_WIDTH - sy, 0),
            }
        else:
            lookup = {
                "hub": _BLUE_HUB,
                "left": Translation3d(ax, _FIELD_WIDTH - sy, 0),
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

    @feedback
    def get_effective_distance(self) -> float:
        """Distance adjusted for robot motion -- use this for shooter lookups."""
        return self.effective_distance

    @feedback
    def get_flight_time(self) -> float:
        return self.flight_time

    @feedback
    def get_radial_velocity(self) -> float:
        """Robot velocity component toward/away from target (m/s).
        Positive = moving toward target."""
        return self.radial_velocity

    @feedback
    def get_tangential_velocity(self) -> float:
        """Robot velocity component perpendicular to target (m/s)."""
        return self.tangential_velocity

    @feedback
    def get_robot_speed(self) -> float:
        return self.robot_speed

    def _lookup_tof(self, distance: float) -> float:
        """Look up flight time based on what the launch parameters would be at
        this distance.

        Flight time is physically a function of launch parameters (angle/speed),
        not distance directly.  We go distance -> (hood RPS, flywheel RPS) ->
        flight time via a 2D bilinear interpolation table so the iterative
        solver naturally couples TOF to the actual launch profile.
        """
        hood_rps = HOOD_RPS_TABLE.get(distance)
        flywheel_rps = FLYWHEEL_RPS_TABLE.get(distance)
        return FLIGHT_TIME_TABLE.get(hood_rps, flywheel_rps)

    def execute(self) -> None:
        curr_pose = self.drivetrain.get_pose()

        # Shooter is offset from robot center
        self.shooter_pose = curr_pose.transformBy(self.shooter_offset)
        shooter_x = self.shooter_pose.translation().x
        shooter_y = self.shooter_pose.translation().y

        target_x = self.active_target.translation().x
        target_y = self.active_target.translation().y

        # Get robot velocity (drivetrain already smooths with a weighted average)
        robotvx = self.drivetrain.vx
        robotvy = self.drivetrain.vy
        self.robot_speed = sqrt(robotvx**2 + robotvy**2)

        if not self.sotf_enabled or self.robot_speed < 0.05:
            # Stationary shot -- simple geometry, no SOTF compensation
            dx = target_x - shooter_x
            dy = target_y - shooter_y
            self.field_distance = sqrt(dx**2 + dy**2)
            self.effective_distance = self.field_distance
            self.field_angle = atan2(dy, dx)
            self.flight_time = self._lookup_tof(self.field_distance)
            self.radial_velocity = 0.0
            self.tangential_velocity = 0.0
        else:
            # --- Shoot-on-the-fly compensation ---

            # Phase delay: project the shooter position forward by system latency
            # so we're aiming from where the robot will be when the ball leaves
            adj_shooter_x = shooter_x + robotvx * self.phase_delay_s
            adj_shooter_y = shooter_y + robotvy * self.phase_delay_s

            # Step 1: Compute stationary distance for initial TOF estimate
            dx = target_x - adj_shooter_x
            dy = target_y - adj_shooter_y
            static_dist = sqrt(dx**2 + dy**2)
            tof = self._lookup_tof(static_dist)

            # Step 2: Iterative TOF convergence
            # The problem: TOF depends on effective distance, which depends on
            # the velocity-adjusted aim point, which depends on TOF. We iterate
            # to converge. Typically converges in 2-3 iterations.
            for _ in range(int(self.tof_iterations)):
                # Compute the angle from shooter to static target
                angle_to_target = atan2(dy, dx)

                # Decompose robot velocity into radial (toward target) and
                # tangential (perpendicular to target line) components.
                # Radial positive = moving toward target.
                v_radial = robotvx * cos(angle_to_target) + robotvy * sin(angle_to_target)
                v_tangential = -robotvx * sin(angle_to_target) + robotvy * cos(angle_to_target)

                # 254-style: compute the shot speed the ball needs in the radial
                # direction, accounting for our radial motion
                shot_speed_radial = static_dist / tof - v_radial
                if shot_speed_radial < 0.1:
                    shot_speed_radial = 0.1

                # Yaw correction to cancel tangential velocity
                yaw_correction = atan2(-v_tangential, shot_speed_radial)

                # Effective distance = what the shooter "sees" after velocity
                # compensation. This is the distance to use for hood/flywheel lookups.
                eff_dist = tof * sqrt(v_tangential**2 + shot_speed_radial**2)

                # Update TOF based on the new effective distance
                tof = self._lookup_tof(eff_dist)

            # Store results
            self.flight_time = tof
            self.radial_velocity = v_radial
            self.tangential_velocity = v_tangential
            self.field_distance = static_dist
            self.effective_distance = eff_dist
            self.field_angle = angle_to_target + yaw_correction

        # Publish the compensated aim point for AdvantageScope visualization
        aim_dist = self.effective_distance
        aim_x = shooter_x + aim_dist * cos(self.field_angle)
        aim_y = shooter_y + aim_dist * sin(self.field_angle)
        self.goal_pose = Pose3d(
            Translation3d(aim_x, aim_y, self.active_target.translation().z),
            Rotation3d(0, 0, 0),
        )
        self.targets.set(self.goal_pose)
