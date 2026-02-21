import math
from collections import deque

import magicbot
import ntcore
import wpilib
from phoenix6.configs import (
    CANcoderConfiguration,
    ClosedLoopGeneralConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
)
from phoenix6.controls import DutyCycleOut, PositionVoltage, VelocityVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import FeedbackSensorSourceValue, InvertedValue, NeutralModeValue
from wpimath.controller import (
    ProfiledPIDControllerRadians,
    SimpleMotorFeedforwardMeters,
    PIDController,
)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.trajectory import TrapezoidProfileRadians
from components.gyro import GyroComponent
from generated.tuner_constants_swerve import TunerConstants
from utilities.game import is_match, is_red
from choreo.trajectory import SwerveSample


class SwerveModule:

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        drive_id: int,
        steer_id: int,
        encoder_id: int,
        *,
        busname: str,
        mag_offset: float = 0.0,
        drive_reversed: bool = False,
        steer_reversed: bool = False
    ):
        """
        x, y: where the module is relative to the center of the robot
        *_id: can ids of steer and drive motors and absolute encoder
        *: signifies anything after this must be a named parameter
        busname: name of CAN bus the module is on
        drive_reversed: Should the drive motor be reversed?
        steer_reversed: Should the steer motor be reversed?
        """
        self.name = name
        self.busname = busname
        self.translation = Translation2d(x, y)
        self.state = SwerveModuleState(0, Rotation2d(0))
        self.mag_offset = mag_offset

        # Create Motor and encoder objects
        self.steer = TalonFX(steer_id, self.busname)
        self.drive = TalonFX(drive_id, self.busname)
        self.encoder = CANcoder(encoder_id, self.busname)

        # Configure CANcoder for FusedCANCoder - use builder pattern
        enc_config = CANcoderConfiguration()
        enc_config.magnet_sensor.with_magnet_offset(mag_offset)
        self.encoder.configurator.apply(enc_config)  # type: ignore

        steer_motor_config = MotorOutputConfigs()
        steer_motor_config.neutral_mode = NeutralModeValue.BRAKE
        # The SDS Mk4i rotation has one pair of gears.
        steer_motor_config.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if steer_reversed
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        # Configure FusedCANCoder feedback
        steer_feedback_config = FeedbackConfigs()
        steer_feedback_config.feedback_remote_sensor_id = encoder_id
        steer_feedback_config.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        steer_feedback_config.sensor_to_mechanism_ratio = 1.0
        steer_feedback_config.rotor_to_sensor_ratio = TunerConstants._steer_gear_ratio

        # configuration for motor pid
        steer_pid = TunerConstants._steer_gains
        steer_closed_loop_config = ClosedLoopGeneralConfigs()
        steer_closed_loop_config.continuous_wrap = True

        self.steer.configurator.apply(steer_motor_config)
        self.steer.configurator.apply(steer_pid, 0.01)
        self.steer.configurator.apply(steer_feedback_config)
        self.steer.configurator.apply(steer_closed_loop_config)

        # Configure drive motor
        drive_motor_config = MotorOutputConfigs()
        drive_motor_config.neutral_mode = NeutralModeValue.BRAKE
        drive_motor_config.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if drive_reversed
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        wheel_circumference = TunerConstants._wheel_radius * math.tau
        # sensor_to_mechanism_ratio converts motor rotations to meters
        # = motor_rotations_per_meter = 1 / meters_per_motor_rotation
        # = 1 / (wheel_circumference * drive_ratio)
        # where drive_ratio = wheel_rotations / motor_rotations (output/input)
        drive_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            1 / (wheel_circumference * (1 / TunerConstants._drive_gear_ratio))
        )

        # configuration for motor pid and feedforward
        self.drive_pid = TunerConstants._drive_gains
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.01, kV=0.09, kA=0.0)

        self.drive.configurator.apply(drive_motor_config)
        self.drive.configurator.apply(self.drive_pid, 0.01)
        self.drive.configurator.apply(drive_gear_ratio_config)

        self.central_angle = Rotation2d(x, y)

        # Create Phoenix 6 control requests
        self.steer_request = PositionVoltage(0).with_slot(0)
        self.drive_request = VelocityVoltage(0)
        self.stop_request = VoltageOut(0)

    def get_angle_absolute(self) -> float:
        """Gets steer angle (rot) from absolute encoder (now fused with motor)"""
        return self.steer.get_position().value * math.tau

    def get_rotation(self) -> Rotation2d:
        """Get the steer angle as a Rotation2d"""
        return Rotation2d(self.get_angle_absolute())

    def get_speed(self) -> float:
        # velocity is in rot/s, return in m/s
        return self.drive.get_velocity().value

    def get_distance_traveled(self) -> float:
        return self.drive.get_position().value

    def set(self, desired_state: SwerveModuleState):
        no_steer = False
        no_drive = False
        self.state = desired_state
        current_angle = self.get_rotation()
        self.state.optimize(current_angle)

        target_displacement = self.state.angle - current_angle
        target_angle_rotations = self.state.angle.radians() / math.tau
        wpilib.SmartDashboard.putNumber("tar", target_angle_rotations)
        diff = self.state.angle - current_angle
        if no_steer is False:
            if (abs(diff.degrees()) < 1):
                # self.steer.set_control(DutyCycleOut(0))
                ...
            else:
                ...
                # Use Phoenix 6 closed-loop position control with FusedCANCoder
                # self.steer.set_control(self.steer_request.with_position(target_angle_rotations))

        if no_drive is False:
            # rescale the speed target based on how close we are to being correctly
            # aligned with where we want to go
            target_speed = self.state.speed * target_displacement.cos() ** 2
            if abs(self.state.speed) < 0.01:
                # self.drive.set_control(self.stop_request)
                ...
            else:
                ...
                # self.drive.set_control(self.drive_request.with_velocity(target_speed))


    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_distance_traveled(), self.get_rotation())

    def get(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_speed(), self.get_rotation())


class DrivetrainComponent:
    # Here's where we inject the other components
    # Note that you can't use the components directly in the __init__ method
    # You have to use them in the setup() method
    gyro: GyroComponent

    HEADING_TOLERANCE = math.radians(1)

    chassis_speeds = magicbot.will_reset_to(ChassisSpeeds(0, 0, 0))

    send_modules = magicbot.tunable(True)
    snapping_to_heading = magicbot.tunable(False)

    def __init__(self) -> None:
        # Theoretical max RPM that a Kraken X60 can reach
        DRIVE_MOTOR_MAX_RPM = 200

        # maxiumum speed for any wheel
        wheel_circumference = TunerConstants._wheel_radius * math.tau
        drive_motor_rev_to_meters = wheel_circumference / TunerConstants._drive_gear_ratio
        self.max_wheel_speed = drive_motor_rev_to_meters * DRIVE_MOTOR_MAX_RPM

        # Placeholders for current robot velocity
        self.vx = 0
        self.vy = 0
        # Buffers for weighted moving average of velocity; used to populate
        # vx and vy
        self._velocity_samples = 10
        self._vx_samples: deque[float] = deque(maxlen=self._velocity_samples)
        self._vy_samples: deque[float] = deque(maxlen=self._velocity_samples)
        # Weights for exponential weighting (most recent sample has highest weight)
        self._velocity_weights = [1.2 ** i for i in range(self._velocity_samples)]

        # Plotting the location of this in AdvantageScope shows the robot's
        # estimated position on the field
        self.fused_pose_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("FusedPose", Pose2d)
            .publish()
        )

        # Used to lock the robot onto a heading; currently not used.
        self.heading_controller = ProfiledPIDControllerRadians(
            0.5, 0, 0, TrapezoidProfileRadians.Constraints(3 * math.tau, 49 * 6)
        )
        self.heading_controller.enableContinuousInput(-math.pi, math.pi)
        self.heading_controller.setTolerance(self.HEADING_TOLERANCE)
        self.snap_heading: float | None = None

        # Used for path following and driving directly to a specific point
        self.path_pid_control = PIDController(10, 0, 0)
        self.path_heading_pid_control = PIDController(8.2, 0, 0)

        # Define each of the four swerve modules using the SwerveModule class
        # also found in this file.
        self.modules = (
            # Front Left
            SwerveModule(
                "Front Left",
                TunerConstants._front_left_x_pos,
                TunerConstants._front_left_y_pos,
                TunerConstants._front_left_drive_motor_id,
                TunerConstants._front_left_steer_motor_id,
                TunerConstants._front_left_encoder_id,
                busname=TunerConstants.canbus.name,
                mag_offset=TunerConstants._front_left_encoder_offset,
                steer_reversed=TunerConstants._front_left_steer_motor_inverted,
                drive_reversed=TunerConstants._invert_left_side,
            ),
            # Front Right
            SwerveModule(
                "Front Right",
                TunerConstants._front_right_x_pos,
                TunerConstants._front_right_y_pos,
                TunerConstants._front_right_drive_motor_id,
                TunerConstants._front_right_steer_motor_id,
                TunerConstants._front_right_encoder_id,
                busname=TunerConstants.canbus.name,
                mag_offset=TunerConstants._front_right_encoder_offset,
                steer_reversed=TunerConstants._front_right_steer_motor_inverted,
                drive_reversed=TunerConstants._invert_right_side,
            ),
            # Back Left
            SwerveModule(
                "Back Left",
                TunerConstants._back_left_x_pos,
                TunerConstants._back_left_y_pos,
                TunerConstants._back_left_drive_motor_id,
                TunerConstants._back_left_steer_motor_id,
                TunerConstants._back_left_encoder_id,
                busname=TunerConstants.canbus.name,
                mag_offset=TunerConstants._back_left_encoder_offset,
                steer_reversed=TunerConstants._back_left_steer_motor_inverted,
                drive_reversed=TunerConstants._invert_left_side,
            ),
            # Back Right
            SwerveModule(
                "Back Right",
                TunerConstants._back_right_x_pos,
                TunerConstants._back_right_y_pos,
                TunerConstants._back_right_drive_motor_id,
                TunerConstants._back_right_steer_motor_id,
                TunerConstants._back_right_encoder_id,
                busname=TunerConstants.canbus.name,
                mag_offset=TunerConstants._back_right_encoder_offset,
                steer_reversed=TunerConstants._back_right_steer_motor_inverted,
                drive_reversed=TunerConstants._invert_right_side,
            ),
        )

        self.kinematics = SwerveDrive4Kinematics(
            self.modules[0].translation,
            self.modules[1].translation,
            self.modules[2].translation,
            self.modules[3].translation,
        )

        # nt = ntcore.NetworkTableInstance.getDefault().getTable("/components/drivetrain")
        # module_states_table = nt.getSubTable("module_states")
        # if not is_match():
        #     self.setpoints_publisher = module_states_table.getStructArrayTopic(
        #         "setpoints", SwerveModuleState
        #     ).publish()
        #     self.measurements_publisher = module_states_table.getStructArrayTopic(
        #         "measured", SwerveModuleState
        #     ).publish()

        #     wpilib.SmartDashboard.putData("Heading PID", self.heading_controller)

    def get_chassis_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.get_module_states())

    def get_module_states(self) -> tuple[
        SwerveModuleState,
        SwerveModuleState,
        SwerveModuleState,
        SwerveModuleState,
    ]:
        return (
            self.modules[0].get(),
            self.modules[1].get(),
            self.modules[2].get(),
            self.modules[3].get(),
        )

    def get_heading(self) -> Rotation2d:
        return self.gyro.get_Rotation2d()

    def setup(self) -> None:
        initial_pose = Pose2d(Translation2d(0, 0), Rotation2d(0))

        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_heading(),
            self.get_module_positions(),
            initial_pose,
            stateStdDevs=(0.01, 0.01, 0.01),  # How much to trust wheel odometry
            visionMeasurementStdDevs=(0.4, 0.4, 0.2),
        )
        self.set_pose(initial_pose)
        heading = 180 if is_red() else 0
        self.gyro.reset_heading(heading)

    def drive_field(self, vx: float, vy: float, omega: float) -> None:
        """Field oriented drive commands"""
        current_heading = self.get_rotation()
        self.chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega, current_heading
        )

    def drive_to_pose(self, target_pose: Pose2d):
        self.drive_to_position(target_pose.x, target_pose.y, target_pose.rotation().radians())

    def drive_to_position(self, x: float, y: float, o: float) -> None:
        robot_pose = self.get_pose()
        xvel = self.path_pid_control.calculate(robot_pose.x, x)
        yvel = self.path_pid_control.calculate(robot_pose.y, y)
        ovel = self.path_heading_pid_control.calculate(robot_pose.rotation().radians(), o)
        self.drive_field(xvel, yvel, ovel)

    def follow_path(self, sample: SwerveSample) -> None:
        robot_pose = self.get_pose()
        xvel = sample.vx + self.path_pid_control.calculate(robot_pose.x, sample.x)
        yvel = sample.vy + self.path_pid_control.calculate(robot_pose.y, sample.y)
        ovel = sample.omega + self.path_heading_pid_control.calculate(robot_pose.rotation().radians(), sample.heading)
        self.drive_field(xvel, yvel, ovel)

    def get_robot_speeds(self) -> tuple[float, float]:
        vx = self.chassis_speeds.vx
        vy = self.chassis_speeds.vy
        total_speed = math.sqrt(vx * vx + vy * vy)
        return total_speed, self.chassis_speeds.omega

    def halt(self):
        self.drive_local(0, 0, 0)

    def drive_local(self, vx: float, vy: float, omega: float) -> None:
        """Robot oriented drive commands"""
        # if abs(omega) < 0.01 and self.snap_heading is None:
        #     self.snap_to_heading(self.get_heading().radians())
        self.chassis_speeds = ChassisSpeeds(vx, vy, omega)

    # Note that haeding should be in radians
    def snap_to_heading(self, heading: float) -> None:
        """set a heading target for the heading controller"""
        self.snapping_to_heading = True
        self.snap_heading = heading
        self.heading_controller.setGoal(self.snap_heading)

    def stop_snapping(self) -> None:
        """stops the heading_controller"""
        self.snapping_to_heading = False
        self.snap_heading = None

    def execute(self) -> None:
        if self.snapping_to_heading:
            self.chassis_speeds.omega = self.heading_controller.calculate(
                self.get_rotation().radians()
            )
        else:
            self.heading_controller.reset(
                self.get_rotation().radians(), self.get_rotational_velocity()
            )

        desired_speeds = self.chassis_speeds
        desired_states = self.kinematics.toSwerveModuleStates(desired_speeds)
        desired_states = self.kinematics.desaturateWheelSpeeds(
            desired_states, attainableMaxSpeed=self.max_wheel_speed
        )

        for state, module in zip(desired_states, self.modules, strict=True):
            module.set(state)

        self.update_odometry()

    def on_enable(self) -> None:
        """update the odometry so the pose estimator doesn't have an empty
        buffer

        While we should be building the pose buffer while disabled, this
        accounts for the edge case of crashing mid match and immediately
        enabling with an empty buffer"""
        self.update_odometry()

    def get_rotational_velocity(self) -> float:
        v = self.gyro.pigeon.get_angular_velocity_z_world().value
        return math.radians(v)

    def update_odometry(self) -> None:
        orig_pose = self.get_pose()
        self.estimator.update(self.gyro.get_Rotation2d(), self.get_module_positions())
        curr_pose = self.get_pose()

        dx = curr_pose.x - orig_pose.x
        dy = curr_pose.y - orig_pose.y
        # Add new velocity samples to the buffers
        self._vx_samples.append(dx / 0.02)
        self._vy_samples.append(dy / 0.02)
        # Calculate weighted moving average (recent samples weighted more heavily)
        n = len(self._vx_samples)
        weights = self._velocity_weights[:n]
        total_weight = sum(weights)
        self.vx = sum(v * w for v, w in zip(self._vx_samples, weights, strict=True)) / total_weight
        self.vy = sum(v * w for v, w in zip(self._vy_samples, weights, strict=True)) / total_weight

        self.fused_pose_pub.set(curr_pose)
        # if self.send_modules:
        #     self.setpoints_publisher.set([module.state for module in self.modules])
        #     self.measurements_publisher.set([module.get() for module in self.modules])

    def set_pose(self, pose: Pose2d) -> None:
        self.estimator.resetPosition(
            self.gyro.get_Rotation2d(), self.get_module_positions(), pose
        )
        self.fused_pose_pub.set(pose)

    def reset_yaw(self) -> None:
        """Sets pose to current pose but with a heading of forwards"""
        cur_pose = self.estimator.getEstimatedPosition()
        default_heading = math.pi if is_red() else 0
        self.set_pose(Pose2d(cur_pose.translation(), Rotation2d(default_heading)))

    def get_module_positions(self) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return (
            self.modules[0].get_position(),
            self.modules[1].get_position(),
            self.modules[2].get_position(),
            self.modules[3].get_position(),
        )

    def get_pose(self) -> Pose2d:
        """Get the current location of the robot relative to ???"""
        return self.estimator.getEstimatedPosition()

    def get_rotation(self) -> Rotation2d:
        """Get the current heading of the robot."""
        return self.get_pose().rotation()
