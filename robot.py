import math
import wpilib

from magicbot import tunable, MagicRobot
from components.shot_calculator import ShotCalculatorComponent
from components.battery_monitor import BatteryMonitorComponent
from components.drivetrain import DrivetrainComponent
from components.vision import VisionComponent
from components.gyro import GyroComponent
from components.kicker import KickerComponent
from components.climber import ClimberComponent
from components.intake import IntakeComponent
from components.leds import LEDComponent
from components.shooter import ShooterComponent
from utilities.scalers import rescale_js
from wpimath.geometry import Pose2d, Rotation2d
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from hid.xbox_driver import RebuiltDriver
from hid.xbox_operator import RebuiltOperator

from controllers.tanker import Tanker
from controllers.gaspump import GasPump
from utilities.game import is_sim, is_red, is_left, init_side_chooser
from utilities.scalers import clamp_degrees
from choreo import load_swerve_trajectory
from math import cos, sin



class MyRobot(MagicRobot):
    # Declare components and controllers here
    # Controllers (must be declared before components)
    tanker: Tanker
    gaspump: GasPump

    # Components, the order they are declared in determines which one's
    # execute() goes first. This means we can order things a bit.
    kicker: KickerComponent
    # climber: ClimberComponent
    intake: IntakeComponent
    gyro: GyroComponent
    vision: VisionComponent
    drivetrain: DrivetrainComponent
    shot_calc: ShotCalculatorComponent
    shooter: ShooterComponent
    # leds: LEDComponent
    battery_monitor: BatteryMonitorComponent

    apriltags = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    # Robot's max speed in X/Y plane
    max_speed = tunable(8.0)
    # Robot's max rotation speed in radians per second
    max_rotation = tunable(4*math.tau)

    def createObjects(self):
        # Create logging and such here; actual robot components are above
        self.data_log = wpilib.DataLogManager.getLog()
        self.field = wpilib.Field2d()
        wpilib.DriverStation.startDataLog(self.data_log, logJoysticks=True)
        wpilib.SmartDashboard.putData(self.field)

        self._last_auton_selection = None
        self._last_config_key = None
        init_side_chooser()
        wpilib.SmartDashboard.putBoolean("Load Trajectories", False)
        wpilib.SmartDashboard.putBoolean("Trajectories Ready", False)

        if is_sim():
            self.control_loop_wait_time = 0.1
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def autonomousInit(self):
        curr_pose = self.drivetrain.get_pose()
        # resetPosition (called inside set_pose) re-syncs the gyro offset
        # internally, so we don't need to reset the hardware gyro
        self.drivetrain.set_pose(curr_pose)
        self.tanker.engage()
        self.gaspump.engage()

    def autonomousPeriodic(self):
        # MagicBot handles periodic execution; this never runs but is left as
        # a placeholder to keep others from trying to implement it thinking
        # it will be called.
        ...

    def teleopInit(self):
        self.driver_controller = RebuiltDriver()
        self.operator_controller = RebuiltOperator()
        self.tanker.engage()
        self.gaspump.engage()
        self.gaspump.go_stop()
        self.tanker.go_drive_local()
        self.shot_calc.set_target("hub")
        self.drivetrain.stop_snapping()
        # self.drivetrain.set_pose(Pose2d(14.2, 5.0, -1.34))
        # self.gyro.reset_heading(math.degrees(-1.34))

    def teleopPeriodic(self):
        if self.battery_monitor.is_stop_active():
            print('dead battery')
            # return  # We do NOTHING if the battery is too low. No more robot for you!
        x = -rescale_js(self.driver_controller.get_left_y(), 0.05, 1.0) * self.max_speed
        y = -rescale_js(self.driver_controller.get_left_x(), 0.05, 1.0) * self.max_speed
        omega = -rescale_js(self.driver_controller.get_right_x(), 0.10, 2.0) * self.max_rotation

        self.tanker.set_stick_values(x, y, omega)

        # Force the drive mode to field relative and put the driver in control,
        # aborts any drive to point or trajectory following
        if self.driver_controller.field_centric():
            self.tanker.go_drive_field()

        if self.driver_controller.robot_centric():
            self.tanker.go_drive_local()

        if self.driver_controller.reset_yaw():
            self.drivetrain.reset_yaw()
        if self.driver_controller.set_heading_to_vision():
            curr_pose = self.drivetrain.get_pose()
            # resetPosition (called inside set_pose) re-syncs the gyro offset
            # internally, so we don't need to reset the hardware gyro
            self.drivetrain.set_pose(curr_pose)


        if self.driver_controller.intake_up():
            self.intake.rotate_up()

        if self.driver_controller.target_lob_left():
            self.shot_calc.set_target("left")
            self.tanker.go_drive_auto_target()
        elif self.driver_controller.target_lob_right():
            self.shot_calc.set_target("right")
            self.tanker.go_drive_auto_target()
        elif self.driver_controller.target_hub():
            self.shot_calc.set_target("hub")
            self.tanker.go_drive_auto_target()
        else:
            self.tanker.go_drive_last_mode()

        # operator_turret = rescale_js(self.operator_controller.turret_movement(), 0.05, 1.0)
        # driver_turret = self.driver_controller.turret_left() + self.driver_controller.turret_right()
        # self.turret.set_manual_speed(operator_turret if operator_turret != 0 else driver_turret)

        if self.operator_controller.intake_on():  # Right trigger
            self.intake.on()
            self.intake.rotate_down()
        if self.operator_controller.intake_flip():  # Y button
            self.intake.rotate_tilt()
        if self.operator_controller.intake_idle():  # Left bumper
            self.intake.off()
        # if self.operator_controller.eject():  # X
        #     self.gaspump.go_eject()
        if self.operator_controller.shooter_shoot(): # Right bumper
            self.gaspump.go_shoot()
        if self.operator_controller.shooter_off(): # A button
            self.gaspump.go_shoot_off()

        if self.operator_controller.turret_aim_hub():
            self.shot_calc.set_target("hub")
        if self.operator_controller.turret_aim_left():
            self.shot_calc.set_target("left")
        if self.operator_controller.turret_aim_right():
            self.shot_calc.set_target("right")


    def disabledPeriodic(self):
        # this keeps us updating odometry even when disabled, vision will put
        # us where it can
        self.vision.execute()
        self.drivetrain.update_odometry()
        # Force the gyro to honor our estimated pose that way it'll align when we begin auton
        heading = clamp_degrees(self.drivetrain.get_pose().rotation().degrees())
        wpilib.SmartDashboard.putNumber("Estimated Heading", heading)
        self.gyro.reset_heading(heading)

        selected = self._automodes.chooser.getSelected()
        config_key = ('red' if is_red() else 'blue') + ('_left' if is_left() else '_right')

        # Auto-set pose and clear Ready when selection or side changes
        if selected != self._last_auton_selection or config_key != self._last_config_key:
            self._last_auton_selection = selected
            self._last_config_key = config_key
            wpilib.SmartDashboard.putBoolean("Trajectories Ready", False)
            if selected:
                selected.set_initial_pose()

        # Manual Load button — force recompute + set Ready indicator
        if wpilib.SmartDashboard.getBoolean("Load Trajectories", False):
            wpilib.SmartDashboard.putBoolean("Load Trajectories", False)
            if selected:
                selected.pose_set = False
                selected.set_initial_pose()
                wpilib.SmartDashboard.putBoolean("Trajectories Ready", True)

        # Vision quality indicator
        vision_ok = self.vision.has_good_vision()
        wpilib.SmartDashboard.putBoolean("StartPose/VisionReady", vision_ok)

        # Live pose agreement indicator
        if selected and selected.cached_initial_pose is not None:
            robot_pose = self.drivetrain.get_pose()
            expected = selected.cached_initial_pose
            dx = robot_pose.x - expected.x
            dy = robot_pose.y - expected.y
            dist = math.hypot(dx, dy)
            raw_err = abs(math.degrees(
                robot_pose.rotation().radians() - expected.rotation().radians()
            ))
            heading_err = min(raw_err, 360.0 - raw_err)
            wpilib.SmartDashboard.putNumber("StartPose/DistanceM", dist)
            wpilib.SmartDashboard.putNumber("StartPose/XErrorM", dx)
            wpilib.SmartDashboard.putNumber("StartPose/YErrorM", dy)
            wpilib.SmartDashboard.putNumber("StartPose/HeadingErrDeg", heading_err)
            wpilib.SmartDashboard.putBoolean("StartPose/PoseMatch", dist < 0.15 and heading_err < 5.0)

