import math
import wpilib

from magicbot import tunable, MagicRobot
from components.shot_calculator import ShotCalculatorComponent
from components.battery_monitor import BatteryMonitorComponent
from components.drivetrain import DrivetrainComponent
from components.vision import VisionComponent
from components.gyro import GyroComponent
from components.turret import TurretComponent
from components.kicker import KickerComponent
from components.climber import ClimberComponent
from components.singulator import SingulatorComponent
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
from utilities.game import is_sim, is_red
from utilities.scalers import clamp_degrees
from choreo import load_swerve_trajectory
from math import cos, sin


OPERATOR_DEBUG = False


class MyRobot(MagicRobot):
    # Declare components and controllers here
    # Controllers (must be declared before components)
    tanker: Tanker
    gaspump: GasPump

    # Components, the order they are declared in determines which one's
    # execute() goes first. This means we can order things a bit.
    kicker: KickerComponent
    climber: ClimberComponent
    singulator: SingulatorComponent
    intake: IntakeComponent
    gyro: GyroComponent
    vision: VisionComponent
    drivetrain: DrivetrainComponent
    shot_calc: ShotCalculatorComponent
    turret: TurretComponent
    shooter: ShooterComponent
    leds: LEDComponent
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

        if is_sim():
            self.control_loop_wait_time = 0.1
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def autonomousInit(self):
        self.drivetrain.reset_yaw()
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
        if not OPERATOR_DEBUG:
            self.gaspump.engage()
        self.tanker.go_drive_local()
        self.shot_calc.set_target("hub")
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
            self.singulator.forward()
        if self.operator_controller.intake_flip():  # Y button
            self.intake.tilt()
        if self.operator_controller.intake_idle():  # Left bumper
            self.intake.off()
        if self.operator_controller.eject():  # X
            self.gaspump.go_eject()
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
        # If a new auton mode is selected set the drivetrain's position to that
        # vision will correct any subtle differences (hopefully!) before it
        # starts running.

        selected = self._automodes.chooser.getSelected()
        if selected != self._last_auton_selection:
            self._last_auton_selection = selected
            if selected:
                initial_pose = selected.get_initial_pose()
                self.drivetrain.set_pose(initial_pose)

