import math
import wpilib

from magicbot import tunable, MagicRobot
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
from choreo import load_swerve_trajectory
from math import cos, sin


OPERATOR_DEBUG = False



class MyRobot(MagicRobot):
    # Declare components and controllers here
    # Controllers (must be declared before components)
    tanker: Tanker
    gaspump: GasPump

    # Components
    gyro: GyroComponent
    drivetrain: DrivetrainComponent
    vision: VisionComponent
    turret: TurretComponent
    kicker: KickerComponent
    climber: ClimberComponent
    singulator: SingulatorComponent
    intake: IntakeComponent
    shooter: ShooterComponent
    leds: LEDComponent
    battery_monitor: BatteryMonitorComponent

    apriltags = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    # Robot's max speed in X/Y plane
    max_speed = tunable(8.0)
    # Robot's max rotation speed in radians per second
    max_rotation = tunable(4*math.tau)
    target_tag = tunable(21)
    shooter_rps_target = tunable(50.0)

    def createObjects(self):
        # Create logging and such here; actual robot components are above
        self.data_log = wpilib.DataLogManager.getLog()
        self.field = wpilib.Field2d()
        wpilib.DriverStation.startDataLog(self.data_log, logJoysticks=True)
        wpilib.SmartDashboard.putData(self.field)

        self.auton_chooser = wpilib.SendableChooser()
        self.auton_chooser.setDefaultOption("Do Nothing", "")
        self.auton_chooser.addOption("Ball Path", "ball_path")
        self.auton_chooser.addOption("Go To Player Station", "go_to_player_station")
        self.auton_chooser.addOption("Left Blue Simple", "Left_blue_simple")
        self.auton_chooser.addOption("Middle Hang", "middle_hang")
        self.auton_chooser.addOption("Right Blue Simple", "right_blue_simple")
        self.auton_chooser.addOption("Test Path", "test_path")
        wpilib.SmartDashboard.putData("Auto Mode", self.auton_chooser)
        self._last_auton_selection = None

        if is_sim():
            self.control_loop_wait_time = 0.1
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def autonomousInit(self):
        self.drivetrain.reset_yaw()
        self.tanker.engage()
        selected = self.auton_chooser.getSelected()
        self.turret.set_target("hub")
        if selected:
            self.tanker.go_follow_path(selected)
        else:
            self.tanker.go_drive_field()

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
        self.tanker.go_drive_field()
        self.turret.set_target("hub")

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

        if self.driver_controller.intake_up():
            self.intake.rotate_up()

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
            self.tanker.go_drive_hub_lockon()
        if self.operator_controller.shooter_off(): # A button
            self.gaspump.go_shoot_off()
            self.tanker.go_drive_field()

        if self.operator_controller.turret_aim_hub():
            self.turret.set_target("hub")
        if self.operator_controller.turret_aim_left():
            self.turret.set_target("left")
        if self.operator_controller.turret_aim_right():
            self.turret.set_target("right")


    def disabledPeriodic(self):
        self.vision.execute()
        selected = self.auton_chooser.getSelected()
        if selected != self._last_auton_selection:
            self._last_auton_selection = selected
            if selected:
                traj = load_swerve_trajectory(selected)
                sample = traj.sample_at(0.0, is_red())
                assert sample
                self.drivetrain.set_pose(sample.get_pose())

