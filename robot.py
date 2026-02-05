import math
import wpilib

from magicbot import tunable, MagicRobot
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.turret import TurretComponent, clamp_angle
from components.kicker import KickerComponent
from components.climber import ClimberComponent
from components.singulator import SingulatorComponent
from components.Intake import IntakeComponent
from utilities.scalers import rescale_js
from wpimath.geometry import Pose2d
from hid.xbox_driver import RebuiltDriver

from controllers.tanker import Tanker
from utilities.game import is_sim



class MyRobot(MagicRobot):
    # Declare components and controllers here
    # Controllers (must be declared before components)
    tanker: Tanker

    # Components
    gyro: GyroComponent
    drivetrain: DrivetrainComponent
    turret: TurretComponent
    kicker: KickerComponent
    climber: ClimberComponent
    singulator: SingulatorComponent
    intake: IntakeComponent

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

    def autonomousInit(self):
        self.tanker.engage()

    def autonomousPeriodic(self):
        # MagicBot handles periodic execution; this never runs but is left as
        # a placeholder to keep others from trying to implement it thinking
        # it will be called.
        ...

    def teleopInit(self):
        self.tanker.engage()
        self.tanker.go_drive_field()
        self.turret.set_hub_target()
        self.driver_controller = RebuiltDriver()
        self.operator_controller = wpilib.XboxController(1)

    def teleopPeriodic(self):
        pose = self.drivetrain.get_pose()

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

        if self.driver_controller.shooter():
            self.turret.shoot_fuel()

        if self.driver_controller.intake_on():
            self.intake.intake_on()
        else:
            self.intake.intake_off()
        """
        if self.driver_controller.getLeftBumperPressed():
            self.climber.set_speed(100)
            self.climber.raise_climber()

        if self.driver_controller.getRightBumperPressed():
            # TODO: Fix error
            self.singulator.singulator_forward()

        if self.driver_controller.getPOV(180):
            # TODO: Fix error
            self.singulator.singulator_reverse()


        if self.driver_controller.getRightTriggerAxis() > 0.55:
            self.kicker.kicker_forward()
        elif self.driver_controller.getRightTriggerAxis() < 0.45:
            self.kicker.kicker_off()

        if self.driver_controller.getLeftTriggerAxis() > 0.55:
            self.kicker.kicker_reverse()
        elif self.driver_controller.getLeftTriggerAxis() < 0.45:
            self.kicker.kicker_off()
        """

    def disabledPeriodic(self):
        ...

