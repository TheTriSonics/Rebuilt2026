import math
import wpilib

from magicbot import tunable, MagicRobot
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.turret import TurretComponent, clamp_angle
from components.kicker import KickerComponent
from components.climber import ClimberComponent
from components.singulater import SingulaterComponent
from utilities.scalers import rescale_js
from wpimath.geometry import Pose2d

from controllers.tanker import Tanker



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
    singulater: SingulaterComponent

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
        self.driver_controller = wpilib.XboxController(0)

    def teleopPeriodic(self):
        pose = self.drivetrain.get_pose()

        x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 1.0) * self.max_speed
        y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 1.0) * self.max_speed
        omega = -rescale_js(self.driver_controller.getRightX(), 0.10, 2.0) * self.max_rotation

        self.tanker.set_stick_values(x, y, omega)

        # Let's check to see if button B is pressed and if so create a point
        # 1 meter away from where we are to drive to
        if self.driver_controller.getBButtonPressed():
            self.target_x = pose.x + 1.0
            self.target_y = pose.y
            self.target_o = clamp_angle(pose.rotation().radians() + math.pi)
            target_pose = Pose2d(self.target_x, self.target_y, self.target_o)
            self.tanker.go_drive_pose(target_pose)


        # Toggle between field relative and robot relative
        if self.driver_controller.getRawButtonPressed(8):
            # Menu / hamburger button
            self.tanker.toggle_mode()

        # Force the drive mode to field relative and put the driver in control,
        # aborts any drive to point or trajectory following
        if self.driver_controller.getYButtonPressed():
            self.tanker.go_drive_field()

        if self.driver_controller.getAButtonPressed():
            self.turret.shoot_fuel()

        if self.driver_controller.getXButtonPressed():
            self.tanker.go_follow_path('test_path')
        
        if self.driver_controller.getLeftBumperPressed():
            self.climber.set_speed(100)
            self.climber.raise_climber()

        if self.driver_controller.getRightBumperPressed():
            self.singulater.singulater_forward

        if self.driver_controller.getPOV(180):
            self.singulater.singulater_reverse




        if self.driver_controller.getRightTriggerAxis() > 0.55:
            self.kicker.kicker_forward()
        elif self.driver_controller.getRightTriggerAxis() < 0.45:
            self.kicker.kicker_off()

        if self.driver_controller.getLeftTriggerAxis() > 0.55:
            self.kicker.kicker_reverse()
        elif self.driver_controller.getLeftTriggerAxis() < 0.45:
            self.kicker.kicker_off()


    def disabledPeriodic(self):
        ...

