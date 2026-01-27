import math
import wpilib

from magicbot import tunable, MagicRobot
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.turret import TurretComponent
from utilities.scalers import rescale_js


class MyRobot(MagicRobot):
    # Declare components and controllers here
    # Controllers (must be declared before components)

    # Components
    gyro: GyroComponent
    drivetrain: DrivetrainComponent
    turret: TurretComponent

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

    def autonomousInit(self): ...

    def autonomousPeriodic(self):
        # MagicBot handles periodic execution; this never runs but is left as
        # a placeholder to keep others from trying to implement it thinking
        # it will be called.
        ...

    def teleopInit(self):
        self.driver_controller = wpilib.XboxController(0)
        self.drive_method = self.drivetrain.drive_field
        ...

    def teleopPeriodic(self):
        x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 1.0) * self.max_speed
        y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 1.0) * self.max_speed
        rot = -rescale_js(self.driver_controller.getRawAxis(3), 0.10, 2.0) * self.max_rotation
        self.drive_method(x, y, rot)

        if self.driver_controller.getAButtonPressed():
            self.turret.shoot_fuel()

        if self.driver_controller.getRawButtonPressed(8):
            print('toggle drive mode')
            if self.drive_method == self.drivetrain.drive_field:
                self.drive_method = self.drivetrain.drive_local
            else:
                self.drive_method = self.drivetrain.drive_field

    def disabledPeriodic(self):
        ...

