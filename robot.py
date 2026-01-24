import math

import magicbot
import wpilib
from magicbot import feedback, tunable

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.turret import TurretComponent
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Declare components and controllers here
    # Controllers (must be declared before components)

    # Components
    gyro: GyroComponent
    drivetrain: DrivetrainComponent
    turret: TurretComponent

    max_speed = tunable(5.0)
    max_rotation = tunable(math.tau)

    fuel_launch_vel = tunable(2.0)
    fuel_launch_zvel = tunable(5.0)


    def createObjects(self):
        # Create logging and such here; actual robot components are above
        self.data_log = wpilib.DataLogManager.getLog()
        self.field = wpilib.Field2d()
        wpilib.DriverStation.startDataLog(self.data_log, logJoysticks=True)
        wpilib.SmartDashboard.putData(self.field)

        self.driver_controller = wpilib.XboxController(0)

    def autonomousInit(self): ...

    def autonomousPeriodic(self):
        # MagicBot handles periodic execution; this never runs but is left as
        # a placeholder to keep others from trying to implement it thinking
        # it will be called.
        ...

    def teleopInit(self):
        ...

    def teleopPeriodic(self):
        x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 1.0) * self.max_speed
        y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 1.0) * self.max_speed
        rot = -rescale_js(self.driver_controller.getRawAxis(3), 0.10, 2.0) * self.max_rotation
        self.drivetrain.drive_local(x, y, rot)

        if self.driver_controller.getAButtonPressed():
            self.turret.shoot_fuel()

    def disabledPeriodic(self):
        ...

