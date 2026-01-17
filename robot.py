import math
import wpilib
import magicbot

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Declare components and controllers here
    # Controllers (must be declared before components)

    # Components
    gyro: GyroComponent
    drivetrain: DrivetrainComponent

    def createObjects(self):
        # Create logging and such here; actual robot components are above
        self.data_log = wpilib.DataLogManager.getLog()
        wpilib.DriverStation.startDataLog(self.data_log, logJoysticks=True)
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def autonomousInit(self): ...

    def autonomousPeriodic(self):
        # MagicBot handles periodic execution; this never runs but is left as
        # a placeholder to keep others from trying to implement it thinking
        # it will be called.
        ...

    def teleopInit(self):
        self.driver_controller = wpilib.XboxController(0)

    def teleopPeriodic(self):
        xyfudge = 500
        rotfudge = 200 * math.pi
        drive_x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 1.0) * xyfudge
        drive_y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 1.0) * xyfudge
        drive_rot = rescale_js(self.driver_controller.getRightX(), 0.05, 1.0) * rotfudge
        self.drivetrain.drive_local(drive_x, drive_y, drive_rot)

    def disabledPeriodic(self):
        ...

