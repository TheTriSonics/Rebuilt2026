import math
import wpilib
import magicbot

from dataclasses import dataclass
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from utilities.scalers import rescale_js

from magicbot import feedback

pn = wpilib.SmartDashboard.putNumber

@dataclass
class BallProperties:
    xpos: float
    ypos: float
    zpos: float
    xvel: float
    yvel: float
    zvel: float

class MyRobot(magicbot.MagicRobot):
    # Declare components and controllers here
    # Controllers (must be declared before components)

    # Components
    gyro: GyroComponent
    drivetrain: DrivetrainComponent

    xyfudge = 250.0
    rotfudge = 2.0 * math.pi

    balls: list[BallProperties] = []


    def createObjects(self):
        # Create logging and such here; actual robot components are above
        self.data_log = wpilib.DataLogManager.getLog()
        wpilib.DriverStation.startDataLog(self.data_log, logJoysticks=True)
        self.field = wpilib.Field2d()
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

    @feedback
    def get_drive_x(self) -> float:
        drive_x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 1.0) * self.xyfudge
        return drive_x
    
    @feedback
    def get_drive_y(self) -> float:
        drive_y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 1.0) * self.xyfudge
        return drive_y

    @feedback
    def get_drive_rot(self) -> float:
        drive_rot = rescale_js(self.driver_controller.getRawAxis(3), 0.05, 2.0) * self.rotfudge
        return drive_rot

    def teleopPeriodic(self):
        current_pose = self.drivetrain.get_pose()
        x, y, rot = (
            self.get_drive_x(),
            self.get_drive_y(),
            self.get_drive_rot(),
        )
        self.drivetrain.drive_local(x, y, rot)
        if self.driver_controller.getAButtonPressed():
            self.balls.append(
                BallProperties(
                    xpos=current_pose.X(),
                    ypos=current_pose.Y(),
                    zpos=1.0,
                    xvel=2.0,
                    yvel=0.0,
                    zvel=5.0,)
            )
        print('Balls:', len(self.balls))

    def disabledPeriodic(self):
        ...

