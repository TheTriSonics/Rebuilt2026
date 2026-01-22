import math
import wpilib
import magicbot

from dataclasses import dataclass
from components.drivetrain import DrivetrainComponent
from components.turret import TurretComponent, traj_calc
from components.gyro import GyroComponent
from utilities.scalers import rescale_js

from magicbot import feedback, tunable

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
    turret: TurretComponent

    max_speed = tunable(5.0)
    max_rotation = tunable(math.tau)

    balls: list[BallProperties] = []

    fuel_launch_vel = tunable(2.0)
    fuel_launch_zvel = tunable(5.0)


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
        drive_x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 1.0) * self.max_speed
        return drive_x
    
    @feedback
    def get_drive_y(self) -> float:
        drive_y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 1.0) * self.max_speed
        return drive_y

    @feedback
    def get_drive_rot(self) -> float:
        drive_rot = -rescale_js(self.driver_controller.getRawAxis(3), 0.10, 2.0) * self.max_rotation
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
            field_shot_angle = self.turret.measured_angle - self.gyro.get_Rotation2d().radians()
            distance_to_goal = self.turret.distance_to_goal * 0.90
            vhoriz, vz, t = traj_calc(distance_to_goal)
            xvel = vhoriz * math.cos(field_shot_angle)
            yvel = vhoriz * math.sin(field_shot_angle)
            self.fuel_launch_vel = vhoriz
            self.fuel_launch_zvel = vz
            xvel += self.drivetrain.vx
            yvel += self.drivetrain.vy
            self.balls.append(
                BallProperties(
                    xpos=current_pose.X(),
                    ypos=current_pose.Y(),
                    zpos=0.15,
                    xvel=xvel,
                    yvel=yvel,
                    zvel=vz)
            )
            print('Field shot angle (deg):', math.degrees(field_shot_angle))

    def disabledPeriodic(self):
        ...

