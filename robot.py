import math
import wpilib

from magicbot import tunable, MagicRobot
from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from components.turret import TurretComponent
from utilities.scalers import rescale_js
from wpimath.geometry import Pose2d
from components.turret import clamp_angle 


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
        self.target_x = None
        self.target_y = None
        self.target_o = None

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
        at_pos = False
        pose = self.drivetrain.get_pose()
        x = -rescale_js(self.driver_controller.getLeftY(), 0.05, 1.0) * self.max_speed
        y = -rescale_js(self.driver_controller.getLeftX(), 0.05, 1.0) * self.max_speed
        rot = -rescale_js(self.driver_controller.getRightX(), 0.10, 2.0) * self.max_rotation

        # Let's check to see if button B is pressed and if so create a point
        # 1 meter away from where we are to drive to
        if self.driver_controller.getBButtonPressed():
            self.target_x = pose.x + 1.0
            self.target_y = pose.y
            self.target_o = clamp_angle(pose.rotation().radians() + math.pi)
            self.target_pose = Pose2d(self.target_x, self.target_y, self.target_o)
            self.drivetrain.target_pose_pub.set(self.target_pose)

        if self.target_x is None:
            self.drive_method(x, y, rot)
        else:
            assert self.target_x is not None
            assert self.target_y is not None
            assert self.target_o is not None
            self.drivetrain.drive_to_position(self.target_x, self.target_y, self.target_o)
            tdiff = self.target_pose.relativeTo(pose)
            dist = math.sqrt(tdiff.x**2 + tdiff.y**2)
            at_pos = dist < 0.02 and tdiff.rotation().degrees() < 1

        if self.driver_controller.getYButtonPressed() or at_pos:
            self.target_x = None
            self.target_y = None
            self.target_o = None

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

