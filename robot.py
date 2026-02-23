import math
import wpilib

from magicbot import tunable, MagicRobot
from components.battery_monitor import BatteryMonitorComponent
from components.drivetrain import DrivetrainComponent
from components.vision import VisionComponent
from components.gyro import GyroComponent
from components.intake import IntakeComponent
from components.leds import LEDComponent
from utilities.scalers import rescale_js
from wpimath.geometry import Pose2d, Rotation2d
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from hid.xbox_driver import RebuiltDriver
from hid.xbox_operator import RebuiltOperator

from controllers.tanker import Tanker
from utilities.game import is_sim
from math import cos, sin



class MyRobot(MagicRobot):
    # Declare components and controllers here
    # Controllers (must be declared before components)
    tanker: Tanker

    # Components
    # vision: VisionComponent
    gyro: GyroComponent
    drivetrain: DrivetrainComponent
    vision: VisionComponent
    intake: IntakeComponent
    leds: LEDComponent
    battery_monitor: BatteryMonitorComponent

    apriltags = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    # Robot's max speed in X/Y plane
    max_speed = tunable(8.0)
    # Robot's max rotation speed in radians per second
    max_rotation = tunable(4*math.tau)
    target_tag = tunable(21)

    def createObjects(self):
        # Create logging and such here; actual robot components are above
        self.data_log = wpilib.DataLogManager.getLog()
        self.field = wpilib.Field2d()
        wpilib.DriverStation.startDataLog(self.data_log, logJoysticks=True)
        wpilib.SmartDashboard.putData(self.field)

        if is_sim():
            self.control_loop_wait_time = 0.1
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def autonomousInit(self):
        self.tanker.engage()

    def autonomousPeriodic(self):
        # MagicBot handles periodic execution; this never runs but is left as
        # a placeholder to keep others from trying to implement it thinking
        # it will be called.
        ...

    def teleopInit(self):
        self.driver_controller = RebuiltDriver()
        self.operator_controller = RebuiltOperator()
        self.tanker.engage()
        self.tanker.go_drive_field()

    def teleopPeriodic(self):
        if self.battery_monitor.is_stop_active():
            print('dead battery')
            # return  # We do NOTHING if the battery is too low. No more robot for you!
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

        if self.driver_controller.intake_on():
            self.intake.intake_on()
        elif self.driver_controller.intake_reverse():
            self.intake.intake_reverse()
        # elif self.operator_controller.intake_on():
        #     self.intake.intake_on()
        # elif self.operator_controller.intake_reverse():
        #     self.intake.intake_reverse()
        else:
            self.intake.intake_off()

        if self.driver_controller.go_to_point():
            target_pose = self.apriltags.getTagPose(self.target_tag)
            assert target_pose
            twod_pose = target_pose.toPose2d()
            dist_away = 2.0
            newx = target_pose.x + dist_away * cos(twod_pose.rotation().radians())
            newy = target_pose.y + dist_away * sin(twod_pose.rotation().radians())
            target_pose = Pose2d(newx, newy, twod_pose.rotation().rotateBy(Rotation2d.fromDegrees(180)))
            self.tanker.go_drive_pose(target_pose)

        if self.driver_controller.reset_yaw():
            omega = 0

    def disabledPeriodic(self):
        ...

