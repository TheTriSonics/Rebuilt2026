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
    shooter_rps = tunable(30.0)

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
        self.turret.set_hub_target()

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

        operator_turret = rescale_js(self.operator_controller.turret_movement(), 0.05, 1.0)
        driver_turret = self.driver_controller.turret_left() + self.driver_controller.turret_right()
        self.turret.set_manual_speed(operator_turret if operator_turret != 0 else driver_turret)

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

        if self.operator_controller.singulator_forward():
            self.singulator.singulator_forward()
        elif self.operator_controller.singulator_reverse():
            self.singulator.singulator_reverse()
        else:
            self.singulator.singulator_off()

        if self.operator_controller.kicker_on():
            self.kicker.kicker_forward()
        else:
            self.kicker.kicker_off()
        
        if self.operator_controller.shooter_shoot():
            # self.turret.shoot_fuel()
            self.shooter.spin_up(self.shooter_rps)
        else:
            self.shooter.stop()

        
        

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

        """if self.operator_controller.status == 'on':
            self.turret.shoot_fuel()"""

    def disabledPeriodic(self):
        ...

