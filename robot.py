"""
Commissioning robot.py — barebones drivetrain characterization environment.

This is NOT the competition robot.py. It exists on the 'commissioning' branch
to provide a minimal execution environment for the Commissioning controller.

Only DrivetrainComponent and GyroComponent are active. All game-specific
components (intake, shooter, vision, etc.) are absent so they cannot
interfere with characterization tests.

See controllers/commissioning.py for full usage instructions and the
step-by-step tuning procedure.
"""

import math
import wpilib

from collections.abc import Callable

from magicbot import MagicRobot, tunable
from phoenix6 import SignalLogger

from components.drivetrain import DrivetrainComponent
from components.gyro import GyroComponent
from controllers.commissioning import Commissioning
from controllers.motor_commissioning import MotorCommissioning
from hid.xbox_tech import TechController
from hid.xbox_tech2 import TechController2
from utilities.game import is_sim


class MyRobot(MagicRobot):
    # Controllers (declared before components)
    commissioning: Commissioning
    motor_commissioning: MotorCommissioning

    # Components
    gyro: GyroComponent
    drivetrain: DrivetrainComponent

    max_speed = tunable(8.0)
    max_rotation = tunable(4 * math.tau)

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()
        wpilib.DriverStation.startDataLog(self.data_log, logJoysticks=True)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        # Tracks whether SignalLogger is currently recording
        self._signal_logger_running = False

        if is_sim():
            self.control_loop_wait_time = 0.1
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def teleopInit(self) -> None:
        self.tech_controller = TechController()
        self.tech_controller2 = TechController2()
        self.commissioning.engage()
        self.motor_commissioning.engage()
        curr_pose = self.drivetrain.get_pose()
        self.drivetrain.set_pose(curr_pose)

    def teleopPeriodic(self) -> None:
        self.tech_controller.update()
        self.tech_controller2.update()

        # --- SignalLogger toggle (Back button on either controller) ---
        if (
            self.tech_controller.toggle_signal_logger()
            or self.tech_controller2.toggle_signal_logger()
        ):
            if self._signal_logger_running:
                SignalLogger.stop()
                self._signal_logger_running = False
            else:
                SignalLogger.start()
                self._signal_logger_running = True

        # --- Emergency stop ---
        if self.tech_controller.emergency_stop():
            self.commissioning.go_idle()
            if self._signal_logger_running:
                SignalLogger.stop()
                self._signal_logger_running = False
        if self.tech_controller2.emergency_stop():
            self.motor_commissioning.go_idle()
            if self._signal_logger_running:
                SignalLogger.stop()
                self._signal_logger_running = False

        # --- Test triggers ---
        # Auto-starts SignalLogger when a test is triggered so you don't have to
        # manually press Back first, but you can still toggle it independently.
        if self.tech_controller.turn_quasistatic_fwd():
            self._trigger_test(self.commissioning.go_turn_quasistatic_fwd)
        elif self.tech_controller.turn_quasistatic_rev():
            self._trigger_test(self.commissioning.go_turn_quasistatic_rev)
        elif self.tech_controller.turn_dynamic_fwd():
            self._trigger_test(self.commissioning.go_turn_dynamic_fwd)
        elif self.tech_controller.turn_dynamic_rev():
            self._trigger_test(self.commissioning.go_turn_dynamic_rev)
        elif self.tech_controller.drive_quasistatic_fwd():
            self._trigger_test(self.commissioning.go_drive_quasistatic_fwd)
        elif self.tech_controller.drive_quasistatic_rev():
            self._trigger_test(self.commissioning.go_drive_quasistatic_rev)
        elif self.tech_controller.drive_dynamic_fwd():
            self._trigger_test(self.commissioning.go_drive_dynamic_fwd)
        elif self.tech_controller.drive_dynamic_rev():
            self._trigger_test(self.commissioning.go_drive_dynamic_rev)
        elif self.tech_controller.snap_0():
            self._trigger_test(
                lambda: self.commissioning.go_heading_snap_test(
                    self.commissioning.snap_target_0_deg
                )
            )
        elif self.tech_controller.snap_90():
            self._trigger_test(
                lambda: self.commissioning.go_heading_snap_test(
                    self.commissioning.snap_target_1_deg
                )
            )
        elif self.tech_controller.snap_180():
            self._trigger_test(
                lambda: self.commissioning.go_heading_snap_test(
                    self.commissioning.snap_target_2_deg
                )
            )
        elif self.tech_controller.snap_270():
            self._trigger_test(
                lambda: self.commissioning.go_heading_snap_test(
                    self.commissioning.snap_target_3_deg
                )
            )
        elif self.tech_controller.translation_test():
            self._trigger_test(self.commissioning.go_translation_test)

        # --- Generic motor commissioning tests (TechController2, port 3) ---
        if self.tech_controller2.quasistatic_fwd():
            self._trigger_test(self.motor_commissioning.go_quasistatic_fwd)
        elif self.tech_controller2.quasistatic_rev():
            self._trigger_test(self.motor_commissioning.go_quasistatic_rev)
        elif self.tech_controller2.dynamic_fwd():
            self._trigger_test(self.motor_commissioning.go_dynamic_fwd)
        elif self.tech_controller2.dynamic_rev():
            self._trigger_test(self.motor_commissioning.go_dynamic_rev)

        wpilib.SmartDashboard.putBoolean("SignalLogger Running", self._signal_logger_running)

    def _trigger_test(self, go_fn: Callable[[], None]) -> None:
        """Auto-start SignalLogger if needed, then invoke a commissioning test."""
        if not self._signal_logger_running:
            SignalLogger.start()
            self._signal_logger_running = True
        go_fn()

    def disabledPeriodic(self) -> None:
        # Keep odometry fresh while disabled so the first snap/translation test
        # has an accurate starting pose.
        self.drivetrain.update_odometry()
        self.field.setRobotPose(self.drivetrain.get_pose())
