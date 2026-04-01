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

    # Dashboard button — toggles SignalLogger (momentary, same as Back on either controller)
    dash_toggle_signal_logger = tunable(False)

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

        # --- SignalLogger toggle (Back on either controller, or dashboard button) ---
        if (
            self.tech_controller.toggle_signal_logger()
            or self.tech_controller2.toggle_signal_logger()
            or self._consume(self, "dash_toggle_signal_logger")
        ):
            if self._signal_logger_running:
                SignalLogger.stop()
                self._signal_logger_running = False
            else:
                SignalLogger.start()
                self._signal_logger_running = True

        # --- Emergency stop (controller Start or dashboard button) ---
        c = self.commissioning
        mc = self.motor_commissioning
        if self.tech_controller.emergency_stop() or self._consume(c, "dash_emergency_stop"):
            c.go_idle()
            if self._signal_logger_running:
                SignalLogger.stop()
                self._signal_logger_running = False
        if self.tech_controller2.emergency_stop() or self._consume(mc, "dash_emergency_stop"):
            mc.go_idle()
            if self._signal_logger_running:
                SignalLogger.stop()
                self._signal_logger_running = False

        # --- Drivetrain commissioning tests (TechController port 2 + dashboard) ---
        # Auto-starts SignalLogger when a test is triggered so you don't have to
        # manually press Back first, but you can still toggle it independently.
        if self.tech_controller.turn_quasistatic_fwd() or self._consume(
            c, "dash_turn_quasistatic_fwd"
        ):
            self._trigger_test(c.go_turn_quasistatic_fwd)
        elif self.tech_controller.turn_quasistatic_rev() or self._consume(
            c, "dash_turn_quasistatic_rev"
        ):
            self._trigger_test(c.go_turn_quasistatic_rev)
        elif self.tech_controller.turn_dynamic_fwd() or self._consume(c, "dash_turn_dynamic_fwd"):
            self._trigger_test(c.go_turn_dynamic_fwd)
        elif self.tech_controller.turn_dynamic_rev() or self._consume(c, "dash_turn_dynamic_rev"):
            self._trigger_test(c.go_turn_dynamic_rev)
        elif self.tech_controller.drive_quasistatic_fwd() or self._consume(
            c, "dash_drive_quasistatic_fwd"
        ):
            self._trigger_test(c.go_drive_quasistatic_fwd)
        elif self.tech_controller.drive_quasistatic_rev() or self._consume(
            c, "dash_drive_quasistatic_rev"
        ):
            self._trigger_test(c.go_drive_quasistatic_rev)
        elif self.tech_controller.drive_dynamic_fwd() or self._consume(c, "dash_drive_dynamic_fwd"):
            self._trigger_test(c.go_drive_dynamic_fwd)
        elif self.tech_controller.drive_dynamic_rev() or self._consume(c, "dash_drive_dynamic_rev"):
            self._trigger_test(c.go_drive_dynamic_rev)
        elif self.tech_controller.snap_0() or self._consume(c, "dash_snap_0"):
            self._trigger_test(lambda: c.go_heading_snap_test(c.snap_target_0_deg))
        elif self.tech_controller.snap_90() or self._consume(c, "dash_snap_90"):
            self._trigger_test(lambda: c.go_heading_snap_test(c.snap_target_1_deg))
        elif self.tech_controller.snap_180() or self._consume(c, "dash_snap_180"):
            self._trigger_test(lambda: c.go_heading_snap_test(c.snap_target_2_deg))
        elif self.tech_controller.snap_270() or self._consume(c, "dash_snap_270"):
            self._trigger_test(lambda: c.go_heading_snap_test(c.snap_target_3_deg))
        elif self.tech_controller.translation_test() or self._consume(c, "dash_translation_test"):
            self._trigger_test(c.go_translation_test)

        # --- Generic motor commissioning tests (TechController2 port 3 + dashboard) ---
        if self.tech_controller2.quasistatic_fwd() or self._consume(mc, "dash_quasistatic_fwd"):
            self._trigger_test(mc.go_quasistatic_fwd)
        elif self.tech_controller2.quasistatic_rev() or self._consume(mc, "dash_quasistatic_rev"):
            self._trigger_test(mc.go_quasistatic_rev)
        elif self.tech_controller2.dynamic_fwd() or self._consume(mc, "dash_dynamic_fwd"):
            self._trigger_test(mc.go_dynamic_fwd)
        elif self.tech_controller2.dynamic_rev() or self._consume(mc, "dash_dynamic_rev"):
            self._trigger_test(mc.go_dynamic_rev)

        wpilib.SmartDashboard.putBoolean("SignalLogger Running", self._signal_logger_running)

    def _consume(self, obj: object, attr: str) -> bool:
        """Read a momentary dashboard tunable, reset it to False, return its value."""
        if getattr(obj, attr, False):
            setattr(obj, attr, False)
            return True
        return False

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
