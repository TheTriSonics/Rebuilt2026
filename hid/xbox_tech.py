"""
TechController — Xbox controller for robot commissioning and characterization (DS port 2).

This HID wrapper is ONLY used with the Commissioning controller during first-boot
hardware validation and drivetrain characterization. It is never instantiated during
normal competition operation.

BUTTON REFERENCE
================
Back  (btn 7)  — Toggle CTRE SignalLogger (start/stop .hoot recording)
Start (btn 8)  — Emergency stop (abort test, return to idle)

TURN MOTOR CHARACTERIZATION (SysId — steer motors)
  A             — Turn quasistatic forward  (slow voltage ramp, steer fwd)
  B             — Turn quasistatic reverse  (slow voltage ramp, steer rev)
  X             — Turn dynamic forward      (step voltage, steer fwd)
  Y             — Turn dynamic reverse      (step voltage, steer rev)

DRIVE MOTOR CHARACTERIZATION (SysId — drive motors)
  Left Bumper   — Drive quasistatic forward
  Right Bumper  — Drive quasistatic reverse
  Left Trigger  — Drive dynamic forward     (hold >50%)
  Right Trigger — Drive dynamic reverse     (hold >50%)

HEADING PID TUNING
  D-pad Up      — Snap to snap_target_0_deg  (default 0°)
  D-pad Right   — Snap to snap_target_1_deg  (default 90°)
  D-pad Down    — Snap to snap_target_2_deg  (default 180°)
  D-pad Left    — Snap to snap_target_3_deg  (default 270°)

TRANSLATION PID TUNING
  Left Stick    — Drive straight for translation_distance_m then stop
  (Button 9)
"""

import wpilib


class TechController:
    """Thin wrapper around XboxController(2) for commissioning use.

    All methods return booleans (pressed-edge for buttons, level for axes).
    Call update() once per teleopPeriodic loop before reading any methods.
    No state other than the pressed-edge tracking fields is held here.
    """

    TRIGGER_THRESHOLD = 0.5

    def __init__(self) -> None:
        self._controller = wpilib.XboxController(2)
        self._lt_was_pressed = False
        self._rt_was_pressed = False
        self._pov_prev = -1

    def update(self) -> None:
        """Must be called once per loop before reading trigger or POV pressed events."""
        self._lt_was_pressed = self._controller.getLeftTriggerAxis() > self.TRIGGER_THRESHOLD
        self._rt_was_pressed = self._controller.getRightTriggerAxis() > self.TRIGGER_THRESHOLD
        self._pov_prev = self._controller.getPOV()

    # --- SignalLogger / safety ---

    def toggle_signal_logger(self) -> bool:
        """Back button — toggle CTRE SignalLogger on/off (pressed-edge)."""
        return self._controller.getBackButtonPressed()

    def emergency_stop(self) -> bool:
        """Start button — abort current test and return to idle (pressed-edge)."""
        return self._controller.getStartButtonPressed()

    # --- Turn motor characterization ---

    def turn_quasistatic_fwd(self) -> bool:
        """A button — begin steer quasistatic forward sweep (pressed-edge)."""
        return self._controller.getAButtonPressed()

    def turn_quasistatic_rev(self) -> bool:
        """B button — begin steer quasistatic reverse sweep (pressed-edge)."""
        return self._controller.getBButtonPressed()

    def turn_dynamic_fwd(self) -> bool:
        """X button — begin steer dynamic forward step (pressed-edge)."""
        return self._controller.getXButtonPressed()

    def turn_dynamic_rev(self) -> bool:
        """Y button — begin steer dynamic reverse step (pressed-edge)."""
        return self._controller.getYButtonPressed()

    # --- Drive motor characterization ---

    def drive_quasistatic_fwd(self) -> bool:
        """Left bumper — begin drive quasistatic forward sweep (pressed-edge)."""
        return self._controller.getLeftBumperButtonPressed()

    def drive_quasistatic_rev(self) -> bool:
        """Right bumper — begin drive quasistatic reverse sweep (pressed-edge)."""
        return self._controller.getRightBumperButtonPressed()

    def drive_dynamic_fwd(self) -> bool:
        """Left trigger >50% — begin drive dynamic forward step (rising-edge)."""
        is_pressed = self._controller.getLeftTriggerAxis() > self.TRIGGER_THRESHOLD
        return is_pressed and not self._lt_was_pressed

    def drive_dynamic_rev(self) -> bool:
        """Right trigger >50% — begin drive dynamic reverse step (rising-edge)."""
        is_pressed = self._controller.getRightTriggerAxis() > self.TRIGGER_THRESHOLD
        return is_pressed and not self._rt_was_pressed

    # --- Heading PID tuning ---

    def snap_0(self) -> bool:
        """D-pad Up — snap to snap_target_0_deg (rising-edge)."""
        return self._controller.getPOV() == 0 and self._pov_prev != 0

    def snap_90(self) -> bool:
        """D-pad Right — snap to snap_target_1_deg (rising-edge)."""
        return self._controller.getPOV() == 90 and self._pov_prev != 90

    def snap_180(self) -> bool:
        """D-pad Down — snap to snap_target_2_deg (rising-edge)."""
        return self._controller.getPOV() == 180 and self._pov_prev != 180

    def snap_270(self) -> bool:
        """D-pad Left — snap to snap_target_3_deg (rising-edge)."""
        return self._controller.getPOV() == 270 and self._pov_prev != 270

    # --- Translation PID tuning ---

    def translation_test(self) -> bool:
        """Left stick button (button 9) — drive straight for translation_distance_m (pressed-edge)."""
        return self._controller.getRawButtonPressed(9)
