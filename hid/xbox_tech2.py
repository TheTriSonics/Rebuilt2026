"""
TechController2 — Xbox controller for generic motor commissioning (DS port 3).

This HID wrapper is used with the MotorCommissioning controller for
characterizing and setting up individual mechanism motors.

BUTTON REFERENCE
================
Back  (btn 7)  — Toggle CTRE SignalLogger (start/stop .hoot recording)
Start (btn 8)  — Emergency stop (abort test, return to idle)

MOTOR CHARACTERIZATION (SysId)
  A             — Quasistatic forward   (slow voltage ramp, forward)
  B             — Quasistatic reverse   (slow voltage ramp, reverse)
  X             — Dynamic forward       (step voltage, forward)
  Y             — Dynamic reverse       (step voltage, reverse)
"""

import wpilib


class TechController2:
    """Thin wrapper around XboxController(3) for generic motor commissioning.

    All methods return booleans (pressed-edge). Call update() once per
    teleopPeriodic loop before reading any methods.
    """

    def __init__(self) -> None:
        self._controller = wpilib.XboxController(3)

    def update(self) -> None:
        """Must be called once per loop before reading any pressed events."""
        pass  # No trigger/POV edge tracking needed for this controller

    # --- SignalLogger / safety ---

    def toggle_signal_logger(self) -> bool:
        """Back button — toggle CTRE SignalLogger on/off (pressed-edge)."""
        return self._controller.getBackButtonPressed()

    def emergency_stop(self) -> bool:
        """Start button — abort current test and return to idle (pressed-edge)."""
        return self._controller.getStartButtonPressed()

    # --- Motor characterization ---

    def quasistatic_fwd(self) -> bool:
        """A button — begin quasistatic forward sweep (pressed-edge)."""
        return self._controller.getAButtonPressed()

    def quasistatic_rev(self) -> bool:
        """B button — begin quasistatic reverse sweep (pressed-edge)."""
        return self._controller.getBButtonPressed()

    def dynamic_fwd(self) -> bool:
        """X button — begin dynamic forward step (pressed-edge)."""
        return self._controller.getXButtonPressed()

    def dynamic_rev(self) -> bool:
        """Y button — begin dynamic reverse step (pressed-edge)."""
        return self._controller.getYButtonPressed()
