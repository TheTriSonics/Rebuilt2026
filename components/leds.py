import magicbot
import wpilib
from wpilib import Timer

from phoenix6.hardware import CANdle
from phoenix6.controls import EmptyAnimation, SolidColor
from phoenix6.configs import CANdleConfiguration
from phoenix6.signals import StripTypeValue, RGBWColor

from components.vision import VisionComponent
from components.drivetrain import DrivetrainComponent
from utilities.game import _phase_time_remaining
import ids

NUM_LEDS = 125

WHITE = RGBWColor(255, 255, 255)
RED = RGBWColor(255, 0, 0)
GREEN = RGBWColor(0, 255, 0)
BLACK = RGBWColor(0, 0, 0)


class LEDComponent:
    vision: VisionComponent
    drivetrain: DrivetrainComponent

    # Reset every cycle — robot.py must call set_targeting() each loop
    _is_targeting = magicbot.will_reset_to(False)

    def __init__(self):
        self.candle = CANdle(ids.CANdleId.CANDLE.id, ids.CANdleId.CANDLE.bus)

        cfg = CANdleConfiguration()
        cfg.led.strip_type = StripTypeValue.GRB
        cfg.led.brightness_scalar = 0.8
        self.candle.configurator.apply(cfg)

        for i in range(8):
            self.candle.set_control(EmptyAnimation(i))

        self._game_msg = ""

    def set_targeting(self, targeting: bool) -> None:
        """Call each loop from robot.py when the driver is auto-targeting."""
        self._is_targeting = targeting

    def set_game_msg(self, msg: str) -> None:
        """Set the FMS game-specific message for hub phase tracking."""
        self._game_msg = msg

    @staticmethod
    def _countdown_frequency(phase_remaining: float) -> float:
        """Pulse frequency for the hub phase-transition countdown.

        Returns 0 for solid (no pulse) or Hz for the blink rate.
        """
        if phase_remaining > 5.0 or phase_remaining <= 0.0:
            return 0.0
        if phase_remaining > 4.0:
            return 0.0  # 5-4 s: solid
        if phase_remaining > 3.0:
            return 2.0  # 4-3 s
        if phase_remaining > 2.0:
            return 3.0  # 3-2 s
        if phase_remaining > 1.0:
            return 4.0  # 2-1 s
        return 5.0  # 1-0 s

    def execute(self) -> None:
        # ── Base colour from targeting state ──────────────────────────
        if self._is_targeting:
            locked = self.vision.has_stable_pose() and self.drivetrain.is_heading_aligned()
            color = GREEN if locked else RED
        else:
            color = WHITE

        # ── Hub phase-change countdown overlay ────────────────────────
        match_time = wpilib.DriverStation.getMatchTime()
        if match_time < 0:
            match_time = 0.0
        phase_remaining = _phase_time_remaining(match_time)
        freq = self._countdown_frequency(phase_remaining)

        if freq > 0.0:
            period = 1.0 / freq
            t = Timer.getFPGATimestamp() % period
            if t >= period / 2:
                color = BLACK

        # ── Apply to full strip ───────────────────────────────────────
        self.candle.set_control(SolidColor(0, NUM_LEDS, color))
