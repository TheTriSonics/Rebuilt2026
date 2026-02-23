import wpilib

from phoenix6.hardware import CANdle
from phoenix6.controls import EmptyAnimation, SolidColor
from phoenix6.configs import CANdleConfiguration
from phoenix6.signals import StripTypeValue, RGBWColor

from components.intake import IntakeComponent
import ids


# Number of physical LEDs per logical "bulb"
LEDS_PER_BULB = 50

# Visual size of each LED bulb in the Mechanism2d (meters)
LED_VIZ_SIZE = 0.06

# Spacing between bulbs in the visualization (meters)
LED_VIZ_SPACING = 0.02
LED_HEIGHT = 0.6

# Labels for each bulb
BULB_LABELS = ["Intake", "Drivetrain"]


def _rgbw_to_color8bit(color: RGBWColor) -> wpilib.Color8Bit:
    return wpilib.Color8Bit(color.red, color.green, color.blue)


class LEDComponent:
    intake: IntakeComponent

    RED = RGBWColor(255, 0, 0)
    GREEN = RGBWColor(0, 255, 0)
    BLUE = RGBWColor(0, 0, 255)
    YELLOW = RGBWColor(255, 255, 0)
    CYAN = RGBWColor(0, 255, 255)
    PURPLE = RGBWColor(128, 0, 255)
    WHITE = RGBWColor(255, 255, 255)
    GREY = RGBWColor(128, 128, 128)
    BLACK = RGBWColor(0, 0, 0)
    OFF = RGBWColor(0, 0, 0, 0)

    # Bulb assignments
    BULB_INTAKE = 0
    BULB_DRIVETRAIN = 1
    NUM_BULBS = 2

    def __init__(self):
        self.candle = CANdle(ids.CANdleId.CANDLE.id, ids.CANdleId.CANDLE.bus)

        cfg = CANdleConfiguration()
        cfg.led.strip_type = StripTypeValue.GRB
        cfg.led.brightness_scalar = 0.8

        self.candle.configurator.apply(cfg)

        for i in range(0, 8):
            self.candle.set_control(EmptyAnimation(i))

        self.bulb_colors: list[RGBWColor] = [self.OFF] * self.NUM_BULBS

        # Build Mechanism2d visualization
        total_width = self.NUM_BULBS * LED_VIZ_SIZE + (self.NUM_BULBS - 1) * LED_VIZ_SPACING
        self.mech = wpilib.Mechanism2d(total_width, LED_HEIGHT + LED_VIZ_SIZE)

        self.bulb_ligaments: list[wpilib.MechanismLigament2d] = []
        for i in range(self.NUM_BULBS):
            x = i * (LED_VIZ_SIZE + LED_VIZ_SPACING)
            root = self.mech.getRoot(BULB_LABELS[i], x, LED_HEIGHT)
            ligament = root.appendLigament(
                BULB_LABELS[i],
                LED_VIZ_SIZE,
                90,
                LED_VIZ_SIZE * 100,  # line width in pixels — scale up for visibility
                wpilib.Color8Bit(0, 0, 0),
            )
            self.bulb_ligaments.append(ligament)

        wpilib.SmartDashboard.putData("LEDs", self.mech)

    def set_bulb(self, bulb: int, color: RGBWColor) -> None:
        self.bulb_colors[bulb] = color

    def execute(self) -> None:
        if not wpilib.DriverStation.isDSAttached() and wpilib.DriverStation.isDisabled():
            for i in range(self.NUM_BULBS):
                self.bulb_colors[i] = self.RED
        elif wpilib.DriverStation.isDSAttached() and wpilib.DriverStation.isDisabled():
            for i in range(self.NUM_BULBS):
                self.bulb_colors[i] = self.BLUE
        else:
            if self.intake.target_speed < 0:
                self.bulb_colors[self.BULB_INTAKE] = self.GREEN
            elif self.intake.target_speed > 0:
                self.bulb_colors[self.BULB_INTAKE] = self.RED
            else:
                self.bulb_colors[self.BULB_INTAKE] = self.YELLOW
            self.bulb_colors[self.BULB_DRIVETRAIN] = self.BLUE

        # Update physical LEDs and visualization
        for i, color in enumerate(self.bulb_colors):
            start = i * LEDS_PER_BULB
            self.candle.set_control(SolidColor(start, LEDS_PER_BULB, color))
            self.bulb_ligaments[i].setColor(_rgbw_to_color8bit(color))
