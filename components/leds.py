import wpilib

from phoenix6.hardware import CANdle
from phoenix6.controls import EmptyAnimation, SolidColor
from phoenix6.configs import CANdleConfiguration
from phoenix6.signals import StripTypeValue, RGBWColor

from components.intake import IntakeComponent
from components.kicker import KickerComponent
from components.shooter import ShooterComponent
import ids


# Number of physical LEDs per logical "bulb"
LEDS_PER_BULB = 50

# Visual size of each LED bulb in the Mechanism2d (meters)
LED_VIZ_SIZE = 0.06

# Spacing between bulbs in the visualization (meters)
LED_VIZ_SPACING = 0.02
LED_HEIGHT = 0.6

# Bulb names in physical wiring order — index is derived automatically
BULB_NAMES = ["Drivetrain", "Climber", "Turret", "Intake", "Singulator", "Kicker", "Shooter"]
BULB = {name: i for i, name in enumerate(BULB_NAMES)}
NUM_BULBS = len(BULB_NAMES)


def _rgbw_to_color8bit(color: RGBWColor) -> wpilib.Color8Bit:
    return wpilib.Color8Bit(color.red, color.green, color.blue)


class LEDComponent:
    # intake: IntakeComponent
    kicker: KickerComponent
    shooter: ShooterComponent

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


    def __init__(self):
        self.candle = CANdle(ids.CANdleId.CANDLE.id, ids.CANdleId.CANDLE.bus)

        cfg = CANdleConfiguration()
        cfg.led.strip_type = StripTypeValue.GRB
        cfg.led.brightness_scalar = 0.8

        self.candle.configurator.apply(cfg)

        for i in range(0, 8):
            self.candle.set_control(EmptyAnimation(i))

        self.bulb_colors: list[RGBWColor] = [self.OFF] * NUM_BULBS

        # Build Mechanism2d visualization
        total_width = NUM_BULBS * LED_VIZ_SIZE + (NUM_BULBS - 1) * LED_VIZ_SPACING
        self.mech = wpilib.Mechanism2d(total_width, LED_HEIGHT + LED_VIZ_SIZE)

        self.bulb_ligaments: list[wpilib.MechanismLigament2d] = []
        for i in range(NUM_BULBS):
            x = i * (LED_VIZ_SIZE + LED_VIZ_SPACING)
            root = self.mech.getRoot(BULB_NAMES[i], x, LED_HEIGHT)
            ligament = root.appendLigament(
                BULB_NAMES[i],
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
        if self.intake.target_speed < 0:
            self.bulb_colors[BULB["Intake"]] = self.GREEN
        elif self.intake.target_speed > 0:
            self.bulb_colors[BULB["Intake"]] = self.RED
        else:
            self.bulb_colors[BULB["Intake"]] = self.YELLOW
        if self.kicker.is_active():
            self.bulb_colors[BULB["Kicker"]] = self.GREEN
        else:
            self.bulb_colors[BULB["Kicker"]] = self.GREY

        self.bulb_colors[BULB["Singulator"]] = self.CYAN

        if self.shooter.is_at_speed():
            self.bulb_colors[BULB["Shooter"]] = self.GREEN
        elif self.shooter.is_active():
            self.bulb_colors[BULB["Shooter"]] = self.WHITE
        else:
            self.bulb_colors[BULB["Shooter"]] = self.BLACK
        self.bulb_colors[BULB["Turret"]] = self.PURPLE
        self.bulb_colors[BULB["Climber"]] = self.RED
        self.bulb_colors[BULB["Drivetrain"]] = self.BLUE

        # Update physical LEDs and visualization
        for i, color in enumerate(self.bulb_colors):
            start = i * LEDS_PER_BULB
            self.candle.set_control(SolidColor(start, LEDS_PER_BULB, color))
            self.bulb_ligaments[i].setColor(_rgbw_to_color8bit(color))
