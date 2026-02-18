import wpilib
import magicbot

from phoenix6.hardware import CANdle
from phoenix6.controls import EmptyAnimation,SolidColor,RainbowAnimation
from phoenix6.configs import CANdleConfiguration
from phoenix6.signals import StripTypeValue,RGBWColor

from components.intake import IntakeComponent
from controllers.tanker import Tanker




class LEDComponent:
    intake: IntakeComponent
    tanker: Tanker

    RED = RGBWColor(255, 0, 0)
    GREEN = RGBWColor(0, 255, 0)
    BLUE = RGBWColor(0, 0, 255)
    YELLOW = RGBWColor(255, 255, 0)
    OFF = RGBWColor(0,0,0,0)

    def __init__(self):
        self.candle = CANdle(5, "Drive")

        cfg = CANdleConfiguration()
        cfg.led.strip_type = StripTypeValue.GRB
        cfg.led.brightness_scalar = 0.8

        self.candle.configurator.apply(cfg)

        for i in range(0, 8):
            self.candle.set_control(EmptyAnimation(i))
        
        self.candle.set_control(RainbowAnimation(0,307,0,1.0))

    def setColor(self, color: RGBWColor) -> None:
        self.candle.set_control(SolidColor(0,307,color))

    def execute(self) -> None:
        

        if(wpilib.DriverStation.isDSAttached == False and wpilib.DriverStation.isDisabled == True):
             self.setColor(self.RED)
        elif(wpilib.DriverStation.isDSAttached == True and wpilib.DriverStation.isDisabled == True):
            self.setColor(self.BLUE)
        
        else:
            self.setColor(self.OFF)


