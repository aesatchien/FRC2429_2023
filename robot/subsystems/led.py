import enum
import math
import commands2
import wpilib
from wpilib import AddressableLED
from wpilib import Color
import constants


class Led(commands2.SubsystemBase):
    class Mode(enum.Enum):
        CONE = 'CONE' # yellow
        CUBE = 'CUBE' # purple
        PICKUP_COMPLETE = 'PICKUP_COMPLETE' # flashing green
        VISION_TARGET_FAILURE = 'VISION_TARGET_FAILURE' # red
        VISION_TARGET_SUCCESS = 'VISION_TARGET_SUCCESS' # flashing blue
        AUTO_STRAFE_COMPLETE = 'AUTO_STRAFE_COMPLETE' # solid blue
        RAINBOW = 'RAINBOW' # Haochen really wanted it

    def __init__(self):
        super().__init__()
        self.setName('Led')
        self.counter = 0
        self.animation_counter = 0

        self.led_count = constants.k_led_count
        self.led_strip = AddressableLED(constants.k_led_pwm_port)
        self.led_data = [AddressableLED.LEDData() for _ in range(self.led_count)]

        [led.setRGB(0, 0, 0) for led in self.led_data]

        self.led_strip.setLength(self.led_count)
        self.led_strip.setData(self.led_data)
        self.led_strip.start()
        self.mode = Led.Mode.CONE
        self.prev_mode = self.mode

    def set_mode(self, mode: Mode) -> None:
        self.prev_mode = self.mode
        self.mode = mode

    def get_mode(self) -> Mode:
        return self.mode

    def get_prev_mode(self) -> Mode:
        return self.prev_mode

    def periodic(self) -> None:
        # update LEDs
        if self.counter % 5 == 0:
            wpilib.SmartDashboard.putString('led_mode', self.mode.value)

            self.animation_counter += 1

            for i in range(constants.k_led_count):
                led = self.led_data[i]

                if self.mode == Led.Mode.CONE:
                    # solid yellow
                    led.setRGB(255, 220, 0)

                elif self.mode == Led.Mode.CUBE:
                    # solid purple
                    led.setRGB(255, 0, 255)

                elif self.mode == Led.Mode.PICKUP_COMPLETE:
                    # flashing green
                    freq = 5 # 10 /s > 2x /s
                    cycle = math.floor(self.animation_counter / freq)

                    if cycle % 2 == 0:
                        led.setRGB(0, 0, 0)
                    else:
                        led.setRGB(0, 255, 0)

                elif self.mode == Led.Mode.VISION_TARGET_FAILURE:
                    # solid red
                    led.setRGB(255, 0, 0)

                elif self.mode == Led.Mode.VISION_TARGET_SUCCESS:
                    # flashing blue
                    freq = 5  # 10 /s > 2x /s
                    cycle = math.floor(self.animation_counter / freq)

                    if cycle % 2 == 0:
                        led.setRGB(0, 0, 0)
                    else:
                        led.setRGB(255, 0, 0)

                elif self.mode == Led.Mode.AUTO_STRAFE_COMPLETE:
                    # solid blue
                    led.setRGB(0, 0, 255)

                elif self.mode == Led.Mode.RAINBOW:
                    # rainbow
                    hue = (i + self.animation_counter) % constants.k_led_count
                    hue /= constants.k_led_count
                    hue *= 255

                    led.setHSV(math.floor(hue), 255, 255)

            self.led_strip.setData(self.led_data)

        self.counter += 1








