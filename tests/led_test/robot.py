import wpilib
from wpilib import AddressableLED


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.pwm_port = 7
        self.led_count = 36
        self.led_strip = AddressableLED(self.pwm_port)
        self.led_data = [AddressableLED.LEDData() for _ in range(self.led_count)]

        [led.setRGB(255, 0, 0) for led in self.led_data]

        self.led_strip.setLength(self.led_count)
        self.led_strip.setData(self.led_data)
        self.led_strip.start()

        self.counter = 0

    def teleopPeriodic(self):
        self.counter += 1

        if self.counter % 25 == 0:
            [led.setRGB(255, 0, 0) for led in self.led_data]
            self.led_strip.setData(self.led_data)

if __name__ == "__main__":
    wpilib.run(Robot)