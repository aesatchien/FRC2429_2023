#!/usr/bin/env python3

# This is REV's SmartMotion tuning tool.
# NOTE SmartMotion only uses ONE PIDF slot - the one for velocity.  You aren't actually changing position control PIDs.
# Also not the trickery about conversion factors in the SparkMax and how they affect decel and PIDFs
# (they all scale with the factor, and if you change position, also change velocity)
# Good starting values for NEO motor with no load  below.
# probably need to bump kFF first once we have a load.  With no load I'm not seeing much kp influence.
# Conversion factor = 1 :  kp 5e-5, ki 1e-6, kd 0, kiz 1e-5, kFF 1.6e-4
# Conversion factor = 0.1 :  kp 5e-5, ki 1e-6, kd 0, kiz 1e-5, kFF 1.9e-3

import rev
import wpilib


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        pwm_port = 0
        controller_port = 0

        self.spark = wpilib.Spark(pwm_port)
        self.controller = wpilib.XboxController(controller_port)

    def teleopPeriodic(self):
        power = 0.2
        if self.controller.getRawButton(1):
            self.spark.set(power)
        elif self.controller.getRawButton(2):
            self.spark.set(-power)
        else:
            self.spark.stopMotor()

if __name__ == "__main__":
    wpilib.run(Robot)