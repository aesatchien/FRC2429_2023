#!/usr/bin/env python3

# This is REV's SmartMotion tuning tool.
# NOTE SmartMotion only uses ONE PIDF slot - the one for velocity.  You aren't actually changing position control PIDs.
# Also not the trickery about conversion factors in the SparkMax and how they affect decel and PIDFs
# (they all scale with the factor, and if you change position, also change velocity)
# Good starting values for NEO motor with no load  below.
# probably need to bump kFF first once we have a load.  With no load I'm not seeing much kp influence.
# Conversion factor = 1 :  kp 5e-5, ki 1e-6, kd 0, kiz 1e-5, kFF 1.6e-4
# Conversion factor = 0.1 :  kp 5e-5, ki 1e-6, kd 0, kiz 1e-5, kFF 1.9e-3

import wpilib
import time

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        controller_port = 0

        self.controller = wpilib.XboxController(controller_port)
        self.counter = 1

        self.t0 = -1
        self.prev_was_none = False
        self.pov_dir = 90

    def teleopPeriodic(self):
        pov = self.controller.getPOV(0)

        if pov == self.pov_dir and self.t0 < 0:
            self.t0 = time.time()

        if pov == self.pov_dir and self.prev_was_none:
            t1 = time.time()
            print(f'DELTA T: {t1 - self.t0}', flush=True)
            self.t0 = t1

        # print(f'co status ({self.counter%50:02}/50): {pov}', flush=True)

        self.counter += 1
        self.prev_was_none = pov != self.pov_dir

if __name__ == "__main__":
    wpilib.run(Robot)