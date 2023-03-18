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
        self.turning_motors = {
            'lf': {'id': 20},
            'rf': {'id': 22},
            'lb': {'id': 24},
            'rb': {'id': 26},
        }

        for (key, motor_info) in self.turning_motors.items():
            motor = rev.CANSparkMax(motor_info['id'], rev.CANSparkMax.MotorType.kBrushless)
            temp_dict = {
                'id': motor_info['id'],
                'motor': motor,
                'encoder': motor.getEncoder(),
                'analog_encoder': motor.getAnalog()
            }
            self.turning_motors[key].update(temp_dict)

        print(self.turning_motors)
    def teleopPeriodic(self):
        for (key, motor_info) in self.turning_motors.items():
            wpilib.SmartDashboard.putNumber(f"AbsEncoder_{motor_info['id']}", motor_info['analog_encoder'].getPosition())
            wpilib.SmartDashboard.putNumber(f"RegEncoder_{motor_info['id']}", motor_info['encoder'].getPosition())

if __name__ == "__main__":
    wpilib.run(Robot)