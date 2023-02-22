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
        can_id = 9  # change as needed

        # if you are going to change position factor, you should also change velocity.
        # smart motion does weird things on the decel if you don't.
        # also note that if you do change these, the PIDs will have to change as well
        # the default values below are good for conversion factors of 1.
        position_conversion_factor = 1  # test to see if this changes PIDFs on velocity

        self.motor = rev.CANSparkMax(can_id, rev.CANSparkMax.MotorType.kBrushless)
        self.motor.restoreFactoryDefaults()
        self.motor.setInverted(True)

        self.pid_controller = self.motor.getPIDController()
        self.encoder = self.motor.getEncoder()
        self.encoder.setPositionConversionFactor(position_conversion_factor)
        self.encoder.setVelocityConversionFactor(position_conversion_factor)

        self.counter = 0

        self.kP = 5e-5
        self.kI = 1e-6
        self.kD = 0
        self.kIz = 0.00001
        self.kFF = 0.000156
        self.kMaxOutput = 0.35
        self.kMinOutput = -0.35
        self.max_rpm = 5700

        # Smart Motion coefficients
        self.max_vel = 2000  # rpm
        self.max_acc = 1500
        self.min_vel = 0
        self.allowed_err = 0.2

        self.pid_controller.setP(self.kP)
        self.pid_controller.setI(self.kI)
        self.pid_controller.setD(self.kD)
        self.pid_controller.setIZone(self.kIz)
        self.pid_controller.setFF(self.kFF)
        self.pid_controller.setOutputRange(self.kMinOutput, self.kMaxOutput)

        smart_motion_slot = 0
        self.pid_controller.setSmartMotionMaxVelocity(self.max_vel, smart_motion_slot)
        self.pid_controller.setSmartMotionMinOutputVelocity(self.min_vel, smart_motion_slot)
        self.pid_controller.setSmartMotionMaxAccel(self.max_acc, smart_motion_slot)
        self.pid_controller.setSmartMotionAllowedClosedLoopError(self.allowed_err, smart_motion_slot)

        wpilib.SmartDashboard.putNumber("P Gain", self.kP)
        wpilib.SmartDashboard.putNumber("I Gain", self.kI)
        wpilib.SmartDashboard.putNumber("D Gain", self.kD)
        wpilib.SmartDashboard.putNumber("I Zone", self.kIz)
        wpilib.SmartDashboard.putNumber("Feed Forward", self.kFF)
        wpilib.SmartDashboard.putNumber("Max Output", self.kMaxOutput)
        wpilib.SmartDashboard.putNumber("Min Output", self.kMinOutput)

        wpilib.SmartDashboard.putNumber("Max Velocity", self.max_vel)
        wpilib.SmartDashboard.putNumber("Min Velocity", self.min_vel)
        wpilib.SmartDashboard.putNumber("Max Acceleration", self.max_acc)
        wpilib.SmartDashboard.putNumber("Allowed Closed Loop Error", self.allowed_err)
        wpilib.SmartDashboard.putNumber("Set Position", 0)
        wpilib.SmartDashboard.putNumber("Set Velocity", 0)

        wpilib.SmartDashboard.putBoolean("Mode", True)

        wpilib.SmartDashboard.putNumber("conversion_factor", self.encoder.getPositionConversionFactor())

    def teleopPeriodic(self):

        mode = wpilib.SmartDashboard.getBoolean("Mode", False)
        if mode:
            setpoint = wpilib.SmartDashboard.getNumber("Set Velocity", 0)
            self.pid_controller.setReference(setpoint, rev.CANSparkMax.ControlType.kVelocity)
            pv = self.encoder.getVelocity()
        else:
            setpoint = wpilib.SmartDashboard.getNumber("Set Position", 0)
            self.pid_controller.setReference(setpoint, rev.CANSparkMax.ControlType.kSmartMotion)
            pv = self.encoder.getPosition()

        self.counter += 1
        if self.counter % 2 == 0:

            p = wpilib.SmartDashboard.getNumber("P Gain", 0)
            i = wpilib.SmartDashboard.getNumber("I Gain", 0)
            d = wpilib.SmartDashboard.getNumber("D Gain", 0)
            iz = wpilib.SmartDashboard.getNumber("I Zone", 0)
            ff = wpilib.SmartDashboard.getNumber("Feed Forward", 0)
            max_out = wpilib.SmartDashboard.getNumber("Max Output", 0)
            min_out = wpilib.SmartDashboard.getNumber("Min Output", 0)
            maxV = wpilib.SmartDashboard.getNumber("Max Velocity", 0)
            minV = wpilib.SmartDashboard.getNumber("Min Velocity", 0)
            maxA = wpilib.SmartDashboard.getNumber("Max Acceleration", 0)
            allE = wpilib.SmartDashboard.getNumber("Allowed Closed Loop Error", 0)

            if p != self.kP:
                self.pid_controller.setP(p)
                self.kP = p
            if i != self.kI:
                self.pid_controller.setI(i)
                self.kI = i
            if d != self.kD:
                self.pid_controller.setD(d)
                self.kD = d
            if iz != self.kIz:
                self.pid_controller.setIZone(iz)
                self.kIz = iz
            if ff != self.kFF:
                self.pid_controller.setFF(ff)
                self.kFF = ff
            if max_out != self.kMaxOutput or min_out != self.kMinOutput:
                self.pid_controller.setOutputRange(min_out, max_out)
                self.kMinOutput = min_out
                self.kMaxOutput = max_out
            if maxV != self.max_vel:
                self.pid_controller.setSmartMotionMaxVelocity(maxV, 0)
                self.max_vel = maxV
            if minV != self.min_vel:
                self.pid_controller.setSmartMotionMinOutputVelocity(minV, 0)
                self.min_vel = minV
            if maxA != self.max_acc:
                self.pid_controller.setSmartMotionMaxAccel(maxA, 0)
                self.max_acc = maxA
            if allE != self.allowed_err:
                self.pid_controller.setSmartMotionAllowedClosedLoopError(allE, 0)
                self.allowed_err = allE

            wpilib.SmartDashboard.putNumber("SetPoint", setpoint)
            wpilib.SmartDashboard.putNumber("POSITION", self.encoder.getPosition())
            wpilib.SmartDashboard.putNumber("VELOCITY", self.encoder.getVelocity())

        # do these guys every time
        else:
            wpilib.SmartDashboard.putNumber("Process Variable", pv)
            wpilib.SmartDashboard.putNumber("Output", self.motor.getAppliedOutput())


if __name__ == "__main__":
    wpilib.run(Robot)