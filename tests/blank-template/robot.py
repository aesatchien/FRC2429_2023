#!/usr/bin/env python3

import wpilib
from wpilib import Spark
from wpimath.filter import MedianFilter
import rev

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        # self.analog_0 = wpilib.AnalogInput(0)
        # self.analog_1 = wpilib.AnalogInput(1)
        # self.analog_2 = wpilib.AnalogInput(2)
        # self.analog_3 = wpilib.AnalogInput(3)
        # self.analog_4 = wpilib.AnalogInput(4)
        self.analogs = [wpilib.AnalogInput(i) for i in range(5)]
        self.digital_ins = [wpilib.DigitalInput(i) for i in range(5)]
        self.digital_outs = [wpilib.DigitalOutput(i) for i in range(5, 10)]
        self.relay = wpilib.Relay(0, wpilib.Relay.Direction.kForwardOnly)
        self.spark = Spark(0)
        self.sparkmax = rev.CANSparkMax(1, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.current_filter = MedianFilter(10)

    def teleopPeriodic(self) -> None:
        pass

    def robotPeriodic(self) -> None:
        wpilib.SmartDashboard.putBoolean('im_alive', True)
        # wpilib.SmartDashboard.putNumber('analog_0', self.analogs[0].getVoltage())
        # wpilib.SmartDashboard.putNumber('analog_1', self.analogs[1].getVoltage())
        pedal_output = max(0, self.analogs[0].getVoltage() - 0.9) / 3.1
        pedal_output = self.clamp(pedal_output)
        wpilib.SmartDashboard.putNumber('pedal_output', pedal_output)
        [wpilib.SmartDashboard.putNumber(f'analog_{i}', self.analogs[i].getVoltage()) for i in range(5)]
        [wpilib.SmartDashboard.putBoolean(f'digital_ins{i}', self.digital_ins[i].get()) for i in range(5)]
        self.digital_outs[4].set(self.digital_ins[0].get())
        [wpilib.SmartDashboard.putBoolean(f'digital_outs{i}', self.digital_outs[i].get()) for i in range(5)]
        self.relay.set(wpilib.Relay.Value.kOn)
        self.spark.set(pedal_output)
        self.sparkmax.set(pedal_output)
        wpilib.SmartDashboard.putNumber(f'spark_voltage', self.sparkmax.getAppliedOutput())
        current = self.sparkmax.getOutputCurrent()
        wpilib.SmartDashboard.putNumber(f'spark_current', current)
        wpilib.SmartDashboard.putNumber(f'spark_filtered_current', self.current_filter.calculate(current))

    def clamp(self, x):
        return min(1, max(-1, x))

if __name__ == "__main__":
    wpilib.run(MyRobot)

