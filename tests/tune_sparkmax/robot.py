#!/usr/bin/env python3

"""  20230215 CJH
Test robot designed to:
1) set up a sparkmax
2) put PIDF values on the dashboard
3) read dash values and send to sparkmax
4) set a position or velocity setpoint on the sparkmax
5) update and optionally burn values to sparkmax
6) save values to a dictionary (todo)

"""

import wpilib
import rev
import ntcore
import time

# noinspection PyAttributeOutsideInit
class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.button_reset = True
        self.mechanism = 'turret'  # set this to choose everything else
        set_conversion_factor = True
        self.mechanism_dict = {
            'turret': {'canid': 9, 'inverted': True, 'conversion_factor': 360 / 462.2},
            'elevator': {'canid': 10, 'inverted': True, 'conversion_factor': 0.25 * 0.0254}
        }

        self.counter = 0
        self.keys = ['kP', 'kI', 'kD', 'kIz', 'kFF']
        ntinst = ntcore.NetworkTableInstance.getDefault()
        self.sparkmax_table = ntinst.getTable('Sparkmax')
        self.keys = ['kP', 'kI', 'kD', 'kIz', 'kFF']
        self.PID_dict_pos = {'kP': 0.002, 'kI': 0, 'kD': 0.002, 'kIz': 0, 'kFF': 0.008, 'kArbFF': 0,
                        'kMaxOutput': 0.99, 'kMinOutput': -0.99}
        self.PID_dict_vel = {'kP': 0.05, 'kI': 0.0005, 'kD': 0.00, 'kIz': 0.2, 'kFF': 0.17, 'kArbFF': 0,
                        'kMaxOutput': 0.99, 'kMinOutput': -0.99}
        self.smartmotion_maxvel = 1000  # rpm
        self.smartmotion_maxacc = 30  # rpm/s?

        self.PID_multiplier = 1000  # small numbers do not show up well on the dash
        for key in self.keys:
            self.sparkmax_table.putNumber(key + '_pos', self.PID_multiplier * self.PID_dict_pos[key])
            self.sparkmax_table.putNumber(key + '_vel', self.PID_multiplier * self.PID_dict_vel[key])

        # make some user-editable values for targets
        self.sparkmax_table.putNumber('vel_sp', 1)
        self.sparkmax_table.putNumber('pos_sp', 1)
        self.sparkmax_table.putNumber('pos_arbff', 0)
        self.sparkmax_table.putNumber('vel_arbff', 0)
        self.sparkmax_table.putNumber('pos_max_vel', 20)
        self.sparkmax_table.putNumber('vel_max_accel', self.smartmotion_maxacc)
        self.sparkmax_table.putNumber('pos_max_accel', self.smartmotion_maxacc)

        # create objects for robot hardware
        self.joystick = wpilib.Joystick(0)

        # get a can sparkmax for the ID
        self.canid = self.mechanism_dict[self.mechanism]['canid']
        self.sparkmax = rev.CANSparkMax(self.canid, rev.CANSparkMax.MotorType.kBrushless)
        inverted = self.mechanism_dict[self.mechanism]['inverted']
        self.sparkmax.setInverted(inverted)  # set to true or false as needed

        self.spark_encoder = self.sparkmax.getEncoder()
        self.pid_controller = self.sparkmax.getPIDController()

        # conversion factors: turret is 360 / 462.2, elevator is 0.25 (inches), etc
        if set_conversion_factor:
            self.spark_encoder_conversion_factor = self.mechanism_dict[self.mechanism]['conversion_factor']
            self.spark_encoder.setPositionConversionFactor(self.spark_encoder_conversion_factor)


    def teleopPeriodic(self) -> None:
        self.counter += 1

        b1 = self.joystick.getRawButton(1)  # stop sparkmax
        b2 = self.joystick.getRawButton(2)  # go to position SP
        b3 = self.joystick.getRawButton(3)  # go to vel SP
        b4 = self.joystick.getRawButton(4)  # use stick to move mechanism
        b5 = self.joystick.getRawButton(5)  # update sparkmax
        b6 = self.joystick.getRawButton(6)  # update sparkmax and burn
        b7 = self.joystick.getRawButton(7)  # print dictionaries
        b8 = self.joystick.getRawButton(8)  # re-enable buttons

        stick = - self.joystick.getY()

        if self.button_reset and any([b2,b3,b4,b5,b6,b7]):  # only do this if the buttons are enabled - kludge to only do things once
            self.button_reset = False

            if b1:  # (A) stop motor, ignore the button reset state on this one
                # self.controller.set(0)
                self.pid_controller.setReference(0, rev.CANSparkMax.ControlType.kVoltage)
                print(f'set sparkmax pid controller power out to zero')

            if b2:  # (B) go to position setpoint
                current_value = self.spark_encoder.getPosition()
                setpoint = self.sparkmax_table.getNumber('pos_sp', current_value)
                response = self.pid_controller.setReference(setpoint, rev.CANSparkMax.ControlType.kPosition)
                print(f'set sparkmax postion to {setpoint} with response {response}')

            if b3:  # (X) go to velocity setpoint
                setpoint = self.sparkmax_table.getNumber('vel_sp', 0)
                response = self.pid_controller.setReference(setpoint, rev.CANSparkMax.ControlType.kPosition)
                print(f'set sparkmax velocity to {setpoint} with response {response}')

            if b4:  # (Y) only move the mechanism if holding down Y
                self.sparkmax.set(stick)

            if b5:  # (LB) update the sparkmax but do not burn
                self.update_sparkmax(pid_only=False, burn_flash=False)
            if b6:  # (RB) update the sparkmax and burn
                self.update_sparkmax(pid_only=False, burn_flash=True)
            if b7:  # (BACK) print the dictionary
                print(self.PID_dict_pos, self.PID_dict_vel)

        if b8 or b1:  # (START and A)
            self.button_reset = True

        if self.counter % 10 == 0:
            wpilib.SmartDashboard.putNumber("stick", stick)
            wpilib.SmartDashboard.putBoolean('position', self.spark_encoder.getPosition())
            wpilib.SmartDashboard.putBoolean('velocity', self.spark_encoder.getVelocity())


    def update_keys(self):
        # update the PIDs from the dash
        for key in self.keys:
            self.PID_dict_pos[key] = self.sparkmax_table.getNumber(key + '_pos', 0) / self.PID_multiplier
            self.PID_dict_vel[key] = self.sparkmax_table.getNumber(key + '_vel', 0) / self.PID_multiplier

    def update_sparkmax(self, pid_only=False, burn_flash=False):
        # update the PIDs from the dash
        self.update_keys()

        """Set the PIDs, etc for the controllers, slot 0 is position and slot 1 is velocity"""
        i = 0
        self.error_dict = {}
        self.error_dict.update({'kP0_'+str(i):self.pid_controller.setP(self.PID_dict_pos['kP'], 0)})
        self.error_dict.update({'kP1_'+str(i):self.pid_controller.setP(self.PID_dict_vel['kP'], 1)})
        self.error_dict.update({'kI0_'+str(i):self.pid_controller.setI(self.PID_dict_pos['kI'], 0)})
        self.error_dict.update({'kI1_'+str(i):self.pid_controller.setI(self.PID_dict_vel['kI'], 1)})
        self.error_dict.update({'kD0_'+str(i):self.pid_controller.setD(self.PID_dict_pos['kD'], 0)})
        self.error_dict.update({'kD_1'+str(i):self.pid_controller.setD(self.PID_dict_vel['kD'], 1)})
        self.error_dict.update({'kFF_0'+str(i):self.pid_controller.setFF(self.PID_dict_pos['kFF'], 0)})
        self.error_dict.update({'kFF_1'+str(i):self.pid_controller.setFF(self.PID_dict_vel['kFF'], 1)})
        self.error_dict.update({'kIZ_0'+str(i):self.pid_controller.setIZone(self.PID_dict_pos['kIz'], 0)})
        self.error_dict.update({'kIZ_1'+str(i):self.pid_controller.setIZone(self.PID_dict_vel['kIz'], 1)})
        self.error_dict.update({'MinMax0_'+str(i):self.pid_controller.setOutputRange(self.PID_dict_pos['kMinOutput'], self.PID_dict_pos['kMaxOutput'], 0)})
        self.error_dict.update({'MinMax0_'+str(i):self.pid_controller.setOutputRange(self.PID_dict_vel['kMinOutput'], self.PID_dict_vel['kMaxOutput'], 1)})
        self.error_dict.update({'Accel0_'+str(i):self.pid_controller.setSmartMotionMaxVelocity(self.smartmotion_maxvel, 0)}) #
        self.error_dict.update({'Accel1_'+str(i):self.pid_controller.setSmartMotionMaxVelocity(self.smartmotion_maxvel, 1)}) #
        self.error_dict.update({'Vel0_'+str(i):self.pid_controller.setSmartMotionMaxAccel(self.smartmotion_maxacc, 0)}) #
        self.error_dict.update({'Vel1_'+str(i):self.pid_controller.setSmartMotionMaxAccel(self.smartmotion_maxacc, 1)}) #
        self.pid_controller.setSmartMotionAllowedClosedLoopError(1, slotID=0)

        if not pid_only:
            self.error_dict.update({'VoltComp_'+str(i):self.sparkmax.enableVoltageCompensation(12)})

        if len(set(self.error_dict)) > 1:
            print('\n*Sparkmax setting*     *Response*')
            for key in sorted(self.error_dict.keys()):
                print(f'     {key:15} \t {self.error_dict[key]}', flush=True)
        else:
            print(f'\n *All SparkMax report {list(set(self.error_dict))[0]}')

        if burn_flash:
            start_time = time.time()
            can_error = self.sparkmax.burnFlash()
            print(f'Burn flash on controller {i}: {can_error} {int(1000*(time.time()-start_time)):2d}ms after starting')


if __name__ == "__main__":
    wpilib.run(MyRobot)
