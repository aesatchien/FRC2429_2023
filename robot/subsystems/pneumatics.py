"""
pneumatic subsystem
Shares the following with networktables:  manipulator_open, close_loop_enable
manipulator needs to reliably open and close on a double solenoid
"""
import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, Solenoid, Compressor, AnalogInput, DoubleSolenoid

import constants


class Pneumatics(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Pneumatics')
        self.counter = 0
        self.compressor = Compressor(0, wpilib.PneumaticsModuleType.CTREPCM)
        self.manipulator_piston = DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, constants.k_manipulator_open_port, constants.k_manipulator_closed_port)
        self.pressure_sensor = AnalogInput(0)  # may skip this unless we really want one
        self.close_loop_enable = True

        competition = True  # need to put this in robot container, not here
        if competition:
            self.start_compressor()
        else:
            self.stop_compressor()

        # Decide on init piston position - always closed because we will hold a game piece
        self.manipulator_open = False

        SmartDashboard.putBoolean('manipulator_open', self.manipulator_open)
        SmartDashboard.putBoolean('compressor_close_loop', self.close_loop_enable)

    def set_manipulator_piston(self, position='open'):
        if position == 'open':
            self.manipulator_open = True
            self.manipulator_piston.set(DoubleSolenoid.Value.kForward)
        elif position == 'close':
            self.manipulator_open = False
            self.manipulator_piston.set(DoubleSolenoid.Value.kReverse)
        SmartDashboard.putBoolean('manipulator_open', self.manipulator_open)

    def toggle_manipulator(self):
        self.manipulator_piston.toggle()
        self.manipulator_open = not self.manipulator_open
        SmartDashboard.putBoolean('manipulator_open', self.manipulator_open)

    def get_manipulator_state(self):
        return self.manipulator_piston.get()

    def stop_compressor(self):
        self.compressor.disable()   # should now be disable, as of 2022
        self.close_loop_enable = False
        SmartDashboard.putBoolean('compressor_close_loop', self.close_loop_enable)

    def start_compressor(self):
        self.compressor.enableDigital()  # setClosedLoopControl(True) is no longer a call in 2022
        self.close_loop_enable = True
        SmartDashboard.putBoolean('compressor_close_loop', self.close_loop_enable)

    def toggle_compressor(self):
        if self.close_loop_enable:
            self.stop_compressor()
        else:
            self.start_compressor()

    def periodic(self) -> None:
        
        self.counter += 1

        if self.counter % 25 == 1:
            # the compressor turns itself off and on, so we have to ask it its state
            SmartDashboard.putBoolean('compressor_state', self.compressor.enabled())
            # SmartDashboard.putNumber('pressure', self.get_analog_pressure())
            # todo: intergrate pressure sensor into compressor class




        





  
