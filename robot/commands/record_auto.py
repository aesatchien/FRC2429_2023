import commands2
import json
from wpilib import SmartDashboard


class RecordAuto(commands2.CommandBase):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Start Recording')
        self.container = container

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        print('** INITIALIZING RECORDAUTO **')
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.counter = 0
        self.input_log = []
        self.input_data = {
            'driver_controller': {
                'axis': {},
                'button': {}
            },
            'co_driver_controller': {
                'axis': {},
                'button': {}
            }
        }

    def execute(self) -> None:
        print('** EXECUTING RECORDAUTO **')
        # Get driver inputs
        for axis in range(0, 6):
            self.input_data['driver_controller']['axis'][f'axis{axis}'] = self.container.driver_controller.getRawAxis(axis)

        self.input_data['driver_controller']['button']['A'] = self.container.driver_controller.getRawButton(1)
        self.input_data['driver_controller']['button']['B'] = self.container.driver_controller.getRawButton(2)
        self.input_data['driver_controller']['button']['X'] = self.container.driver_controller.getRawButton(3)
        self.input_data['driver_controller']['button']['Y'] = self.container.driver_controller.getRawButton(4)
        self.input_data['driver_controller']['button']['LB'] = self.container.driver_controller.getRawButton(5)
        self.input_data['driver_controller']['button']['RB'] = self.container.driver_controller.getRawButton(6)
        self.input_data['driver_controller']['button']['Back'] = self.container.driver_controller.getRawButton(7)
        self.input_data['driver_controller']['button']['Start'] = self.container.driver_controller.getRawButton(8)
        self.input_data['driver_controller']['button']['LS'] = self.container.driver_controller.getRawButton(9)
        self.input_data['driver_controller']['button']['RS'] = self.container.driver_controller.getRawButton(10)

        # Get operator inputs
        for axis in range(0, 6):
            self.input_data['driver_controller']['axis'][f'axis{axis}'] = self.container.co_driver_controller.getRawAxis(axis)

        self.input_data['co_driver_controller']['button']['A'] = self.container.co_driver_controller.getRawButton(1)
        self.input_data['co_driver_controller']['button']['B'] = self.container.co_driver_controller.getRawButton(2)
        self.input_data['co_driver_controller']['button']['X'] = self.container.co_driver_controller.getRawButton(3)
        self.input_data['co_driver_controller']['button']['Y'] = self.container.co_driver_controller.getRawButton(4)
        self.input_data['co_driver_controller']['button']['LB'] = self.container.co_driver_controller.getRawButton(5)
        self.input_data['co_driver_controller']['button']['RB'] = self.container.co_driver_controller.getRawButton(6)
        self.input_data['co_driver_controller']['button']['Back'] = self.container.co_driver_controller.getRawButton(7)
        self.input_data['co_driver_controller']['button']['Start'] = self.container.co_driver_controller.getRawButton(8)
        self.input_data['co_driver_controller']['button']['LS'] = self.container.co_driver_controller.getRawButton(9)
        self.input_data['co_driver_controller']['button']['RS'] = self.container.co_driver_controller.getRawButton(10)

        # Add captured inputs to the list
        self.input_log.append(self.input_data)
        self.counter += 1


    def isFinished(self) -> bool:
        return self.counter >= 750
    
    def runsWhenDisabled(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        with open('input_log.json', 'w') as input_json:
            json.dump(self.input_log, input_json)

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")