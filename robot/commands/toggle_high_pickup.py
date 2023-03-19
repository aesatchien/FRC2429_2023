import commands2

import constants

from arm_move import ArmMove
from elevator_move import ElevatorMove
from manipulator_auto_grab import ManipulatorAutoGrab
from wrist_move import WristMove
from turret_move import TurretMove

from subsystems.pneumatics import Pneumatics
from subsystems.wrist import Wrist
from subsystems.elevator import Elevator
from subsystems.arm import Arm
from playingwithfusion import TimeOfFlight
from subsystems.vision import Vision

class ToggleHighPickup(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('ToggleHighPickup')  # change this to something appropriate for this command
        self.container = container

        #move elevator
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator, setpoint=Elevator.positions['upper_pickup']).withTimeout(1.5))

        #open wrist
        wrist_setpoint=Wrist.positions['flat']  #ToDo: logical statement that checks if we're picking up a cone or a cube
        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=wrist_setpoint).withTimeout(1.0)) #not sure what timeout is best

        #detect gamepiece
        angle=0 #the detected angle of the game piece

        #rotate turret
        self.addCommands(TurretMove(container=self.container, turret=self.container.turret, setpoint=angle).withTimeout(1.0)) #not sure what timeout is best

        #extend arm
        distance = self.container.pneumatics.pneumatics.target_distance_sensor.getRange() 
        self.addCommands(ArmMove(container=self.container, arm=self.container.turret, setpoint=distance).withTimeout(1.0)) #not sure what timeout is best. Also not sure what unit the "setpoint" is in.

        #clamp
            #do we need to check whether or not the gamepiece is in the manipulator before closing?
        self.addCommands(ManipulatorAutoGrab(container=self.container, pneumatics=self.container.pneumatics).withTimeout(0.5)) #not sure what timeout is best. Also not sure what unit the "setpoint" is in.

        #stow
            # What should happen?
            # Elevator goes all the way down
            # Wrist stowed
            # Arm stowed
            # Turret zeroed



