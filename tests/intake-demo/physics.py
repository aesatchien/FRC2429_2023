#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import hal
import wpilib
import wpilib.simulation
from robotpy_ext.common_drivers.distance_sensors_sim import SharpIR2Y0A41Sim

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units
import wpimath

from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor

import dataclasses
import math
import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


RED = wpilib.Color8Bit(wpilib.Color.kRed)
BLUE = wpilib.Color8Bit(wpilib.Color.kBlue)
GREEN = wpilib.Color8Bit(wpilib.Color.kGreen)
GRAY = wpilib.Color8Bit(wpilib.Color.kGray)

HSV_RED = 0
HSV_BLUE = 120
HSV_GREEN = 60


@dataclasses.dataclass
class Shape:
    root: wpilib.MechanismRoot2d
    items: typing.List[wpilib.MechanismLigament2d]

    def setPosition(self, x: float, y: float):
        self.root.setPosition(x, y)

    def setColor(self, c: wpilib.Color8Bit):
        for item in self.items:
            item.setColor(c)


class Mechanism(wpilib.Mechanism2d):
    def __init__(self, width: float, height: float) -> None:
        super().__init__(width, height)

    def make_point(self, name: str, cx: float, cy: float, sz: int = 10) -> Shape:
        root = self.getRoot(name, cx, cy)
        return self._make(root, 0.50, 0, 1, sz)

    def make_triangle(
        self, name: str, cx: float, cy: float, side: float, line_width=6
    ) -> Shape:
        x = cx + side / 2
        y = cy - side / 2
        root = self.getRoot(name, x, y)
        return self._make(root, side, 120, 3, line_width)

    def make_hex(
        self, name: str, x: float, y: float, side: float, line_width=6
    ) -> Shape:
        # x, y are near bottom right corner
        root = self.getRoot(name, x, y)
        return self._make(root, side, 60, 6, line_width)

    def _make(
        self,
        root: wpilib.MechanismRoot2d,
        side: float,
        angle: float,
        n: int,
        line_width,
    ) -> Shape:
        item = root
        items = []
        for i in range(n):
            item = item.appendLigament(f"{i}", side, angle, line_width)
            items.append(item)

        return Shape(root, items)


BALL_DIAMETER = 9.5
BALL_RADIUS = 4.75
BALL_Y = 10.5
BALL_MOVE = 12 / 604.0

# positions along the intake
EDGE = 8
INDEXER_LEN = 12
SENSOR_Y = 15

ENTRY_MOTOR_START = 0
ENTRY_MOTOR_END = EDGE + 2


ENTRY_SENSOR_POS = 9
EXIT_SENSOR_POS = 20

BELT_MOTOR_START = EDGE - BALL_RADIUS
BELT_MOTOR_END = EDGE + INDEXER_LEN

SHOOTER_START = EDGE + INDEXER_LEN
SHOOTER_END = SHOOTER_START + BALL_RADIUS

BEAM_SIZE = 0.5


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller

        # Motors and sensors

        self.entry_sensor = SharpIR2Y0A41Sim(robot.entry_sensor)
        self.exit_sensor = SharpIR2Y0A41Sim(robot.exit_sensor)

        self.entry_motor = wpilib.simulation.PWMSim(robot.entry_motor.getChannel())
        self.belt_motor = wpilib.simulation.PWMSim(robot.belt_motor.getChannel())
        self.shooter_motor = wpilib.simulation.PWMSim(robot.shooter_motor.getChannel())

        # TODO: these motor sim values are bogus, but it probably doesn't matter
        #       for our purposes here
        self.entry_motor_sim = wpilib.simulation.DCMotorSim(DCMotor.NEO(), 1, 0.0005)
        self.belt_motor_sim = wpilib.simulation.DCMotorSim(DCMotor.NEO(), 1, 0.0005)
        self.shooter_motor_sim = wpilib.simulation.FlywheelSim(DCMotor.NEO(), 4, 0.0005)

        # balls

        # Intake tuning
        self.intake_tuner = hal.SimDevice("Intake Tuner")

        # Represents the end of when the intake motor affects the ball's center
        self.intake_pos_start = self.intake_tuner.createDouble(
            "intake pos start", False, ENTRY_MOTOR_START
        )
        self.intake_pos_end = self.intake_tuner.createDouble(
            "intake pos end", False, ENTRY_MOTOR_END
        )

        # Represents the start/end of when the belt affects the ball's center
        self.belt_pos_start = self.intake_tuner.createDouble(
            "belt pos start", False, BELT_MOTOR_START
        )
        self.belt_pos_end = self.intake_tuner.createDouble(
            "belt pos end", False, BELT_MOTOR_END
        )

        # Represents the start/end of when the shooter affects the ball's center
        self.shooter_pos_start = self.intake_tuner.createDouble(
            "shooter pos start", False, SHOOTER_START
        )
        self.shooter_pos_end = self.intake_tuner.createDouble(
            "shooter pos end", False, SHOOTER_END
        )

        self.entry_sensor_pos = self.intake_tuner.createDouble(
            "entry sensor pos", False, ENTRY_SENSOR_POS
        )
        self.exit_sensor_pos = self.intake_tuner.createDouble(
            "exit sensor pos", False, EXIT_SENSOR_POS
        )

        # Ball control
        self.ball_device = hal.SimDevice("Balls")
        self.ball_insert = self.ball_device.createBoolean("insert", False, False)

        # drawn robot model (scale 1inch=1)

        self.model = Mechanism(30, 30)
        wpilib.SmartDashboard.putData("Model", self.model)

        outside = self.model.getRoot("outside", EDGE, 10)
        l = outside.appendLigament("l1", INDEXER_LEN, 0, color=GRAY)
        # l = l.appendLigament("l2", 20, 25, color=GRAY)
        # l = l.appendLigament("l3", 20, 25, color=GRAY)
        # l = l.appendLigament("l4", 20, 25, color=GRAY)
        # l = l.appendLigament("l5", 30, 30, color=GRAY)
        # l = l.appendLigament("l6", 20, 20, color=GRAY)

        inside = self.model.getRoot("inside", EDGE, 20)
        inside.appendLigament("l1", INDEXER_LEN, 0, color=GRAY)

        self.entry_sensor_pt = self.model.make_point(
            "entry-sensor", self.entry_sensor_pos.get(), SENSOR_Y
        )

        self.exit_sensor_pt = self.model.make_point(
            "exit-sensor",
            self.exit_sensor_pos.get(),
            SENSOR_Y,
        )

        self.entry_motor_pt = self.model.make_hex("intake-motor", 4, 20, 2.5, 4)
        self.entry_motor_pt.setColor(GRAY)

        self.belt_motor_pt = self.model.make_hex("belt-motor", 10, 4, 2.5, 4)
        self.belt_motor_pt.setColor(GRAY)

        self.shooter = self.model.make_hex("shooter", 27, 20, 3, 2)
        self.shooter.setColor(BLUE)

        # The 'value' of each ball is either 'nan' (not present) or it
        # is the distance the ball lies along the track in inches
        self.balls = []

        for i in range(2):
            v = self.ball_device.createDouble(f"center {i}", False, float("nan"))
            m = self.model.make_hex(f"ball {i}", -400, BALL_Y, 5, 1)
            m.setColor(RED)

            self.balls.append((v, m))

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        self.intake_simulation(tm_diff)

        self.update_positions()

    def intake_simulation(self, tm_diff: float) -> None:

        # Update motor movement
        v = wpilib.simulation.RoboRioSim.getVInVoltage()
        self.entry_motor_sim.setInputVoltage(self.entry_motor.getSpeed() * v)
        self.entry_motor_sim.update(tm_diff)
        intake_move = self.entry_motor_sim.getAngularVelocity() * BALL_MOVE * tm_diff
        # self.entry_motor_sim.getAngularPosition()
        # print(
        #     self.entry_motor_sim.getAngularVelocity()
        # )  # max is 604, whatever that means...

        self.belt_motor_sim.setInputVoltage(self.belt_motor.getSpeed() * v)
        self.belt_motor_sim.update(tm_diff)
        belt_move = self.belt_motor_sim.getAngularVelocity() * BALL_MOVE * tm_diff

        self.shooter_motor_sim.setInputVoltage(self.shooter_motor.getSpeed() * v)
        self.shooter_motor_sim.update(tm_diff)
        shooter_move = self.shooter_motor_sim.getAngularVelocity() * 0.1 * tm_diff

        # self.shooter_motor_sim.getAngularVelocity()

        #
        # ball movement
        #

        # Has a ball just been inserted?
        if self.ball_insert.value:
            # 'insert' a new ball by setting its position at the starting point
            for ball, _ in self.balls:
                if math.isnan(ball.value):
                    ball.value = 0
                    print("Ball inserted!")
                    break

            self.ball_insert.value = False

        # Valid balls
        balls = [ball for ball in self.balls if not math.isnan(ball[0].value)]

        # Sensors
        self.entry_sensor.setDistance(40)
        self.exit_sensor.setDistance(40)

        entry_sensor_pos = self.entry_sensor_pos.get()
        (entry_sensor_start, entry_sensor_end) = (
            entry_sensor_pos - BEAM_SIZE,
            entry_sensor_pos + BEAM_SIZE,
        )

        exit_sensor_pos = self.exit_sensor_pos.get()
        (exit_sensor_start, exit_sensor_end) = (
            exit_sensor_pos - BEAM_SIZE,
            exit_sensor_pos + BEAM_SIZE,
        )

        ball_positions = []

        for ball, mball in balls:

            #
            # Compute ball movement
            # - if the center of the ball overlaps the range of the specified motor,
            #   then it is moved using the value computed above
            #

            ball_position = ball.value

            if (
                ball_position >= self.intake_pos_start.get()
                and ball_position <= self.intake_pos_end.get()
            ):
                ball_position += intake_move
                ball.value = ball_position

            if (
                ball_position >= self.belt_pos_start.get()
                and ball_position <= self.belt_pos_end.get()
            ):
                ball_position += belt_move
                ball.value = ball_position

            if (
                ball_position >= self.shooter_pos_start.get()
                and ball_position <= self.shooter_pos_end.get()
            ):
                ball_position += shooter_move
                ball.value = ball_position

            ball_positions.append(ball_position)

            #
            # Compute sensor detections
            # - If either edge of the ball lies between the sensors start
            #   or end position, set the voltage appropriately
            #

            ball_start = ball_position - BALL_RADIUS
            ball_end = ball_position + BALL_RADIUS

            if (
                entry_sensor_start >= ball_start and entry_sensor_start <= ball_end
            ) or (entry_sensor_end >= ball_start and entry_sensor_end <= ball_end):
                self.entry_sensor.setDistance(10)

            if (exit_sensor_start >= ball_start and exit_sensor_start <= ball_end) or (
                exit_sensor_end >= ball_start and exit_sensor_end <= ball_end
            ):
                self.exit_sensor.setDistance(10)

            # If the ball was shot, remove it
            if ball_position < 0:
                print("Ball removed!")
                ball.value = float("nan")
            if ball_position > self.shooter_pos_end.value:
                print("Ball shot")
                ball.value = float("nan")

        # Finally, determine if any of the balls overlapped each other
        # - Sort by distance to make it easier to compute ball overlap
        ball_positions = sorted(ball_positions)
        for i in range(1, len(ball_positions)):
            if ball_positions[i] - ball_positions[i - 1] < BALL_DIAMETER:
                print("=" * 72)
                print(" " * 20, "FAIL: balls overlapped!!")
                print(" " * 20, ", ".join("%.3f" % bp for bp in ball_positions))
                print("=" * 72)
                for ballv, _ in self.balls:
                    ballv.value = float("nan")
                break

    def get_motor_color(self, vel, fwd, rev):
        vel = min(max(round(vel), -255), 255)
        if vel < 0:
            return wpilib.Color8Bit(wpilib.Color.fromHSV(rev, 255, -vel))
        else:
            return wpilib.Color8Bit(wpilib.Color.fromHSV(fwd, 255, vel))

    def update_positions(self):

        # set motor colors to indicate movement
        self.entry_motor_pt.setColor(
            self.get_motor_color(
                self.entry_motor_sim.getAngularVelocity() / (600 / 255.0),
                HSV_GREEN,
                HSV_RED,
            )
        )

        self.belt_motor_pt.setColor(
            self.get_motor_color(
                self.belt_motor_sim.getAngularVelocity() / (600 / 255.0),
                HSV_GREEN,
                HSV_RED,
            )
        )

        self.shooter.setColor(
            self.get_motor_color(
                self.shooter_motor_sim.getAngularVelocity() / (150 / 255.0),
                HSV_GREEN,
                HSV_RED,
            )
        )

        # set sensor colors to indicate detection
        self.entry_sensor_pt.setPosition(self.entry_sensor_pos.get(), SENSOR_Y)
        if self.entry_sensor.getDistance() > 20:
            self.entry_sensor_pt.setColor(GRAY)
        else:
            self.entry_sensor_pt.setColor(RED)

        self.exit_sensor_pt.setPosition(self.exit_sensor_pos.get(), SENSOR_Y)
        if self.exit_sensor.getDistance() > 20:
            self.exit_sensor_pt.setColor(GRAY)
        else:
            self.exit_sensor_pt.setColor(RED)

        # set ball positions
        for ballv, ballm in self.balls:
            v = ballv.value
            if math.isnan(v):
                ballm.setPosition(-400, BALL_Y)
            else:
                ballm.setPosition(v + 2.5, BALL_Y)
