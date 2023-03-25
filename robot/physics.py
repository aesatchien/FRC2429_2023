"""
    CJH Copied from robotpy docs example drivetrain physics, keeping only swerve  2023/03/19

    .. warning:: These drivetrain models are not particularly realistic, and
                 if you are using a tank drive style drivetrain you should use
                 the :class:`.TankModel` instead.

    Based on input from various drive motors, these helper functions
    simulate moving the robot in various ways. Many thanks to
    `Ether <http://www.chiefdelphi.com/forums/member.php?u=34863>`_
    for assistance with the motion equations.

    When specifying the robot speed to the below functions, the following
    may help you determine the approximate speed of your robot:

    * Slow: 4ft/s
    * Typical: 5 to 7ft/s
    * Fast: 8 to 12ft/s

    Obviously, to get the best simulation results, you should try to
    estimate the speed of your robot accurately.

    Here's an example usage of the drivetrains::

        import hal.simulation
        from pyfrc.physics import drivetrains

        class PhysicsEngine:

            def __init__(self, physics_controller):
                self.physics_controller = physics_controller
                self.drivetrain = drivetrains.TwoMotorDrivetrain(deadzone=drivetrains.linear_deadzone(0.2))

                self.l_motor = hal.simulation.PWMSim(1)
                self.r_motor = hal.simulation.PWMSim(2)

            def update_sim(self, now, tm_diff):
                l_motor = self.l_motor.getSpeed()
                r_motor = self.r_motor.getSpeed()

                speeds = self.drivetrain.calculate(l_motor, r_motor)
                self.physics_controller.drive(speeds, tm_diff)

                # optional: compute encoder
                # l_encoder = self.drivetrain.wheelSpeeds.left * tm_diff

    .. versionchanged:: 2020.1.0

       The input speeds and output rotation angles were changed to reflect
       the current WPILib drivetrain/field objects. Wheelbases and default
       speeds all require units.
"""
import math
import typing

import wpilib
import wpilib.simulation as simlib
from pyfrc.physics.core import PhysicsEngine, Pose2d
from pyfrc.physics.drivetrains import DeadzoneCallable, linear_deadzone
from wpimath.kinematics import ChassisSpeeds


class PhysicsEngine(PhysicsEngine):

    def __init__(self, physics_controller):
        self.physics_controller = physics_controller
        self.field = wpilib.Field2d()

        offset = 16
        self.lf_motor = simlib.PWMSim(21 - offset)
        self.lr_motor = simlib.PWMSim(23-offset)
        self.rf_motor = simlib.PWMSim(25-offset)
        self.rr_motor = simlib.PWMSim(27-offset)

        self.lf_angle = simlib.PWMSim(20-offset)
        self.lr_angle = simlib.PWMSim(22-offset)
        self.rf_angle = simlib.PWMSim(24-offset)
        self.rr_angle = simlib.PWMSim(26-offset)

        # self.drivetrain = drivetrains.four_motor_swerve_drivetrain(deadzone=drivetrains.linear_deadzone(0.1))

    def update_sim(self, now, tm_diff):
        velocity_scale, angular_scale = 10, 10  # faking PWM signals in from -1 to 1, multiply by to x get velocities,
        lr_motor = self.lr_motor.getSpeed() * velocity_scale
        rr_motor = self.rr_motor.getSpeed() * velocity_scale
        lf_motor = self.lf_motor.getSpeed() * velocity_scale
        rf_motor = self.rf_motor.getSpeed() * velocity_scale

        lr_angle = self.lr_angle.getSpeed() * angular_scale   # multiply by 10 to get dummy angles from simulated pwms
        rr_angle = self.rr_angle.getSpeed() * angular_scale
        lf_angle = self.lf_angle.getSpeed() * angular_scale
        rf_angle = self.rf_angle.getSpeed() * angular_scale

        # had to mess with the orders and the vx, vy to get things to sim properly
        speeds = four_motor_swerve_drivetrain(lf_motor, rf_motor, lr_motor, rr_motor,
        lf_angle, rf_angle, lr_angle, rr_angle, x_wheelbase = 2, y_wheelbase = 2,
        speed=5, deadzone=None,)

        self.physics_controller.drive(speeds, tm_diff)
        self.field.setRobotPose(self.physics_controller.get_pose())
        x = self.field.getRobotPose().X()
        y = self.field.getRobotPose().Y()
        theta = self.field.getRobotPose().rotation().degrees()
        wpilib.SmartDashboard.putNumberArray('sim_pose', [x,y,theta])
        # optional: compute encoder
        # l_encoder = self.drivetrain.wheelSpeeds.left * tm_diff

def four_motor_swerve_drivetrain(
    lr_motor: float,
    rr_motor: float,
    lf_motor: float,
    rf_motor: float,
    lr_angle: float,
    rr_angle: float,
    lf_angle: float,
    rf_angle: float,
    x_wheelbase=2,
    y_wheelbase=2,
    speed=10,
    deadzone=None,
) -> ChassisSpeeds:
    """
    Four motors that can be rotated in any direction

    If any motors are inverted, then you will need to multiply that motor's
    value by -1.

    :param lr_motor:   Left rear motor value (-1 to 1); 1 is forward
    :param rr_motor:   Right rear motor value (-1 to 1); 1 is forward
    :param lf_motor:   Left front motor value (-1 to 1); 1 is forward
    :param rf_motor:   Right front motor value (-1 to 1); 1 is forward

    :param lr_angle:   Left rear motor angle in degrees (0 to 360 measured clockwise from forward position)
    :param rr_angle:   Right rear motor angle in degrees (0 to 360 measured clockwise from forward position)
    :param lf_angle:   Left front motor angle in degrees (0 to 360 measured clockwise from forward position)
    :param rf_angle:   Right front motor angle in degrees (0 to 360 measured clockwise from forward position)

    :param x_wheelbase: The distance in feet between right and left wheels.
    :param y_wheelbase: The distance in feet between forward and rear wheels.
    :param speed:       Speed of robot in feet per second (see above)
    :param deadzone:    A function that adjusts the output of the motor (see :func:`linear_deadzone`)

    :returns: ChassisSpeeds that can be passed to 'drive'

    .. versionchanged:: 2020.1.0

       The output rotation angle was changed from CW to CCW to reflect the
       current WPILib drivetrain/field objects
    """

    if deadzone:
        lf_motor = deadzone(lf_motor)
        lr_motor = deadzone(lr_motor)
        rf_motor = deadzone(rf_motor)
        rr_motor = deadzone(rr_motor)

    # Calculate speed of each wheel
    lr = lr_motor * speed
    rr = rr_motor * speed
    lf = lf_motor * speed
    rf = rf_motor * speed

    # Calculate angle in radians
    lr_rad = lr_angle
    rr_rad = rr_angle
    lf_rad = lf_angle
    rf_rad = rf_angle

    # Calculate wheelbase radius
    wheelbase_radius = math.hypot(x_wheelbase / 2.0, y_wheelbase / 2.0)

    # Calculates the Vx and Vy components
    # Sin an Cos inverted because forward is 0 on swerve wheels
    Vy = (
        (math.sin(lr_rad) * lr)
        + (math.sin(rr_rad) * rr)
        + (math.sin(lf_rad) * lf)
        + (math.sin(rf_rad) * rf)
    )
    Vx = (
        (math.cos(lr_rad) * lr)
        + (math.cos(rr_rad) * rr)
        + (math.cos(lf_rad) * lf)
        + (math.cos(rf_rad) * rf)
    )

    # Adjusts the angle corresponding to a diameter that is perpendicular to the radius (add or subtract 45deg)
    lr_rad = (lr_rad + (math.pi / 4)) % (2 * math.pi)
    rr_rad = (rr_rad - (math.pi / 4)) % (2 * math.pi)
    lf_rad = (lf_rad - (math.pi / 4)) % (2 * math.pi)
    rf_rad = (rf_rad + (math.pi / 4)) % (2 * math.pi)

    # Finds the rotational velocity by finding the torque and adding them up
    Vw = wheelbase_radius * (
        (math.cos(lr_rad) * -lr)
        + (math.cos(rr_rad) * rr)
        + (math.cos(lf_rad) * -lf)
        + (math.cos(rf_rad) * rf)
    )

    Vx *= 0.25
    Vy *= 0.25
    Vw *= 0.25

    wpilib.SmartDashboard.putNumberArray('sim_chassis', [Vx, Vy, Vw])
    return ChassisSpeeds.fromFeet(Vx, Vy, Vw)

