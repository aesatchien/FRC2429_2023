"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.

2023 robot for team 2429 - the blockheads
"""

k_competition_mode = False  # use for compressor and some joystick settings
k_burn_flash = False  # if we want to burn the settings to the sparkmaxes

# --------------  OI  ---------------
# ID for the driver's joystick (template)
k_driver_controller_port = 0
k_co_driver_controller_port = 1

# --------------  DRIVETRAIN  ---------------
# The CAN IDs for the drivetrain SparkMAX motor controllers
k_left_motor1_port = 1
k_left_motor2_port = 2
k_right_motor1_port = 3
k_right_motor2_port = 4

# --------------  SCORING  SUBSYSTEMS ---------------

# --------------  TURRET  ---------------
k_turret_motor_port = 9  # sparkmax with a NEO550 - full speed is 11k
k_turret_abs_encoder_port = 1  # analog absolute encoder on turret
k_turret_encoder_conversion_factor = 360 / 462.2  # Armabot has 462:1 gear ratio?  Circle has 360 degrees-->  0.779°/rot
# TODO: verify turret velocity PID values, burn to slot 0
k_PID_dict_vel_turret = {'kP': 5e-5, 'kI': 1e-6, 'kD': 0, 'kIz': 1e-5, 'kFF': 1.6e-4, 'kArbFF':0,
                         'kMaxOutput': 0.99, 'kMinOutput': -0.99, 'SM_MaxVel':10000/k_turret_encoder_conversion_factor,
                         'SM_MaxAccel':10000/k_turret_encoder_conversion_factor}

# --------------  ELEVATOR  ---------------
k_elevator_motor_port = 10  # sparkmax with a NEO
k_elevator_timeoflight = 13  # time of flight CAN ID
# 6.24mm/rev - comes from 16x reduction motor to shaft, one sprocket rot is 4.05in, so 0.253in / rot * 25.4 mm/in
k_elevator_encoder_conversion_factor = 0.253 * 25.4   # 6.24
# TODO: verify elevator velocity PID values
k_PID_dict_vel_elevator = {'kP': 0.002, 'kI': 0.004, 'kD': 0, 'kIz': 0.002, 'kFF': 0.0075, 'kArbFF':0,
                         'kMaxOutput': 0.99, 'kMinOutput': -0.99, 'SM_MaxVel':5000/k_elevator_encoder_conversion_factor,
                           'SM_MaxAccel':5000/k_elevator_encoder_conversion_factor}

# --------------  WRIST  ---------------
k_wrist_motor_port = 11  # sparkmax with a NEO
# 81x reduction motor to shaft, so in degrees it's 360./81.  Half a second at 6k rpm
k_wrist_encoder_conversion_factor = 360. / 81  # 4.44 degrees per revolution
# TODO: verify wrist velocity PID values
k_PID_dict_vel_wrist = {'kP': 5e-5, 'kI': 1e-6, 'kD': 0, 'kIz': 1e-5, 'kFF': 0.0075, 'kArbFF':0,
                         'kMaxOutput': 0.99, 'kMinOutput': -0.99, 'SM_MaxVel':5000/k_wrist_encoder_conversion_factor,
                        'SM_MaxAccel':5000/k_wrist_encoder_conversion_factor}

# --------------  ARM  ---------------
k_arm_motor_port = 12  # sparkmax with a NEO550 - full speed is 11k
# 60x reduction motor to shaft, one drum rot is 1.9*pi inch, then x 25.4 so we are measuring in mm
k_arm_encoder_conversion_factor = (1.9 * 3.14 / 60) * 25.4  # 2.52 mm per revolution
# TODO: verify arm velocity PID values
k_PID_dict_vel_arm = {'kP': 5e-5, 'kI': 1e-6, 'kD': 0, 'kIz': 1e-5, 'kFF': 0.8e-4, 'kArbFF':0,
                         'kMaxOutput': 0.99, 'kMinOutput': -0.99, 'SM_MaxVel':10000/k_arm_encoder_conversion_factor,
                      'SM_MaxAccel':10000/k_arm_encoder_conversion_factor}

# TODO: determine which systems need a limit switch
k_wrist_limit_switch = 1  # DIO for the wrist limit switch
k_arm_limit_switch = 3

# --------------  PNEUMATICS  ---------------
k_manipulator_open_port = 0  # Double solenoid port 1 of two
k_manipulator_closed_port = 1  #


# --------------  SIMULATION  ---------------
k_start_x = 7.647
k_start_y = 1.935
k_start_heading = -90  # looking at the drawing originally tried -109
k_drivetrain_motor_count = 4
k_wheel_diameter_m = 6 * 0.0254  # wheel diameter in meters
k_gear_ratio = 5.39  # 4.17 # high gear 2022
k_track_width_meters = 24 * 0.0254
robot_characterization = {'ks':0.291, 'kv':1.63, 'ka':0.293, 'track_width':0.89}  # 2022 climberbot
ks_volts = robot_characterization['ks']  # so far this is only used in the Ramsete command, but in 2021 we used it in tank model as well
kv_volt_seconds_per_meter = robot_characterization['kv']  # used in physics.py LinearSystemId and Ramsete
ka_volt_seconds_squared_per_meter = robot_characterization['ka']  # used in physics.py LinearSystemId and Ramsete

# --------------  HELPER FUNCTIONS  ---------------
def clamp(value: float, bottom: float, top: float) -> float:
    return max(bottom, min(value, top))