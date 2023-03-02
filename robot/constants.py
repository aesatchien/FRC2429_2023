"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.

2023 robot for team 2429 - the blockheads
"""

k_competition_mode = True  # use for compressor and some joystick settings
k_burn_flash = False  # if we want to burn the settings to the sparkmaxes
k_enable_soft_limts = True

# --------------  OI  ---------------
# ID for the driver's joystick (template)
k_driver_controller_port = 0
k_co_driver_controller_port = 1
k_controller_thrust_axis = 1
k_controller_twist_axis = 4
k_arcade_thrust_scale = 0.7  # used in drive by joystick arcade mode
k_arcade_twist_scale = 0.45  # used in drive by joystick arcade mode
k_max_thrust_velocity = 150  # meters per MINUTE  for smartmotion
k_max_twist_velocity = 90 # meters per MINUTE - for smartmotion

#  co-driver
k_controller_elevator_axis = 1
k_controller_turret_axis = 4


# --------------  DRIVETRAIN  ---------------
# The CAN IDs for the drivetrain SparkMAX motor controllers
k_left_motor1_port = 1
k_left_motor2_port = 2
k_right_motor1_port = 3
k_right_motor2_port = 4

# drivetrain constants
k_wheel_diameter_in = 6  # wheel diameter in inches
k_wheel_diameter_m =  k_wheel_diameter_in * 0.0254  # wheel diameter in meters
k_robot_length = 33 * 0.0254
k_track_width_meters = 27 * 0.0254
k_robot_wheelbase = 18 * 0.5 * 0.0254
k_gear_ratio = 10.72  # REV 6in slow (10T) is 11.79:  medium (11T) is 10.72.  Both use the 30/68, so 2.26 * 52/PinionT
k_sparkmax_conversion_factor_meters = k_wheel_diameter_m * 3.14159 / k_gear_ratio  # used in drivetrain

# testing ON BLOCKS shows that flat out at 90% power we top out at 4m/s - still pretty fast
# kff = 0.24 using k and k/60  but this screws smart motion - it ain't that smart so 0.0040 is the kFF and m/min is vel

k_PID_dict_pos = {'kP': 0.002, 'kI': 0, 'kD': 0.002, 'kIz': 0, 'kFF': 0.008, 'kArbFF':0, 'kMaxOutput': 0.99, 'kMinOutput': -0.99,
                'SM_MaxVel': 5000 / k_sparkmax_conversion_factor_meters, 'SM_MaxAccel': 5000 / k_sparkmax_conversion_factor_meters}
k_PID_dict_vel = {'kP': 0.0, 'kI': 0.000, 'kD': 0.00, 'kIz': 0.001, 'kFF': 0.0040, 'kArbFF':0, 'kMaxOutput': 0.95,
                'kMinOutput': -0.95, 'SM_MaxVel':180, 'SM_MaxAccel':120}  # 180 is 3 m/s and 3m/s/s
k_PID_dict_vel_slow = {'kP': 1e-5, 'kI': 1e-6, 'kD': 0.00, 'kIz': 0.001, 'kFF': 0.0040, 'kArbFF':1, 'kMaxOutput': 0.95,
                'kMinOutput': -0.95, 'SM_MaxVel':120, 'SM_MaxAccel':120}  # 120 is 2 m/s and 2m/s/s

# --------------  SCORING  SUBSYSTEMS ---------------

# --------------  TURRET  ---------------
k_turret_motor_port = 9  # sparkmax with a NEO550 - full speed is 11k
k_turret_abs_encoder_port = 1  # analog absolute encoder on turret
k_turret_encoder_conversion_factor = 360 / 462.2  # Armabot has 462:1 gear ratio?  Circle has 360 degrees-->  0.779°/rot
# TODO: verify turret velocity PID values, burn to slot 0  - tested on 2023 0226
k_PID_dict_vel_turret = {'kP': 0, 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF': 1.4e-4, 'kArbFF':0,
                         'kMaxOutput': 0.8, 'kMinOutput': -0.8, 'SM_MaxVel':5500,
                         'SM_MaxAccel':3800}

# --------------  ELEVATOR  ---------------
k_elevator_motor_port = 10  # sparkmax with a NEO
k_elevator_timeoflight = 13  # time of flight CAN ID
# 4.11mm/rev - comes from 25x reduction motor to shaft, one sprocket rot is 4.05in, so 0.162in / rot * 25.4 mm/in
k_elevator_encoder_conversion_factor = 0.162 * 25.4   # 4.11
# TODO: verify elevator velocity PID values  - # 25000V and 35000A worked well in practice  20230121
k_PID_dict_vel_elevator = {'kP': 0, 'kI': 0, 'kD': 0, 'kIz': 2e-4, 'kFF': 4.1e-5, 'kArbFF':0,
                         'kMaxOutput': 0.9, 'kMinOutput': -0.9, 'SM_MaxVel':30000,
                         'SM_MaxAccel':30000}

# --------------  ARM  ---------------
k_arm_motor_port = 11  # sparkmax with a NEO550 - full speed is 11k
# 60x reduction motor to shaft, one drum rot is 1.9*pi inch, then x 25.4 so we are measuring in mm
k_arm_encoder_conversion_factor = (1.9 * 3.14 / 60) * 25.4  # 2.52 mm per revolution
# TODO: verify arm velocity PID values
k_PID_dict_vel_arm = {'kP': 1e-5, 'kI': 1e-5, 'kD': 0, 'kIz': 1e-5, 'kFF': 4.1e-5, 'kArbFF':0,
                         'kMaxOutput': 0.95, 'kMinOutput': -0.95, 'SM_MaxVel':25000,
                      'SM_MaxAccel':25000}

# --------------  WRIST  ---------------
k_wrist_motor_port = 12  # sparkmax with a NEO
# 9*9*7 = 567x reduction motor to shaft, so in degrees it's 360./567.  Half a second at 6k rpm
k_wrist_encoder_conversion_factor = 360. / 567  # 0.635 degrees per revolution
# TODO: verify wrist velocity PID values
k_PID_dict_vel_wrist = {'kP': 0, 'kI': 0, 'kD': 0, 'kIz': 1e-5, 'kFF': 1.56e-4, 'kArbFF':0,
                         'kMaxOutput': 0.6, 'kMinOutput': -0.6, 'SM_MaxVel':4400,
                        'SM_MaxAccel':4800}

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
