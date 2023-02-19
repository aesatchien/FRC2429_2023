import time
import rev

def configure_controller(sparkmax: rev.CANSparkMax, pid_controller : rev.SparkMaxPIDController, pid_dict : dict,
                         id=0, slot=0, pid_only=False, burn_flash=False):
    """Set the PIDs, etc for the controllers, slot 0 is often position and slot 1 is often velocity
    Expects a sparkmax and a pid controller, as well as a dictionary with values - looks like this:
    pid_dict = {'kP': 0.002, 'kI': 0.004, 'kD': 0, 'kIz': 0.002, 'kFF': 0.0075, 'kArbFF': 0,
                           'kMaxOutput': 0.99, 'kMinOutput': -0.99, 'SM_MaxVel':5000, 'SM_MaxAccel':5000}
    """
    error_dict = {}
    error_dict.update({'kP0_' + str(id): pid_controller.setP(pid_dict['kP'], slot)})
    error_dict.update({'kI0_' + str(id): pid_controller.setI(pid_dict['kI'], slot)})
    error_dict.update({'kD0_' + str(id): pid_controller.setD(pid_dict['kD'], slot)})
    error_dict.update({'kFF_0' + str(id): pid_controller.setFF(pid_dict['kFF'], slot)})
    error_dict.update({'kIZ_0' + str(id): pid_controller.setIZone(pid_dict['kIz'], slot)})
    error_dict.update({'MinMax0_' + str(id): pid_controller.setOutputRange(pid_dict['kMinOutput'], pid_dict['kMaxOutput'], slot)})
    error_dict.update({'Accel0_' + str(id): pid_controller.setSmartMotionMaxVelocity(pid_dict['SM_MaxVel'], slot)})
    error_dict.update({'Vel0_' + str(id): pid_controller.setSmartMotionMaxAccel(pid_dict['SM_MaxAccel'], slot)})

    if not pid_only:
        error_dict.update({'VoltComp_' + str(id): sparkmax.enableVoltageCompensation(12)})

    if len(set(error_dict)) > 1:
        print('\n*Sparkmax setting*     *Response*')
        for key in sorted(error_dict.keys()):
            print(f'     {key:15} \t {error_dict[key]}', flush=True)
    else:
        print(f'\n *All SparkMax report {list(set(error_dict))[0]}')

    if burn_flash:
        start_time = time.time()
        can_error = sparkmax.burnFlash()
        time.sleep(0.05)
        print(f'Burn flash on controller {id}: {can_error} {int(1000 * (time.time() - start_time)):2d}ms after starting')

    return error_dict
