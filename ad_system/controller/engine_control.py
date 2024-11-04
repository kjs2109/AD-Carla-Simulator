
def calc_engine_control_command(desired_acceleration, max_acceleration):

    if desired_acceleration > 0:
        throttle_cmd = desired_acceleration / max_acceleration 
        if throttle_cmd < 0.65: 
            throttle_cmd = 2 * throttle_cmd
            if throttle_cmd > 0.65: 
                throttle_cmd = 0.65
    else:
        throttle_cmd = 0

    return throttle_cmd
