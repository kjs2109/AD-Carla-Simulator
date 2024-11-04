
def calc_brake_command(desired_acceleration, max_deceleration):

    if desired_acceleration >= 0:
        brake_cmd = 0
    else:
        brake_cmd = -desired_acceleration / 7 



    return brake_cmd
