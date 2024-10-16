

def arbitration(acceleration_by_acc, acceleration_by_aeb, steer_by_lfa):

    # longitudinal arbitration
    acceleration = acceleration_by_acc

    if acceleration_by_aeb:
        acceleration = acceleration_by_aeb

    # lateral arbitration
    steer = steer_by_lfa

    return acceleration, steer