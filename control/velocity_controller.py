def velocity_controller(
    desired_velocity,
    current_velocity,
    kp,
    ki,
    integral_error,
    dt,
    max_acceleration
    ):

    # calculate velocity error
    error = desired_velocity - current_velocity

    # calculate what the integral error would be
    new_integral_error = integral_error + error * dt

    # calculate what the controller WOULD command
    # if we accepted the new integral value
    test_command = (
        kp * error
        + ki * new_integral_error
    )

    # check if the actuator would be saturated
    saturated_high = test_command > max_acceleration
    saturated_low = test_command < -max_acceleration

    # only update the integral if:
    # 1. we are not saturated, OR
    # 2. the error is helping bring us OUT of saturation
    if not (
        (saturated_high and error > 0)
        or
        (saturated_low and error < 0)
    ):
        integral_error = new_integral_error

    # final pi controller command
    acceleration_command = (
        kp * error
        + ki * integral_error
    )

    return acceleration_command, integral_error