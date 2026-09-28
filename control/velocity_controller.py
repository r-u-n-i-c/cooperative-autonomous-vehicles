class PIController:
    def  __init__(self, kp, ki, max_acceleration):
        # gains
        self.kp = kp
        self.ki = ki
        
        # physical actuator limit used for anti wind up (reduced overshoot)
        self.max_acceleration = max_acceleration

        # integral term stores accumulated error
        self.integral_error = 0.0

    def update(self, desired_velocity, current_velocity, dt):
        # current velocity tracking error
        error = desired_velocity - current_velocity

        # calculate what the integral would become
        new_integral_error = (
            self.integral_error
            + error * dt
        )

        # calc the command using the integral value
        test_command = (
            self.kp * error
            + self.ki * new_integral_error
        )

            # check if the actuator would be saturated
        saturated_high = test_command > self.max_acceleration
        saturated_low = test_command < -self.max_acceleration

        # only update the integral if:
        # 1. we are not saturated, OR
        # 2. the error is helping bring us OUT of saturation
        if not (
            (saturated_high and error > 0)
            or
            (saturated_low and error < 0)
        ):
            self.integral_error = new_integral_error

        # final pi controller command
        acceleration_command = (
            self.kp * error
            + self.ki * self.integral_error
        )

        return acceleration_command