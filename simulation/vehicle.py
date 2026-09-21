# creating vehicle class for future implementation of multiple vehicles
# each vehicle has its own position and velocity

class Vehicle:
    # stores position and velocity to this particular car
    def __init__(
        self,
        position=0.0,
        velocity=0.0,
        max_acceleration=2.0,
        drag_coefficient=0.5
    ):
        self.position = position
        self.velocity = velocity
        self.max_acceleration = max_acceleration
        self.drag_coefficient = drag_coefficient

    # how to update position and velocity
    def update(self, acceleration, dt):

        # limit acceleration to what physical vehicle can prod
        # inside checks the upper limit (+) outside checks lower (-)
        motor_acceleration = max(
            -self.max_acceleration,
            min(acceleration, self.max_acceleration)
        )

        # drag opposes the vehicles motion
        # vdot = accel_motor - c*v
        drag_acceleration = self.drag_coefficient * self.velocity

        # net acceleration of vehicle
        net_acceleration = motor_acceleration - drag_acceleration

        # update vehicle state
        self.position += self.velocity * dt
        self.velocity += net_acceleration * dt

        return motor_acceleration, net_acceleration
        