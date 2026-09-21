import matplotlib.pyplot as plt

from simulation.vehicle import Vehicle
from control.velocity_controller import velocity_controller

# simulation settings
dt = 0.1
total_time = 5.0

# controller settings
desired_velocity = 1.0
kp = 5.0
ki = 2.0

# create a vehicle
car = Vehicle(position=0.0, velocity=0.0)


# lists to store info
times = []
positions = []
velocities = []
acceleration_commands = []
motor_accelerations = []
net_accelerations = []

steps = int(total_time / dt)
integral_error = 0.0

for i in range(steps):
    # acceleration requested by controller
    acceleration_command, integral_error = velocity_controller(
        desired_velocity,
        car.velocity,
        kp,
        ki,
        integral_error,
        dt,
        car.max_acceleration
    )

    # apply accel
    motor_acceleration, net_acceleration = car.update(
        acceleration_command,
        dt
    )

    time = (i + 1) * dt

    times.append(time)
    positions.append(car.position)
    velocities.append(car.velocity)
    acceleration_commands.append(acceleration_command)
    motor_accelerations.append(motor_acceleration)
    net_accelerations.append(net_acceleration)

    print(
        f"t = {time:.1f} s, "
        f"x = {car.position:.2f} m, "
        f"v = {car.velocity:.2f} m/s"
    )

# plots

# time
plt.figure()
plt.plot(times, positions)
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title("Vehicle Position")
plt.grid()

# vel
plt.figure()
plt.plot(times, velocities)
plt.axhline(desired_velocity, linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title("Vehicle Velocity")
plt.grid()

# accel
plt.figure()
plt.plot(
    times,
    acceleration_commands,
    label="Controller Command"
)

plt.plot(
    times,
    motor_accelerations,
    label="Motor Acceleration"
)

plt.plot(
    times,
    net_accelerations,
    label="Net Acceleration"
)

plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.title("Vehicle Acceleration")
plt.legend()
plt.grid()

plt.show()