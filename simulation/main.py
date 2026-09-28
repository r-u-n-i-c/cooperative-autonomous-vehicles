import matplotlib.pyplot as plt

from simulation.vehicle import Vehicle
from control.following_controller import FollowingController
from control.velocity_controller import PIController

# simulation settings
dt = 0.1
total_time = 8.0

# create a vehicles
car1 = Vehicle(
    position=0.0,
    velocity=0.0,
    max_acceleration=2.0,
    drag_coefficient=0.5
)

car2 = Vehicle(
    position=-2.0,
    velocity=0.0,
    max_acceleration=2.0,
    drag_coefficient=0.5
)

# init controller
car1_controller = PIController(
    kp=5.0,
    ki=2.0,
    max_acceleration=car1.max_acceleration
)

car2_controller = PIController(
    kp=5.0,
    ki=2.0,
    max_acceleration=car2.max_acceleration
)

# higher level controller used by car 2 to maintain spacing behind car 1
car2_follwing_controller = FollowingController(
    desired_distance=2.0,
    distance_gain=0.5
)


# lists to store info
history = []

steps = int(total_time / dt)

for i in range(steps):
    time = i * dt

    # ------------------------------------------------------------
    # lead vehicle
    # ------------------------------------------------------------

    # planned stop for CAR 1 ONLY:
    # drive 3 m/s for the first 3 seconds
    # then command the vehicle to stop
    if time < 3.0:
        desired_velocity_1 =  3.0
    else:
        desired_velocity_1 = 0.0

    # ------------------------------------------------------------
    # follower
    # ------------------------------------------------------------

    # car2 desired velocity based on car1
    desired_velocity_2, current_distance, distance_error = (
        car2_follwing_controller.update(
            leader_position=car1.position,
            leader_velocity=car1.velocity,
            follower_position=car2.position
        )
    )

    # acceleration requested by controller
    acceleration_command_1 = car1_controller.update(
        desired_velocity_1,
        car1.velocity,
        dt
    )

    acceleration_command_2 = car2_controller.update(
        desired_velocity_2,
        car2.velocity,
        dt
    )

    # apply accel
    motor_acceleration_1, net_acceleration_1 = car1.update(
        acceleration_command_1,
        dt
    )

    motor_acceleration_2, net_acceleration_2 = car2.update(
        acceleration_command_2,
        dt
    )

    history.append({
        "time": time,

        "car1": {
            "position": car1.position,
            "velocity": car1.velocity,
            "desired_velocity": desired_velocity_1,
            "acceleration_command": acceleration_command_1,
            "motor_acceleration": motor_acceleration_1,
            "net_acceleration": net_acceleration_1
        },

        "car2": {
            "position": car2.position,
            "velocity": car2.velocity,
            "desired_velocity": desired_velocity_2,
            "acceleration_command": acceleration_command_2,
            "motor_acceleration": motor_acceleration_2,
            "net_acceleration": net_acceleration_2
        },

        "following": {
            "distance": current_distance,
            "distance_error": distance_error
        }
    })

    time = (i + 1) * dt

# extract car information
times = [step["time"] for step in history]

car1_positions = [
    step["car1"]["position"]
    for step in history
]

car2_positions = [
    step["car2"]["position"]
    for step in history
]

car1_velocities = [
    step["car1"]["velocity"]
    for step in history
]

car2_velocities = [
    step["car2"]["velocity"]
    for step in history
]

distances = [
    step["following"]["distance"]
    for step in history
]

distance_errors = [
    step["following"]["distance_error"]
    for step in history
]

# plots

# pos
plt.figure()
plt.plot(
    times,
    car1_positions,
    label="Car 1")

plt.plot(
    times,
    car2_positions,
    label="Car 2")

plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title("Vehicle Positions")
plt.legend()
plt.grid()

# vel
plt.figure()
plt.plot(
    times,
    car1_velocities,
    label="Car 1"
)   

plt.plot(
    times,
    car2_velocities,
    label="Car 2"
)

plt.xlabel("Time")
plt.ylabel("Velocity (m/s)")
plt.title("Planned Stop")
plt.legend()
plt.grid()

# spacing
plt.figure()
plt.plot(
    times,
    distances,
    label="Actual Distance"
)

plt.axhline(
    car2_follwing_controller.desired_distance,
    linestyle="--",
    label="Desired Distance"
)

plt.xlabel("Time")
plt.ylabel("Distance (m)")
plt.title("Car 1 to Car 2 Follwing Distance")
plt.legend()
plt.grid()

plt.show()