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

car3 = Vehicle(
    position=-4.0,
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

car3_controller = PIController(
    kp=5.0,
    ki=2.0,
    max_acceleration=car3.max_acceleration
)


# higher level controller used by car 2 to maintain spacing behind car 1
car2_follwing_controller = FollowingController(
    desired_distance=2.0,
    distance_gain=0.5
)

# higher level controller used by car 3 to maintain spacing behind car 2
car3_follwing_controller = FollowingController(
    desired_distance=2.0,
    distance_gain=0.5
)

# brake message logic
communication_enabled = True
message_delay = 0.1             # seconds per hop

car1_brake_sent = False

car2_receive_time = None
car3_receive_time = None

car2_braking = False
car3_braking = False

# lists to store info
history = []

steps = int(total_time / dt)

for i in range(steps):
    time = i * dt

    if communication_enabled:

        # car 1 sends once when its planned stop begins
        if time >= 3.0 and not car1_brake_sent:
            car1_brake_sent = True
            car2_receive_time = time + message_delay

            print(f"{time:.2f}s: Car 1 sends BRAKE to Car 2")

        # car 2 receives and immediately relays
        if (
            car2_receive_time is not None
            and time + 1e-9 >= car2_receive_time
            and not car2_braking
        ):
            car2_braking = True
            car3_receive_time = time + message_delay

            print(f"{time:.2f}s: Car 2 receives BRAKE, relays to Car 3")

        # car 3 receives the relay
        if (
            car3_receive_time is not None
            and time + 1e-9 >= car3_receive_time
            and not car3_braking
        ):
            car3_braking = True
        

            print(f"{time:.2f}s: Car 3 receives BRAKE")

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

    # car3 desired velocity based on car2
    desired_velocity_3, current_distance_23, distance_error_23 = (
        car3_follwing_controller.update(
            leader_position=car2.position,
            leader_velocity=car2.velocity,
            follower_position=car3.position
        )
    )

    # brake messages override normal following
    if car2_braking:
        desired_velocity_2 = 0.0

    if car3_braking:
        desired_velocity_3 = 0.0

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

    acceleration_command_3 = car3_controller.update(
        desired_velocity_3,
        car3.velocity,
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

    motor_acceleration_3, net_acceleration_3 = car3.update(
        acceleration_command_3,
        dt
    )

    history.append({
        "time": (i + 1) * dt,

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

        "car3": {
            "position": car3.position,
            "velocity": car3.velocity,
            "desired_velocity": desired_velocity_3,
            "acceleration_command": acceleration_command_3,
            "motor_acceleration": motor_acceleration_3,
            "net_acceleration": net_acceleration_3
        },

        "following": {
            "distance": car1.position - car2.position,
            "distance_error": (
                car1.position - car2.position
                - car2_follwing_controller.desired_distance
            )
        },

        "following_23": {
            "distance": car2.position - car3.position,
            "distance_error": (
                car2.position - car3.position
                - car3_follwing_controller.desired_distance
            )
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

car3_positions = [
    step["car3"]["position"]
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

car3_velocities = [
    step["car3"]["velocity"]
    for step in history
]

distances = [
    step["following"]["distance"]
    for step in history
]

distances_23 = [
    step["following_23"]["distance"]
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

plt.plot(
    times,
    car3_positions,
    label="Car 3")

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

plt.plot(
    times,
    car3_velocities,
    label="Car 3"
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
    label="Car 1 to Car 2"
)

plt.plot(
    times,
    distances_23,
    label="Car 2 to Car 3"
)

plt.axhline(
    car2_follwing_controller.desired_distance,
    linestyle="--",
    label="Desired Distance"
)

plt.xlabel("Time")
plt.ylabel("Distance (m)")
plt.title("Follwing Distances")
plt.legend()
plt.grid()

plt.show()