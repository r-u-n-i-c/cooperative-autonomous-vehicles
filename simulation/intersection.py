import matplotlib.pyplot as plt

from simulation.vehicle import Vehicle
from control.velocity_controller import PIController

# simulation settings
DT = 0.01
TOTAL_TIME = 12.0

CRUISE_SPEED = 1.0      # m/s
VEHICLE_LENGTH = 0.28   # meters

# crossing area extends from -0.30 to +0.30
# along each cars direction of travel
CROSSING_HALF_WIDTH = 0.30

# extra time between one car clearing and the next entering
CLEARANCE_TIME = 0.5

# each position represents the FRONT of the car.
# negative position = approaching the intersection.
# zero = intersection center.
# positive position = moving beyond the center.
car1 = Vehicle(
    position=-2.0,
    velocity=CRUISE_SPEED,
    max_acceleration=2.0,
    drag_coefficient=0.5
)

car2 = Vehicle(
    position=-2.0,
    velocity=CRUISE_SPEED,
    max_acceleration=2.0,
    drag_coefficient=0.5
)

car1_controller = PIController(
    kp=5.0,
    ki=2.0,
    max_acceleration=car1.max_acceleration
)

car2_controller = PIController(
    kp=5.0,
    ki=2.0,
    max_acceleration=car1.max_acceleration
)

# init controllers to balance drag while cruising
# this is so the car start traveling immediately upon sim start
for car, controller in [
    (car1, car1_controller),
    (car2, car2_controller),
]:
    controller.integral_error = (
        car.drag_coefficient * CRUISE_SPEED / controller.ki
    )

# Positions defining entry and complete clearance
ENTRY_POSITION = -CROSSING_HALF_WIDTH
CLEAR_POSITION = CROSSING_HALF_WIDTH + VEHICLE_LENGTH

# predict clearance assuming car 1 maintains current speed
car1_time_to_clear = (
    (CLEAR_POSITION - car1.position) / car1.velocity
)

# earliest planned entry time for car2, measured from t=0
car2_entry_time = car1_time_to_clear + CLEARANCE_TIME

# calc the approach speed for car2
car2_distance_to_entry = ENTRY_POSITION - car2.position

car2_average_approach_speed = (
    car2_distance_to_entry / car2_entry_time
)

print(f"Car 1 predicted clearance: {car1_time_to_clear:.2f} s")
print(f"Car 2 earliest planned entry: {car2_entry_time:.2f} s")
print(f"Car 2 required average speed: {car2_average_approach_speed:.3f} m/s")

history = []

steps = int(TOTAL_TIME / DT)

for i in range(steps):
    time  = i * DT

    # car 1 has priority and keep cruising
    desired_velocity_1 = CRUISE_SPEED

    # car 2 adjusts its approach speed
    distance_remaining = ENTRY_POSITION - car2.position
    time_remaining = car2_entry_time - time

    if time_remaining > 0.0:
        desired_velocity_2 = (
            max(0.0, distance_remaining / time_remaining)
        )

        # dont exceed cruising speed
        desired_velocity_2 = min(
            desired_velocity_2,
            CRUISE_SPEED
        )
    else:
        desired_velocity_2 = CRUISE_SPEED

    # convert desired speed into accel commands
    acceleration_command_1 = car1_controller.update(
        desired_velocity_1,
        car1.velocity,
        DT
    )

    acceleration_command_2 = car2_controller.update(
        desired_velocity_2,
        car2.velocity,
        DT
    )

    # move forward one simulation step
    car1.update(acceleration_command_1, DT)
    car2.update(acceleration_command_2, DT)

    # save the updated states
    history.append({
        "time": (i + 1) * DT,
        "car1_position": car1.position,
        "car2_position": car2.position,
        "car1_velocity": car1.velocity,
        "car2_velocity": car2.velocity,
        "car1_desired_velocity": desired_velocity_1,
        "car2_desired_velocity": desired_velocity_2,
    })

car1_actual_clear = next(
    (
        step["time"] for step in history
        if step["car1_position"] >= CLEAR_POSITION
    ),
    None
)

car2_actual_entry = next(
    (
        step["time"] for step in history
        if step["car2_position"] >= ENTRY_POSITION
    ),
    None
)

if car1_actual_clear is not None and car2_actual_entry is not None:
    actual_margin = car2_actual_entry - car1_actual_clear

    print(f"Car 1 actually clears: {car1_actual_clear:.3f} s")
    print(f"Car 2 actually enters: {car2_actual_entry:.3f} s")
    print(f"Actual clearance margin: {actual_margin:.3f} s")
    print(f"Requested margin: {CLEARANCE_TIME:.3f} s")
else:
    print("One or both crossing events did not occur.")

# extract data
times = [step["time"] for step in history]

positions_1 = [step["car1_position"] for step in history]
positions_2 = [step["car2_position"] for step in history]

velocities_1 = [step["car1_velocity"] for step in history]
velocities_2 = [step["car2_velocity"] for step in history]

targets_2 = [
    step["car2_desired_velocity"]
    for step in history
]

fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# speed plot
axes[0].plot(times, velocities_1, label="Car 1 actual")
axes[0].plot(times, velocities_2, label="Car 2 actual")
axes[0].plot(
    times, targets_2,
    linestyle="--",
    label="Car 2 target"
)

axes[0].set_ylabel("Speed (m/s)")
axes[0].set_title("Intersection Approach Speeds")

# position plot
axes[1].plot(times, positions_1, label="Car 1 front")
axes[1].plot(times, positions_2, label="Car 2 front")

axes[1].axhspan(
    ENTRY_POSITION,
    CLEAR_POSITION,
    color="gray",
    alpha=0.15,
    label="vehicle occupies crossing"
)

axes[1].axhline(
    ENTRY_POSITION,
    color="gray",
    linestyle=":",
    label="Front enters crossing"
)

axes[1].axhline(
    CLEAR_POSITION,
    color="black",
    linestyle=":",
    label="Rear clears crossing"
)

axes[1].set_ylabel("Front position along own road (m)")
axes[1].set_xlabel("Time (s)")
axes[1].set_title("Crossing Entry and Clearance")

for ax in axes:
    ax.axvline(
        car2_entry_time,
        color="purple",
        linestyle="--",
        label="Car 2 planned entry"
    )
    ax.grid()
    ax.legend()

# focus on the intersection area
axes[1].set_xlim(0, 5)
axes[1].set_ylim(-2.1, 1.2)

plt.tight_layout()
plt.show()