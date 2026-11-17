import matplotlib.pyplot as plt

from simulation.vehicle import Vehicle
from control.velocity_controller import PIController
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from matplotlib.patches import FancyBboxPatch

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

# Positions defining entry and complete clearance
ENTRY_POSITION = -CROSSING_HALF_WIDTH
CLEAR_POSITION = CROSSING_HALF_WIDTH + VEHICLE_LENGTH

# stop settings for car2
STOP_POSITION = ENTRY_POSITION - 0.15
BRAKING_DECELERATION = 0.8  # m/s²
STOP_SPEED_THRESHOLD = 0.01  # m/s

def run_simulation(mode="v2v"):
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
        max_acceleration=car2.max_acceleration
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

    # predict clearance assuming car 1 maintains current speed
    car1_time_to_clear = (
        (CLEAR_POSITION - car1.position) / car1.velocity
    )

    car2_entry_time = car1_time_to_clear + CLEARANCE_TIME

    car2_distance_to_entry = ENTRY_POSITION - car2.position

    # v = d/t
    car2_average_approach_speed = (
        car2_distance_to_entry / car2_entry_time
    )

    print(f"Car 1 predicted clearance: {car1_time_to_clear:.2f} s")
    print(f"Car 2 earliest planned entry: {car2_entry_time:.2f} s")
    print(f"Car 2 required average speed: {car2_average_approach_speed:.3f} m/s")

    braking_started = False
    car2_has_stopped = False
    car2_released = False
    car1_clear_time = None

    history = []

    steps = int(TOTAL_TIME / DT)

    for i in range(steps):
        time = i * DT

        desired_velocity_1 = CRUISE_SPEED

        if car1.position >= CLEAR_POSITION and car1_clear_time is None:
            car1_clear_time = time

        if mode == "v2v":
            distance_remaining = ENTRY_POSITION - car2.position
            time_remaining = car2_entry_time - time

            if time_remaining > 0.0:
                desired_velocity_2 = max(
                    0.0,
                    distance_remaining / time_remaining
                )
            
                desired_velocity_2 = min(
                    desired_velocity_2,
                    CRUISE_SPEED
                )

            else:
                desired_velocity_2 = CRUISE_SPEED

        elif mode == "stop_yield":
        # stopping logic
            distance_to_stop = STOP_POSITION - car2.position

            # vf^2 = vi^2 + 2a * dx
            stopping_distance = (
                car2.velocity ** 2 / (2 * BRAKING_DECELERATION)
            )

            braking_buffer = 2 * car2.velocity * DT

            if distance_to_stop <= stopping_distance + braking_buffer:
                braking_started = True

            # braking commands
            if car2_has_stopped and car1_clear_time is not None:
                if time >= car1_clear_time + CLEARANCE_TIME:      
                    car2_released = True
            
            if car2_released:
                desired_velocity_2 = CRUISE_SPEED
            elif braking_started:
                desired_velocity_2 = 0.0
            else:
                desired_velocity_2 = CRUISE_SPEED

        else:
            raise ValueError(f"Unknown simulation mode: {mode}")
    
        # convert desired speed into accel commands
        acceleration_command_1 = car1_controller.update(
            desired_velocity_1,
            car1.velocity,
            DT
        )

        if mode == "stop_yield" and braking_started and not car2_released:
            if car2.velocity <= STOP_SPEED_THRESHOLD:
                car2.velocity = 0.0
                car2_has_stopped = True

                acceleration_command_2 = 0.0
                car2_controller.integral_error = 0.0
            else:
                acceleration_command_2 = (
                    -BRAKING_DECELERATION
                    + car2.drag_coefficient * car2.velocity
                )
        else:
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

    # check clearance time
    if car1_actual_clear is not None and car2_actual_entry is not None:
        actual_margin = car2_actual_entry - car1_actual_clear

        print(f"Car 1 actually clears: {car1_actual_clear:.3f} s")
        print(f"Car 2 actually enters: {car2_actual_entry:.3f} s")
        print(f"Actual clearance margin: {actual_margin:.3f} s")
        print(f"Requested margin: {CLEARANCE_TIME:.3f} s")
    else:
        print("One or both crossing events did not occur.")

    return history, car2_entry_time

# find the time both cars have fully cleared
def get_completion_time(history):
    for step in history:
        if (
            step["car1_position"] >= CLEAR_POSITION
            and step["car2_position"] >= CLEAR_POSITION
        ):
            return step["time"]
    return None

# find hwo long car2 is stopped
def get_stopped_time(history):
    stopped_time = 0.0

    for step in history:
        if step["car2_position"] >= CLEAR_POSITION:
            break

        if abs(step["car2_velocity"]) <= STOP_SPEED_THRESHOLD:
            stopped_time += DT

    return stopped_time

# check if cars are over the same crossing
def get_first_overlap_time(history):
    for step in history:
        car1_in_crossing = (
            ENTRY_POSITION <= step["car1_position"] < CLEAR_POSITION
        )

        car2_in_crossing = (
            ENTRY_POSITION <= step["car2_position"] < CLEAR_POSITION
        )

        if car1_in_crossing and car2_in_crossing:
            return step["time"]
        
    return None

# run simulations
print("\n--- V2V coordination ---")
history_v2v, entry_time_v2v = run_simulation(mode="v2v")

print("\n--- Stop and yield ---")
history_stop, entry_time_stop = run_simulation(mode="stop_yield")

# output overlap if any
overlap_v2v = get_first_overlap_time(history_v2v)
overlap_stop = get_first_overlap_time(history_stop)

print("\n--- Crossing occupancy check ---")

if overlap_v2v is None:
    print("V2V: no simultaneous crossing occupancy detected.")
else:
    print(f"V2V: crossing overlap at {overlap_v2v:.2f} s")

if overlap_stop is None:
    print("Stop and yield: no simultaneous crossing occupancy detected.")
else:
    print(f"Stop and yield: crossing overlap at {overlap_stop:.2f} s")

# output stopped time for each run
stopped_v2v = get_stopped_time(history_v2v)
stopped_stop = get_stopped_time(history_stop)

print("\n--- Car 2 stopped time ---")
print(f"V2V: {stopped_v2v:.2f} s")
print(f"Stop and yield: {stopped_stop:.2f} s")

# output
completion_v2v = get_completion_time(history_v2v)
completion_stop = get_completion_time(history_stop)

if completion_v2v is not None and completion_stop is not None:
    time_saved = completion_stop - completion_v2v

    print("\n--- Completion comparison ---")
    print(f"V2V: both cars clear at {completion_v2v:.2f} s")
    print(f"Stop and yield: both cars clear at {completion_stop:.2f} s")
    print(f"Time saved with V2V: {time_saved:.2f} s")
else:
    print("At least one scenario did not finish.")

# get car2 speed from each run
times_v2v = [step["time"] for step in history_v2v]
speeds_v2v = [step["car2_velocity"] for step in history_v2v]

times_stop = [step["time"] for step in history_stop]
speeds_stop = [step["car2_velocity"] for step in history_stop]

fig, ax = plt.subplots(figsize=(10, 5))

ax.plot(
    times_v2v,
    speeds_v2v,
    label="V2V: Coordinated Approach"
)

ax.plot(
    times_stop,
    speeds_stop,
    label="No V2V: Stop and Yield"
)

ax.set_title("Car 2 Speed: V2V vs. Stop-and-Yield")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Speed (m/s)")

ax.set_xlim(0, 5)
ax.set_ylim(-0.05, 1.1)

ax.legend()
ax.grid(alpha=0.3)

plt.tight_layout()

fig.savefig(
    "intersection_speed_comparison.png",
    dpi=300,
    bbox_inches="tight"
)

# animation
# Animation colors
BACKGROUND = "#ffffff"
ROAD = "#e2e4e7"
MARKINGS = "#9399a1"
TEXT = "#20252b"
MUTED = "#626b75"
BLUE = "#4da1c7"
CORAL = "#e56868"
GREEN = "#29865f"

animation_fig, animation_axes = plt.subplots(
    1, 2,
    figsize=(12, 6),
    facecolor=BACKGROUND
)

animation_fig.suptitle(
    "One intersection",
    color=TEXT,
    fontsize=22,
    fontweight="bold",
    y=0.97
)

histories = [history_v2v, history_stop]
titles = ["With V2V coordination", "Stop and yield"]
subtitles = [
    "Adjust speed early",
    "Stop before the crossing"
]

car_patches = []
time_labels = []

CAR_WIDTH = 0.12

for index, ax in enumerate(animation_axes):
    ax.set_facecolor(BACKGROUND)
    ax.set_xlim(-2.3, 1.3)
    ax.set_ylim(-2.3, 1.3)
    ax.set_aspect("equal")
    ax.axis("off")

    # Rounded road strips
    for x, y, width, height in [
        (-2.3, -0.30, 3.6, 0.60),
        (-0.30, -2.3, 0.60, 3.6),
    ]:
        road = FancyBboxPatch(
            (x, y), width, height,
            boxstyle="round,pad=0,rounding_size=0.04",
            facecolor=ROAD,
            edgecolor="none",
            zorder=1
        )
        ax.add_patch(road)

    # Center markings, interrupted at the crossing
    for start, end in [(-2.3, -0.30), (0.30, 1.3)]:
        ax.plot(
            [start, end], [0, 0],
            color=MARKINGS,
            linewidth=1.5,
            linestyle=(0, (5, 6)),
            zorder=2
        )
        ax.plot(
            [0, 0], [start, end],
            color=MARKINGS,
            linewidth=1.5,
            linestyle=(0, (5, 6)),
            zorder=2
        )

    # Subtle highlight of the shared crossing
    ax.add_patch(Rectangle(
        (-0.30, -0.30), 0.60, 0.60,
        facecolor="#64748b",
        edgecolor="none",
        alpha=0.07,
        zorder=2
    ))

    # Stop line belongs only to the stop-and-yield scenario
    if index == 1:
        ax.plot(
            [-0.27, 0.27],
            [STOP_POSITION, STOP_POSITION],
            color=CORAL,
            linewidth=3,
            solid_capstyle="butt",
            zorder=3
        )

    car1_patch = FancyBboxPatch(
        (-2.0 - VEHICLE_LENGTH, -CAR_WIDTH / 2),
        VEHICLE_LENGTH, CAR_WIDTH,
        boxstyle="round,pad=0,rounding_size=0.035",
        facecolor=BLUE,
        edgecolor="none",
        zorder=5
    )

    car2_patch = FancyBboxPatch(
        (-CAR_WIDTH / 2, -2.0 - VEHICLE_LENGTH),
        CAR_WIDTH, VEHICLE_LENGTH,
        boxstyle="round,pad=0,rounding_size=0.035",
        facecolor=CORAL,
        edgecolor="none",
        zorder=5
    )

    ax.add_patch(car1_patch)
    ax.add_patch(car2_patch)
    car_patches.append((car1_patch, car2_patch))

    time_labels.append(ax.text(
        0.02, 0.96, "",
        transform=ax.transAxes,
        color=MUTED,
        fontsize=11,
        va="top"
    ))

    ax.text(
        0.0, -0.05, titles[index],
        transform=ax.transAxes,
        color=TEXT,
        fontsize=17,
        fontweight="bold"
    )

    ax.text(
        0.0, -0.12, subtitles[index],
        transform=ax.transAxes,
        color=MUTED,
        fontsize=11
    )


def update_animation(frame):
    for index, history in enumerate(histories):
        step = history[frame]
        car1_patch, car2_patch = car_patches[index]

        car1_patch.set_x(
            step["car1_position"] - VEHICLE_LENGTH
        )
        car2_patch.set_y(
            step["car2_position"] - VEHICLE_LENGTH
        )

        time_labels[index].set_text(
            f'Time  {step["time"]:.2f} s\n'
            f'Car 2  {step["car2_velocity"]:.2f} m/s'
        )

        position = step["car2_position"]
        speed = step["car2_velocity"]

        if position >= CLEAR_POSITION:
            status = "Crossing complete"
        elif position >= ENTRY_POSITION:
            status = "Crossing"
        elif speed <= STOP_SPEED_THRESHOLD:
            status = "Stopped · waiting"
        elif index == 0:
            status = "Coordinating approach"
        else:
            status = "Approaching" if speed > 0.99 else "Moving to crossing"

frame_indices = range(
    0,
    min(len(history_v2v), len(history_stop), int(5.0 / DT)),
    5
)

animation = FuncAnimation(
    animation_fig,
    update_animation,
    frames=frame_indices,
    interval=5 * DT * 1000,
    repeat=True,
    repeat_delay=1000
)

animation_fig.text(
    0.5, 0.025,
    "Blue: Car 1 · Coral: Car 2   |   Ideal two-car simulation",
    ha="center",
    color=MUTED,
    fontsize=10
)

animation_fig.subplots_adjust(
    left=0.04, right=0.96,
    top=0.86, bottom=0.19,
    wspace=0.12
)

animation.save(
    "intersection_comparison.gif",
    writer="pillow",
    fps=20,
    dpi=120
)

plt.show()