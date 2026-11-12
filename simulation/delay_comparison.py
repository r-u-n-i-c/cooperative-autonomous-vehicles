import matplotlib.pyplot as plt

from simulation.vehicle import Vehicle
from control.following_controller import FollowingController
from control.velocity_controller import PIController


DT = 0.001
TOTAL_TIME = 8.0
BRAKE_TIME = 3.0

CRUISE_SPEED = 2.0
VEHICLE_LENGTH = 0.28

GAPS = [0.30, 0.50, 1.00]

CASES = [
    ("Messages off", False, 0.0),
    ("5 ms", True, 0.005),
    ("10 ms", True, 0.010),
    ("20 ms", True, 0.020),
    ("50 ms", True, 0.050),
]


def run_simulation(clear_gap, communication_enabled, message_delay):
    # Positions use the same reference point on every car.
    # Reference-point spacing = car length + clear bumper gap.
    spacing = VEHICLE_LENGTH + clear_gap

    cars = [
        Vehicle(
            position=-i * spacing,
            velocity=CRUISE_SPEED,
            max_acceleration=2.0,
            drag_coefficient=0.5
        )
        for i in range(3)
    ]

    controllers = [
        PIController(
            kp=5.0,
            ki=2.0,
            max_acceleration=car.max_acceleration
        )
        for car in cars
    ]

    # Start in steady cruising equilibrium:
    # PI output must balance drag at the initial speed.
    for car, controller in zip(cars, controllers):
        controller.integral_error = (
            car.drag_coefficient * CRUISE_SPEED / controller.ki
        )

    # Existing FollowingController subtracts reference positions.
    # Therefore give it the length + desired clear gap.
    followers = [
        FollowingController(
            desired_distance=spacing,
            distance_gain=0.5
        )
        for _ in range(2)
    ]

    stop_requested = [False, False, False]
    receive_at = [None, None, None]

    braking_times = [None, None, None]
    first_contact_time = None

    history = [{
        "time": 0.0,
        "speeds": [car.velocity for car in cars],
        "gaps": [clear_gap, clear_gap]
    }]

    for i in range(round(TOTAL_TIME / DT)):
        time = i * DT

        # Car 1 initiates the stop.
        if time + 1e-9 >= BRAKE_TIME and not stop_requested[0]:
            stop_requested[0] = True

            if communication_enabled:
                receive_at[1] = time + message_delay

        # Car 2 receives and immediately relays.
        if (
            communication_enabled
            and receive_at[1] is not None
            and time + 1e-9 >= receive_at[1]
            and not stop_requested[1]
        ):
            stop_requested[1] = True
            receive_at[2] = time + message_delay

        # Car 3 receives Car 2's relay.
        if (
            communication_enabled
            and receive_at[2] is not None
            and time + 1e-9 >= receive_at[2]
            and not stop_requested[2]
        ):
            stop_requested[2] = True

        desired_speeds = [
            0.0 if stop_requested[0] else CRUISE_SPEED
        ]

        # Compute both followers' targets before updating any car.
        for follower_index in (1, 2):
            leader = cars[follower_index - 1]
            follower = cars[follower_index]

            desired_speed, _, _ = followers[follower_index - 1].update(
                leader_position=leader.position,
                leader_velocity=leader.velocity,
                follower_position=follower.position
            )

            if stop_requested[follower_index]:
                desired_speed = 0.0

            desired_speeds.append(desired_speed)

        commands = [
            controller.update(target, car.velocity, DT)
            for controller, target, car in zip(
                controllers, desired_speeds, cars
            )
        ]

        for index, (car, command) in enumerate(zip(cars, commands)):
            _, net_acceleration = car.update(command, DT)

            # First deceleration interval after the leader's stop.
            if (
                time + 1e-9 >= BRAKE_TIME
                and braking_times[index] is None
                and net_acceleration < -1e-6
            ):
                braking_times[index] = time

        gaps = [
            cars[0].position - cars[1].position - VEHICLE_LENGTH,
            cars[1].position - cars[2].position - VEHICLE_LENGTH
        ]

        sample_time = (i + 1) * DT

        history.append({
            "time": sample_time,
            "speeds": [car.velocity for car in cars],
            "gaps": gaps
        })

        # Stop at first sampled contact: no collision physics modeled.
        if min(gaps) <= 0.0:
            first_contact_time = sample_time
            break

    braking_history = [
        sample for sample in history
        if sample["time"] >= BRAKE_TIME - 1e-9
    ]

    minimum_gaps = [
        min(sample["gaps"][pair] for sample in braking_history)
        for pair in (0, 1)
    ]

    return {
        "history": history,
        "minimum_gaps": minimum_gaps,
        "braking_times": braking_times,
        "contact_time": first_contact_time
    }


def format_time(value):
    return "N/A" if value is None else f"{value:.3f}"


if __name__ == "__main__":
    fig, axes = plt.subplots(
        len(GAPS), 2,
        figsize=(12, 10),
        sharex=True
    )

    for row, clear_gap in enumerate(GAPS):
        print(f"\nStarting clear gap: {clear_gap:.2f} m")
        print(
            f"{'Case':<15}"
            f"{'Min gap 1-2':>14}"
            f"{'Min gap 2-3':>14}"
            f"{'B slows (s)':>14}"
            f"{'C slows (s)':>14}"
            f"{'Contact (s)':>14}"
        )

        for label, enabled, delay in CASES:
            result = run_simulation(clear_gap, enabled, delay)

            history = result["history"]
            gap_12, gap_23 = result["minimum_gaps"]
            _, brake_2, brake_3 = result["braking_times"]
            contact = result["contact_time"]

            print(
                f"{label:<15}"
                f"{gap_12:>14.3f}"
                f"{gap_23:>14.3f}"
                f"{format_time(brake_2):>14}"
                f"{format_time(brake_3):>14}"
                f"{'None' if contact is None else format_time(contact):>14}"
            )

            # Plot time relative to the leader's stop request.
            for pair in (0, 1):
                axes[row, pair].plot(
                    [sample["time"] - BRAKE_TIME for sample in history],
                    [sample["gaps"][pair] for sample in history],
                    label=label
                )

        for pair in (0, 1):
            ax = axes[row, pair]

            ax.axhline(
                clear_gap,
                color="gray",
                linestyle=":",
                label="Starting gap"
            )
            ax.axhline(
                0.0,
                color="black",
                linestyle="--",
                label="Contact"
            )
            ax.axvline(0.0, color="gray", linestyle="--")

            ax.set_title(
                f"Car {pair + 1} to Car {pair + 2}"
                f" — starting gap {clear_gap:.2f} m"
            )
            ax.set_ylabel("Bumper clearance (m)")
            ax.set_xlabel("Time since leader stop request (s)")
            ax.set_xlim(-0.2, TOTAL_TIME - BRAKE_TIME)
            ax.grid()
            ax.legend(fontsize=8)

    fig.suptitle(
        f"Gap and brake-message delay comparison at {CRUISE_SPEED:.1f} m/s"
    )
    fig.tight_layout()
    plt.show()