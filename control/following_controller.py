class FollowingController:
    def __init__(self, desired_distance, distance_gain):
        # desired distance between follower and leader
        self.desired_distance = desired_distance
        # how strongly the follower reacts to spacing error
        self.distance_gain = distance_gain

    def update(
        self,
        leader_position,
        leader_velocity,
        follower_position
    ):
        
        # current distance between the two vehicles
        current_distance = leader_position - follower_position

        # positive error -> follower is too far away
        # negative error -> follower is too close
        distance_error = current_distance - self.desired_distance

        # start w leader velocity as baseline
        # then correct it bases on error
        desired_velocity = (
            leader_velocity
            + self.distance_gain * distance_error
        )

        # no reverse motion
        desired_velocity = max(0.0, desired_velocity)

        return desired_velocity, current_distance, distance_error