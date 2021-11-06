namespace pick_objects {

// Possible states of the robot
enum RobotState {
    FINISHED = 0,
    FAILED = 1,
    MOVING = 2
};
}  // namespace pick_objects


namespace add_markers {

// Possible states of the marker
enum MarkerState {
    FINISHED = 0,
    PICKUP = 1,
    DROPOFF = 2
};
}  // namespace add_markers
