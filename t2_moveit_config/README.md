# t2_moveit_config

Initial MoveIt 2 configuration for plan-only validation of the T2 arms and
Revo2 hands.

The launch file consumes `/t2/joint_states`, `/t2/tf`, and `/t2/tf_static`.
Controller names describe the future `FollowJointTrajectory` robot-side
bridges. This package does not implement those action servers.

Simulation currently publishes only the 31 body joints. Before planning with
the Revo2 groups, its 12 active hand states must also be merged into
`/t2/joint_states`. Arm-only planning can be validated first.

```bash
colcon build --packages-select t2_description t2_moveit_config
source install/setup.bash
ros2 launch t2_moveit_config moveit.launch.py
```
