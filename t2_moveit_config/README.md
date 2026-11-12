# t2_moveit_config

MoveIt 2 planning and execution configuration for the two seven-DOF T2 arms.
The Revo2 hands remain in the robot model for collision checking, but their
finger joints are controlled separately from MoveIt.

The launch file consumes `/t2/joint_states`, `/t2/tf`, and `/t2/tf_static`.
The configured controllers expect robot-side `FollowJointTrajectory` action
servers for the left and right arms.

Accurate hand joint states must still be merged into `/t2/joint_states` because
MoveIt uses the hand geometry while checking arm collisions.

```bash
colcon build --packages-select t2_description t2_moveit_config
source install/setup.bash
ros2 launch t2_moveit_config moveit.launch.py
```
