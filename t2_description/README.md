# t2_description

Static model resources for the Booster T2 with Revo2 hands.

- `robot/` retains the upstream Booster model and meshes.
- `urdf/t2.urdf.xacro` is the project model consumed by RViz and MoveIt.

The model is supplied directly to MoveIt as its `robot_description` parameter.
This package publishes no ROS topics. Live joint state and TF remain owned by
`t2_robot_side`.
