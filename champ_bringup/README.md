# champ_bringup

Bringup and spawn utilities for Spot in Ignition Gazebo (ros_gz_sim) using the unified `spot_description`.

## TF prefix (real-robot parity)

To match the real Spot frame tree, you can pass a `tf_prefix` that will be applied consistently:

- Forwarded to the Spot xacros in `spot_description` (as `tf_prefix`)
- Used for all auxiliary static TF publishers created by this bringup
- Used to generate a temporary, prefixed ros2_control YAML so controller joint names match the prefixed model

Example:

```bash
ros2 launch champ_bringup spot_spawn.launch.py tf_prefix:=spot_BD_42910021/
```

Notes:

- Include the trailing slash if you want one (e.g. `spot_BD_42910021/`).
- Experiments in `experiments_spot_gazebo_sim` default this prefix so the same code runs on sim and the real robot.
- You can omit the prefix (`tf_prefix:=''`) to use unprefixed frames.

## Key launch arguments

- `urdf_file`: Path to Spot's xacro (defaults to `spot_description/urdf/spot.urdf.xacro`).
- `urdf_mapping`: Xacro parameters (e.g., `arm:=True`, `add_ros2_control_tag:=True`, `simulate_cameras:=True`).
- `tf_prefix`: Frame prefix for all Spot frames (default: empty).
- `use_sim_time`: Use simulated clock (default: `True`).
- `start_quadruped_controller`: Start CHAMP quadruped controller after controllers activate.

## Controllers

`spot_spawn.launch.py` loads:

- `joint_state_broadcaster`
- `joint_trajectory_controller`
- `arm_trajectory_controller`
- `gripper_trajectory_controller`

When `tf_prefix` is set, a temporary controller config file is generated with prefixed joint names and handed to the `gz_ros2_control` plugin via the URDF to ensure controller activation.
