from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from os import environ
import os
import xacro
import yaml
import tempfile

def evaluate_nodes(context, *args, **kwargs):

    urdf_path = LaunchConfiguration("urdf_file").perform(context)
    mapping_dict = LaunchConfiguration("urdf_mapping").perform(context)
    mapping_yaml = yaml.safe_load(mapping_dict)
    # Resolve tf prefix (to match real Spot frame naming like "spot_<ID>/...")
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    # Pass prefix down to xacro if supported by the robot description
    # Many xacros accept a parameter named "prefix" to preprend to link/joint (frame) names
    # If the xacro ignores it, the rest of this launch still provides prefixed frame IDs
    # for auxiliary publishers below.
    if tf_prefix is None:
        tf_prefix = ""
    # Ensure mapping exists and add prefix without guessing trailing separators
    # The provided value should include any desired trailing slash (e.g., "spot_BD_42910021/")
    mapping_yaml = mapping_yaml or {}
    # spot_description expects 'tf_prefix' (not 'prefix')
    mapping_yaml.setdefault('tf_prefix', tf_prefix)

    # Generate a ros2_control YAML with prefixed joint names so controllers match the URDF when tf_prefix is used
    # Default template path from spot_description
    controllers_template = os.path.join(get_package_share_directory('spot_description'), 'config', 'ros2_control_spot.yaml')
    try:
        with open(controllers_template, 'r') as f:
            ctrl_cfg = yaml.safe_load(f)
    except Exception as e:
        ctrl_cfg = None

    def _prefix_joints_in_controller(ctrl_section: dict, joints_key='joints', gains_key='gains'):
        if not ctrl_section or 'ros__parameters' not in ctrl_section:
            return
        params = ctrl_section['ros__parameters']
        # Prefix joints list
        if joints_key in params and isinstance(params[joints_key], list):
            params[joints_key] = [f"{tf_prefix}{j}" for j in params[joints_key]]
        # Prefix gains map keys
        if gains_key in params and isinstance(params[gains_key], dict):
            params[gains_key] = {f"{tf_prefix}{k}": v for k, v in params[gains_key].items()}

    if ctrl_cfg and tf_prefix:
        # Update known controllers if present
        for key in ['joint_trajectory_controller', 'arm_trajectory_controller', 'gripper_trajectory_controller']:
            _prefix_joints_in_controller(ctrl_cfg.get(key, {}))

        # Write to a temp file
        tmp_yaml = tempfile.NamedTemporaryFile(prefix='spot_ros2_control_', suffix='.yaml', delete=False)
        with open(tmp_yaml.name, 'w') as f:
            yaml.safe_dump(ctrl_cfg, f)
        mapping_yaml['controllers_config'] = tmp_yaml.name
    else:
        # Fallback to default template if no prefix or failed to load
        mapping_yaml['controllers_config'] = controllers_template

    robot_description = xacro.process_file(
       urdf_path, mappings=mapping_yaml).toprettyxml(indent="  ")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")},
            {'robot_description': robot_description}],
        remappings=[('/tf', 'tf'),
            ('/tf_static', 'tf_static')],
        output='screen'
    )

    # Compose commonly used prefixed frame names
    robot_base_link = LaunchConfiguration("robot_base_link").perform(context)
    robot_base_link_prefixed = f"{tf_prefix}{robot_base_link}"
    odom_prefixed = f"{tf_prefix}odom"

    # Fix up the robot's ground truth to publish w.r.t. prefixed odom (to match real Spot)
    ground_truth_node = Node(
        package="champ_gazebo",
        executable="republish_ground_truth",
        name="republish_ground_truth",
        output="screen",
        parameters=[{"robot_pose_topic": "/model/spot/pose", "fixed_frame": odom_prefixed, "robot_frame": robot_base_link_prefixed}]
    )

    # Camera TFs: create identity edges from camera_* frames to "spot/camera_.../*_depth"
    # Prefix the parent frame IDs so the camera frames match real Spot naming when desired.
    broadcast_left_front_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", f"{tf_prefix}camera_frontleft", "--child-frame-id", f"{tf_prefix}camera/frontleft/frontleft_depth"],
        output='screen'
    )

    broadcast_right_front_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", f"{tf_prefix}camera_frontright", "--child-frame-id", f"{tf_prefix}camera/frontright/frontright_depth"],
        output='screen'
    )

    broadcast_left_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", f"{tf_prefix}camera_left", "--child-frame-id", f"{tf_prefix}camera/left/left_depth"],
        output='screen'
    )

    broadcast_right_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", f"{tf_prefix}camera_right", "--child-frame-id", f"{tf_prefix}camera/right/right_depth"],
        output='screen'
    )

    broadcast_back_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", f"{tf_prefix}camera_back", "--child-frame-id", f"{tf_prefix}camera/back/back_depth"],
        output='screen'
    )

    odom_republish_node = Node(
        package='champ_gazebo',
        executable='helper_publish_base_pose',
        output='screen',
        parameters=[{"fixed_frame": odom_prefixed, "robot_frame": robot_base_link_prefixed}]
    )

    return [
        robot_state_publisher_node,
        ground_truth_node,
        odom_republish_node,
        broadcast_left_front_cam,
        broadcast_right_front_cam,
        broadcast_left_cam,
        broadcast_right_cam,
        broadcast_back_cam,
    ]

##################################################################
def generate_launch_description():

  launch_args = [
    DeclareLaunchArgument("urdf_file",
        default_value=os.path.join( get_package_share_directory('spot_description'), 'urdf/spot.urdf.xacro')),
    DeclareLaunchArgument("urdf_mapping",
        default_value="{'arm': 'True', 'add_ros2_control_tag': 'True', 'hardware_interface_type': 'gazebo', 'simulate_cameras': 'True', 'feet': 'True'}"),
    DeclareLaunchArgument("robot_base_link", default_value="body"),
    DeclareLaunchArgument(
        'x',
        default_value='5.0',
        description='X at which to spawn Spot. 0.84 for ground plane, 0 for marsyard is good'),
    DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='y at which to spawn Spot.'),
    DeclareLaunchArgument(
        'z',
        default_value='0.84',
        description='Height at which to spawn Spot.'),
    DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='Roll at which to spawn Spot.'),
    DeclareLaunchArgument(
        'yaw',
        default_value='-0.5',
        description='Yaw at which to spawn Spot.'),
    DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='whether to use Gazebo Simulation.'),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='If true, use simulated clock'),
    DeclareLaunchArgument(
        'tf_prefix',
        default_value='',
        description='...'),
    DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(
            get_package_share_directory('champ_bringup'), 'config', 'robot_localization_params.yaml'),
        description='Path to the vox_nav parameters file.'),
    DeclareLaunchArgument(
        'start_quadruped_controller', default_value='False')
  ]

  use_simulator = LaunchConfiguration('use_simulator')
  use_sim_time = LaunchConfiguration('use_sim_time', default=True)
  tf_prefix = LaunchConfiguration('tf_prefix')

  # Robot publisher
  nodes_eval = OpaqueFunction(function=evaluate_nodes)

  # Bridge
  bridge_config_file = os.path.join(get_package_share_directory('champ_bringup'), 'config', "spot_bridge.yaml")

  bridge = Node(
      package='ros_gz_bridge',
      executable='parameter_bridge',
      parameters=[{'config_file': bridge_config_file}],
      additional_env={'GZ_VERSION': '8'},  # Use Gazebo Harmonic
      #arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
      output='screen'
  )

  # Spawn the robot in Gazebo (Harmonic)
  spawn_entity_to_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'spot',
                   '-topic', '/robot_description',
                    "-x", LaunchConfiguration("x"),
                    "-y", LaunchConfiguration("y"),
                   "-z", LaunchConfiguration("z"),
                   "-R", LaunchConfiguration("roll"),
                   "-Y", LaunchConfiguration("yaw"),
                  ],
        parameters=[{"use_sim_time": use_sim_time}],
        additional_env={'GZ_VERSION': '8'},  # Use Gazebo Harmonic
        output='screen',
        condition=IfCondition(use_simulator)
  )

  # Fix up the robot's ground truth to publish w.r.t. world
  # by default it is published w.r.t. the world's name, rather than "world"
  # moved into evaluate_nodes to apply tf_prefix consistently


  load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--switch-timeout", "100.0"],
        name="start_joint_state_broadcaster",
        output='screen'
  )


  load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--switch-timeout", "100.0"],
        name="start_joint_trajectory_controller",
        output='screen',
  )

  load_arm_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "-c", "/controller_manager", "--switch-timeout", "100.0"],
        name="start_arm_trajectory_controller",
        output='screen',
  )

  load_gripper_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_trajectory_controller", "-c", "/controller_manager", "--switch-timeout", "100.0"],
        name="start_gripper_trajectory_controller",
        output='screen',
  )


  # camera TFs and odom republisher moved into evaluate_nodes to apply tf_prefix consistently

  # Start quadruped controller
  launch_quadruped_controller = IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('champ_bringup'), 'launch', 'spot_quadruped_controller.launch.py']),
      launch_arguments={
        'tf_prefix': LaunchConfiguration('tf_prefix')
      }.items()
  )

  return LaunchDescription(
    launch_args +
    [
      SetParameter(name='use_sim_time', value=True),
      nodes_eval,
      bridge,
      spawn_entity_to_gazebo_node,
      # ground_truth_node is created inside nodes_eval to use prefixed frames
      RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=spawn_entity_to_gazebo_node,
              on_exit=[load_joint_state_controller],
          )
      ),
      RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=load_joint_state_controller,
              on_exit=[load_joint_trajectory_controller, load_arm_trajectory_controller, load_gripper_trajectory_controller],
          )
      ),
      RegisterEventHandler(
          event_handler=OnProcessExit(
              target_action=load_joint_trajectory_controller,
              on_exit=[launch_quadruped_controller],
          ),
          condition=IfCondition(LaunchConfiguration("start_quadruped_controller"))
      ),
      # camera broadcasters and odom republisher are created inside nodes_eval to use prefixed frames
    ])
