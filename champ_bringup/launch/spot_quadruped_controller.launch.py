# Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import yaml
import tempfile

def generate_launch_description():

  # Get here directories of packages
  champ_bringup_share_dir = get_package_share_directory('champ_bringup')

  launch_args = [

    DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='If true, use simulated clock'),
    DeclareLaunchArgument(
        'champ_params',
        default_value=os.path.join(champ_bringup_share_dir, 'config', 'champ_params_spot_arm.yaml'),
        description='Path to CHAMP params (will be prefixed at runtime if tf_prefix is set).'),
    DeclareLaunchArgument(
        'tf_prefix',
        default_value='',
        description='TF frame and joint name prefix used by the robot model (e.g., spot_BD_42910021/).'
    ),
    DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(
            champ_bringup_share_dir, 'config', 'robot_localization_params.yaml'),
        description='Path to the vox_nav parameters file.')
  ]
  
  use_sim_time = LaunchConfiguration('use_sim_time', default=True)
  champ_params = LaunchConfiguration('champ_params')  
  tf_prefix = LaunchConfiguration('tf_prefix')

  # Build a temporary, prefixed CHAMP params file when tf_prefix is provided
  def build_prefixed_params(context):
    src_path = champ_params.perform(context)
    prefix = tf_prefix.perform(context) or ''
    if not prefix:
      return src_path

    try:
      with open(src_path, 'r') as f:
        cfg = yaml.safe_load(f)
    except Exception:
      return src_path

    data = cfg or {}
    # ROS 2 global namespace key is '/**' (no trailing colon in the key)
    params = data.get('/**', {}).get('ros__parameters', {})

    # Frames and base link
    params['odom_frame'] = f"{prefix}odom"
    # Some CHAMP configs use 'base' or 'base_link_frame' to identify base
    params['base'] = f"{prefix}base_link"
    params['base_link_frame'] = f"{prefix}base_link"

    # Helper to prefix list entries
    def _prefix_list(lst):
      if isinstance(lst, list):
        return [f"{prefix}{x}" for x in lst]
      return lst

    # Prefix link and joint name lists if present
    for key in [
      'left_front_links', 'left_hind_links', 'right_front_links', 'right_hind_links',
      'left_front_joints', 'left_hind_joints', 'right_front_joints', 'right_hind_joints']:
      if key in params:
        params[key] = _prefix_list(params[key])

    # Write out a temp YAML with same structure
    tmpf = tempfile.NamedTemporaryFile(prefix='champ_params_', suffix='.yaml', delete=False)
    data.setdefault('/**', {})['ros__parameters'] = params
    with open(tmpf.name, 'w') as f:
      yaml.safe_dump(data, f)
    return tmpf.name

  # Build the node after resolving launch configurations using OpaqueFunction
  def create_quadruped_node(context):
    pref_path = build_prefixed_params(context)
    node = Node(
      package='champ_base',
      executable='quadruped_controller',
      name='quadruped_controller',
      output='screen',
      arguments=['--ros-args', '--log-level', 'INFO'],
      parameters=[
        {"use_sim_time": use_sim_time},
        pref_path
      ],
      remappings=[('cmd_vel', 'vox_nav/cmd_vel')]
    )
    return [node]
    
  return LaunchDescription(
    launch_args + 
    [
      OpaqueFunction(function=create_quadruped_node),
      #declare_state_estimation_node,
      #declare_rviz_launch_include,
      #declare_localization_params,
      #footprint_to_odom_ekf,
      #base_to_footprint_ekf,
      #broadcast_left_front_cam,
      #broadcast_right_front_cam
      #joint_state_publisher_gui  # added by diya 9/6      
    ])
