import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, SetParameter
from launch.substitutions import Command

def generate_launch_description():
  urdf_file = os.path.join(
    get_package_share_directory('simple_arm_camera'), 'urdf', 'simple.xacro'
  )
  semantic_file = os.path.join(
    get_package_share_directory('simple_arm_camera'), 'config', 'simple_arm.srdf'
  )
  description = Command(['xacro ', urdf_file])
  robot_description = {'robot_description': description}

  robot_description_semantic = {'robot_description_semantic': Command(['cat ', semantic_file])}

  return LaunchDescription([
    SetParameter(name='use_sim_time', value=True),
    Node(
      name='move_arm',
      package='simple_arm_camera',
      executable='move_arm',
      output='screen',
      parameters=[
        robot_description,
        robot_description_semantic,
      ]
    )
  ])