import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, SetParameter
from launch.substitutions import Command
import yaml

def load_yaml(package_name, file_path):
  package_path = get_package_share_directory(package_name)
  absolute_file_path = os.path.join(package_path, file_path)

  try:
    with open(absolute_file_path, 'r') as file:
      return yaml.safe_load(file)
  except EnvironmentError:
    return None

def generate_launch_description():
  env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH']}
  world_file = os.path.join(
    get_package_share_directory('simple_arm_camera'), 'worlds', 'world.sdf'
  )
  urdf_file = os.path.join(
    get_package_share_directory('simple_arm_camera'), 'urdf', 'simple.xacro'
  )
  semantic_file = os.path.join(
    get_package_share_directory('simple_arm_camera'), 'config', 'simple_arm.srdf'
  )

  ign_gazebo = ExecuteProcess(
    cmd=['ign gazebo -r', world_file],
    output='screen',
    additional_env=env,
    shell=True
  )
  ign_spawn_entry = Node(
    package='ros_gz_sim',
    executable='create',
    output='screen',
    arguments=[
      '-topic', '/robot_description',
      '-name', 'simple_arm',
      '-allow_renaming', 'true'
    ],
  )

  controllers_yaml = load_yaml('simple_arm_camera', 'config/controllers.yaml')
  moveit_controllers = {
    'moveit_simple_controller_manager': controllers_yaml,
    'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
  }

  description = Command(['xacro ', urdf_file])
  robot_description = {'robot_description': description}

  robot_description_semantic = {'robot_description_semantic': Command(['cat ', semantic_file])}

  ompl_planning_pipeline_config = {
    'move_group': {
      'planning_plugin': 'ompl_interface/OMPLPlanner',
      'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization \
                                default_planner_request_adapters/FixWorkspaceBounds \
                                default_planner_request_adapters/FixStartStateBounds \
                                default_planner_request_adapters/FixStartStateCollision \
                                default_planner_request_adapters/FixStartStatePathConstraints',
      'start_state_max_bounds_error': 0.1
    }
  }

  trajectory_execution = {
    'moveit_manage_controllers': True,
  }

  run_move_group_node = Node(
    package='moveit_ros_move_group',
    executable='move_group',
    output='screen',
    parameters=[
      robot_description,
      robot_description_semantic,
      ompl_planning_pipeline_config,
      trajectory_execution,
      moveit_controllers,
    ]
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[robot_description])
  
  spawn_joint_state_controller = ExecuteProcess(
    cmd=['ros2 run controller_manager spawner joint_state_controller'],
    shell=True,
    output='screen',
  )

  spawn_arm_controller = ExecuteProcess(
    cmd=['ros2 run controller_manager spawner simple_arm_controller'],
    shell=True,
    output='screen',
  )

  bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
      '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
      '/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
      '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
    ],
    output='screen'
  )

  return LaunchDescription([
    SetParameter(name='use_sim_time', value=True),
    ign_gazebo,
    ign_spawn_entry,
    run_move_group_node,
    robot_state_publisher,
    spawn_joint_state_controller,
    spawn_arm_controller,
    bridge,
  ])