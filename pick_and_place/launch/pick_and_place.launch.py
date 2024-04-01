import os

from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import yaml

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    description_loader = RobotDescriptionLoader()

    robot_description_semantic_config = load_file(
        'crane_plus_moveit_config', 'config/crane_plus.srdf'
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml('crane_plus_moveit_config', 'config/kinematics.yaml')

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        Node(
            name='pick_and_place',
            package='pick_and_place',
            executable='pick_and_place',
            output='screen',
            parameters=[{'robot_description': description_loader.load()},
                        robot_description_semantic,
                        kinematics_yaml]
        )
    ])
