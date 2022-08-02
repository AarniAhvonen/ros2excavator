import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("jcb_moveit_config"),
            "config",
            "jcb.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "jcb_moveit_config", "config/jcb_description.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "jcb_moveit_config", "config/kinematics.yaml"
    )

    # MoveGroupInterface demo executable
    move_group_jcb = Node(
        name="move_group_interface_jcb",
        package="jcb_moveit_config",
        executable="move_group_interface_jcb",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )

    test_jcb = Node(
        name="test_node",
        package="jcb_moveit_config",
        executable="test_interface",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )

    return LaunchDescription([move_group_jcb])
