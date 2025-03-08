import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    package_name = "xplorer_description"
    package_dir = get_package_share_directory(package_name)

    # Define the URDF file path
    urdf_file = PathJoinSubstitution([package_dir, "urdf", "xplorer.urdf.xacro"])

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time")

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["xacro ", urdf_file])}],
    )

    # Joint State Publisher GUI (for testing)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=LaunchConfiguration("gui", default="true")
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([package_dir, "config", "rviz_config.rviz"])]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])