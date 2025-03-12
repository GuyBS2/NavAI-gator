from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch joint state publisher GUI'
    )

    # Initialize launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # Define the package and xacro file path
    package_name = 'xplorer_description'
    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'urdf',
        'components',
        'rover',
        'motor',
        'dynamixel_xl430.urdf.xacro'
    ])

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': use_sim_time
        }],
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(gui)
    )

    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'rviz_config.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])