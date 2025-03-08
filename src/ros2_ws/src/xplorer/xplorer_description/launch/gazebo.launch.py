import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_xplorer_description = get_package_share_directory('xplorer_description')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'xplorer', '-file', os.path.join(pkg_xplorer_description, 'urdf', 'xplorer.urdf.xacro')],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_robot
    ])
