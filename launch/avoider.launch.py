import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    avoider_node = Node(
        package='my_turtle_controller_cpp',
        executable='wall_avoider_node',
        name='wall_avoider_node',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        gazebo_launch,
        avoider_node
    ])
