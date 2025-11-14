# launch/avoider.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 설정
    # TurtleBot3 Gazebo 시뮬레이션 패키지의 경로를 가져옵니다.
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    
    # 2. Gazebo 시뮬레이션 환경 실행
    # 'turtlebot3_world.launch.py'는 Gazebo를 열고 TurtleBot3 모델을 로드합니다.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 3. 벽 회피 C++ 노드 실행
    # 'my_turtle_controller_cpp' 패키지에 있는 'wall_avoider_node' 실행 파일을 등록합니다.
    avoider_node = Node(
        package='my_turtle_controller_cpp',
        executable='wall_avoider_node',
        name='wall_avoider_node',
        output='screen',
        emulate_tty=True
    )

    # 4. 런치 디스크립션 반환
    return LaunchDescription([
        gazebo_launch,
        avoider_node
    ])
