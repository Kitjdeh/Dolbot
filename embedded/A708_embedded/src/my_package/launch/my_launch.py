from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            node_executable='path_plan',
            node_name='path_plan'
        ),
        Node(
            package='my_package',
            node_executable='local_path',
            node_name='local_path'
        ),
        Node(
            package='my_package',
            node_executable='drive',
            node_name='drive'
        ),
        Node(
            package='my_package',
            node_executable='load_map',
            node_name='load_map'
        ),
        Node(
            package='my_package',
            node_executable='perception',
            node_name='create_logListId'
        ),
        Node(
            package='my_package',
            node_executable='appliance_ctrl',
            node_name='appliance_ctrl'
        ),
        Node(
            package='my_package',
            node_executable='schedule',
            node_name='schedule'
        ),
        Node(
            package='my_package',
            node_executable='weather_pub',
            node_name='weather_pub'
        ),
        Node(
            package='my_package',
            node_executable='cctv',
            node_name='cctv'
        ),
        Node(
            package='my_package',
            node_executable='follow',
            node_name='follow'
        ), 
        # follow는 실행 XXXXXXX
    ])
