from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # QR Scanner Node
        Node(
            package='qr_scanner',
            executable='qr_scanner_node',
            name='qr_scanner',
            output='screen',
            parameters=[
                {"image_topic": "/camera/color/image_raw"}  # 필요하면 수정
            ]
        ),

        # URL Launcher Node
        Node(
            package='url_launcher',
            executable='url_launcher_node',
            name='url_launcher',
            output='screen'
        ),
    ])
