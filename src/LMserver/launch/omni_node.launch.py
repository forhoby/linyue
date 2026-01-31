from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 omni_node 节点 (Python版本)
        Node(
            package='LMserver',
            executable='omni_node.py',
            name='omni_node',
            output='screen',
            parameters=[
                # 可根据需要添加参数
            ]
        ),
        # 启动 speaker 节点 (C++版本)
        Node(
            package='LMserver',
            executable='speaker',
            name='speaker_node',
            output='screen'
        )
    ])
