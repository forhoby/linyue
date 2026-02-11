from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

    
        Node(
            package='audio_capture',
            executable='audio_recorder',
            name='record_node',
            output='screen',
            parameters=[
                # 可根据需要添加参数，例如：
                # {'sample_rate': 16000},
                # {'channels': 1},
                # {'device': 'hw:1,0'}
            ]
        ),
        
        Node(
            package='audio_capture',
            executable='video_angle.py',
            name='camera_node',
            output='screen',
            parameters=[
                # 可根据需要添加参数，例如：
                # {'sample_rate': 16000},
                # {'channels': 1},
                # {'device': 'hw:1,0'}
            ]
        ),
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
        ),

        Node(
            package='LMserver',
            executable='vad_node',
            name='vad_node',
            output='screen',
            parameters=[
                # 可根据需要添加参数，例如：
                # {'vad_aggressiveness': 1},
                # {'vad_frame_duration_ms': 30}
            ]
        ),
        Node(
            package='servo_control',
            executable='eyes_ctrl',
            name='eyes_control_node',
            output='screen',
            parameters=[
                # 可根据需要添加参数，例如：
                # {'vad_aggressiveness': 1},
                # {'vad_frame_duration_ms': 30}
            ]
        )
])