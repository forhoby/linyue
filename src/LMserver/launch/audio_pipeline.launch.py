from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动音频采集节点 (record.cpp)
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
        # Node(
        #     package='audio_capture',
        #     executable='video2',
        #     name='camera_node',
        #     output='screen',
        #     parameters=[
        #         # 可根据需要添加参数，例如：
        #         # {'sample_rate': 16000},
        #         # {'channels': 1},
        #         # {'device': 'hw:1,0'}
        #     ]
        # ),

        # 启动 VAD 节点 (vad_node.cpp)
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
        )
    ])