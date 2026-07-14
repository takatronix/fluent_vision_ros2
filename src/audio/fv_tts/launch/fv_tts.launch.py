from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fv_tts',
            executable='fv_tts_node',
            name='fv_tts',
            output='screen',
            parameters=[{
                # エンジン差替え: "voicevox"(既定) | "pyopenjtalk"
                'engine': 'voicevox',
                'voicevox_url': 'http://127.0.0.1:50021',
                'voicevox_speaker': 3,
                'output_sample_rate': 48000,
                'default_voice': '',
                'output_topic': 'audio/tts/frame',
                'playback_topic': 'audio/output/frame',
                'use_playback_topic': True,
                'say_topic': '/aspa/tts/say',
                'speaking_topic': '/aspa/audio/speaking',
                'cache_dir': '~/.fluent_voice_cache',
            }],
        )
    ])
