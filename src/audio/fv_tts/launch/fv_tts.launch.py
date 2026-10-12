from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'engine', default_value='pyopenjtalk',
            description='TTS engine: pyopenjtalk (lightweight) or voicevox'),
        DeclareLaunchArgument(
            'default_volume_db', default_value='8.0',
            description='Engine-specific synthesis gain in dB'),
        Node(
            package='fv_tts',
            executable='fv_tts_node',
            name='fv_tts',
            output='screen',
            parameters=[{
                # エンジン差替え: "pyopenjtalk"(軽量・既定) | "voicevox"
                'engine': ParameterValue(
                    LaunchConfiguration('engine'), value_type=str),
                'default_volume_db': ParameterValue(
                    LaunchConfiguration('default_volume_db'), value_type=float),
                'voicevox_url': 'http://127.0.0.1:50021',
                'voicevox_speaker': 30,
                'output_sample_rate': 48000,
                'default_voice': '',
                'output_topic': 'audio/tts/frame',
                'playback_topic': 'audio/output/frame',
                'use_playback_topic': True,
                'say_topic': '/aspa/tts/say',
                'speaking_topic': '/aspa/audio/speaking',
                'settings_topic': '/aspa/tts/settings',
                'cache_dir': '~/.fluent_voice_cache',
            }],
        )
    ])
