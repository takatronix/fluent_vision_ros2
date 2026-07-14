import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('fv_soundboard')
    default_params = os.path.join(share, 'config', 'soundboard.yaml')
    default_registry = os.path.join(share, 'config', 'events.yaml')

    args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='soundboard.yaml (既定パラメータ。output_backends 等はここで指定)'),
        DeclareLaunchArgument(
            'audio_device', default_value='default',
            description='ALSA 再生デバイス。USB スピーカー接続後は `aplay -l` で確認し hw:X,0'),
        DeclareLaunchArgument('master_volume', default_value='0.8'),
        DeclareLaunchArgument(
            'registry_file', default_value=default_registry,
            description='events.yaml 登録表 (案件別に差し替え可)'),
        DeclareLaunchArgument('event_topic', default_value='/fv/event'),
        DeclareLaunchArgument('enable_tts', default_value='true'),
        DeclareLaunchArgument('enable_sonification', default_value='true'),
    ]

    node = Node(
        package='fv_soundboard',
        executable='fv_soundboard_node',
        name='fv_soundboard',
        output='screen',
        parameters=[
            # 1) yaml を base に読み込み (output_backends 等)
            LaunchConfiguration('params_file'),
            # 2) CLI 引数で上書き (型を明示してキャスト)
            {
                'audio_device': LaunchConfiguration('audio_device'),
                'master_volume': ParameterValue(
                    LaunchConfiguration('master_volume'), value_type=float),
                'registry_file': LaunchConfiguration('registry_file'),
                'event_topic': LaunchConfiguration('event_topic'),
                'enable_tts': ParameterValue(
                    LaunchConfiguration('enable_tts'), value_type=bool),
                'enable_sonification': ParameterValue(
                    LaunchConfiguration('enable_sonification'), value_type=bool),
            },
        ],
    )
    return LaunchDescription(args + [node])
