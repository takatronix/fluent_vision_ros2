from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 声キャッシュ (engine=pyopenjtalk 時のみ): 起動スクリプトを変えずに機体ごとに
# 有効化できるよう、既定値を環境変数 (~/.aspa/env など) から取る。
_VOICE_CACHE_ARGS = [
    # (launch arg / ROS param, env var, default, type, description)
    ('voice_cache_engine', 'ASPA_TTS_VOICE_CACHE_ENGINE', '', str,
     '"" = off, "voicevox" = render cached voice with a VOICEVOX-compatible engine (AivisSpeech)'),
    ('voice_cache_url', 'ASPA_TTS_VOICE_CACHE_URL', 'http://127.0.0.1:10101', str,
     'Engine URL for the cached voice'),
    ('voice_cache_speaker', 'ASPA_TTS_VOICE_CACHE_SPEAKER', '-1', int,
     'Style/speaker id for the cached voice'),
    ('voice_cache_dir', 'ASPA_TTS_VOICE_CACHE_DIR', '~/.aspa/tts_cache/voice', str,
     'Directory of cached WAVs + index.json'),
    ('voice_cache_max_mb', 'ASPA_TTS_VOICE_CACHE_MAX_MB', '300', int,
     'Cache size cap (LRU eviction)'),
    ('voice_cache_warmup_file', 'ASPA_TTS_VOICE_CACHE_WARMUP', '', str,
     'YAML of texts rendered in the background at startup (relative = share/fv_tts/config)'),
    ('voice_cache_max_load', 'ASPA_TTS_VOICE_CACHE_MAX_LOAD', '0.0', float,
     'Warm-up waits while 1-min loadavg >= this (0 = no guard)'),
    ('voice_cache_live_budget_s', 'ASPA_TTS_VOICE_CACHE_LIVE_BUDGET', '2.0', float,
     'Not cached: wait up to this for the high-quality engine, else Open JTalk (0 = never wait)'),
    ('voice_cache_volume_db', 'ASPA_TTS_VOICE_CACHE_VOLUME_DB', '0.0', float,
     'Gain for the high-quality voice (AivisSpeech 0 dB ~ Open JTalk +8 dB)'),
]


def generate_launch_description():
    cache_args = [
        DeclareLaunchArgument(name, default_value=EnvironmentVariable(env, default_value=default),
                              description=desc)
        for name, env, default, _t, desc in _VOICE_CACHE_ARGS
    ]
    cache_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=typ)
        for name, _env, _d, typ, _desc in _VOICE_CACHE_ARGS
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            'engine', default_value='pyopenjtalk',
            description='TTS engine: pyopenjtalk (lightweight) or voicevox'),
        DeclareLaunchArgument(
            'default_volume_db', default_value='8.0',
            description='Engine-specific synthesis gain in dB'),
        # VOICEVOX 互換 API の URL (engine=voicevox 用)。機体の ~/.aspa/env で
        # ASPA_VOICEVOX_URL を指定すれば AivisSpeech Engine (:10101) などへ向けられる。
        DeclareLaunchArgument(
            'voicevox_url',
            default_value=EnvironmentVariable(
                'ASPA_VOICEVOX_URL', default_value='http://127.0.0.1:50021'),
            description='VOICEVOX-compatible engine URL (VOICEVOX :50021 / AivisSpeech :10101)'),
        *cache_args,
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
                'voicevox_url': LaunchConfiguration('voicevox_url'),
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
                **cache_params,
            }],
        )
    ])
