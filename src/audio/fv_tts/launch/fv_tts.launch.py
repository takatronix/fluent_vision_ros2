import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def _aivis_url_default() -> str:
    """ASPA_AIVIS_URL。旧名 ASPA_VOICEVOX_URL も一度だけ読む (廃止予定のログ付き)。"""
    url = os.environ.get('ASPA_AIVIS_URL', '').strip()
    if url:
        return url
    legacy = os.environ.get('ASPA_VOICEVOX_URL', '').strip()
    if legacy:
        print(f'[fv_tts.launch] ASPA_VOICEVOX_URL は廃止予定 → ASPA_AIVIS_URL={legacy} として使う'
              ' (~/.aspa/env を ASPA_AIVIS_URL に直してください)', file=sys.stderr)
    return legacy


# AivisSpeech の声 + 声キャッシュ: 起動スクリプトを変えずに機体ごとに
# 有効化できるよう、既定値を環境変数 (~/.aspa/env など) から取る。
_VOICE_CACHE_ARGS = [
    # (launch arg / ROS param, env var, default, type, description)
    ('aivis_speaker', 'ASPA_AIVIS_SPEAKER', '135718976', int,
     'AivisSpeech style id (135718976 = female, 1310138977 = male; the viewer overrides it)'),
    ('voice_cache_dir', 'ASPA_TTS_VOICE_CACHE_DIR', '~/.aspa/tts_cache/voice', str,
     'Directory of cached WAVs + index.json'),
    ('voice_cache_max_mb', 'ASPA_TTS_VOICE_CACHE_MAX_MB', '300', int,
     'Cache size cap (LRU eviction)'),
    ('voice_cache_warmup_file', 'ASPA_TTS_VOICE_CACHE_WARMUP', '', str,
     'YAML of texts rendered in the background at startup (relative = share/fv_tts/config)'),
    ('voice_cache_max_load', 'ASPA_TTS_VOICE_CACHE_MAX_LOAD', '0.0', float,
     'Warm-up waits while 1-min loadavg >= this (0 = no guard)'),
    ('voice_cache_live_budget_s', 'ASPA_TTS_VOICE_CACHE_LIVE_BUDGET', '2.0', float,
     'Not cached: wait up to this for AivisSpeech, else Open JTalk (0 = never wait)'),
    ('voice_cache_volume_db', 'ASPA_TTS_VOICE_CACHE_VOLUME_DB', '0.0', float,
     'Gain for the AivisSpeech voice (0 dB ~ Open JTalk +8 dB)'),
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
            description='TTS engine: pyopenjtalk (lightweight) or aivis (AivisSpeech, Open JTalk fallback)'),
        DeclareLaunchArgument(
            'default_volume_db', default_value='8.0',
            description='Engine-specific synthesis gain in dB'),
        # AivisSpeech Engine の URL。機体の ~/.aspa/env の ASPA_AIVIS_URL。設定があれば
        # engine=pyopenjtalk でも viewer から再起動なしに AivisSpeech へ切り替えられる。
        DeclareLaunchArgument(
            'aivis_url', default_value=_aivis_url_default(),
            description='AivisSpeech Engine URL ("" = not set; engine=aivis then uses http://127.0.0.1:10101)'),
        *cache_args,
        Node(
            package='fv_tts',
            executable='fv_tts_node',
            name='fv_tts',
            output='screen',
            parameters=[{
                # エンジン: "pyopenjtalk"(軽量・既定) | "aivis"
                'engine': ParameterValue(
                    LaunchConfiguration('engine'), value_type=str),
                'default_volume_db': ParameterValue(
                    LaunchConfiguration('default_volume_db'), value_type=float),
                'aivis_url': ParameterValue(LaunchConfiguration('aivis_url'), value_type=str),
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
