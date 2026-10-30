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
# 旧 ASPA_TTS_VOICE_CACHE_LIVE_BUDGET (間に合わなければ Open JTalk) は廃止: 代わりの声は出さない
# (2026-10-24)。待つ長さは ASPA_TTS_AIVIS_TIMEOUT。
_VOICE_CACHE_ARGS = [
    # (launch arg / ROS param, env var, default, type, description)
    ('aivis_remote_urls', 'ASPA_AIVIS_REMOTE_URLS', '', str,
     'AivisSpeech Engines on other machines, tried first (comma separated); falls back to aivis_url when '
     'unreachable or slow ("" = this machine only; the viewer setting aivis_route=local also turns it off)'),
    ('aivis_speaker', 'ASPA_AIVIS_SPEAKER', '1310138977', int,
     'AivisSpeech style id (1310138977 = male (default), 135718976 = female; the viewer overrides it)'),
    ('aivis_speed', 'ASPA_TTS_AIVIS_SPEED', '1.3', float,
     'AivisSpeech speedScale (a little faster than normal; the viewer setting aivis_speed overrides it)'),
    ('aivis_timeout_s', 'ASPA_TTS_AIVIS_TIMEOUT', '15.0', float,
     'Wait this long for one sentence; past it the utterance is skipped (never another voice)'),
    ('aivis_keepalive_s', 'ASPA_TTS_AIVIS_KEEPALIVE', '180.0', float,
     'Synthesize one short text in the background after this much idle time (0 = never)'),
    ('voice_cache_dir', 'ASPA_TTS_VOICE_CACHE_DIR', '~/.aspa/tts_cache/voice', str,
     'Directory of cached WAVs + index.json'),
    ('voice_cache_max_mb', 'ASPA_TTS_VOICE_CACHE_MAX_MB', '300', int,
     'Cache size cap (LRU eviction)'),
    ('voice_cache_warmup_file', 'ASPA_TTS_VOICE_CACHE_WARMUP', '', str,
     'YAML of texts rendered in the background at startup (relative = share/fv_tts/config)'),
    ('voice_cache_max_load', 'ASPA_TTS_VOICE_CACHE_MAX_LOAD', '0.0', float,
     'Warm-up waits while 1-min loadavg >= this (0 = no guard)'),
    ('voice_cache_volume_db', 'ASPA_TTS_VOICE_CACHE_VOLUME_DB', '0.0', float,
     'Gain for the AivisSpeech voice (0 dB ~ Open JTalk +8 dB)'),
]


def generate_launch_description():
    if os.environ.get('ASPA_TTS_VOICE_CACHE_LIVE_BUDGET', '').strip():
        print('[fv_tts.launch] ASPA_TTS_VOICE_CACHE_LIVE_BUDGET は廃止 (選んだエンジンが間に合わなくても'
              ' 別の声では喋りません)。待つ長さは ASPA_TTS_AIVIS_TIMEOUT (既定 15 秒)', file=sys.stderr)
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
            description='TTS engine at start: pyopenjtalk (lightweight) or aivis (AivisSpeech). '
                        'The viewer setting (/aspa/tts/settings) switches it at run time; no fallback voice'),
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
