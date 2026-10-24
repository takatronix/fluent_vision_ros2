"""ROS が無い機械 (開発 PC) でも tts_node の中身を試せるよう、無い時だけ最小の代用を差し込む。

機体 (ROS あり) では何もしない。代用は import を通すだけで、ノードは起動しない
(試験は __init__ を飛ばした素のインスタンスに部品を差し込む)。
"""
import sys
import types


def _stub(name: str, **attrs) -> types.ModuleType:
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    sys.modules[name] = mod
    return mod


def _install_ros_stubs() -> None:
    try:
        import rclpy  # noqa: F401
        return
    except ImportError:
        pass

    class _Node:
        def __init__(self, *_a, **_k):
            pass

        def destroy_node(self):
            return True

    class _Enum:
        def __init__(self, *_a, **_k):
            pass
        RELIABLE = VOLATILE = TRANSIENT_LOCAL = KEEP_LAST = 0

    class _QoS:
        def __init__(self, *_a, **_k):
            pass

    rclpy = _stub("rclpy", init=lambda *a, **k: None, spin=lambda *a, **k: None,
                  shutdown=lambda *a, **k: None, ok=lambda: False)
    rclpy.node = _stub("rclpy.node", Node=_Node)
    rclpy.qos = _stub("rclpy.qos", QoSProfile=_QoS, ReliabilityPolicy=_Enum, HistoryPolicy=_Enum,
                      DurabilityPolicy=_Enum)

    class _Msg:
        def __init__(self, data=None, **kw):
            self.data = data
            for k, v in kw.items():
                setattr(self, k, v)

    class _Header:
        def __init__(self):
            self.stamp = None

    class _AudioFrame:
        def __init__(self):
            self.header = _Header()
            self.encoding = ""
            self.sample_rate = 0
            self.channels = 0
            self.bit_depth = 0
            self.rms = 0.0
            self.peak = 0.0
            self.vad = False
            self.data = b""
            self.epoch = 0

    _stub("std_msgs")
    _stub("std_msgs.msg", Bool=_Msg, Empty=_Msg, String=_Msg)
    _stub("fv_audio")
    _stub("fv_audio.msg", AudioFrame=_AudioFrame)

    class _Speak:
        class Request:
            text = ""
            play = True
            voice_id = ""
            volume_db = 0.0
            cache_key = ""

        class Response:
            success = False
            message = ""
            played = False
            frame = None

    _stub("fv_tts")
    _stub("fv_tts.srv", Speak=_Speak)


_install_ros_stubs()
