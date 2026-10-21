"""aivis_backend の単体テスト (ROS 不要)。ローカルに偽の AivisSpeech API を立てる。"""
import importlib.util
import io
import json
import sys
import threading
import wave
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

_path = Path(__file__).resolve().parents[1] / "fv_tts_py" / "aivis_backend.py"
_spec = importlib.util.spec_from_file_location("aivis_backend", _path)
ab = importlib.util.module_from_spec(_spec)
sys.modules["aivis_backend"] = ab
_spec.loader.exec_module(ab)


def _wav(sr=44100, n=441):
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(b"\x01\x00" * n)
    return buf.getvalue()


@pytest.fixture
def server():
    calls = []

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_POST(self):
            body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
            calls.append((self.path, body))
            if self.path.startswith("/audio_query"):
                out = json.dumps({"speedScale": 1.0}).encode()
                ctype = "application/json"
            else:
                out, ctype = _wav(), "audio/wav"
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_port}", calls
    srv.shutdown()


def test_default_url_is_aivisspeech():
    assert ab.AivisBackend().url == "http://127.0.0.1:10101"


def test_synthesize_two_step_api(server):
    url, calls = server
    pcm, sr, ch, bits = ab.AivisBackend(url).synthesize("テスト", 135718976, {"speedScale": 1.1})
    assert (sr, ch, bits, len(pcm)) == (44100, 1, 16, 882)
    assert calls[0][0].startswith("/audio_query?") and "speaker=135718976" in calls[0][0]
    assert calls[1][0] == "/synthesis?speaker=135718976"
    assert json.loads(calls[1][1]) == {"speedScale": 1.1}


def test_unreachable_is_distinguished():
    with pytest.raises(ab.AivisUnreachable):
        ab.AivisBackend("http://127.0.0.1:9", timeout=1.0).synthesize("テスト", 1)
