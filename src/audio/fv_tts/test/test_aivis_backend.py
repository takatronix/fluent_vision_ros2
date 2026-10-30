"""aivis_backend の単体テスト (ROS 不要)。ローカルに偽の AivisSpeech API を立てる。"""
import importlib.util
import io
import json
import socket
import sys
import threading
import time
import wave
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer
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


# ── AivisRouter: 別の機械のエンジンを先に、だめならこの機械で ─────────────────

class _Now:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _server(delay=0.0, status=200, trickle=False):
    """偽のエンジン (/version・/audio_query・/synthesis)。calls に来た path を残す。"""
    calls = []

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, out, ctype):
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            if trickle:
                for i in range(0, len(out), 200):
                    self.wfile.write(out[i:i + 200])
                    self.wfile.flush()
                    time.sleep(0.05)
            else:
                self.wfile.write(out)

        def do_GET(self):
            calls.append(self.path)
            self._send(b'"latest"', "application/json")

        def do_POST(self):
            self.rfile.read(int(self.headers.get("Content-Length") or 0))
            calls.append(self.path)
            time.sleep(delay)
            if self.path.startswith("/audio_query"):
                self._send(json.dumps({"speedScale": 1.0}).encode(), "application/json")
            else:
                self._send(_wav(), "audio/wav")

    srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{srv.server_port}", calls


@pytest.fixture
def engines():
    made = []

    def make(**kw):
        made.append(_server(**kw))
        return made[-1][1], made[-1][2]

    yield make
    for srv, _u, _c in made:
        srv.shutdown()


def _router(local_url, remote, now=None, **kw):
    ran = []
    router = ab.AivisRouter(ab.AivisBackend(local_url), remote, clock=now or _Now(), spawn=lambda fn: (ran.append(fn), fn()), **kw)
    return router, ran


def _posts(calls):
    return [c for c in calls if c.startswith("/synthesis")]


def test_router_probes_first_then_uses_remote(engines):
    local, lcalls = engines()
    remote, rcalls = engines()
    router, ran = _router(local, [remote])
    router.synthesize("一", 1)                # 起動直後: 声は機体で、裏で確かめる (届かない場所で 1 文目を待たせない)
    assert len(ran) == 1 and "/version" in rcalls and len(_posts(lcalls)) == 1 and router.last_source == "local"
    pcm, sr, ch, bits = router.synthesize("二", 1, {"speedScale": 1.1})
    assert (sr, ch, bits, len(pcm)) == (44100, 1, 16, 882)
    assert router.last_source == remote and len(_posts(rcalls)) == 1 and len(_posts(lcalls)) == 1


def test_router_unreachable_falls_back_in_the_same_sentence_and_holds(engines):
    local, lcalls = engines()
    now = _Now()
    remote, _ = engines()
    router, ran = _router(local, [remote], now=now, hold_s=30.0)
    router.synthesize("一", 1)
    assert router._url == remote
    router._url = "http://127.0.0.1:9"        # 確かめの後で届かなくなった
    t0 = time.monotonic()
    pcm, *_ = router.synthesize("二", 1)
    assert pcm and router.last_source == "local" and time.monotonic() - t0 < 1.0
    n = len(_posts(lcalls))
    probes = len(ran)
    router.synthesize("三", 1)                # hold の間は試しも確かめもしない
    assert len(ran) == probes and router._url == "" and len(_posts(lcalls)) == n + 1
    now.t += 31.0
    router.synthesize("四", 1)                # 過ぎたら声は機体のまま、裏で確かめて戻る
    assert len(ran) == probes + 1 and router._url == remote and router.last_source == "local"


def test_router_black_hole_connect_is_short(monkeypatch, engines):
    """届かない住所 (Tailscale の相手が居ない): 接続の短い時間切れだけで機体へ戻す。"""
    local, _ = engines()
    router, _ran = _router(local, ["http://10.255.255.1:10101"], connect_s=0.1)
    router._url = "http://10.255.255.1:10101"
    t0 = time.monotonic()
    assert router.synthesize("一", 1)[0]
    assert time.monotonic() - t0 < 0.8 and router.last_source == "local"


def test_router_budget_cuts_a_stuck_remote(engines):
    local, _ = engines()
    remote, _ = engines(delay=2.0)
    router, _ran = _router(local, [remote], budget_s=0.2, budget_per_char_s=0.0)
    router._url = remote
    t0 = time.monotonic()
    assert router.synthesize("一", 1)[0]
    assert time.monotonic() - t0 < 1.0 and router.last_source == "local" and router._url == ""


def test_router_budget_cuts_a_trickling_remote(engines):
    """少しずつ届く (遠い回線): socket の時間切れにかからなくても全体の期限で切る。"""
    local, _ = engines()
    remote, _ = engines(trickle=True)
    router, _ran = _router(local, [remote], budget_s=0.12, budget_per_char_s=0.0)
    router._url = remote
    t0 = time.monotonic()
    assert router.synthesize("一", 1)[0]
    assert time.monotonic() - t0 < 1.5 and router.last_source == "local"


def test_router_slow_success_is_used_but_next_goes_local(engines):
    local, lcalls = engines()
    remote, _ = engines(delay=0.15)
    router, _ran = _router(local, [remote], slow_s=0.1, slow_per_char_s=0.0)
    router._url = remote
    assert router.synthesize("一", 1)[0] and router.last_source == remote and router._url == ""
    router.synthesize("二", 1)
    assert router.last_source == "local" and len(_posts(lcalls)) == 1


@pytest.mark.parametrize("status", [404, 500])
def test_router_http_error_falls_back(engines, status):
    """話者が無い・エンジンの不調: 喋らないのではなく機体で作る。"""
    local, _ = engines()
    remote, _ = engines(status=status)
    router, _ran = _router(local, [remote])
    router._url = remote
    assert router.synthesize("一", 1)[0] and router.last_source == "local"


def test_router_redirect_is_not_followed(engines):
    local, _ = engines()
    remote, calls = engines(status=307)
    router, _ran = _router(local, [remote])
    router._url = remote
    assert router.synthesize("一", 1)[0] and router.last_source == "local"
    assert len([c for c in calls if c.startswith("/audio_query")]) == 1


def test_router_hold_doubles_and_resets(engines):
    local, _ = engines()
    now = _Now()
    remote, _ = engines()
    router, _ran = _router(local, ["http://127.0.0.1:9"], now=now, hold_s=30.0, hold_max_s=100.0)
    holds = []
    for _ in range(4):
        router.synthesize("一", 1)            # 確かめは失敗
        holds.append(router._retry_at - now.t)
        now.t = router._retry_at + 1.0
    assert holds == [30.0, 60.0, 100.0, 100.0]
    router.urls = [remote]
    router.synthesize("二", 1)
    router.synthesize("三", 1)                # 速く作れた: 待ちは初めの長さに
    assert router.last_source == remote and router._hold_now == 30.0


def test_router_disabled_uses_local_only(engines):
    local, lcalls = engines()
    remote, rcalls = engines()
    router, ran = _router(local, [remote])
    router.set_enabled(False)
    router.synthesize("一", 1)
    assert not ran and not rcalls and len(_posts(lcalls)) == 1
    router.set_enabled(True)
    router.synthesize("二", 1)
    router.synthesize("三", 1)
    assert router.last_source == remote


def test_router_second_url_is_found_by_probe(engines):
    local, _ = engines()
    remote, _ = engines()
    router, _ran = _router(local, ["http://127.0.0.1:9", remote])
    router.synthesize("一", 1)
    assert router._url == remote


def test_router_warm_keeps_both_loaded(engines):
    local, lcalls = engines()
    remote, rcalls = engines()
    router, _ran = _router(local, [remote])
    router.warm("はい。", 1)                  # 起動時: local を温め、裏で確かめ
    assert len(_posts(lcalls)) == 1 and not _posts(rcalls)
    router.warm("はい。", 1)
    assert len(_posts(lcalls)) == 2 and len(_posts(rcalls)) == 1


def test_router_local_failure_is_raised(engines):
    remote, _ = engines(status=500)
    router, _ran = _router("http://127.0.0.1:9", [remote])
    router._url = remote
    with pytest.raises(ab.AivisUnreachable):
        router.synthesize("一", 1)


def _raw_server(script):
    """接続ごとに script(conn) を走らせる生の TCP サーバー。"""
    srv = socket.create_server(("127.0.0.1", 0))

    def run():
        while True:
            try:
                conn, _ = srv.accept()
            except OSError:
                return
            threading.Thread(target=script, args=(conn,), daemon=True).start()

    threading.Thread(target=run, daemon=True).start()
    return srv, f"http://127.0.0.1:{srv.getsockname()[1]}"


def _dribble(conn):
    """ヘッダーを少しずつ送り続ける (socket の時間切れにかからない)。"""
    try:
        conn.recv(4096)
        conn.sendall(b"HTTP/1.1 200 OK\r\n")
        for i in range(200):
            conn.sendall(f"X-A{i}: 1\r\n".encode())
            time.sleep(0.03)
    except OSError:
        pass


def test_router_budget_cuts_trickling_headers(engines):
    local, _ = engines()
    srv, remote = _raw_server(_dribble)
    router, _ran = _router(local, [remote], budget_s=0.2, budget_per_char_s=0.0)
    router._url = remote
    t0 = time.monotonic()
    assert router.synthesize("一", 1)[0]
    assert time.monotonic() - t0 < 1.0 and router.last_source == "local" and router._url == ""
    srv.close()


def test_router_probe_is_cut_by_its_deadline(engines):
    """確かめが終わらないと以後の確かめが始まらない: 確かめにも全体の期限。"""
    local, _ = engines()
    srv, remote = _raw_server(_dribble)
    router, _ran = _router(local, [remote], probe_s=0.2)
    t0 = time.monotonic()
    router.synthesize("一", 1)
    assert time.monotonic() - t0 < 1.0 and not router._probing and router._url == ""
    srv.close()


def test_short_answer_cut_by_the_deadline_is_not_success():
    """期限で切られて途中で終わった答えを、成功の空の音にしない。"""
    def half(conn):
        try:
            conn.recv(4096)
            conn.sendall(b"HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nabc")
            time.sleep(1.0)
        except OSError:
            pass
    srv, url = _raw_server(half)
    with pytest.raises(ab.AivisUnreachable):
        ab._DeadlineBackend(url, 0.2, 0.2)._request("GET", "/version")
    srv.close()


def test_truncated_answer_falls_back_instead_of_playing_half(engines):
    """Content-Length に足りないまま切れた答え (回線断): 欠けた声を鳴らさず、この機械で作り直す。"""
    wav = _wav()

    def cut(conn):
        try:
            req = conn.recv(4096)
            if req.startswith(b"POST /audio_query"):
                body = b"{}"
            else:
                body = wav
            head = f"HTTP/1.1 200 OK\r\nContent-Length: {len(body)}\r\nConnection: close\r\n\r\n".encode()
            conn.sendall(head + body[:100])
        finally:
            conn.close()

    local, lcalls = engines()
    srv, remote = _raw_server(cut)
    router, _ran = _router(local, [remote])
    router._url = remote
    pcm, *_ = router.synthesize("一", 1)
    assert len(pcm) == 882 and router.last_source == "local" and len(_posts(lcalls)) == 1
    srv.close()
