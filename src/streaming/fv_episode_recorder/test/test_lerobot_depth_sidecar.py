"""depth_hevc10 sidecar export (fv_depth in info.json, byte copy, loader)
and the depth10 encoder argv / 10-bit verification. No ROS, no lerobot.
Video round-trips need ffmpeg with libx265 and are skipped otherwise."""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("pandas")
pa = pytest.importorskip("pyarrow")
pq = pytest.importorskip("pyarrow.parquet")

from fv_episode_recorder import depth10  # noqa: E402
from fv_episode_recorder import lerobot_materializer as lm  # noqa: E402

W, H, N, FPS = 64, 48, 20, 30
BASE_NS = 1_792_544_401_000_000_000


def _have_x265() -> bool:
    if not (shutil.which("ffmpeg") and shutil.which("ffprobe")):
        return False
    out = subprocess.run(["ffmpeg", "-hide_banner", "-encoders"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    return b"libx265" in out.stdout


needs_x265 = pytest.mark.skipif(not _have_x265(), reason="ffmpeg/ffprobe with libx265 unavailable")


def _depth_frames(d_min=400, d_max=4000):
    frames = []
    for i in range(N):
        d = np.linspace(d_min, d_max, W * H).reshape(H, W).astype(np.uint16)
        d = np.roll(d, i * 7, axis=1)
        d[0, :4] = 0          # invalid
        d[1, :4] = d_max + 500  # beyond far → invalid
        frames.append(d)
    return frames


def _encode(video: Path, frames, d_min, d_max, extra):
    cmd = depth10.hevc10_encode_cmd("libx265", extra, video, W, H, FPS)
    uv = np.full((H // 2, W // 2), 512, dtype="<u2").tobytes() * 2
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
    lut = depth10.build_pack_lut(d_min, d_max)
    for d in frames:
        proc.stdin.write(depth10.pack_luma10(d, d_min, d_max, lut).astype("<u2").tobytes())
        proc.stdin.write(uv)
    proc.stdin.close()
    assert proc.wait() == 0


def _make_depth_episode(root: Path, *, name="depth", d_min=400, d_max=4000, scale=1.0,
                        video_bytes: bytes | None = None, extra=None) -> Path:
    ep = root / "ep"
    cam = ep / "videos" / name
    cam.mkdir(parents=True)
    video = cam / "0000.mp4"
    if video_bytes is not None:
        video.write_bytes(video_bytes)
    else:
        _encode(video, _depth_frames(d_min, d_max), d_min, d_max, extra)
    depth10.write_meta(cam, d_min=d_min, d_max=d_max, depth_scale_mm=scale,
                       width=W, height=H, fps=FPS, codec="libx265",
                       encoding_seen="16UC1", video_file="0000.mp4")
    rows = [{"frame_index": i, "segment_file": "0000.mp4", "segment_local_frame": i,
             "ros_stamp_ns": BASE_NS + int(i * 1e9 / FPS), "recv_stamp_ns": 0,
             "source_seq": -1, "dropped_before": 0, "keyframe": False} for i in range(N)]
    from fv_episode_recorder.frames_sidecar import SCHEMA
    pq.write_table(pa.Table.from_pylist(rows, schema=SCHEMA), cam / "frames.parquet")
    meta = {"episode_id": "01EPISODE", "started_at": "2026-10-21T01:00:00Z",
            "cameras": [{"name": name, "kind": "depth_hevc10", "frame_count": N,
                         "video_dir": f"{name}/", "sidecar_file": f"{name}/frames.parquet",
                         "meta_file": f"{name}/depth_meta.json",
                         "segments": [{"file": "0000.mp4", "frame_count": N}]},
                        {"name": "x3", "kind": None, "frame_count": 0}]}
    (ep / "meta.json").write_text(json.dumps(meta))
    return ep


def test_depth_frame_range_floor():
    st = np.array([100, 200, 300, 400], dtype=np.int64)
    assert lm.depth_frame_range(st, 250, 350) == (1, 2)
    assert lm.depth_frame_range(st, 50, 1000) == (0, 3)
    assert lm.depth_frame_range(st, 200, 200) == (1, 1)
    assert lm.depth_frame_range(np.array([], dtype=np.int64), 0, 1) == (0, -1)


def test_info_json_block_and_copy_without_ffmpeg(tmp_path, monkeypatch):
    # fake video bytes: the export copies bytes, never decodes
    monkeypatch.setattr(depth10, "probe_pix_fmt", lambda p: None)
    ep = _make_depth_episode(tmp_path, video_bytes=b"\x00fakehevc" * 10)
    meta = json.loads((ep / "meta.json").read_text())
    srcs = lm.depth_sources(ep, meta, {"depth": "main"})
    assert [s["key"] for s in srcs] == ["observation.depth.main"]

    t0 = BASE_NS + int(5.5 * 1e9 / FPS)  # between frames 5 and 6
    t1 = BASE_NS + int(12 * 1e9 / FPS)
    e0 = lm.plan_depth_entry(srcs[0], 0, "01EPISODE", t0, t1)
    e1 = lm.plan_depth_entry(srcs[0], 1, "01EPISODE", BASE_NS, BASE_NS + 10**9)
    assert (e0["frame_from"], e0["frame_to"], e0["n_frames"]) == (5, 12, 8)
    assert e0["first_frame_timestamp_s"] < 0  # frame 5 was current at t=0
    assert e0["file"] == "depth/observation.depth.main/01EPISODE/0000.mp4"

    out = tmp_path / "ds"
    (out / "meta").mkdir(parents=True)
    (out / "meta" / "info.json").write_text(json.dumps({"codebase_version": "v3.0", "fps": 30}))
    copied = lm.copy_depth_files(out, [e0, e1])
    vid = out / e0["file"]
    assert vid.read_bytes() == (ep / "videos/depth/0000.mp4").read_bytes()
    assert (out / e0["meta_file"]).exists() and (out / e0["frames_file"]).exists()
    assert copied == sum((out / e0[k]).stat().st_size for k in ("file", "meta_file", "frames_file"))

    block = lm.build_fv_depth_block([e0, e1])
    lm.patch_info_json(out, block)
    info = json.loads((out / "meta" / "info.json").read_text())
    assert info["codebase_version"] == "v3.0"  # LeRobot keys preserved
    k = info["fv_depth"]["keys"]["observation.depth.main"]
    assert k["pix_fmt"] == "yuv420p10le"
    assert k["quantisation"] == "inverse_depth_10bit"
    assert (k["d_min_counts"], k["d_max_counts"], k["depth_scale_mm_per_count"]) == (400, 4000, 1.0)
    assert (k["luma_levels"], k["invalid_luma"]) == (1023, 0)
    assert k["decode"] == "fv_episode_recorder.depth10.decode_video"
    assert k["path_template"] == "depth/observation.depth.main/{source_episode_id}/0000.mp4"
    assert set(k["episodes"]) == {"0", "1"}
    assert k["episodes"]["0"]["frame_from"] == 5
    assert not any(kk.startswith("_") for kk in k["episodes"]["0"])
    json.dumps(info)


@needs_x265
def test_load_depth_frames_roundtrip(tmp_path):
    lossless = ["-pix_fmt", "yuv420p10le", "-profile:v", "main10",
                "-x265-params", "log-level=none:lossless=1"]
    d_min, d_max, scale = 700, 15000, 0.1  # D405-style counts
    ep = _make_depth_episode(tmp_path, d_min=d_min, d_max=d_max, scale=scale, extra=lossless)
    meta = json.loads((ep / "meta.json").read_text())
    src = lm.depth_sources(ep, meta, {})[0]
    t0 = BASE_NS + int(3 * 1e9 / FPS)
    e = lm.plan_depth_entry(src, 0, "01EPISODE", t0, BASE_NS + int(9 * 1e9 / FPS))
    assert e["actual_pix_fmt"] == "yuv420p10le" and e["bit_depth_ok"] is True
    out = tmp_path / "ds"
    (out / "meta").mkdir(parents=True)
    (out / "meta" / "info.json").write_text("{}")
    lm.copy_depth_files(out, [e])
    lm.patch_info_json(out, lm.build_fv_depth_block([e]))

    truth = _depth_frames(d_min, d_max)
    pack, unpack = depth10.build_pack_lut(d_min, d_max), depth10.build_unpack_lut(d_min, d_max)
    got = list(lm.load_depth_frames(out, 0, "observation.depth.depth", unit="counts"))
    assert len(got) == 7  # frames 3..9
    for j, (ts, counts) in enumerate(got):
        i = 3 + j
        assert ts == pytest.approx(j / FPS, abs=1e-6)
        expect = unpack[pack[truth[i]]]
        assert counts.dtype == np.uint16 and counts.shape == (H, W)
        np.testing.assert_array_equal(counts, expect)
        assert counts[0, 0] == 0 and counts[1, 0] == 0  # invalid / beyond-far stay 0
    _, mm = next(iter(lm.load_depth_frames(out, 0, "observation.depth.depth")))
    np.testing.assert_array_equal(
        mm, np.rint(unpack[pack[truth[3]]].astype(np.float32) * 0.1).astype(np.uint16))
    _, mmf = next(iter(lm.load_depth_frames(out, 0, "observation.depth.depth", unit="mm_f32")))
    assert mmf.dtype == np.float32
    with pytest.raises(KeyError):
        next(iter(lm.load_depth_frames(out, 5, "observation.depth.depth")))


@needs_x265
def test_8bit_stream_flagged(tmp_path, capsys):
    eight = ["-pix_fmt", "yuv420p", "-x265-params", "log-level=none"]
    ep = _make_depth_episode(tmp_path, extra=eight)
    meta = json.loads((ep / "meta.json").read_text())
    src = lm.depth_sources(ep, meta, {})[0]
    e = lm.plan_depth_entry(src, 0, "01EPISODE", BASE_NS, BASE_NS + 10**9)
    assert e["actual_pix_fmt"] == "yuv420p" and e["bit_depth_ok"] is False
    assert "8-bit" in capsys.readouterr().err
    block = lm.build_fv_depth_block([e])
    assert block["keys"]["observation.depth.depth"]["bit_depth_ok"] is False


def test_hevc10_encoder_argv_requests_10bit_output(tmp_path):
    tries = depth10.hevc10_encoder_tries(16)
    assert [c for c, _ in tries] == ["hevc_nvenc", "libx265"]
    for codec, extra in tries:
        cmd = depth10.hevc10_encode_cmd(codec, extra, tmp_path / "o.mp4", 640, 480, 30)
        i_in = cmd.index("-i")
        out_opts = cmd[i_in + 2:]
        # input is declared 10-bit ...
        assert cmd[cmd.index("-pix_fmt") + 1] == "yuv420p10le" and cmd.index("-pix_fmt") < i_in
        # ... and the OUTPUT options request a 10-bit pix_fmt + Main10 profile
        pf = out_opts[out_opts.index("-pix_fmt") + 1]
        assert depth10.is_10bit_pix_fmt(pf), (codec, pf)
        assert out_opts[out_opts.index("-profile:v") + 1] == "main10"
        assert out_opts[out_opts.index("-c:v") + 1] == codec
        assert cmd[-1] == str(tmp_path / "o.mp4")
    assert not depth10.is_10bit_pix_fmt("nv12")
    assert not depth10.is_10bit_pix_fmt(None)


@needs_x265
def test_probe_encoder_10bit_detects_8bit_output():
    ok, pf = depth10.probe_encoder_10bit("libx265", dict(depth10.hevc10_encoder_tries(16))["libx265"])
    assert ok and pf == "yuv420p10le"
    ok, pf = depth10.probe_encoder_10bit("libx265", ["-pix_fmt", "yuv420p",
                                                     "-x265-params", "log-level=none"])
    assert not ok and pf == "yuv420p"
