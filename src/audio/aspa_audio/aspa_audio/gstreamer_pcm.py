"""GStreamer PCM shaping, resampling, and channel conversion."""

from __future__ import annotations


class GStreamerPcmConverter:
    def __init__(self, output_rate_hz: int, output_channels: int) -> None:
        if output_rate_hz <= 0 or output_channels <= 0:
            raise ValueError("output format must be positive")
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        self.Gst = Gst
        self.output_rate_hz = output_rate_hz
        self.output_channels = output_channels

    def convert(
        self,
        payload: bytes,
        *,
        sample_rate_hz: int,
        channels: int,
        bit_depth: int,
        encoding: str,
    ) -> bytes:
        if encoding != "PCM16LE" or bit_depth != 16:
            raise ValueError("GStreamer boundary accepts PCM16LE only")
        if sample_rate_hz <= 0 or channels <= 0:
            raise ValueError("input format must be positive")
        if len(payload) % (channels * 2):
            raise ValueError("input PCM is not frame aligned")
        if not payload:
            return b""

        Gst = self.Gst
        description = (
            "appsrc name=src format=time ! "
            f"audio/x-raw,format=S16LE,rate={sample_rate_hz},channels={channels},layout=interleaved ! "
            "audioconvert ! audioresample ! "
            f"audio/x-raw,format=S16LE,rate={self.output_rate_hz},"
            f"channels={self.output_channels},layout=interleaved ! "
            "appsink name=sink sync=false"
        )
        pipeline = Gst.parse_launch(description)
        source = pipeline.get_by_name("src")
        sink = pipeline.get_by_name("sink")
        buffer = Gst.Buffer.new_allocate(None, len(payload), None)
        buffer.fill(0, payload)
        input_frames = len(payload) // (channels * 2)
        buffer.pts = 0
        buffer.duration = Gst.util_uint64_scale_int(
            input_frames, Gst.SECOND, sample_rate_hz
        )
        pipeline.set_state(Gst.State.PLAYING)
        try:
            result = source.emit("push-buffer", buffer)
            if result != Gst.FlowReturn.OK:
                raise RuntimeError(f"GStreamer push-buffer failed: {result}")
            source.emit("end-of-stream")
            output = bytearray()
            while True:
                sample = sink.emit("try-pull-sample", Gst.SECOND)
                if sample is None:
                    if sink.get_property("eos"):
                        break
                    message = pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
                    if message is not None:
                        error, debug = message.parse_error()
                        raise RuntimeError(f"GStreamer conversion failed: {error}: {debug}")
                    continue
                output_buffer = sample.get_buffer()
                ok, mapped = output_buffer.map(Gst.MapFlags.READ)
                if not ok:
                    raise RuntimeError("GStreamer output buffer map failed")
                try:
                    output.extend(mapped.data)
                finally:
                    output_buffer.unmap(mapped)
            expected_frames = round(input_frames * self.output_rate_hz / sample_rate_hz)
            expected_bytes = expected_frames * self.output_channels * 2
            if len(output) < expected_bytes:
                output.extend(b"\x00" * (expected_bytes - len(output)))
            return bytes(output[:expected_bytes])
        finally:
            pipeline.set_state(Gst.State.NULL)
