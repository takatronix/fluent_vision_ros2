#pragma once

// Stage 3: runtime bindings and the frame-boundary snapshot scheduler
// (spec sections 6.1/6.4). Transport-neutral: sources push typed values with
// metadata; at a frame boundary the scheduler selects one immutable snapshot
// per the declared policies. No ROS 2, no GPU. Time is always supplied by the
// caller so behavior is deterministic and testable.

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/render/renderer.hpp"
#include "fluent_scene/validator.hpp"

namespace fluent_scene {

// Metadata carried with every pushed value (spec section 6.3). Timestamps are
// seconds in the declared clock domain; zero is data, never "unknown", so
// presence is explicit.
struct ValueMeta {
    bool has_timestamp = false;
    double timestamp = 0.0;
    std::string clock;
    std::string frame;
    uint64_t sequence = 0;
};

// Typed payload for the MVP input families used by the stage-2 renderer.
struct TypedValue {
    enum class Kind { kImage, kDetections, kPoints, kString, kCalibration };
    Kind kind = Kind::kString;

    // kImage (owned RGBA8)
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint8_t> pixels;

    std::vector<DetectionInstance> detections;  // kDetections
    std::vector<Point2f> points;                // kPoints (points2d / polyline2d)
    std::string text;                           // kString
    std::string calibration_id;                 // kCalibration

    ValueMeta meta;
};

enum class QueuePolicy { kLatest, kFifo, kMatched };

struct InputRuntimeConfig {
    QueuePolicy policy = QueuePolicy::kLatest;
    size_t capacity = 1;           // fixed queue capacity (bounded backpressure)
    double max_age_seconds = 0.0;  // 0 = default (0.5s for per_frame, none otherwise)
};

struct SyncGroupConfig {
    double tolerance_seconds = 0.012;  // approximate-match window
};

struct BindingTableOptions {
    std::map<std::string, InputRuntimeConfig> inputs;  // per-input overrides
    std::map<std::string, SyncGroupConfig> groups;     // per-group overrides
    double default_per_frame_max_age = 0.5;
};

enum class EntryStatus {
    kFresh,                // selected from the queue this frame
    kHeld,                 // no new match; last selected value held
    kFallbackValue,        // declared typed fallback value in use
    kFallbackUnavailable,  // declared behavior fallback (output_unavailable)
    kMissing,              // required input absent with no declared fallback
};

const char* toString(EntryStatus status);

// One input inside an immutable frame snapshot (spec section 6.4: records
// source sequence, timestamp, conversion path, age, and validity).
struct SnapshotEntry {
    std::shared_ptr<const TypedValue> value;  // null when nothing is available
    EntryStatus status = EntryStatus::kMissing;
    bool valid = false;
    double age_seconds = 0.0;
    uint64_t sequence = 0;
    std::string conversion_path = "direct";
};

struct FrameSnapshot {
    uint64_t index = 0;
    double now = 0.0;
    bool healthy = true;  // false iff a required input is missing undeclared
    std::map<std::string, SnapshotEntry> entries;

    // Borrows image pixels from the snapshot's shared values: the snapshot
    // must outlive any renderFrame call using the returned inputs.
    FrameInputs toFrameInputs() const;
};

struct InputBindingStats {
    uint64_t pushes = 0;
    uint64_t drops = 0;      // bounded-queue overflow evictions
    uint64_t stale = 0;      // freshness-bound violations at snapshot time
    uint64_t fallbacks = 0;  // snapshots served by a declared fallback
    uint64_t held = 0;       // snapshots served by holding the last value
};

struct BindingStats {
    uint64_t snapshots = 0;
    uint64_t match_failures = 0;  // sync groups that could not match a set
    std::map<std::string, InputBindingStats> inputs;
};

// Runtime binding table for one validated scene. Push is safe to call from a
// source thread; acquireSnapshot is the frame-boundary selection. Values that
// arrive after a snapshot are considered for the next one only.
class BindingTable {
public:
    static std::unique_ptr<BindingTable> create(const ValidationResult& scene,
                                                const BindingTableOptions& options,
                                                DiagnosticList& diagnostics);

    // Pushes one typed value into the named input queue. Contract violations
    // (unknown input, kind mismatch, missing timestamp) produce bind-phase
    // diagnostics and return false.
    bool push(const std::string& input, TypedValue value, DiagnosticList& diagnostics);

    // Selects one immutable snapshot at time `now` (seconds, caller's clock).
    FrameSnapshot acquireSnapshot(double now);

    BindingStats stats() const;

private:
    BindingTable() = default;

    struct InputState {
        TypedValue::Kind kind = TypedValue::Kind::kString;
        bool required = false;
        std::string update;  // per_frame | on_change | static
        std::string group;
        InputRuntimeConfig config;
        double max_age = 0.0;  // resolved freshness bound (0 = none)
        // Declared fallback (resolved from the scene's fallbacks section).
        bool has_fallback = false;
        bool fallback_is_behavior = false;
        std::shared_ptr<const TypedValue> fallback_value;  // null => hide
        // Queue and hold state.
        std::deque<std::shared_ptr<const TypedValue>> queue;
        std::shared_ptr<const TypedValue> last_selected;
        InputBindingStats stats;
    };

    void selectFresh(InputState& state, std::shared_ptr<const TypedValue> value,
                     SnapshotEntry& entry, double now);
    void resolveAbsent(const std::string& name, InputState& state, SnapshotEntry& entry,
                       FrameSnapshot& snapshot, bool stale);

    mutable std::mutex mutex_;
    std::map<std::string, InputState> inputs_;
    std::map<std::string, SyncGroupConfig> groups_;
    uint64_t snapshot_count_ = 0;
    uint64_t match_failures_ = 0;
};

}  // namespace fluent_scene
