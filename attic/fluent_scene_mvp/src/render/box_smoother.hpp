#pragma once

// Internal: temporal smoothing for detection boxes, shared by both renderer
// backends. Incoming detections carry no identity, so each frame is greedily
// associated to the nearest existing track by center distance; matched tracks
// blend toward their target with a rate-independent exponential filter.
//
// Bounded state (spec decision 6): at most `max_tracks` tracks ever exist.
// Deterministic given the same inputs, dt sequence, and time constant. New
// tracks start exactly at their target (no pop-in animation) and unmatched
// tracks drop immediately, so a scene with static inputs renders identically
// with smoothing on or off.

#include <algorithm>
#include <cmath>
#include <vector>

#include "fluent_scene/render/renderer.hpp"

namespace fluent_scene {
namespace render {

class BoxSmoother {
public:
    // `time_constant` (seconds): time to close ~63% of the gap; <= 0 disables.
    std::vector<DetectionInstance> update(std::vector<DetectionInstance> targets, double dt_seconds,
                                          float time_constant, uint64_t max_tracks) {
        if (targets.size() > max_tracks) {
            targets.resize(static_cast<size_t>(max_tracks));
        }
        if (time_constant <= 0.0f) {
            tracks_ = targets;
            return targets;
        }
        const float alpha =
            1.0f - std::exp(-static_cast<float>(std::max(dt_seconds, 0.0)) / time_constant);
        std::vector<bool> used(tracks_.size(), false);
        std::vector<DetectionInstance> next;
        next.reserve(targets.size());
        for (const DetectionInstance& target : targets) {
            const float tx = target.bbox[0] + target.bbox[2] * 0.5f;
            const float ty = target.bbox[1] + target.bbox[3] * 0.5f;
            int best = -1;
            float best_distance = 0.0f;
            for (size_t i = 0; i < tracks_.size(); ++i) {
                if (used[i]) {
                    continue;
                }
                const DetectionInstance& track = tracks_[i];
                const float cx = track.bbox[0] + track.bbox[2] * 0.5f;
                const float cy = track.bbox[1] + track.bbox[3] * 0.5f;
                const float distance = std::hypot(tx - cx, ty - cy);
                if (best < 0 || distance < best_distance) {
                    best = static_cast<int>(i);
                    best_distance = distance;
                }
            }
            DetectionInstance out = target;
            // Association gate: without detection IDs, only a nearby track may
            // claim the target; distant ones are treated as new objects.
            const float gate = 1.5f * std::max({target.bbox[2], target.bbox[3], 40.0f});
            if (best >= 0 && best_distance <= gate) {
                used[static_cast<size_t>(best)] = true;
                const DetectionInstance& track = tracks_[static_cast<size_t>(best)];
                for (int c = 0; c < 4; ++c) {
                    out.bbox[c] = track.bbox[c] + (target.bbox[c] - track.bbox[c]) * alpha;
                }
                out.score = track.score + (target.score - track.score) * alpha;
            }
            next.push_back(std::move(out));
        }
        tracks_ = next;
        return next;
    }

private:
    std::vector<DetectionInstance> tracks_;
};

}  // namespace render
}  // namespace fluent_scene
