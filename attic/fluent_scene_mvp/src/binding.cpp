#include "fluent_scene/binding.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fluent_scene {
namespace {

void bindError(DiagnosticList& diagnostics, std::string code, std::string message,
               std::vector<std::pair<std::string, std::string>> context = {}) {
    diagnostics.add(std::move(code), Severity::kError, Phase::kBind, Span{}, std::move(message),
                    std::move(context));
}

const JsonValue* findByName(const JsonValue* array, const std::string& value) {
    if (array == nullptr || !array->isArray()) {
        return nullptr;
    }
    for (const JsonValue& element : array->elements()) {
        const JsonValue* name = element.find("name");
        if (name != nullptr && name->stringValue() == value) {
            return &element;
        }
    }
    return nullptr;
}

TypedValue::Kind kindFromTypeName(const std::string& type_name, bool& supported) {
    supported = true;
    if (type_name.rfind("image.", 0) == 0) {
        return TypedValue::Kind::kImage;
    }
    if (type_name.rfind("sequence<", 0) == 0) {
        return TypedValue::Kind::kDetections;
    }
    if (type_name == "points2d" || type_name == "polyline2d") {
        return TypedValue::Kind::kPoints;
    }
    if (type_name == "string") {
        return TypedValue::Kind::kString;
    }
    if (type_name == "calibration") {
        return TypedValue::Kind::kCalibration;
    }
    supported = false;
    return TypedValue::Kind::kString;
}

// Converts a normalized fallback literal from the canonical IR into a typed
// value. Returns null for `null` fallbacks (meaning: hide the input).
std::shared_ptr<const TypedValue> fallbackValueFromJson(const JsonValue& json,
                                                        TypedValue::Kind kind) {
    if (json.isNull()) {
        return nullptr;
    }
    auto value = std::make_shared<TypedValue>();
    value->kind = kind;
    value->meta.has_timestamp = false;
    switch (kind) {
        case TypedValue::Kind::kString:
            value->text = json.stringValue();
            break;
        case TypedValue::Kind::kDetections:
            for (const JsonValue& element : json.elements()) {
                DetectionInstance detection;
                const JsonValue* bbox = element.find("bbox");
                if (bbox != nullptr && bbox->isArray() && bbox->elements().size() == 4) {
                    for (size_t i = 0; i < 4; ++i) {
                        detection.bbox[i] = static_cast<float>(bbox->elements()[i].floatValue());
                    }
                }
                const JsonValue* score = element.find("score");
                if (score != nullptr) {
                    detection.score = static_cast<float>(score->floatValue());
                }
                const JsonValue* label = element.find("label");
                if (label != nullptr) {
                    detection.label = label->stringValue();
                }
                value->detections.push_back(std::move(detection));
            }
            break;
        case TypedValue::Kind::kCalibration:
            value->calibration_id = json.stringValue();
            break;
        case TypedValue::Kind::kImage:
        case TypedValue::Kind::kPoints:
            // No literal representation; unreachable by schema.
            return nullptr;
    }
    return value;
}

}  // namespace

const char* toString(EntryStatus status) {
    switch (status) {
        case EntryStatus::kFresh: return "fresh";
        case EntryStatus::kHeld: return "held";
        case EntryStatus::kFallbackValue: return "fallback_value";
        case EntryStatus::kFallbackUnavailable: return "fallback_unavailable";
        case EntryStatus::kMissing: return "missing";
    }
    return "unknown";
}

FrameInputs FrameSnapshot::toFrameInputs() const {
    FrameInputs inputs;
    for (const auto& [name, entry] : entries) {
        if (!entry.valid || entry.value == nullptr) {
            continue;
        }
        switch (entry.value->kind) {
            case TypedValue::Kind::kImage:
                inputs.images[name] =
                    CpuImageView{entry.value->width, entry.value->height, entry.value->pixels.data()};
                break;
            case TypedValue::Kind::kDetections:
                inputs.detections[name] = entry.value->detections;
                break;
            case TypedValue::Kind::kPoints:
                inputs.points[name] = entry.value->points;
                break;
            case TypedValue::Kind::kString:
                inputs.strings[name] = entry.value->text;
                break;
            case TypedValue::Kind::kCalibration:
                break;  // consumed by metadata contracts, not by draws
        }
    }
    return inputs;
}

std::unique_ptr<BindingTable> BindingTable::create(const ValidationResult& scene,
                                                   const BindingTableOptions& options,
                                                   DiagnosticList& diagnostics) {
    if (!scene.ok) {
        bindError(diagnostics, "bind.invalid_input", "cannot bind a scene that did not validate");
        return nullptr;
    }
    auto table = std::unique_ptr<BindingTable>(new BindingTable());
    table->groups_ = options.groups;

    const JsonValue* inputs = scene.ir.find("inputs");
    const JsonValue* fallbacks = scene.ir.find("fallbacks");
    if (inputs == nullptr || !inputs->isArray()) {
        return table;  // a scene without inputs is valid, just static
    }
    for (const JsonValue& input : inputs->elements()) {
        const std::string name = input.find("name")->stringValue();
        InputState state;
        bool supported = true;
        state.kind = kindFromTypeName(input.find("type")->stringValue(), supported);
        if (!supported) {
            bindError(diagnostics, "bind.unsupported_type",
                      "input \"" + name + "\" has a type the MVP binding layer cannot carry",
                      {{"input", name}});
            return nullptr;
        }
        state.required = input.find("required")->boolValue();
        state.update = input.find("update")->stringValue();
        const JsonValue* metadata = input.find("metadata");
        if (metadata != nullptr) {
            const JsonValue* group = metadata->find("synchronization_group");
            if (group != nullptr) {
                state.group = group->stringValue();
            }
        }
        // Queue defaults: grouped per_frame inputs match; ungrouped per_frame
        // keep the latest; on_change/static hold a single value.
        auto override_it = options.inputs.find(name);
        if (override_it != options.inputs.end()) {
            state.config = override_it->second;
        } else if (!state.group.empty() && state.update == "per_frame") {
            state.config.policy = QueuePolicy::kMatched;
            state.config.capacity = 4;
        } else {
            state.config.policy = QueuePolicy::kLatest;
            state.config.capacity = 1;
        }
        state.config.capacity = std::max<size_t>(1, state.config.capacity);
        if (state.config.max_age_seconds > 0.0) {
            state.max_age = state.config.max_age_seconds;
        } else if (state.update == "per_frame") {
            state.max_age = options.default_per_frame_max_age;
        }
        const JsonValue* fallback_name = input.find("fallback");
        if (fallback_name != nullptr) {
            const JsonValue* fallback = findByName(fallbacks, fallback_name->stringValue());
            if (fallback != nullptr) {
                state.has_fallback = true;
                if (fallback->find("behavior") != nullptr) {
                    state.fallback_is_behavior = true;
                } else if (const JsonValue* value = fallback->find("value")) {
                    state.fallback_value = fallbackValueFromJson(*value, state.kind);
                }
            }
        }
        if (!state.group.empty() && state.update == "per_frame" &&
            table->groups_.count(state.group) == 0) {
            table->groups_.emplace(state.group, SyncGroupConfig{});
        }
        table->inputs_.emplace(name, std::move(state));
    }
    return table;
}

bool BindingTable::push(const std::string& input, TypedValue value, DiagnosticList& diagnostics) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = inputs_.find(input);
    if (it == inputs_.end()) {
        bindError(diagnostics, "bind.unknown_input",
                  "push to undeclared input \"" + input + "\"", {{"input", input}});
        return false;
    }
    InputState& state = it->second;
    if (value.kind != state.kind) {
        bindError(diagnostics, "bind.type_mismatch",
                  "pushed value kind does not match the declared type of input \"" + input + "\"",
                  {{"input", input}});
        return false;
    }
    if (!value.meta.has_timestamp) {
        bindError(diagnostics, "bind.missing_timestamp",
                  "values must carry an explicit source timestamp (input \"" + input + "\")",
                  {{"input", input}});
        return false;
    }
    ++state.stats.pushes;
    state.queue.push_back(std::make_shared<const TypedValue>(std::move(value)));
    while (state.queue.size() > state.config.capacity) {
        // Bounded backpressure: latest keeps the newest tail, fifo and matched
        // drop the oldest entry. Either way eviction is counted.
        state.queue.pop_front();
        ++state.stats.drops;
    }
    return true;
}

void BindingTable::selectFresh(InputState& state, std::shared_ptr<const TypedValue> value,
                               SnapshotEntry& entry, double now) {
    entry.value = value;
    entry.status = EntryStatus::kFresh;
    entry.valid = true;
    entry.sequence = value->meta.sequence;
    entry.age_seconds = std::max(0.0, now - value->meta.timestamp);
    state.last_selected = std::move(value);
}

void BindingTable::resolveAbsent(const std::string& name, InputState& state, SnapshotEntry& entry,
                                 FrameSnapshot& snapshot, bool stale) {
    if (stale) {
        ++state.stats.stale;
    }
    if (state.has_fallback) {
        ++state.stats.fallbacks;
        if (state.fallback_is_behavior) {
            entry.status = EntryStatus::kFallbackUnavailable;
            entry.valid = false;
        } else {
            entry.status = EntryStatus::kFallbackValue;
            entry.value = state.fallback_value;  // null => declared hide
            entry.valid = entry.value != nullptr;
        }
        return;
    }
    if (state.required) {
        // Undeclared missing required input: the frame is unhealthy
        // (spec section 9). Keep the last value for observability.
        entry.status = EntryStatus::kMissing;
        entry.value = state.last_selected;
        entry.valid = false;
        snapshot.healthy = false;
        return;
    }
    if (state.last_selected != nullptr) {
        ++state.stats.held;
        entry.status = EntryStatus::kHeld;
        entry.value = state.last_selected;
        entry.valid = true;
        return;
    }
    entry.status = EntryStatus::kMissing;
    entry.valid = false;
    (void)name;
}

FrameSnapshot BindingTable::acquireSnapshot(double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    FrameSnapshot snapshot;
    snapshot.index = snapshot_count_++;
    snapshot.now = now;

    // ---- Pass 1: approximate matching per synchronization group ------------
    std::map<std::string, std::shared_ptr<const TypedValue>> matched;
    for (const auto& [group_name, group_config] : groups_) {
        std::vector<std::pair<std::string, InputState*>> members;
        for (auto& [input_name, state] : inputs_) {
            if (state.group == group_name && state.update == "per_frame" &&
                state.config.policy == QueuePolicy::kMatched) {
                members.emplace_back(input_name, &state);
            }
        }
        if (members.empty()) {
            continue;
        }
        // Membership semantics:
        // - Members with queued data participate as candidates.
        // - An absent OPTIONAL member never blocks the group (the section-11
        //   depth input may never be fed; a stalled optional stream must not
        //   freeze the required ones).
        // - An absent REQUIRED member participates through the timestamp of
        //   its held value, so a value that arrives one tick after its peer
        //   still pairs with it; with no held value at all the set is
        //   incomplete and the match fails.
        std::vector<std::pair<std::string, InputState*>> present;
        std::vector<double> held_required_times;
        bool required_missing = false;
        for (const auto& member : members) {
            if (!member.second->queue.empty()) {
                present.push_back(member);
            } else if (member.second->required) {
                if (member.second->last_selected != nullptr) {
                    held_required_times.push_back(member.second->last_selected->meta.timestamp);
                } else {
                    required_missing = true;
                }
            }
        }
        if (present.empty()) {
            continue;  // nothing arrived; idle, not a failure
        }
        bool matched_ok = !required_missing;
        std::vector<std::shared_ptr<const TypedValue>> candidates;
        if (matched_ok) {
            const auto newestTimestamp = [](const InputState& state) {
                double newest = -std::numeric_limits<double>::infinity();
                for (const auto& value : state.queue) {
                    newest = std::max(newest, value->meta.timestamp);
                }
                return newest;
            };
            // Anchor: the laggard's newest timestamp among present members,
            // never newer than a held required member.
            double anchor = std::numeric_limits<double>::infinity();
            for (const auto& [input_name, state] : present) {
                anchor = std::min(anchor, newestTimestamp(*state));
            }
            for (double held : held_required_times) {
                anchor = std::min(anchor, held);
            }
            double min_time = anchor;
            double max_time = anchor;
            for (const auto& [input_name, state] : present) {
                std::shared_ptr<const TypedValue> best;
                double best_distance = std::numeric_limits<double>::infinity();
                for (const auto& value : state->queue) {
                    const double distance = std::fabs(value->meta.timestamp - anchor);
                    if (distance <= best_distance) {  // ties resolve to the newest
                        best_distance = distance;
                        best = value;
                    }
                }
                candidates.push_back(best);
                min_time = std::min(min_time, best->meta.timestamp);
                max_time = std::max(max_time, best->meta.timestamp);
            }
            for (double held : held_required_times) {
                min_time = std::min(min_time, held);
                max_time = std::max(max_time, held);
            }
            matched_ok = (max_time - min_time) <= group_config.tolerance_seconds;
        }
        if (!matched_ok) {
            ++match_failures_;
            continue;  // members fall through to hold/fallback in pass 2
        }
        for (size_t i = 0; i < present.size(); ++i) {
            InputState& state = *present[i].second;
            const std::shared_ptr<const TypedValue>& selected = candidates[i];
            // Consume everything up to and including the selected value.
            while (!state.queue.empty() &&
                   state.queue.front()->meta.timestamp <= selected->meta.timestamp) {
                state.queue.pop_front();
            }
            matched[present[i].first] = selected;
        }
    }

    // ---- Pass 2: per-input selection, freshness, and fallback --------------
    for (auto& [name, state] : inputs_) {
        SnapshotEntry entry;
        std::shared_ptr<const TypedValue> selected;
        auto matched_it = matched.find(name);
        if (matched_it != matched.end()) {
            selected = matched_it->second;
        } else if (state.config.policy == QueuePolicy::kFifo) {
            if (!state.queue.empty()) {
                selected = state.queue.front();
                state.queue.pop_front();
            }
        } else if (state.config.policy == QueuePolicy::kLatest ||
                   state.config.policy == QueuePolicy::kMatched) {
            // kMatched lands here only when its group failed to match this
            // frame; the queue is preserved so a later frame can match.
            if (state.config.policy == QueuePolicy::kLatest && !state.queue.empty()) {
                selected = state.queue.back();
                state.queue.clear();
            }
        }
        if (selected != nullptr) {
            const double age = now - selected->meta.timestamp;
            if (state.max_age > 0.0 && age > state.max_age) {
                resolveAbsent(name, state, entry, snapshot, /*stale=*/true);
            } else {
                selectFresh(state, std::move(selected), entry, now);
            }
        } else if (state.last_selected != nullptr) {
            const double age = now - state.last_selected->meta.timestamp;
            if (state.max_age > 0.0 && age > state.max_age) {
                resolveAbsent(name, state, entry, snapshot, /*stale=*/true);
            } else {
                ++state.stats.held;
                entry.status = EntryStatus::kHeld;
                entry.value = state.last_selected;
                entry.valid = true;
                entry.sequence = state.last_selected->meta.sequence;
                entry.age_seconds = std::max(0.0, age);
            }
        } else {
            resolveAbsent(name, state, entry, snapshot, /*stale=*/false);
        }
        snapshot.entries.emplace(name, std::move(entry));
    }
    return snapshot;
}

BindingStats BindingTable::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    BindingStats stats;
    stats.snapshots = snapshot_count_;
    stats.match_failures = match_failures_;
    for (const auto& [name, state] : inputs_) {
        stats.inputs[name] = state.stats;
    }
    return stats;
}

}  // namespace fluent_scene
