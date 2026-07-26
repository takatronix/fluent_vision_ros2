#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "fv_tts/contracts.hpp"

namespace fv_tts {

std::vector<SynthesisMark> build_synthesis_marks(
    const std::string &text, const nlohmann::json &source_spans,
    const nlohmann::json &audio_query, std::uint64_t actual_total_frames);

}  // namespace fv_tts
