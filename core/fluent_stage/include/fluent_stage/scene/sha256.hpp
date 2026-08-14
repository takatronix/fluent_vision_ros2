#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace fluent_stage::scene {

// Returns the lowercase hex SHA-256 digest of `data`.
std::string sha256Hex(const void* data, size_t size);
std::string sha256Hex(const std::string& data);

}  // namespace fluent_stage::scene
