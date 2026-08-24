#pragma once

namespace fv::ros_io
{

/**
 * @brief Bind the calling thread to the host's reserved ROS I/O CPU.
 * @throws std::runtime_error when the host contract or affinity operation is invalid.
 */
void bind_current_thread();

}  // namespace fv::ros_io
