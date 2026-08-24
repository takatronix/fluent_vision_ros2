#pragma once

namespace fluent_lib::ros
{

enum class IoThreadScheduler
{
  kSchedFifo,
  kSchedOther,
};

struct IoThreadConfiguration
{
  IoThreadScheduler scheduler;
  int fifo_priority;
};

/**
 * @brief Configure the calling ROS I/O thread for prompt scheduling.
 *
 * The function requests SCHED_FIFO for the calling thread. Permission failures
 * are logged as warnings and the thread continues with SCHED_OTHER.
 */
IoThreadConfiguration configure_current_io_thread(int fifo_priority = 20);

}  // namespace fluent_lib::ros
