#include "fluent_lib/ros/io_thread.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pthread.h>
#include <sched.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace fluent_lib::ros
{
namespace
{

class SchedulerPermissionError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

void setCurrentThreadOther()
{
  sched_param parameters{};
  const int set_result = pthread_setschedparam(pthread_self(), SCHED_OTHER, &parameters);
  if (set_result != 0) {
    throw std::runtime_error(
      std::string("pthread_setschedparam(SCHED_OTHER) failed: ") +
      std::strerror(set_result));
  }

  int applied_policy = 0;
  sched_param applied_parameters{};
  const int get_result = pthread_getschedparam(
    pthread_self(), &applied_policy, &applied_parameters);
  if (get_result != 0) {
    throw std::runtime_error(
      std::string("pthread_getschedparam failed: ") + std::strerror(get_result));
  }
  if (applied_policy != SCHED_OTHER || applied_parameters.sched_priority != 0) {
    throw std::runtime_error("SCHED_OTHER verification failed");
  }
}

void setCurrentThreadFifo(int priority)
{
  const int minimum = sched_get_priority_min(SCHED_FIFO);
  const int maximum = sched_get_priority_max(SCHED_FIFO);
  if (minimum < 0 || maximum < 0) {
    throw std::runtime_error(
      std::string("cannot read SCHED_FIFO priority range: ") + std::strerror(errno));
  }
  if (priority < minimum || priority > maximum) {
    throw std::invalid_argument(
      "SCHED_FIFO priority " + std::to_string(priority) + " is outside " +
      std::to_string(minimum) + "-" + std::to_string(maximum));
  }

  sched_param parameters{};
  parameters.sched_priority = priority;
  const int set_result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters);
  if (set_result == EPERM || set_result == EACCES) {
    throw SchedulerPermissionError(
      "pthread_setschedparam(SCHED_FIFO, priority=" + std::to_string(priority) +
      ") failed: " + std::strerror(set_result));
  }
  if (set_result != 0) {
    throw std::runtime_error(
      "pthread_setschedparam(SCHED_FIFO, priority=" + std::to_string(priority) +
      ") failed: " + std::strerror(set_result));
  }

  int applied_policy = 0;
  sched_param applied_parameters{};
  const int get_result = pthread_getschedparam(
    pthread_self(), &applied_policy, &applied_parameters);
  if (get_result != 0) {
    throw std::runtime_error(
      std::string("pthread_getschedparam failed: ") + std::strerror(get_result));
  }
  if (applied_policy != SCHED_FIFO || applied_parameters.sched_priority != priority) {
    throw std::runtime_error("SCHED_FIFO verification failed");
  }
}

}  // namespace

IoThreadConfiguration configure_current_io_thread(int fifo_priority)
{
  const rclcpp::Logger logger = rclcpp::get_logger("fluent_lib.ros_io");
  try {
    setCurrentThreadFifo(fifo_priority);
    RCLCPP_INFO(
      logger, "ROS I/O thread scheduler=SCHED_FIFO priority=%d", fifo_priority);
    return {IoThreadScheduler::kSchedFifo, fifo_priority};
  } catch (const SchedulerPermissionError& error) {
    RCLCPP_WARN(logger, "SCHED_FIFO unavailable: %s", error.what());
  }

  setCurrentThreadOther();
  RCLCPP_INFO(logger, "ROS I/O thread scheduler=SCHED_OTHER priority=0");
  return {IoThreadScheduler::kSchedOther, 0};
}

}  // namespace fluent_lib::ros
