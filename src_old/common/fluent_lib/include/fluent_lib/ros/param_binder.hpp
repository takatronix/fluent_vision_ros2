#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <functional>

namespace fluent_lib::ros
{

// ParamBinder: declare → get → live-update を1回の記述で
// 使い方:
//   struct Cfg { double voxel{0.005}; bool enable{true}; std::string topic{"/out"}; } cfg;
//   ParamBinder b(node);
//   b.bind("voxel_leaf_size", cfg.voxel, 0.005)
//    .bind("enable_metrics",  cfg.enable, true)
//    .bind("output_topic",    cfg.topic,  std::string("/out"))
//    .apply(); // declare+get & install on_set_parameters_callback

class ParamBinder {
public:
  explicit ParamBinder(const rclcpp::Node::SharedPtr &node) : node_(node) {}

  // Overloads for basic types
  ParamBinder &bind(const std::string &name, double &var, double def) {
    declare_queue_.push_back([=]{ declare<double>(name, def); });
    getters_.push_back([=,&var]{ var = get<double>(name, def); });
    setters_[name] = [&var](const rclcpp::Parameter &p){ if (p.get_type()==rclcpp::ParameterType::PARAMETER_DOUBLE) var = p.as_double(); };
    return *this;
  }
  ParamBinder &bind(const std::string &name, int &var, int def) {
    declare_queue_.push_back([=]{ declare<int>(name, def); });
    getters_.push_back([=,&var]{ var = get<int>(name, def); });
    setters_[name] = [&var](const rclcpp::Parameter &p){ if (p.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER) var = p.as_int(); };
    return *this;
  }
  ParamBinder &bind(const std::string &name, bool &var, bool def) {
    declare_queue_.push_back([=]{ declare<bool>(name, def); });
    getters_.push_back([=,&var]{ var = get<bool>(name, def); });
    setters_[name] = [&var](const rclcpp::Parameter &p){ if (p.get_type()==rclcpp::ParameterType::PARAMETER_BOOL) var = p.as_bool(); };
    return *this;
  }
  ParamBinder &bind(const std::string &name, std::string &var, const std::string &def) {
    declare_queue_.push_back([=]{ declare<std::string>(name, def); });
    getters_.push_back([=,&var]{ var = get<std::string>(name, def); });
    setters_[name] = [&var](const rclcpp::Parameter &p){ if (p.get_type()==rclcpp::ParameterType::PARAMETER_STRING) var = p.as_string(); };
    return *this;
  }

  // Apply: declare all, read all, and install live-update callback
  void apply() {
    for (auto &d : declare_queue_) d();
    for (auto &g : getters_) g();
    // install callback (keep handle alive inside node via shared_ptr capture)
    auto setters = std::make_shared<std::unordered_map<std::string, Setter>>(std::move(setters_));
    callback_handle_ = node_->add_on_set_parameters_callback(
      [setters](const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult res; res.successful = true;
        for (const auto &p : params) {
          auto it = setters->find(p.get_name());
          if (it != setters->end()) it->second(p);
        }
        return res;
      }
    );
  }

private:
  template<typename T>
  void declare(const std::string &name, const T &def) {
    try { (void)node_->declare_parameter<T>(name, def); }
    catch (...) { /* already declared */ }
  }
  template<typename T>
  T get(const std::string &name, const T &def) {
    try { return node_->get_parameter(name).get_value<T>(); }
    catch (...) { return def; }
  }

  using Setter = std::function<void(const rclcpp::Parameter&)>;
  rclcpp::Node::SharedPtr node_;
  std::vector<std::function<void()>> declare_queue_;
  std::vector<std::function<void()>> getters_;
  std::unordered_map<std::string, Setter> setters_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

} // namespace fluent_lib::ros


