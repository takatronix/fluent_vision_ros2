#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <variant>
#include <functional>

namespace fluent_lib::ros {

// Lightweight parameter dictionary with declare/get and live update support.
// Usage:
//   ParamDict params(node);
//   params.bind("voxel_leaf_size", 0.005)
//         .bind("enable_metrics", true)
//         .bind("output_topic", std::string("/out"))
//         .apply();
//   double v = params.get<double>("voxel_leaf_size");
//   bool em = params.get<bool>("enable_metrics");

class ParamDict {
public:
  using Value = std::variant<double, int64_t, bool, std::string>;

  explicit ParamDict(const rclcpp::Node::SharedPtr &node) : node_(node) {}

  // Bind declarations with defaults (fluent style)
  ParamDict &bind(const std::string &name, double def) {
    decls_.push_back([=]{ declare<double>(name, def); });
    getters_.push_back([=]{ set_value(name, node_get<double>(name, def)); });
    types_[name] = Type::Double; return *this;
  }
  ParamDict &bind(const std::string &name, int64_t def) {
    decls_.push_back([=]{ declare<int64_t>(name, def); });
    getters_.push_back([=]{ set_value(name, node_get<int64_t>(name, def)); });
    types_[name] = Type::Integer; return *this;
  }
  ParamDict &bind(const std::string &name, bool def) {
    decls_.push_back([=]{ declare<bool>(name, def); });
    getters_.push_back([=]{ set_value(name, node_get<bool>(name, def)); });
    types_[name] = Type::Bool; return *this;
  }
  ParamDict &bind(const std::string &name, const std::string &def) {
    decls_.push_back([=]{ declare<std::string>(name, def); });
    getters_.push_back([=]{ set_value(name, node_get<std::string>(name, def)); });
    types_[name] = Type::String; return *this;
  }

  void apply() {
    for (auto &d : decls_) d();
    for (auto &g : getters_) g();
    auto cb_values = std::make_shared<Values>(values_);
    auto cb_types  = std::make_shared<Types>(types_);
    callback_handle_ = node_->add_on_set_parameters_callback(
      [cb_values, cb_types](const std::vector<rclcpp::Parameter> &params){
        for (const auto &p : params) {
          auto it = cb_types->find(p.get_name());
          if (it == cb_types->end()) continue;
          switch (it->second) {
            case Type::Double:  if (p.get_type()==rclcpp::ParameterType::PARAMETER_DOUBLE)  (*cb_values)[p.get_name()] = p.as_double(); break;
            case Type::Integer: if (p.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER) (*cb_values)[p.get_name()] = static_cast<int64_t>(p.as_int()); break;
            case Type::Bool:    if (p.get_type()==rclcpp::ParameterType::PARAMETER_BOOL)    (*cb_values)[p.get_name()] = p.as_bool(); break;
            case Type::String:  if (p.get_type()==rclcpp::ParameterType::PARAMETER_STRING)  (*cb_values)[p.get_name()] = p.as_string(); break;
          }
        }
        rcl_interfaces::msg::SetParametersResult res; res.successful = true; return res;
      }
    );
    // reflect any updates written into shared map back to this instance
    values_ = *cb_values;
  }

  template <typename T>
  T get(const std::string &name) const {
    auto it = values_.find(name);
    if (it == values_.end()) return T{};
    return std::get<T>(it->second);
  }

  template <typename T>
  T get(const std::string &name, const T &fallback) const {
    auto it = values_.find(name);
    if (it == values_.end()) return fallback;
    return std::get<T>(it->second);
  }

  const Value* try_get_raw(const std::string &name) const {
    auto it = values_.find(name); return (it==values_.end()) ? nullptr : &it->second;
  }

private:
  enum class Type { Double, Integer, Bool, String };
  using Values = std::unordered_map<std::string, Value>;
  using Types  = std::unordered_map<std::string, Type>;

  template<typename T>
  void declare(const std::string &name, const T &def) {
    try { (void)node_->declare_parameter<T>(name, def); } catch (...) {}
  }
  template<typename T>
  T node_get(const std::string &name, const T &def) const {
    try { return node_->get_parameter(name).get_value<T>(); } catch (...) { return def; }
  }
  void set_value(const std::string &name, const Value &v) { values_[name] = v; }

  rclcpp::Node::SharedPtr node_;
  std::vector<std::function<void()>> decls_;
  std::vector<std::function<void()>> getters_;
  Values values_;
  Types  types_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

} // namespace fluent_lib::ros


