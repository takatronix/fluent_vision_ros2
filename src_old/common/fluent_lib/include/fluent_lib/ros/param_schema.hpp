#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <variant>
#include <optional>
#include <sstream>
#include <fstream>
#include <iomanip>

namespace fluent_lib::ros {

// Parameter schema with type/default/min/max and simple YAML serialization.
// No external YAML dependency; emits ROS2-compatible params file.

class ParamSchema {
public:
  enum class Type { Double, Integer, Bool, String };
  using Value = std::variant<double, int64_t, bool, std::string>;

  struct RangeD { double min{0.0}; double max{0.0}; };
  struct RangeI { int64_t min{0}; int64_t max{0}; };

  struct Entry {
    std::string name;
    Type type{Type::Double};
    Value def;                 // default
    std::optional<RangeD> rd;  // for Double
    std::optional<RangeI> ri;  // for Integer
    std::string description;   // optional
    bool dynamic{true};        // allow live updates
  };

  explicit ParamSchema(const rclcpp::Node::SharedPtr &node) : node_(node) {}

  // Fluent builders
  ParamSchema &add_double(const std::string &name, double def,
                          std::optional<RangeD> range = std::nullopt,
                          const std::string &desc = {}, bool dynamic = true)
  { entries_.push_back({name, Type::Double, def, range, std::nullopt, desc, dynamic}); return *this; }

  ParamSchema &add_int(const std::string &name, int64_t def,
                       std::optional<RangeI> range = std::nullopt,
                       const std::string &desc = {}, bool dynamic = true)
  { entries_.push_back({name, Type::Integer, def, std::nullopt, range, desc, dynamic}); return *this; }

  ParamSchema &add_bool(const std::string &name, bool def,
                        const std::string &desc = {}, bool dynamic = true)
  { entries_.push_back({name, Type::Bool, def, std::nullopt, std::nullopt, desc, dynamic}); return *this; }

  ParamSchema &add_string(const std::string &name, const std::string &def,
                          const std::string &desc = {}, bool dynamic = true)
  { entries_.push_back({name, Type::String, def, std::nullopt, std::nullopt, desc, dynamic}); return *this; }

  // Declare and fetch all parameters; install live update callback.
  void apply() {
    for (const auto &e : entries_) { declare_one(e); current_[e.name] = get_one(e); }
    // Live update
    auto shared_current = std::make_shared<Values>(current_);
    auto shared_types   = std::make_shared<Types>(types_map());
    callback_handle_ = node_->add_on_set_parameters_callback(
      [shared_current, shared_types](const std::vector<rclcpp::Parameter>& params){
        for (const auto &p : params) {
          auto it = shared_types->find(p.get_name()); if (it == shared_types->end()) continue;
          switch (it->second) {
            case Type::Double:  if (p.get_type()==rclcpp::ParameterType::PARAMETER_DOUBLE)  (*shared_current)[p.get_name()] = p.as_double(); break;
            case Type::Integer: if (p.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER) (*shared_current)[p.get_name()] = static_cast<int64_t>(p.as_int()); break;
            case Type::Bool:    if (p.get_type()==rclcpp::ParameterType::PARAMETER_BOOL)    (*shared_current)[p.get_name()] = p.as_bool(); break;
            case Type::String:  if (p.get_type()==rclcpp::ParameterType::PARAMETER_STRING)  (*shared_current)[p.get_name()] = p.as_string(); break;
          }
        }
        rcl_interfaces::msg::SetParametersResult res; res.successful = true; return res;
      }
    );
    current_ = *shared_current;
  }

  // Get current value with fallback to default
  template <typename T>
  T get(const std::string &name, const T &fallback) const {
    auto it = current_.find(name); if (it == current_.end()) return fallback; return std::get<T>(it->second);
  }

  // Set current value and update node parameter
  void set(const std::string &name, const Value &v) {
    current_[name] = v;
    // mirror to node param for consistency
    switch (type_of(name)) {
      case Type::Double:  node_->set_parameter(rclcpp::Parameter(name, std::get<double>(v))); break;
      case Type::Integer: node_->set_parameter(rclcpp::Parameter(name, static_cast<int>(std::get<int64_t>(v)))); break;
      case Type::Bool:    node_->set_parameter(rclcpp::Parameter(name, std::get<bool>(v))); break;
      case Type::String:  node_->set_parameter(rclcpp::Parameter(name, std::get<std::string>(v))); break;
    }
  }

  // Serialize current values to a ROS2 params YAML file (single node)
  std::string to_yaml(const std::string &node_name) const {
    std::ostringstream oss;
    oss << node_name << ":\n  ros__parameters:\n";
    for (const auto &e : entries_) {
      oss << "    " << e.name << ": " << encode_value(current_value_or_def(e)) << "\n";
    }
    return oss.str();
  }
  bool save_yaml(const std::string &file_path, const std::string &node_name) const {
    std::ofstream f(file_path);
    if (!f.good()) return false;
    f << to_yaml(node_name);
    return f.good();
  }

private:
  using Values = std::unordered_map<std::string, Value>;
  using Types  = std::unordered_map<std::string, Type>;

  void declare_one(const Entry &e) {
    try {
      switch (e.type) {
        case Type::Double:  (void)node_->declare_parameter<double>(e.name, std::get<double>(e.def)); break;
        case Type::Integer: (void)node_->declare_parameter<int>(e.name, static_cast<int>(std::get<int64_t>(e.def))); break;
        case Type::Bool:    (void)node_->declare_parameter<bool>(e.name, std::get<bool>(e.def)); break;
        case Type::String:  (void)node_->declare_parameter<std::string>(e.name, std::get<std::string>(e.def)); break;
      }
    } catch (...) {}
  }
  Value get_one(const Entry &e) const {
    try {
      switch (e.type) {
        case Type::Double:  return node_->get_parameter(e.name).get_value<double>();
        case Type::Integer: return static_cast<int64_t>(node_->get_parameter(e.name).get_value<int>());
        case Type::Bool:    return node_->get_parameter(e.name).get_value<bool>();
        case Type::String:  return node_->get_parameter(e.name).get_value<std::string>();
      }
    } catch (...) {}
    return e.def;
  }
  static std::string encode_value(const Value &v) {
    std::ostringstream s; s << std::boolalpha;
    if (std::holds_alternative<double>(v)) {
      s << std::fixed << std::setprecision(6) << std::get<double>(v);
    } else if (std::holds_alternative<int64_t>(v)) {
      s << std::get<int64_t>(v);
    } else if (std::holds_alternative<bool>(v)) {
      s << std::get<bool>(v);
    } else {
      s << '"' << escape(std::get<std::string>(v)) << '"';
    }
    return s.str();
  }
  static std::string escape(const std::string &in) {
    std::string out; out.reserve(in.size());
    for (char c : in) { if (c=='"' || c=='\\') out.push_back('\\'); out.push_back(c); }
    return out;
  }
  Value current_value_or_def(const Entry &e) const {
    auto it = current_.find(e.name); return (it!=current_.end()) ? it->second : e.def;
  }
  Types types_map() const {
    Types t; for (const auto &e : entries_) t[e.name] = e.type; return t;
  }
  Type type_of(const std::string &name) const {
    for (const auto &e : entries_) if (e.name == name) return e.type; return Type::Double;
  }

  rclcpp::Node::SharedPtr node_;
  std::vector<Entry> entries_;
  Values current_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

} // namespace fluent_lib::ros


