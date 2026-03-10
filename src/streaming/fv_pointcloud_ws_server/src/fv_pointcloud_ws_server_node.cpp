#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/system/error_code.hpp>
#include <lz4.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <limits>
#include <vector>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace
{

template<typename T>
T clamp_value(T v, T lo, T hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

uint32_t bswap32(uint32_t v)
{
  return ((v & 0x000000FFu) << 24) |
         ((v & 0x0000FF00u) << 8) |
         ((v & 0x00FF0000u) >> 8) |
         ((v & 0xFF000000u) >> 24);
}

uint32_t read_u32(const uint8_t * ptr, bool is_bigendian)
{
  uint32_t v = 0;
  std::memcpy(&v, ptr, sizeof(uint32_t));
  if (is_bigendian) {
    v = bswap32(v);
  }
  return v;
}

float read_f32(const uint8_t * ptr, bool is_bigendian)
{
  uint32_t u = read_u32(ptr, is_bigendian);
  float f = 0.0f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}

void append_f32(std::vector<uint8_t> & out, float value)
{
  const auto * p = reinterpret_cast<const uint8_t *>(&value);
  out.insert(out.end(), p, p + sizeof(float));
}

enum class CompressionCodec : uint8_t
{
  None = 0,
  Lz4 = 1,
};

CompressionCodec parse_codec(std::string codec_name)
{
  std::transform(
    codec_name.begin(), codec_name.end(), codec_name.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});

  if (codec_name == "lz4") {
    return CompressionCodec::Lz4;
  }
  return CompressionCodec::None;
}

const char * codec_to_string(CompressionCodec codec)
{
  switch (codec) {
    case CompressionCodec::Lz4:
      return "lz4";
    case CompressionCodec::None:
    default:
      return "none";
  }
}

}  // namespace

#pragma pack(push, 1)
struct FvPcHeaderV1
{
  char magic[4];
  uint16_t version;
  uint16_t header_size;
  uint64_t stamp_ns;
  uint32_t seq;
  uint32_t point_count;
  uint8_t point_format;
  uint8_t flags;
  uint16_t reserved;
  float min_range_m;
  float max_range_m;
};

struct FvPcHeaderV2
{
  char magic[4];
  uint16_t version;
  uint16_t header_size;
  uint64_t stamp_ns;
  uint32_t seq;
  uint32_t point_count;
  uint8_t point_format;
  uint8_t flags;
  uint16_t reserved;
  float min_range_m;
  float max_range_m;
  uint8_t codec;
  uint8_t codec_level;
  uint16_t reserved2;
  uint32_t payload_size;
  uint32_t payload_raw_size;
};
#pragma pack(pop)

class ClientSession
{
public:
  explicit ClientSession(tcp::socket socket)
  : ws_(std::move(socket))
  {
    ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    ws_.set_option(websocket::stream_base::decorator(
      [](websocket::response_type & res) {
        res.set(beast::http::field::server, "fv_pointcloud_ws_server");
      }));
  }

  bool accept(beast::error_code & ec)
  {
    ws_.accept(ec);
    return !ec;
  }

  bool send_binary(const std::vector<uint8_t> & data, beast::error_code & ec)
  {
    std::lock_guard<std::mutex> lk(write_mutex_);
    ws_.binary(true);
    ws_.write(net::buffer(data), ec);
    return !ec;
  }

  void close()
  {
    beast::error_code ec;
    std::lock_guard<std::mutex> lk(write_mutex_);
    ws_.close(websocket::close_code::normal, ec);
  }

private:
  websocket::stream<tcp::socket> ws_;
  std::mutex write_mutex_;
};

class FVPointCloudWSServer : public rclcpp::Node
{
public:
  FVPointCloudWSServer()
  : Node("fv_pointcloud_ws_server"),
    ioc_(1),
    running_(false),
    seq_(0),
    last_stats_log_sec_(0.0)
  {
    this->declare_parameter<std::string>("pointcloud_topic", "/fv/d405/depth/points");
    this->declare_parameter<std::string>("server.host", "0.0.0.0");
    this->declare_parameter<int>("server.port", 9070);
    this->declare_parameter<int>("stream.max_points", 20000);
    this->declare_parameter<int>("stream.sample_step", 2);
    this->declare_parameter<double>("stream.min_range_m", 0.20);
    this->declare_parameter<double>("stream.max_range_m", 2.50);
    this->declare_parameter<std::string>("stream.codec", "none");
    this->declare_parameter<int>("stream.compression_min_bytes", 65536);
    this->declare_parameter<double>("lazy.cooldown_sec", 5.0);

    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    server_host_ = this->get_parameter("server.host").as_string();
    server_port_ = this->get_parameter("server.port").as_int();
    max_points_ = this->get_parameter("stream.max_points").as_int();
    sample_step_ = this->get_parameter("stream.sample_step").as_int();
    min_range_m_ = this->get_parameter("stream.min_range_m").as_double();
    max_range_m_ = this->get_parameter("stream.max_range_m").as_double();
    stream_codec_name_ = this->get_parameter("stream.codec").as_string();
    stream_codec_ = parse_codec(stream_codec_name_);
    compression_min_bytes_ = this->get_parameter("stream.compression_min_bytes").as_int();
    lazy_cooldown_sec_ = this->get_parameter("lazy.cooldown_sec").as_double();

    max_points_ = std::max(1000, max_points_);
    sample_step_ = std::max(1, sample_step_);
    min_range_m_ = clamp_value(min_range_m_, 0.0, 50.0);
    max_range_m_ = std::max(min_range_m_ + 0.05, max_range_m_);
    compression_min_bytes_ = clamp_value(compression_min_bytes_, 0, 8 * 1024 * 1024);
    lazy_cooldown_sec_ = clamp_value(lazy_cooldown_sec_, 0.1, 60.0);

    RCLCPP_INFO(this->get_logger(), "Initializing FV PointCloud WS Server");
    RCLCPP_INFO(this->get_logger(), "  pointcloud_topic: %s", pointcloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  ws endpoint: ws://%s:%d", server_host_.c_str(), server_port_);
    RCLCPP_INFO(this->get_logger(), "  max_points: %d, sample_step: %d", max_points_, sample_step_);
    RCLCPP_INFO(this->get_logger(), "  range[m]: %.2f - %.2f", min_range_m_, max_range_m_);
    RCLCPP_INFO(
      this->get_logger(), "  codec: %s (compression_min_bytes=%d)",
      codec_to_string(stream_codec_), compression_min_bytes_);

    start_server();

    using namespace std::chrono_literals;
    lazy_timer_ = this->create_wall_timer(
      500ms, std::bind(&FVPointCloudWSServer::update_subscription_state, this));
  }

  ~FVPointCloudWSServer() override
  {
    running_.store(false);
    stop_server();
    if (accept_thread_.joinable()) {
      accept_thread_.join();
    }
    pointcloud_sub_.reset();
  }

private:
  struct FieldRef
  {
    int offset = -1;
    uint8_t datatype = 0;
  };

  void start_server()
  {
    boost::system::error_code ec;

    auto addr = net::ip::make_address(server_host_, ec);
    if (ec) {
      RCLCPP_WARN(
        this->get_logger(), "Invalid server.host '%s'. Fallback to 0.0.0.0",
        server_host_.c_str());
      addr = net::ip::make_address("0.0.0.0");
    }

    acceptor_ = std::make_unique<tcp::acceptor>(ioc_);

    acceptor_->open(tcp::v4(), ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Acceptor open failed: %s", ec.message().c_str());
      return;
    }

    acceptor_->set_option(net::socket_base::reuse_address(true), ec);
    if (ec) {
      RCLCPP_WARN(this->get_logger(), "set_option(reuse_address) failed: %s", ec.message().c_str());
    }

    acceptor_->bind(tcp::endpoint(addr, static_cast<unsigned short>(server_port_)), ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Acceptor bind failed: %s", ec.message().c_str());
      return;
    }

    acceptor_->listen(net::socket_base::max_listen_connections, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Acceptor listen failed: %s", ec.message().c_str());
      return;
    }

    running_.store(true);
    accept_thread_ = std::thread(&FVPointCloudWSServer::accept_loop, this);

    RCLCPP_INFO(
      this->get_logger(),
      "PointCloud WS server started at ws://%s:%d",
      addr.to_string().c_str(), server_port_);
  }

  void stop_server()
  {
    boost::system::error_code ec;
    if (acceptor_) {
      acceptor_->close(ec);
    }

    std::vector<std::shared_ptr<ClientSession>> to_close;
    {
      std::lock_guard<std::mutex> lk(sessions_mutex_);
      to_close.swap(sessions_);
    }

    for (const auto & s : to_close) {
      if (s) {
        s->close();
      }
    }

    ioc_.stop();
  }

  void accept_loop()
  {
    while (running_.load()) {
      tcp::socket socket(ioc_);
      boost::system::error_code ec;
      acceptor_->accept(socket, ec);

      if (ec) {
        if (!running_.load()) {
          break;
        }
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Accept failed: %s", ec.message().c_str());
        continue;
      }

      auto session = std::make_shared<ClientSession>(std::move(socket));
      beast::error_code ws_ec;
      if (!session->accept(ws_ec)) {
        RCLCPP_WARN(this->get_logger(), "WS handshake failed: %s", ws_ec.message().c_str());
        continue;
      }

      {
        std::lock_guard<std::mutex> lk(sessions_mutex_);
        sessions_.push_back(session);
        no_client_since_valid_ = false;
        RCLCPP_INFO(this->get_logger(), "WS client connected. clients=%zu", sessions_.size());
      }

      ensure_subscription();
    }
  }

  bool has_clients()
  {
    std::lock_guard<std::mutex> lk(sessions_mutex_);
    return !sessions_.empty();
  }

  void ensure_subscription()
  {
    if (pointcloud_sub_) {
      return;
    }
    if (pointcloud_topic_.empty()) {
      RCLCPP_WARN(this->get_logger(), "pointcloud_topic is empty. subscription is not created");
      return;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, qos,
      std::bind(&FVPointCloudWSServer::pointcloud_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", pointcloud_topic_.c_str());
  }

  void update_subscription_state()
  {
    if (has_clients()) {
      no_client_since_valid_ = false;
      ensure_subscription();
      return;
    }

    if (!pointcloud_sub_) {
      return;
    }

    if (!no_client_since_valid_) {
      no_client_since_ = this->now();
      no_client_since_valid_ = true;
      return;
    }

    const double idle_sec = (this->now() - no_client_since_).seconds();
    if (idle_sec >= lazy_cooldown_sec_) {
      pointcloud_sub_.reset();
      no_client_since_valid_ = false;
      RCLCPP_INFO(
        this->get_logger(), "No WS clients for %.1fs, unsubscribed from %s",
        idle_sec, pointcloud_topic_.c_str());
    }
  }

  std::vector<uint8_t> build_binary_frame(const sensor_msgs::msg::PointCloud2 & msg)
  {
    FieldRef fx, fy, fz, frgb;
    for (const auto & f : msg.fields) {
      if (f.name == "x") {
        fx.offset = static_cast<int>(f.offset);
        fx.datatype = f.datatype;
      } else if (f.name == "y") {
        fy.offset = static_cast<int>(f.offset);
        fy.datatype = f.datatype;
      } else if (f.name == "z") {
        fz.offset = static_cast<int>(f.offset);
        fz.datatype = f.datatype;
      } else if (f.name == "rgb" || f.name == "rgba") {
        frgb.offset = static_cast<int>(f.offset);
        frgb.datatype = f.datatype;
      }
    }

    if (fx.offset < 0 || fy.offset < 0 || fz.offset < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "PointCloud2 missing x/y/z fields");
      return {};
    }

    if (fx.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      fy.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      fz.datatype != sensor_msgs::msg::PointField::FLOAT32)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Only FLOAT32 x/y/z are supported in v1");
      return {};
    }

    const auto & data = msg.data;
    const size_t point_step = static_cast<size_t>(msg.point_step);
    if (point_step < 12 || data.size() < point_step) {
      return {};
    }

    const size_t count_by_shape = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    const size_t count_by_bytes = data.size() / point_step;
    const size_t point_count = std::min(count_by_shape > 0 ? count_by_shape : count_by_bytes, count_by_bytes);
    if (point_count == 0) {
      return {};
    }

    const bool bigend = msg.is_bigendian;
    const int sample = std::max(1, sample_step_);

    std::vector<uint8_t> payload;
    payload.reserve(static_cast<size_t>(max_points_) * 16);

    uint32_t out_points = 0;
    for (size_t i = 0; i < point_count; i += static_cast<size_t>(sample)) {
      const size_t base = i * point_step;
      const size_t xoff = static_cast<size_t>(fx.offset);
      const size_t yoff = static_cast<size_t>(fy.offset);
      const size_t zoff = static_cast<size_t>(fz.offset);

      if (base + zoff + 4 > data.size()) {
        break;
      }

      const float x = read_f32(&data[base + xoff], bigend);
      const float y = read_f32(&data[base + yoff], bigend);
      const float z = read_f32(&data[base + zoff], bigend);

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const double dist = std::sqrt(static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z);
      if (!std::isfinite(dist) || dist < min_range_m_ || dist > max_range_m_) {
        continue;
      }

      uint8_t r = 80;
      uint8_t g = 180;
      uint8_t b = 255;
      uint8_t a = 255;

      if (frgb.offset >= 0 && base + static_cast<size_t>(frgb.offset) + 4 <= data.size()) {
        const uint32_t packed = read_u32(&data[base + static_cast<size_t>(frgb.offset)], bigend);
        r = static_cast<uint8_t>((packed >> 16) & 0xFFu);
        g = static_cast<uint8_t>((packed >> 8) & 0xFFu);
        b = static_cast<uint8_t>(packed & 0xFFu);
      }

      append_f32(payload, x);
      append_f32(payload, y);
      append_f32(payload, z);
      payload.push_back(r);
      payload.push_back(g);
      payload.push_back(b);
      payload.push_back(a);

      ++out_points;
      if (out_points >= static_cast<uint32_t>(max_points_)) {
        break;
      }
    }

    if (out_points == 0) {
      return {};
    }

    const uint32_t frame_seq = ++seq_;
    const uint64_t stamp_ns =
      static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(msg.header.stamp.nanosec);

    if (
      stream_codec_ == CompressionCodec::Lz4 &&
      static_cast<int>(payload.size()) >= compression_min_bytes_ &&
      payload.size() <= static_cast<size_t>(std::numeric_limits<int>::max()))
    {
      const int raw_size = static_cast<int>(payload.size());
      const int max_comp_size = LZ4_compressBound(raw_size);
      std::vector<uint8_t> compressed(static_cast<size_t>(max_comp_size));
      const int comp_size = LZ4_compress_default(
        reinterpret_cast<const char *>(payload.data()),
        reinterpret_cast<char *>(compressed.data()),
        raw_size, max_comp_size);

      if (comp_size > 0 && comp_size < raw_size) {
        FvPcHeaderV2 hdr{};
        std::memcpy(hdr.magic, "FVPC", 4);
        hdr.version = 2;
        hdr.header_size = static_cast<uint16_t>(sizeof(FvPcHeaderV2));
        hdr.stamp_ns = stamp_ns;
        hdr.seq = frame_seq;
        hdr.point_count = out_points;
        hdr.point_format = 0;
        hdr.flags = 0x03;  // little-endian + compressed payload
        hdr.reserved = 0;
        hdr.min_range_m = static_cast<float>(min_range_m_);
        hdr.max_range_m = static_cast<float>(max_range_m_);
        hdr.codec = static_cast<uint8_t>(CompressionCodec::Lz4);
        hdr.codec_level = 0;
        hdr.reserved2 = 0;
        hdr.payload_size = static_cast<uint32_t>(comp_size);
        hdr.payload_raw_size = static_cast<uint32_t>(raw_size);

        std::vector<uint8_t> frame(sizeof(FvPcHeaderV2) + static_cast<size_t>(comp_size));
        std::memcpy(frame.data(), &hdr, sizeof(FvPcHeaderV2));
        std::memcpy(frame.data() + sizeof(FvPcHeaderV2), compressed.data(), static_cast<size_t>(comp_size));
        return frame;
      }
    }

    FvPcHeaderV1 hdr{};
    std::memcpy(hdr.magic, "FVPC", 4);
    hdr.version = 1;
    hdr.header_size = static_cast<uint16_t>(sizeof(FvPcHeaderV1));
    hdr.stamp_ns =
      static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(msg.header.stamp.nanosec);
    hdr.seq = frame_seq;
    hdr.point_count = out_points;
    hdr.point_format = 0;
    hdr.flags = 0x01;
    hdr.reserved = 0;
    hdr.min_range_m = static_cast<float>(min_range_m_);
    hdr.max_range_m = static_cast<float>(max_range_m_);

    std::vector<uint8_t> frame(sizeof(FvPcHeaderV1) + payload.size());
    std::memcpy(frame.data(), &hdr, sizeof(FvPcHeaderV1));
    std::memcpy(frame.data() + sizeof(FvPcHeaderV1), payload.data(), payload.size());
    return frame;
  }

  void broadcast_binary(const std::vector<uint8_t> & frame)
  {
    std::lock_guard<std::mutex> lk(sessions_mutex_);
    for (auto it = sessions_.begin(); it != sessions_.end();) {
      beast::error_code ec;
      if (!(*it)->send_binary(frame, ec)) {
        (*it)->close();
        it = sessions_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg || !has_clients()) {
      return;
    }

    ++frames_in_;
    auto frame = build_binary_frame(*msg);
    if (frame.empty()) {
      return;
    }

    broadcast_binary(frame);
    ++frames_out_;

    const double now_sec = this->now().seconds();
    if ((now_sec - last_stats_log_sec_) > 2.0) {
      size_t client_count = 0;
      {
        std::lock_guard<std::mutex> lk(sessions_mutex_);
        client_count = sessions_.size();
      }

      RCLCPP_INFO(
        this->get_logger(),
        "streaming: clients=%zu, frames_in=%lu, frames_out=%lu",
        client_count,
        static_cast<unsigned long>(frames_in_.load()),
        static_cast<unsigned long>(frames_out_.load()));
      last_stats_log_sec_ = now_sec;
    }
  }

  std::string pointcloud_topic_;
  std::string server_host_;
  int server_port_;

  int max_points_;
  int sample_step_;
  double min_range_m_;
  double max_range_m_;
  std::string stream_codec_name_;
  CompressionCodec stream_codec_{CompressionCodec::None};
  int compression_min_bytes_;
  double lazy_cooldown_sec_;

  net::io_context ioc_;
  std::unique_ptr<tcp::acceptor> acceptor_;
  std::thread accept_thread_;
  std::atomic<bool> running_;

  std::vector<std::shared_ptr<ClientSession>> sessions_;
  std::mutex sessions_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::TimerBase::SharedPtr lazy_timer_;

  rclcpp::Time no_client_since_;
  bool no_client_since_valid_{false};

  std::atomic<uint32_t> seq_;
  std::atomic<uint64_t> frames_in_{0};
  std::atomic<uint64_t> frames_out_{0};
  double last_stats_log_sec_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FVPointCloudWSServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
