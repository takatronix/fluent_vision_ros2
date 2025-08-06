/**
 * @file fv_object_mask_generator_node.cpp
 * @brief Fluent Vision オブジェクトマスク生成ノードのメイン実装ファイル
 * @details UNetを使用したセグメンテーションマスク生成と可視化
 * @author Takashi Otsuka
 * @date 2024
 * @version 1.0
 */

#include "fv_object_mask_generator/fv_object_mask_generator_node.hpp"
#include <chrono>

namespace fv_object_mask_generator
{

/**
 * @brief コンストラクタ
 * @param options ノードオプション
 * @details ノードの初期化、パラメータ読み込み、UNetモデル設定を行う
 * 
 * 初期化内容：
 * - ROS2パラメータの宣言と取得
 * - トピックの初期化
 * - UNetモデルの読み込みと初期化
 * - 処理タイマーの設定
 */
FVObjectMaskGeneratorNode::FVObjectMaskGeneratorNode(const rclcpp::NodeOptions & options)
: Node("fv_object_mask_generator", options),
  has_new_image_(false),
  model_initialized_(false)
{
  RCLCPP_INFO(this->get_logger(), "Initializing FV UNet Segmentation Node");
  
  // ===== 初期化処理 =====
  declareParameters();
  initializeTopics();
  initializeUNetModel();
  
  // ===== 処理タイマーの作成 =====
  auto timer_callback = [this]() { processImage(); };
  processing_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / processing_frequency_)),
    timer_callback);
    
  RCLCPP_INFO(this->get_logger(), "FV UNet Segmentation Node initialized successfully");
}

/**
 * @brief パラメータの宣言と取得
 * @details ROS2パラメータを宣言し、メンバー変数に取得
 * 
 * 設定内容：
 * - トピック名（入力画像、出力マスク）
 * - 処理パラメータ（頻度、可視化）
 * - UNetモデル設定（パス、デバイス、サイズ）
 * - クラス設定（名前、色）
 */
void FVObjectMaskGeneratorNode::declareParameters()
{
  // ===== トピックパラメータ =====
  this->declare_parameter("input_image_topic", "/camera/color/image_raw");                    // 入力画像トピック
  this->declare_parameter("output_segmentation_mask_topic", "/segmentation_mask/image");      // セグメンテーションマスク出力トピック
  this->declare_parameter("output_colored_mask_topic", "/segmentation_mask/colored");         // カラーマスク出力トピック
  
  // ===== 処理パラメータ =====
  this->declare_parameter("processing_frequency", 10.0);                                      // 処理頻度（Hz）
  this->declare_parameter("enable_visualization", true);                                      // 可視化有効化
  
  // ===== UNetモデルパラメータ =====
  this->declare_parameter("model.type", "unet");                                              // モデルタイプ
  this->declare_parameter("model.model_path", "/models/unet_asparagus_ch16_256_v1.0_ep20.xml"); // モデルファイルパス
  this->declare_parameter("model.device", "GPU");                                             // 推論デバイス
  this->declare_parameter("model.input_width", 256);                                          // 入力画像幅
  this->declare_parameter("model.input_height", 256);                                         // 入力画像高さ
  this->declare_parameter("model.confidence_threshold", 0.5);                                 // 信頼度閾値
  
  // ===== クラス設定 =====
  this->declare_parameter("class_names", std::vector<std::string>{"Background", "Asparagus"}); // クラス名リスト
  this->declare_parameter("class_colors", std::vector<int64_t>{0, 0, 0, 0, 255, 0});           // クラス色（BGR形式：黒、緑）
  
  // ===== パラメータ取得 =====
  input_image_topic_ = this->get_parameter("input_image_topic").as_string();
  output_segmentation_mask_topic_ = this->get_parameter("output_segmentation_mask_topic").as_string();
  output_colored_mask_topic_ = this->get_parameter("output_colored_mask_topic").as_string();
  
  processing_frequency_ = this->get_parameter("processing_frequency").as_double();
  enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
  
  model_type_ = this->get_parameter("model.type").as_string();
  model_path_ = this->get_parameter("model.model_path").as_string();
  device_ = this->get_parameter("model.device").as_string();
  input_width_ = this->get_parameter("model.input_width").as_int();
  input_height_ = this->get_parameter("model.input_height").as_int();
  confidence_threshold_ = static_cast<float>(this->get_parameter("model.confidence_threshold").as_double());
  
  class_names_ = this->get_parameter("class_names").as_string_array();
  auto color_vec = this->get_parameter("class_colors").as_integer_array();
  
  // ===== 色ベクトルをcv::Scalarに変換（BGR形式） =====
  for (size_t i = 0; i < color_vec.size(); i += 3) {
    if (i + 2 < color_vec.size()) {
      class_colors_.push_back(cv::Scalar(color_vec[i], color_vec[i+1], color_vec[i+2]));
    }
  }
  
  // ===== パラメータ読み込みログ出力 =====
  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  Input image topic: %s", input_image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output segmentation mask topic: %s", output_segmentation_mask_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output colored mask topic: %s", output_colored_mask_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Processing frequency: %.1f Hz", processing_frequency_);
  RCLCPP_INFO(this->get_logger(), "  Model: %s (%s)", model_type_.c_str(), model_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Device: %s", device_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Input size: %dx%d", input_width_, input_height_);
}

/**
 * @brief トピックの初期化
 * @details パブリッシャーとサブスクライバーを作成
 * 
 * 作成内容：
 * - セグメンテーションマスクパブリッシャー
 * - カラーマスクパブリッシャー
 * - 入力画像サブスクライバー
 */
void FVObjectMaskGeneratorNode::initializeTopics()
{
  // ===== パブリッシャー作成 =====
  segmentation_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_segmentation_mask_topic_, 10);  // セグメンテーションマスク出力
  colored_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_colored_mask_topic_, 10);       // カラーマスク出力
  
  // ===== サブスクライバー作成 =====
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_image_topic_, 10,  // 入力画像
    std::bind(&FVObjectMaskGeneratorNode::imageCallback, this, std::placeholders::_1));
    
  RCLCPP_INFO(this->get_logger(), "Topics initialized");
}

/**
 * @brief UNetモデルの初期化
 * @details OpenVINOを使用してUNetセグメンテーションモデルを読み込み、コンパイル
 * 
 * 初期化内容：
 * - OpenVINOモデルの読み込み
 * - 指定デバイスでのコンパイル
 * - 初期化状態の設定
 */
void FVObjectMaskGeneratorNode::initializeUNetModel()
{
  try {
    RCLCPP_INFO(this->get_logger(), "Initializing UNet model: %s", model_path_.c_str());
    
    // ===== OpenVINOモデルの読み込みとコンパイル =====
    ov::Core core;
    auto model = core.read_model(model_path_);
    compiled_model_ = core.compile_model(model, device_);
    
    model_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "UNet model initialized successfully");
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize UNet model: %s", e.what());
    model_initialized_ = false;
  }
}

/**
 * @brief 画像コールバック関数
 * @param msg 受信した画像メッセージ
 * @details 入力画像を受信し、最新画像として保存
 * 
 * 処理内容：
 * - 画像メッセージの受信
 * - BGR8形式への変換
 * - スレッドセーフな最新画像の更新
 * - 新画像フラグの設定
 */
void FVObjectMaskGeneratorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  try {
    // ===== 画像をBGR8形式に変換 =====
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    latest_image_ = cv_ptr->image.clone();
    has_new_image_ = true;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

/**
 * @brief 画像処理メイン関数
 * @details 定期的にUNetセグメンテーション処理を実行
 * 
 * 処理フロー：
 * - 新画像の可用性チェック
 * - UNet推論実行（時間計測付き）
 * - セグメンテーションマスクのパブリッシュ
 * - 可視化画像の生成とパブリッシュ
 * - 新画像フラグのリセット
 */
void FVObjectMaskGeneratorNode::processImage()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // ===== 新画像とモデルの可用性チェック =====
  if (!has_new_image_ || latest_image_.empty() || !model_initialized_) {
    return;
  }
  
  // ===== UNetセグメンテーション推論実行（時間計測付き） =====
  auto start_time = std::chrono::high_resolution_clock::now();
  cv::Mat segmentation_mask = inferUNet(latest_image_);
  auto end_time = std::chrono::high_resolution_clock::now();
  double inference_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  if (!segmentation_mask.empty()) {
    // ===== 生セグメンテーションマスクのパブリッシュ =====
    cv_bridge::CvImage mask_msg;
    mask_msg.header.stamp = this->now();
    mask_msg.header.frame_id = "camera_frame";
    mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
    mask_msg.image = segmentation_mask;
    segmentation_mask_pub_->publish(*mask_msg.toImageMsg());
    
    // ===== カラー可視化画像の生成とパブリッシュ =====
    if (enable_visualization_) {
      cv::Mat colored_mask = drawSegmentationResult(latest_image_, segmentation_mask, inference_time_ms);
      if (!colored_mask.empty()) {
        cv_bridge::CvImage colored_msg;
        colored_msg.header.stamp = this->now();
        colored_msg.header.frame_id = "camera_frame";
        colored_msg.encoding = sensor_msgs::image_encodings::BGR8;
        colored_msg.image = colored_mask;
        colored_mask_pub_->publish(*colored_msg.toImageMsg());
      }
    }
  }
  
  // ===== 新画像フラグのリセット =====
  has_new_image_ = false;
}

/**
 * @brief UNet推論実行
 * @param image 入力画像
 * @return cv::Mat セグメンテーションマスク
 * @details OpenVINOを使用してUNetセグメンテーション推論を実行
 * 
 * 処理フロー：
 * - モデル入力形状の取得
 * - 画像の前処理（CHW形式変換）
 * - OpenVINO推論実行
 * - 出力の後処理（sigmoid、閾値処理）
 * - 元画像サイズへのリサイズ
 */
cv::Mat FVObjectMaskGeneratorNode::inferUNet(const cv::Mat& image)
{
  if (!model_initialized_) {
    return cv::Mat();
  }
  
  try {
    // ===== モデル入力形状の取得 =====
    auto input_port = compiled_model_.input();
    auto input_shape = input_port.get_shape();
    int in_c = input_shape[1];                    // 入力チャンネル数
    int in_h = input_shape[input_shape.size() - 2]; // 入力高さ
    int in_w = input_shape[input_shape.size() - 1]; // 入力幅

    // ===== 前処理（CHW形式変換） =====
    std::vector<float> blob_data = preprocessToCHW(image, in_c, in_h, in_w);
    ov::Tensor tensor(input_port.get_element_type(), input_port.get_shape(), blob_data.data());

    // ===== OpenVINO推論実行 =====
    auto infer_request = compiled_model_.create_infer_request();
    infer_request.set_input_tensor(tensor);
    infer_request.infer();

    // ===== 出力の取得 =====
    ov::Tensor output_tensor = infer_request.get_output_tensor();
    auto out_shape = output_tensor.get_shape();
    int out_h = out_shape[out_shape.size() - 2];  // 出力高さ
    int out_w = out_shape[out_shape.size() - 1];  // 出力幅
    const float* output_data = output_tensor.data<const float>();

    // ===== セグメンテーションマスクの作成（sigmoid + 閾値処理） =====
    cv::Mat seg_map(out_h, out_w, CV_32F, (void*)output_data);
    cv::Mat sigmoid_map;
    cv::exp(-seg_map, sigmoid_map);
    sigmoid_map = 1.0 / (1.0 + sigmoid_map); // sigmoid関数適用
    
    cv::Mat mask;
    cv::threshold(sigmoid_map, mask, confidence_threshold_, 1.0, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8U, 255.0);

    // ===== 元画像サイズへのリサイズ =====
    cv::Mat mask_resized;
    if (!mask.empty() && !image.empty()) {
      cv::resize(mask, mask_resized, image.size(), 0, 0, cv::INTER_NEAREST);
      return mask_resized;
    }
    
    return cv::Mat();
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "UNet inference error: %s", e.what());
    return cv::Mat();
  }
}

/**
 * @brief セグメンテーション結果の可視化描画
 * @param image 元画像
 * @param seg_map セグメンテーションマスク
 * @param inference_time_ms 推論時間（ミリ秒）
 * @return cv::Mat 可視化画像
 * @details セグメンテーション結果を元画像にオーバーレイして可視化
 * 
 * 描画内容：
 * - アスパラガス領域の緑色オーバーレイ（アルファブレンド）
 * - モデル情報と推論時間の表示
 * - 背景付きテキスト表示
 */
cv::Mat FVObjectMaskGeneratorNode::drawSegmentationResult(const cv::Mat& image, const cv::Mat& seg_map, double inference_time_ms)
{
  if (image.empty() || seg_map.empty() || class_colors_.size() < 2) {
    return cv::Mat();
  }
  
  // ===== カラーマスクオーバーレイの作成 =====
  cv::Mat result = image.clone();
  cv::Scalar asparagus_color = class_colors_[1]; // アスパラガス用の緑色
  
  // ===== ピクセル単位でのアルファブレンド =====
  for (int y = 0; y < result.rows; ++y) {
    for (int x = 0; x < result.cols; ++x) {
      if (seg_map.at<uchar>(y, x) > 0) {
        // アルファブレンド：50%元画像 + 50%緑色オーバーレイ
        cv::Vec3b& pixel = result.at<cv::Vec3b>(y, x);
        pixel[0] = static_cast<uchar>(0.5 * pixel[0] + 0.5 * asparagus_color[0]); // B
        pixel[1] = static_cast<uchar>(0.5 * pixel[1] + 0.5 * asparagus_color[1]); // G
        pixel[2] = static_cast<uchar>(0.5 * pixel[2] + 0.5 * asparagus_color[2]); // R
      }
    }
  }
  
  // Add text information
  int font_face = cv::FONT_HERSHEY_SIMPLEX;
  double font_scale = 0.6;
  int thickness = 2;
  cv::Scalar text_color(255, 255, 255); // White text
  cv::Scalar bg_color(0, 0, 0); // Black background
  
  // Extract model filename from full path
  std::string model_filename = model_path_;
  size_t last_slash = model_filename.find_last_of("/");
  if (last_slash != std::string::npos) {
    model_filename = model_filename.substr(last_slash + 1);
  }
  
  // Prepare text lines
  std::vector<std::string> text_lines = {
    "Model: " + model_type_,
    "File: " + model_filename,
    "Device: " + device_,
    "Inference: " + std::to_string(static_cast<int>(inference_time_ms + 0.5)) + "ms"
  };
  
  // Calculate text position (top-left corner with margin)
  int margin = 10;
  int line_height = 25;
  
  for (size_t i = 0; i < text_lines.size(); ++i) {
    cv::Point text_pos(margin, margin + (i + 1) * line_height);
    
    // Get text size for background rectangle
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text_lines[i], font_face, font_scale, thickness, &baseline);
    
    // Draw background rectangle
    cv::rectangle(result, 
                  cv::Point(text_pos.x - 5, text_pos.y - text_size.height - 5),
                  cv::Point(text_pos.x + text_size.width + 5, text_pos.y + baseline + 5),
                  bg_color, cv::FILLED);
    
    // Draw text
    cv::putText(result, text_lines[i], text_pos, font_face, font_scale, text_color, thickness);
  }
  
  return result;
}

/**
 * @brief 画像の前処理（CHW形式変換）
 * @param src 入力画像
 * @param out_ch 出力チャンネル数
 * @param out_h 出力高さ
 * @param out_w 出力幅
 * @return std::vector<float> CHW形式の画像データ
 * @details 画像をUNetモデル入力用のCHW形式に変換
 * 
 * 処理内容：
 * - 画像のリサイズ
 * - 正規化（0-1範囲）
 * - BGR→RGB変換（3チャンネル時）
 * - CHW形式への変換
 * - グレースケール変換（1チャンネル時）
 */
std::vector<float> FVObjectMaskGeneratorNode::preprocessToCHW(const cv::Mat& src, int out_ch, int out_h, int out_w)
{
  // ===== 画像のリサイズ =====
  cv::Mat resized;
  cv::resize(src, resized, cv::Size(out_w, out_h));
  
  // ===== 正規化（0-1範囲） =====
  cv::Mat float_img;
  resized.convertTo(float_img, CV_32F, 1.0f / 255.0f);
  
  std::vector<float> blob_data(out_ch * out_h * out_w);
  
  if (out_ch == 3 && float_img.channels() == 3) {
    // ===== BGR→RGB変換とCHW形式変換 =====
    std::vector<cv::Mat> channels;
    cv::split(float_img, channels);
    
    // CHW順序でコピー：RGB
    std::memcpy(blob_data.data(), channels[2].data, out_h * out_w * sizeof(float)); // R
    std::memcpy(blob_data.data() + out_h * out_w, channels[1].data, out_h * out_w * sizeof(float)); // G
    std::memcpy(blob_data.data() + 2 * out_h * out_w, channels[0].data, out_h * out_w * sizeof(float)); // B
  } else if (out_ch == 1) {
    // ===== グレースケール変換 =====
    cv::Mat gray;
    if (float_img.channels() == 3) {
      cv::cvtColor(float_img, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = float_img;
    }
    std::memcpy(blob_data.data(), gray.data, out_h * out_w * sizeof(float));
  }
  
  return blob_data;
}

}  // namespace fv_object_mask_generator

/**
 * @brief メイン関数
 * @param argc コマンドライン引数の数
 * @param argv コマンドライン引数の配列
 * @return int 終了コード
 * @details ROS2ノードの初期化と実行
 * 
 * 実行内容：
 * - ROS2の初期化
 * - オブジェクトマスク生成ノードの作成
 * - ノードの実行（スピン）
 * - 適切な終了処理
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<fv_object_mask_generator::FVObjectMaskGeneratorNode>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}