// 最小・高可読版 fv_aspara_analyzer_node（v2新規）
// 目的: 画像パススルー + OverlayEngine（fluent_ui）による矩形/ラベル/収穫タイトル描画
// 仕様: 15fps固定（ui.fpsで変更可）、α補間、選択/非選択のスタイル差、待機画面の安定出力

#include "fv_aspara_analyzer/fv_aspara_analyzer_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include "fluent_lib/fluent.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <functional>
#include <regex>

namespace fv_aspara_analyzer {
// ========== ユーティリティ関数（小さく独立） ==========
std::vector<std::pair<int, cv::Rect>> FvAsparaAnalyzerNode::buildOverlayTargetsFromDetections(
    const fv_stem_detector::msg::StemDetectionArray::SharedPtr &msg) const
{
    std::vector<std::pair<int, cv::Rect>> targets;
    if (!msg) return targets;
    // StemDetectionArray: 各要素に region_id と cut-point(px) と bbox(w,h) が含まれる（拡張仕様）。
    for (const auto &d : msg->detections) {
        // 固定モード（SOURCE_FIXED）は detected=false でも常時表示対象とする
        if (!d.detected && d.source != fv_stem_detector::msg::StemDetection::SOURCE_FIXED) continue;
        // d.x,d.y は bbox左上。d.w,d.h はサイズ。
        int bx = std::max(0, d.x);
        int by = std::max(0, d.y);
        int bw = std::max(0, d.w);
        int bh = std::max(0, d.h);
        cv::Rect b(bx, by, bw, bh);
        int id = d.region_id;
        targets.emplace_back(id, b);
    }
    return targets;
}

// StemDetectionArray -> AsparaInfo（最小）
std::vector<AsparaInfo> FvAsparaAnalyzerNode::buildAsparaInfosFromStem(
    const fv_stem_detector::msg::StemDetectionArray::SharedPtr &msg) const
{
    std::vector<AsparaInfo> out;
    if (!msg) return out;
    // 既定フィルタ: detected=true, bbox(w,h)>0
    for (const auto &d : msg->detections) {
        // 固定モード（SOURCE_FIXED）は detected=false でも可視対象に含める
        if (!d.detected && d.source != fv_stem_detector::msg::StemDetection::SOURCE_FIXED) continue;
        if (d.w <= 0 || d.h <= 0) continue;
        AsparaInfo info;
        info.id = d.region_id;
        info.bbox = cv::Rect(std::max(0, d.x), std::max(0, d.y), std::max(0, d.w), std::max(0, d.h));
        // 距離（前方距離）を長さ近似に使わない。長さは別系だが、ここでは保持しない。
        // 必要なら camera_z を格納した別フィールドを追加可能。
        info.confidence_0_1 = std::numeric_limits<double>::quiet_NaN();
        info.length_m = std::numeric_limits<double>::quiet_NaN();
        out.push_back(info);
    }
    return out;
}

cv::Mat FvAsparaAnalyzerNode::drawWaitScreen(const cv::Size &size)
{
    cv::Size sz = size.width>0 && size.height>0 ? size : cv::Size(640, 480);
    cv::Mat m(sz, CV_8UC3, cv::Scalar(0,0,0));
    // 日本語フォントで描画（fluent_lib）
    fluent::text::drawShadow(m, std::string("待機中..."), {std::max(10, sz.width/2-80), std::max(24, sz.height/2)},
                             {255,255,255}, {0,0,0}, 0.9, 2, 0);
    return m;
}


FvAsparaAnalyzerNode::FvAsparaAnalyzerNode() : rclcpp::Node("fv_aspara_analyzer_node")
{
    // パラメータ宣言（最小）
    this->declare_parameter("ui.fps", 15.0);
    // デフォルトは高速追従寄り（15fps基準で1フレーム大きく追従）
    this->declare_parameter("overlay.smooth_gain", 8.0);
    this->declare_parameter("overlay.fade_in_gain", 10.0);
    this->declare_parameter("overlay.fade_out_gain", 8.0);
    this->declare_parameter("overlay.selected.alpha", 0.8);
    this->declare_parameter("overlay.unselected.alpha", 0.4);
    // 線幅（現行キー）
    this->declare_parameter("overlay.selected.thickness", 2);
    this->declare_parameter("overlay.unselected.thickness", 1);
    // 互換キー（旧）
    this->declare_parameter("overlay.selected.thickness_px", 2);
    this->declare_parameter("overlay.unselected.thickness_px", 1);
    // 色（BGR）
    this->declare_parameter("overlay.selected.color_bgr", std::vector<int64_t>{0, 255, 0});
    this->declare_parameter("overlay.unselected.color_bgr", std::vector<int64_t>{128, 128, 128});
    this->declare_parameter("overlay.unselected.draw_active_only", true);
    this->declare_parameter("label.bg_alpha", 0.4);
    // 固定ソース(固定枠)の青枠描画の有効/無効
    this->declare_parameter("overlay.fixed_source_blue_bbox", false);
    // 最小表示: YOLO矩形を半透明で塗るだけ（他は全て抑止）
    this->declare_parameter("overlay.only_yolo_transparent", false);
    this->declare_parameter("overlay.yolo_fill_alpha", 0.2);
    this->declare_parameter("overlay.yolo_fill_color_bgr", std::vector<int64_t>{0, 255, 0});
    // 情報ウィンドウと非選択ラインのα（YAMLで設定可能に）
    this->declare_parameter("info.window_alpha", 0.8);
    this->declare_parameter("overlay.nonselected_lines_alpha", 0.4);
    this->declare_parameter("harvest.enabled", true);
    this->declare_parameter("harvest.selected_only", true);
    this->declare_parameter("harvest.threshold_cm", 23.0);
    this->declare_parameter("harvest.title_bg_alpha", 0.8);
    // 表示用フィルタ（2D/3Dのconfidenceスレッショルド）
    this->declare_parameter("filter.yolo.min_confidence", 0.0);
    this->declare_parameter("filter.pca.min_confidence", 0.0);
    // 自動保存先と設定元ヒント
    this->declare_parameter("filters.save_yaml_path", std::string(""));
    this->declare_parameter("config.source_file", std::string(""));
    // 残像抑制・描画スキップのしきい値（パラメータ化）
    this->declare_parameter("overlay.erase.alpha_min", 0.02);   // αがこの値未満かつ一定時間経過で削除
    this->declare_parameter("overlay.erase.age_ms", 500.0);     // 削除までの経過時間[ms]
    this->declare_parameter("overlay.draw.skip_alpha", 0.05);    // αがこの値未満の要素は描画しない
    this->declare_parameter("overlay.draw.skip_alpha_if_inactive", 0.15); // 今フレーム未観測ならこの値未満で描画しない
    // 待機画面の既定サイズ（画像未着時の解像度）
    this->declare_parameter("ui.wait_width", 640);
    this->declare_parameter("ui.wait_height", 480);

    // ===== HUD/ウィンドウ/アプローチ関連パラメータ宣言（YAML対応） =====
    // 左/右ウィンドウの有効トグル
    // 左側の点群ウィンドウは廃止
    this->declare_parameter("left_panel_enabled", false);
    this->declare_parameter("info_window_enabled", true);
    // 情報ウィンドウ固定配置（任意）
    this->declare_parameter("info.fixed.enabled", true);
    this->declare_parameter("info.fixed.x", 0);
    this->declare_parameter("info.fixed.y", 0);
    this->declare_parameter("info.fixed.width", 0);
    this->declare_parameter("info.fixed.height", 0);
    // 既定ロック状態を適用
    try { right_info_window_.setLocked(this->get_parameter("info.fixed.enabled").as_bool()); } catch (...) {}
    // 選択状態の自動変更を無効化するフラグ（既定: OFF）
    this->declare_parameter("aspara_selection.auto_enabled", false);
    // アプローチポイント（2Dオーバーレイ）
    this->declare_parameter("approach.enable", true);
    this->declare_parameter("approach.y_mode", std::string("center")); // center | center_to_bottom_ratio | root_scanline
    this->declare_parameter("approach.y_ratio", 0.75);
    this->declare_parameter("approach.x_offset_px", 0);
    this->declare_parameter("approach.y_offset_px", 0);
    this->declare_parameter("approach.overlay.enable", true);
    this->declare_parameter("approach.overlay.color_bgr", std::vector<int64_t>{255, 0, 255});
    this->declare_parameter("approach.overlay.radius", 6);
    this->declare_parameter("approach.overlay.thickness", 2);
    this->declare_parameter("approach.depth.window_px", 5);
    this->declare_parameter("approach.depth.min_valid", 5);
    this->declare_parameter("approach.depth.max_range_m", 2.0);

    // アプローチ色の即時反映（描画で使用）
    try {
        auto carr = this->get_parameter("approach.overlay.color_bgr").as_integer_array();
        if (carr.size() == 3) {
            color_approach_ = cv::Scalar(static_cast<int>(carr[0]), static_cast<int>(carr[1]), static_cast<int>(carr[2]));
        }
    } catch (...) {}

    // 入出力トピック名は設定ファイル（YAML）から読み取り、複数キーに対応（旧設計互換）
    // カメラ識別子（サービス応答等でも返す）
    try { camera_id_ = this->declare_parameter<std::string>("camera_id", std::string("d405")); } catch (...) {}
    std::string image_topic = "image";
    try { image_topic = this->declare_parameter<std::string>("image_topic", image_topic); } catch(...) {}
    try { auto v = this->declare_parameter<std::string>("camera_topic", image_topic); if (v != image_topic) image_topic = v; } catch(...) {}
    try { auto v = this->declare_parameter<std::string>("topics.image", image_topic); if (v != image_topic) image_topic = v; } catch(...) {}
    try { auto v = this->declare_parameter<std::string>("input_color_image_topic", image_topic); if (v != image_topic) image_topic = v; } catch(...) {}

    // /fv/<camera> のプレフィックスを image_topic から自動抽出（YAML未指定時の混線防止）
    std::string ns_prefix = "/fv/" + camera_id_;
    if (!image_topic.empty() && image_topic.rfind("/fv/", 0) == 0) {
        size_t p = image_topic.find('/', 4);
        if (p != std::string::npos) ns_prefix = image_topic.substr(0, p);
    }

    // 茎検出ノード（fv_stem_detector）の統合出力にデフォルトで接続
    // 優先順位: detections_topic > topics.detections > input_detection_topic > detection_topic（後方互換）
    std::string detections_topic_default = ns_prefix + std::string("/stem_detector/detections");
    std::string detections_topic = detections_topic_default;
    bool det_set = false;
    try {
        auto v = this->declare_parameter<std::string>("detections_topic", detections_topic_default);
        detections_topic = v;
        det_set = (v != detections_topic_default);
    } catch(...) {}
    // 後続キーは、まだ値が設定されていない場合のみ採用
    try {
        auto v = this->declare_parameter<std::string>("topics.detections", detections_topic);
        if (!det_set && v != detections_topic) { detections_topic = v; det_set = true; }
    } catch(...) {}
    try {
        auto v = this->declare_parameter<std::string>("input_detection_topic", detections_topic);
        if (!det_set && v != detections_topic) { detections_topic = v; det_set = true; }
    } catch(...) {}
    try {
        auto v = this->declare_parameter<std::string>("detection_topic", detections_topic);
        if (!det_set && v != detections_topic) { detections_topic = v; det_set = true; }
    } catch(...) {}

    std::string annotated_topic = "annotated";
    try { annotated_topic = this->declare_parameter<std::string>("annotated_image_topic", annotated_topic); } catch(...) {}
    try { auto v = this->declare_parameter<std::string>("output_annotated_image_topic", annotated_topic); if (v != annotated_topic) annotated_topic = v; } catch(...) {}
    try { auto v = this->declare_parameter<std::string>("topics.annotated", annotated_topic); if (v != annotated_topic) annotated_topic = v; } catch(...) {}

    std::string annotated_compressed_topic = annotated_topic + "/compressed";
    try { annotated_compressed_topic = this->declare_parameter<std::string>("annotated_compressed_topic", annotated_compressed_topic); } catch(...) {}

    // 入出力
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(),
        std::bind(&FvAsparaAnalyzerNode::imageCallback, this, std::placeholders::_1));
    // 深度購読（存在すれば使用）
    std::string depth_topic = ns_prefix + std::string("/depth/image_rect_raw");
    try { depth_topic = this->declare_parameter<std::string>("depth_topic", depth_topic); } catch (...) {}
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg){
            rclcpp::Time cur(msg->header.stamp);
            if (!latest_depth_ || cur > last_depth_stamp_) { latest_depth_ = msg; last_depth_stamp_ = cur; }
        });

    // カメラ内参
    std::string camera_info_topic = ns_prefix + std::string("/color/camera_info");
    try { camera_info_topic = this->declare_parameter<std::string>("camera_info_topic", camera_info_topic); } catch (...) {}
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg){ latest_camera_info_ = msg; });
    detections_sub_ = this->create_subscription<fv_stem_detector::msg::StemDetectionArray>(
        detections_topic, rclcpp::SensorDataQoS(),
        std::bind(&FvAsparaAnalyzerNode::detectionCallback, this, std::placeholders::_1));
    // 画像出力（未接続でも配信を継続するQoSに設定）
    rclcpp::QoS img_qos = rclcpp::SensorDataQoS();
    annotated_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(annotated_topic, img_qos);
    annotated_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(annotated_compressed_topic, img_qos);
    // 選択アスパラ情報のトピック配信を廃止し、サービスで提供
    try {
        // サービス名はパラメータで上書き可能（デフォルト: <service.ns>/get_selected_asparagus）
        std::string srv_tail = "get_selected_asparagus";
        try { srv_tail = this->declare_parameter<std::string>("services.get_selected", srv_tail); } catch (...) {}
        // サービス基底NS（絶対名）。既定は /fv/<camera_id>
        std::string service_ns = ns_prefix; // e.g., /fv/d405
        try { service_ns = this->declare_parameter<std::string>("service.ns", service_ns); } catch (...) {}
        if (service_ns.empty() || service_ns[0] != '/') service_ns = std::string("/") + service_ns;
        if (service_ns.back() != '/') service_ns.push_back('/');
        std::string srv_name = service_ns + srv_tail;
        get_selected_srv_ = this->create_service<fv_aspara_analyzer::srv::GetSelectedAsparagus>(
            srv_name,
            std::bind(&FvAsparaAnalyzerNode::onGetSelected, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "service (get_selected): %s", srv_name.c_str());
    } catch (...) {}

    RCLCPP_INFO(this->get_logger(), "subscribe camera: %s", image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "subscribe stem detections: %s", detections_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "publish annotated: %s", annotated_topic.c_str());

    // ROI点群出力は廃止

    // 選択操作サービス（ノードNSに依存しない絶対名: service.ns + tail）
    auto make_abs_service_name = [this, &ns_prefix](const char* tail){
        std::string base = ns_prefix; // default /fv/<camera>
        try { base = this->get_parameter("service.ns").as_string(); } catch (...) {}
        if (base.empty() || base[0] != '/') base = std::string("/") + base;
        if (base.back() != '/') base.push_back('/');
        return base + tail;
    };
    std::string next_tail = "next_asparagus";
    std::string prev_tail = "prev_asparagus";
    try { next_tail = this->declare_parameter<std::string>("services.next", next_tail); } catch (...) {}
    try { prev_tail = this->declare_parameter<std::string>("services.prev", prev_tail); } catch (...) {}
    const std::string next_service_name = make_abs_service_name(next_tail.c_str());
    const std::string prev_service_name = make_abs_service_name(prev_tail.c_str());

    next_srv_ = this->create_service<std_srvs::srv::Trigger>(
        next_service_name,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            try {
                if (last_id_to_rect_.empty()) { res->success = false; return; }
                int next_id = aspara_list_.selectNext(selected_id_);
                if (next_id < 0) { res->success = false; return; }
                selected_id_ = next_id;
                auto itb = last_id_to_rect_.find(selected_id_);
                if (itb != last_id_to_rect_.end()) {
                    const auto &r = itb->second; cursor_.SetPosition(r.x + r.width/2, r.y + r.height/2); cursor_.ShowCursor();
                }
                res->success = true;
            } catch (...) {
                res->success = false;
            }
        });
    prev_srv_ = this->create_service<std_srvs::srv::Trigger>(
        prev_service_name,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            try {
                if (last_id_to_rect_.empty()) { res->success = false; return; }
                int prev_id = aspara_list_.selectPrev(selected_id_);
                if (prev_id < 0) { res->success = false; return; }
                selected_id_ = prev_id;
                auto itb = last_id_to_rect_.find(selected_id_);
                if (itb != last_id_to_rect_.end()) {
                    const auto &r = itb->second; cursor_.SetPosition(r.x + r.width/2, r.y + r.height/2); cursor_.ShowCursor();
                }
                res->success = true;
            } catch (...) {
                res->success = false;
            }
        });

    // サービスの実フルパスをログに出す
    RCLCPP_INFO(this->get_logger(), "service (next): %s", next_service_name.c_str());
    RCLCPP_INFO(this->get_logger(), "service (prev): %s", prev_service_name.c_str());

    // Overlayゲイン適用
    overlay_.setGains(
        this->get_parameter("overlay.smooth_gain").as_double(),
        this->get_parameter("overlay.fade_in_gain").as_double(),
        this->get_parameter("overlay.fade_out_gain").as_double());
    overlay_.setRemovalPolicy(
        static_cast<float>(this->get_parameter("overlay.erase.alpha_min").as_double()),
        this->get_parameter("overlay.erase.age_ms").as_double());

    // カーソル設定（最小）
    this->declare_parameter("cursor.enabled", true);
    this->declare_parameter("cursor.keep_ms", 5000);
    this->declare_parameter("cursor.smooth_time_ms", 150);
    this->declare_parameter("cursor.color_bgr", std::vector<int64_t>{0,255,0});
    this->declare_parameter("cursor.size_px", 20);
    this->declare_parameter("cursor.thickness_px", 2);
    this->declare_parameter("cursor.auto_on", true);
    this->declare_parameter("cursor.auto_off_if_miss", true);
    this->declare_parameter("cursor.input_topic", std::string("/fv/cursor/click"));
    {
        bool cen = true; try { cen = this->get_parameter("cursor.enabled").as_bool(); } catch (...) {}
        int keep = 5000; try { keep = this->get_parameter("cursor.keep_ms").as_int(); } catch (...) {}
        int sm = 150; try { sm = this->get_parameter("cursor.smooth_time_ms").as_int(); } catch (...) {}
        std::vector<int64_t> carr; try { carr = this->get_parameter("cursor.color_bgr").as_integer_array(); } catch (...) {}
        cv::Scalar col(0,255,0); if (carr.size()==3) col = cv::Scalar(static_cast<int>(carr[0]), static_cast<int>(carr[1]), static_cast<int>(carr[2]));
        int size_px = 20; try { size_px = this->get_parameter("cursor.size_px").as_int(); } catch (...) {}
        int thick_px = 2; try { thick_px = this->get_parameter("cursor.thickness_px").as_int(); } catch (...) {}
        bool a_on = true; try { a_on = this->get_parameter("cursor.auto_on").as_bool(); } catch (...) {}
        bool a_off = true; try { a_off = this->get_parameter("cursor.auto_off_if_miss").as_bool(); } catch (...) {}
        cursor_.Configure(keep, sm, col, size_px, thick_px, a_on, a_off);
        if (!cen) cursor_.HideCursor();
    }

    // RQTなどからのクリック入力を購読（geometry_msgs/Point）
    try {
        std::string click_topic = this->get_parameter("cursor.input_topic").as_string();
        cursor_click_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            click_topic, 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg){
                int x = static_cast<int>(std::round(msg->x));
                int y = static_cast<int>(std::round(msg->y));
                if (!latest_color_) return; // 画像未着時は無視
                x = std::clamp(x, 0, static_cast<int>(latest_color_->width) - 1);
                y = std::clamp(y, 0, static_cast<int>(latest_color_->height) - 1);
                // カーソル移動
                cursor_.SetPosition(x, y);
                cursor_.ShowCursor();
                // 優先順位順にヒットチェック（AsparaList 使用）
                int hit = aspara_list_.selectByPosition(x, y, last_id_to_rect_);
                if (hit >= 0) { selected_id_ = hit; }
            });
        RCLCPP_INFO(this->get_logger(), "subscribe cursor click: %s", click_topic.c_str());
    } catch (...) {}

    // 画面更新タイマー
    double fps = this->get_parameter("ui.fps").as_double();
    int interval_ms = static_cast<int>(std::max(1.0, std::round(1000.0 / std::max(1.0, fps))));
    animation_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this, fps]() {
            // 補間tick
            double dt = 1.0 / std::max(1.0, fps);
            // アニメーション制御（固定時はtickを抑止）
            bool info_fixed=false; try { info_fixed = this->get_parameter("info.fixed.enabled").as_bool(); } catch (...) {}
            overlay_.tick(dt);
            cursor_.Tick(dt);
            approach_window_.tick(dt);
            if (!info_fixed) right_info_window_.tick(dt);
            // 画像未着時も待機画面を出力
            publishCurrentImage();
        });

    // 深度単位・座標参照・深度バンド
    try { depth_unit_m_16u_ = this->declare_parameter<double>("depth_unit_m_16u", depth_unit_m_16u_); } catch (...) {}
    try { use_color_camera_info_flag_ = this->declare_parameter<bool>("use_color_camera_info", true); } catch (...) {}
    try { depth_band_enable_ = this->declare_parameter<bool>("depth_band.enable", true); } catch (...) {}
    try { depth_band_minus_m_ = this->declare_parameter<double>("depth_band.minus_m", 0.05); } catch (...) {}
    try { depth_band_plus_m_ = this->declare_parameter<double>("depth_band.plus_m", 0.05); } catch (...) {}

    // 動的パラメータ反映（フィルタ値の自動保存）
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&FvAsparaAnalyzerNode::onParamSet, this, std::placeholders::_1));

    // 収穫可否/長さ制約（パラメータ）
    try { asparagus_length_impossible_min_m_ = this->declare_parameter<double>("asparagus_length.min_impossible_m", asparagus_length_impossible_min_m_); } catch (...) {}
    try { asparagus_length_impossible_max_m_ = this->declare_parameter<double>("asparagus_length.max_impossible_m", asparagus_length_impossible_max_m_); } catch (...) {}
    try { harvest_min_length_m_ = this->declare_parameter<double>("harvest.min_length_m", harvest_min_length_m_); } catch (...) {}
    try { harvest_max_length_m_ = this->declare_parameter<double>("harvest.max_length_m", harvest_max_length_m_); } catch (...) {}
}

FvAsparaAnalyzerNode::~FvAsparaAnalyzerNode() {}

rcl_interfaces::msg::SetParametersResult FvAsparaAnalyzerNode::onParamSet(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult res; res.successful = true; res.reason = "ok";
    bool changed = false; double yolo_min=0.0, pca_min=0.0; bool yolo_set=false, pca_set=false;
    for (const auto &p : params) {
        if (p.get_name() == "filter.yolo.min_confidence" && p.get_type()==rclcpp::ParameterType::PARAMETER_DOUBLE) { yolo_min = p.as_double(); yolo_set=true; changed = true; }
        else if (p.get_name() == "filter.pca.min_confidence" && p.get_type()==rclcpp::ParameterType::PARAMETER_DOUBLE) { pca_min = p.as_double(); pca_set=true; changed = true; }
    }
    if (changed) {
        std::string save_path;
        try { save_path = this->get_parameter("filters.save_yaml_path").as_string(); } catch (...) { save_path.clear(); }
        if (save_path.empty()) {
            try { save_path = this->get_parameter("config.source_file").as_string(); } catch (...) { save_path.clear(); }
        }
        if (!save_path.empty()) {
            std::ifstream ifs(save_path);
            if (!ifs) {
                RCLCPP_WARN(this->get_logger(), "filters auto-save failed: cannot open %s", save_path.c_str());
            } else {
                std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
                ifs.close();
                // 1) 既存の filter: ブロックを top-level で置換（インデント不問）
                std::regex re_filter_block("(^\\s*filter:\\s*\\n[\\s\\S]*?)(?=^\\S|\\z)", std::regex_constants::multiline);
                std::ostringstream new_block;
                if (!yolo_set) { try { yolo_min = this->get_parameter("filter.yolo.min_confidence").as_double(); } catch(...) { yolo_min = 0.0; } }
                if (!pca_set)  { try { pca_min  = this->get_parameter("filter.pca.min_confidence").as_double(); } catch(...) { pca_min = 0.0; } }
                new_block << "filter:\n";
                new_block << "  yolo:\n";
                new_block << "    min_confidence: " << yolo_min << "\n";
                new_block << "  pca:\n";
                new_block << "    min_confidence: " << pca_min << "\n";
                std::string replaced = std::regex_replace(content, re_filter_block, new_block.str());
                if (replaced == content) {
                    // 2) filter:が無ければ ros__parameters: の直後に挿入（無ければ末尾）
                    const std::string key = "ros__parameters:";
                    std::size_t p = content.find(key);
                    std::string ins;
                    // 標準インデント（4スペース）で追加
                    ins = std::string("    ") + new_block.str();
                    if (p != std::string::npos) {
                        std::size_t line_end = content.find('\n', p);
                        if (line_end == std::string::npos) line_end = content.size(); else line_end += 1;
                        replaced = content.substr(0, line_end) + ins + content.substr(line_end);
                    } else {
                        if (!content.empty() && content.back() != '\n') content.push_back('\n');
                        replaced = content + ins;
                    }
                }
                std::ofstream ofs(save_path, std::ios::out | std::ios::trunc);
                if (!ofs) {
                    RCLCPP_WARN(this->get_logger(), "filters auto-save failed: cannot write %s", save_path.c_str());
                } else {
                    ofs << replaced;
                    ofs.close();
                    RCLCPP_INFO(this->get_logger(), "filters auto-saved to %s (filter block updated in-place)", save_path.c_str());
                }
            }
        }
    }
    return res;
}

double FvAsparaAnalyzerNode::computeIoU(const cv::Rect &a, const cv::Rect &b)
{
    int inter = (a & b).area(); int uni = a.area() + b.area() - inter; if (uni <= 0) return 0.0; return static_cast<double>(inter) / static_cast<double>(std::max(1, uni));
}

void FvAsparaAnalyzerNode::onGetSelected(
    const std::shared_ptr<fv_aspara_analyzer::srv::GetSelectedAsparagus::Request>,
    std::shared_ptr<fv_aspara_analyzer::srv::GetSelectedAsparagus::Response> res)
{
    res->success = false;
    res->message = "no selection";
    res->id = -1;
    res->camera_id = camera_id_;
    if (selected_id_ < 0) return;
    auto itd = last_det_by_id_.find(selected_id_);
    if (itd == last_det_by_id_.end()) return;
    const auto &d = itd->second;
    res->success = true;
    res->message = "ok";
    res->id = selected_id_;
    // 概要メトリクス
    res->distance_m = std::isfinite(d.approach_distance_m) ? d.approach_distance_m : std::numeric_limits<float>::quiet_NaN();
    res->length_m = std::isfinite(d.length_m) ? d.length_m : std::numeric_limits<float>::quiet_NaN();
    res->confidence_2d = std::isfinite(d.yolo_score) ? d.yolo_score : std::numeric_limits<float>::quiet_NaN();
    res->confidence_3d = std::isfinite(d.pca_score) ? d.pca_score : std::numeric_limits<float>::quiet_NaN();
    // アプローチ
    res->approach_px_x = d.approach_px_x;
    res->approach_px_y = d.approach_px_y;
    res->approach_camera = d.approach_camera;
    // 根本
    res->root_px_x = d.root_px_x;
    res->root_px_y = d.root_px_y;
    res->root_camera = d.root_camera;
    // 先端
    res->tip_px_x = d.tip_px_x;
    res->tip_px_y = d.tip_px_y;
    res->tip_camera = d.tip_camera;
}

std::vector<std::pair<int, cv::Rect>> FvAsparaAnalyzerNode::assignPersistentIds(const std::vector<cv::Rect> &rects)
{
    std::vector<std::pair<int, cv::Rect>> out; out.reserve(rects.size()); std::map<int,bool> used;
    for (const auto &r : rects) {
        double best = 0.0; int best_id = -1;
        for (const auto &kv : last_id_to_rect_) {
            if (used[kv.first]) {
                continue;
            }
            double iou = computeIoU(r, kv.second);
            if (iou > best) {
                best = iou;
                best_id = kv.first;
            }
        }
        // IoUの閾値を下げて永続IDを維持しやすく（アニメーション継続のため）
        if (best_id >= 0 && best >= 0.1) { used[best_id] = true; out.emplace_back(best_id, r); }
        else { int nid = next_id_++; out.emplace_back(nid, r); }
    }
    last_id_to_rect_.clear(); for (const auto &p : out) last_id_to_rect_[p.first] = p.second; return out;
}

void FvAsparaAnalyzerNode::updateSelection(const std::vector<std::pair<int, cv::Rect>> &targets)
{
    if (targets.empty()) { selected_id_ = -1; return; }
    auto it = std::find_if(targets.begin(), targets.end(), [this](const auto &p){ return p.first == selected_id_; }); if (it != targets.end()) return;
    selected_id_ = std::max_element(targets.begin(), targets.end(), [](const auto &a, const auto &b){ return a.second.area() < b.second.area(); })->first;
}

void FvAsparaAnalyzerNode::detectionCallback(const fv_stem_detector::msg::StemDetectionArray::SharedPtr msg)
{
    // 受信確認用ログ（1秒間隔）
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "[aspara_analyzer] stem detections received: %zu", msg ? msg->detections.size() : 0);
    // 1) stem_detector から最小 AsparaInfo リストを構築（基本フィルタ込み）
    aspara_infos_ = buildAsparaInfosFromStem(msg);
    // 1.2) 2D/3D信頼度スレッショルドで追加フィルタ（表示専用）
    try {
        double yolo_min = this->get_parameter("filter.yolo.min_confidence").as_double();
        double pca_min  = this->get_parameter("filter.pca.min_confidence").as_double();
        if ((yolo_min > 0.0) || (pca_min > 0.0)) {
            std::vector<AsparaInfo> filtered; filtered.reserve(aspara_infos_.size());
            for (const auto &a : aspara_infos_) {
                // msg内のStemDetectionとIoU最大でマッチし、スコアを参照
                double best_iou = 0.0; const fv_stem_detector::msg::StemDetection* best = nullptr;
                for (const auto &d : msg->detections) {
                    if (!d.detected || d.w <= 0 || d.h <= 0) continue;
                    cv::Rect rb(std::max(0, d.x), std::max(0, d.y), std::max(0, d.w), std::max(0, d.h));
                    double iou = computeIoU(a.bbox, rb);
                    if (iou > best_iou) { best_iou = iou; best = &d; }
                }
                bool pass = true;
                if (best) {
                    // 固定モードの要素には表示フィルタを適用しない
                    if (best->source != fv_stem_detector::msg::StemDetection::SOURCE_FIXED) {
                        if (std::isfinite(best->yolo_score) && best->yolo_score < static_cast<float>(yolo_min)) pass = false;
                        if (std::isfinite(best->pca_score)  && best->pca_score  < static_cast<float>(pca_min))  pass = false;
                    }
                }
                if (pass) filtered.push_back(a);
            }
            aspara_infos_.swap(filtered);
        }
    } catch (...) {}
    // 1.5) さらにアスパラとして不適合長さを除去（length未設定は通過）
    {
        std::vector<AsparaInfo> filtered; filtered.reserve(aspara_infos_.size());
        for (const auto &a : aspara_infos_) {
            if (!a.hasLength()) { filtered.push_back(a); continue; }
            if (a.IsValidAsAsparagus(asparagus_length_impossible_min_m_, asparagus_length_impossible_max_m_)) {
                filtered.push_back(a);
            }
        }
        aspara_infos_.swap(filtered);
    }
    // 直前の矩形マップを退避（選択IDが消えたフレームでの固定表示に使用）
    auto prev_id_to_rect = last_id_to_rect_;
    // 2) 永続IDの割当（IoUで前フレームとマッチ → 既存ID継続／新規採番）
    std::vector<std::pair<int, cv::Rect>> basic; basic.reserve(aspara_infos_.size());
    for (const auto &a : aspara_infos_) basic.emplace_back(a.id, a.bbox);

    // 2.1) 永続IDを割り当て（矩形のIoUベース）
    std::vector<cv::Rect> rects; rects.reserve(basic.size());
    for (const auto &p : basic) rects.push_back(p.second);
    auto with_ids = assignPersistentIds(rects);

    // 2.2) aspara_infos_ の id を永続IDに置換（IoU最大のものを採用）
    for (auto &a : aspara_infos_) {
        double best_iou = 0.0; int best_id = -1;
        for (const auto &pid : with_ids) {
            double iou = computeIoU(a.bbox, pid.second);
            if (iou > best_iou) { best_iou = iou; best_id = pid.first; }
        }
        if (best_id >= 0) a.id = best_id;
    }

    // 3) 選択更新（自動切替が有効な場合のみ）
    bool auto_sel = false; try { auto_sel = this->get_parameter("aspara_selection.auto_enabled").as_bool(); } catch (...) {}
    if (auto_sel) {
        updateSelection(with_ids);
    }

    // 3.5) 自動選択OFF時も「最後に選択した枠は常に表示」
    //       今フレームに存在しない場合は前回矩形を追加
    if (!auto_sel && selected_id_ >= 0) {
        bool selected_present = false;
        for (const auto &p : with_ids) { if (p.first == selected_id_) { selected_present = true; break; } }
        if (!selected_present) {
            auto it_prev = prev_id_to_rect.find(selected_id_);
            if (it_prev != prev_id_to_rect.end() && it_prev->second.area() > 0) {
                with_ids.emplace_back(selected_id_, it_prev->second);
            }
        }
    }
    // 既存ID矩形更新（アニメーションの連続性: 前回マップを更新して維持）
    last_id_to_rect_.clear(); for (const auto &p : with_ids) last_id_to_rect_[p.first] = p.second;

    // 4) オーバーレイへ目標を設定
    overlay_.setTargets(with_ids);

    // 4.5) 今フレームで観測されたID集合を保存（描画側でのゴミ抑制に使用）
    active_ids_.clear();
    for (const auto &p : with_ids) active_ids_.insert(p.first);
    
    // 4.6) カーソル自動ロジック（自動選択ON時のみ）
    if (auto_sel) {
        std::vector<std::pair<int, cv::Rect>> sorted = with_ids;
        std::sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b){ return a.second.area() > b.second.area(); });
        // 優先順位の更新
        aspara_list_.updatePriority(last_id_to_rect_);
        int hit_id = cursor_.UpdateAutoLogic(sorted);
        if (hit_id >= 0) { selected_id_ = hit_id; }
    } else {
        // 自動OFF時は優先順位のみ更新（選択は保持）
        aspara_list_.updatePriority(last_id_to_rect_);
        // 非観測でも選択IDがスキップされないよう active に加える
        if (selected_id_ >= 0) { active_ids_.insert(selected_id_); }
    }

    // 5) 直近検出詳細を保存（右ウィンドウ/オーバーレイ用）
    last_det_by_id_.clear();
    for (const auto &d : msg->detections) {
        // 永続IDへマップ（region_idは一時IDの場合があるため、with_idsの矩形一致で探す）
        // 矩形がない要素はスキップ
        // 固定モード（SOURCE_FIXED）は detected=false でも保存対象とする
        if ((!d.detected && d.source != fv_stem_detector::msg::StemDetection::SOURCE_FIXED) || d.w <= 0 || d.h <= 0) continue;
        cv::Rect rb(std::max(0, d.x), std::max(0, d.y), std::max(0, d.w), std::max(0, d.h));
        int pid = -1; double best = 0.0;
        for (const auto &p : last_id_to_rect_) {
            double iou = computeIoU(rb, p.second);
            if (iou > best) { best = iou; pid = p.first; }
        }
        if (pid >= 0) {
            // 2DのY座標で上下関係を判断し、tip(先端)がroot(根本)より下側にある場合は反転して整合性を保つ
            auto fixed = d; // コピーして加工
            bool root_px_valid = (fixed.root_px_x >= 0 && fixed.root_px_y >= 0);
            bool tip_px_valid  = (fixed.tip_px_x  >= 0 && fixed.tip_px_y  >= 0);
            // (0,0)の未算出を誤判定しない
            bool tip_is_nonzero = (fixed.tip_px_x != 0 || fixed.tip_px_y != 0);
            if (root_px_valid && tip_px_valid && tip_is_nonzero) {
                // 画像座標は下向きが+Y。tipがrootより大きいY(=下側)なら入れ替え
                if (fixed.tip_px_y > fixed.root_px_y) {
                    // swap 2D px
                    std::swap(fixed.root_px_x, fixed.tip_px_x);
                    std::swap(fixed.root_px_y, fixed.tip_px_y);
                    // swap 3D camera座標
                    std::swap(fixed.root_camera, fixed.tip_camera);
                    // swap 距離[m]
                    std::swap(fixed.root_distance_m, fixed.tip_distance_m);
                }
            }
            last_det_by_id_[pid] = fixed;
        }
    }
}

void FvAsparaAnalyzerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 前回より新しいものだけを受理（古いstampでの巻き戻りを防止）
    rclcpp::Time cur_stamp(msg->header.stamp);
    if (!latest_color_ || cur_stamp > last_color_stamp_) {
        latest_color_ = msg; last_color_stamp_ = cur_stamp;
    }
}

bool FvAsparaAnalyzerNode::computeApproachDistanceM(const cv::Point &px, double &out_m) const
{
    if (!latest_depth_) return false;
    try {
        cv::Mat depth;
        if (latest_depth_->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            depth = cv_bridge::toCvShare(latest_depth_, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        } else if (latest_depth_->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            depth = cv_bridge::toCvShare(latest_depth_, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        } else {
            return false;
        }
        if (depth.empty()) return false;
        int win = 5; try { win = this->get_parameter("approach.depth.window_px").as_int(); } catch (...) {}
        int min_valid = 5; try { min_valid = this->get_parameter("approach.depth.min_valid").as_int(); } catch (...) {}
        double max_range = 2.0; try { max_range = this->get_parameter("approach.depth.max_range_m").as_double(); } catch (...) {}
        int x0 = std::clamp(px.x - win, 0, depth.cols-1);
        int x1 = std::clamp(px.x + win, 0, depth.cols-1);
        int y0 = std::clamp(px.y - win, 0, depth.rows-1);
        int y1 = std::clamp(px.y + win, 0, depth.rows-1);
        std::vector<double> zs; zs.reserve((x1-x0+1)*(y1-y0+1));
        for (int y=y0; y<=y1; ++y) for (int x=x0; x<=x1; ++x) {
            if (depth.type()==CV_16UC1) {
                uint16_t v = depth.at<uint16_t>(y,x); if (v==0) continue; double m = v * depth_unit_m_16u_; if (m>0.0 && m<=max_range) zs.push_back(m);
            } else {
                float v = depth.at<float>(y,x); if (!std::isfinite(v) || v<=0.0f || v>max_range) continue; zs.push_back(static_cast<double>(v));
            }
        }
        if (static_cast<int>(zs.size()) < min_valid) return false;
        std::nth_element(zs.begin(), zs.begin()+zs.size()/2, zs.end());
        out_m = zs[zs.size()/2];
        return std::isfinite(out_m) && out_m > 0.0;
    } catch (...) { return false; }
}

void FvAsparaAnalyzerNode::publishCurrentImage()
{
    cv::Mat out;
    if (latest_color_) {
        // 最新フレームのBGRを生成
        out = cv_bridge::toCvShare(latest_color_, sensor_msgs::image_encodings::BGR8)->image.clone();
    } else {
        int dw = 640, dh = 480;
        try { dw = this->get_parameter("ui.wait_width").as_int(); } catch (...) {}
        try { dh = this->get_parameter("ui.wait_height").as_int(); } catch (...) {}
        out = drawWaitScreen({dw, dh});
    }
    // 描画パラメータの取得（YAMLに揃える・旧キー互換）
    auto get_color = [this](const std::string &key) -> cv::Scalar {
        try {
            auto arr = this->get_parameter(key).as_integer_array();
            if (arr.size() == 3) {
                return cv::Scalar(static_cast<int>(arr[0]), static_cast<int>(arr[1]), static_cast<int>(arr[2]));
            }
        } catch (...) {}
        return cv::Scalar(255,255,255);
    };
    auto get_thickness = [this](const std::string &key_new, const std::string &key_old, int defv) -> int {
        try { return this->get_parameter(key_new).as_int(); } catch (...) {}
        try { return this->get_parameter(key_old).as_int(); } catch (...) {}
        return defv;
    };
    cv::Scalar color_sel = get_color("overlay.selected.color_bgr");
    cv::Scalar color_uns = get_color("overlay.unselected.color_bgr");
    int thick_sel = get_thickness("overlay.selected.thickness", "overlay.selected.thickness_px", 2);
    int thick_uns = get_thickness("overlay.unselected.thickness", "overlay.unselected.thickness_px", 1);

    // 非選択用の半透明ラインをまとめて合成するレイヤ
    cv::Mat overlay_lines = cv::Mat::zeros(out.size(), out.type());
    bool overlay_lines_drawn = false;

    // 最小表示モード（YOLO半透明）
    bool yolo_fill_only = false; try { yolo_fill_only = this->get_parameter("overlay.only_yolo_transparent").as_bool(); } catch (...) {}
    if (yolo_fill_only) {
        double fill_alpha = 0.2; try { fill_alpha = this->get_parameter("overlay.yolo_fill_alpha").as_double(); } catch (...) {}
        cv::Scalar fill_color(0,255,0);
        try {
            auto ca = this->get_parameter("overlay.yolo_fill_color_bgr").as_integer_array();
            if (ca.size()==3) fill_color = cv::Scalar((int)ca[0],(int)ca[1],(int)ca[2]);
        } catch (...) {}
        cv::Mat yolo_layer = cv::Mat::zeros(out.size(), out.type());
        for (const auto &item : overlay_.snapshot()) {
            cv::rectangle(yolo_layer, item.bbox, fill_color, cv::FILLED);
        }
        // 合成
        cv::addWeighted(out, 1.0, yolo_layer, std::clamp(fill_alpha, 0.0, 1.0), 0.0, out);
        // 情報ウィンドウや追加オーバーレイは抑止
        right_info_window_.hide();
        approach_window_.hide();
        // カーソルも抑止
        // HUD合成して終了（フラグOFFなら描画しない）
        bool info_enabled_min = true; try { info_enabled_min = this->get_parameter("info_window_enabled").as_bool(); } catch (...) {}
        if (info_enabled_min) { right_info_window_.render(out); }
        auto hdr = latest_color_ ? latest_color_->header : std_msgs::msg::Header();
        if (annotated_image_pub_ && annotated_image_pub_->get_subscription_count() > 0) {
            auto msg = cv_bridge::CvImage(hdr, sensor_msgs::image_encodings::BGR8, out).toImageMsg();
            annotated_image_pub_->publish(*msg);
        }
        if (annotated_compressed_pub_ && annotated_compressed_pub_->get_subscription_count() > 0) {
            try {
                auto cimg = cv_bridge::CvImage(hdr, sensor_msgs::image_encodings::BGR8, out);
                sensor_msgs::msg::CompressedImage cmsg; cmsg.header = hdr; cmsg.format = "jpeg"; cimg.toCompressedImageMsg(cmsg);
                annotated_compressed_pub_->publish(cmsg);
            } catch (...) {}
        }
        return;
    }

    // snapshotして単純枠描画（短いコードに徹する）
    double skip_alpha = 0.05; try { skip_alpha = this->get_parameter("overlay.draw.skip_alpha").as_double(); } catch (...) {}
    double skip_alpha_inactive = 0.15; try { skip_alpha_inactive = this->get_parameter("overlay.draw.skip_alpha_if_inactive").as_double(); } catch (...) {}
    bool draw_unselected_active_only = true; try { draw_unselected_active_only = this->get_parameter("overlay.unselected.draw_active_only").as_bool(); } catch (...) {}
    bool any_selected_drawn = false;
    // パラメータはループ外で取得（毎フレーム取得しない）
    bool info_enabled = true; try { info_enabled = this->get_parameter("info_window_enabled").as_bool(); } catch (...) {}
    // 固定枠の青色表示の可否（既定: 無効）
    bool fixed_blue_enabled = false; try { fixed_blue_enabled = this->get_parameter("overlay.fixed_source_blue_bbox").as_bool(); } catch (...) {}
    for (const auto &item : overlay_.snapshot()) {
        // 今フレームで観測されていないIDで、かつαが十分小さい場合は完全スキップ
        if (active_ids_.find(item.id) == active_ids_.end() && item.alpha < static_cast<float>(skip_alpha_inactive)) { continue; }
        // αが十分に小さい要素は描画スキップ（フェードアウト中の残像対策）
        if (item.alpha < static_cast<float>(skip_alpha)) { continue; }
        bool is_sel = (item.id == selected_id_);
        if (!is_sel && draw_unselected_active_only && active_ids_.find(item.id) == active_ids_.end()) { continue; }
        // 固定モード（SOURCE_FIXED）は常に青で描画（長さ・収穫可否に依らず）
        bool is_fixed_src = false;
        {
            auto itd_src = last_det_by_id_.find(item.id);
            if (itd_src != last_det_by_id_.end()) {
                is_fixed_src = (itd_src->second.source == fv_stem_detector::msg::StemDetection::SOURCE_FIXED);
            }
        }
        if (is_fixed_src && fixed_blue_enabled) {
            cv::Scalar blue(255,0,0);
            int th = is_sel ? thick_sel : std::max(1, thick_uns);
            cv::rectangle(out, item.bbox, blue, th);
            // 固定はここで完了（以降の長さ/収穫可否ロジックは適用しない）
            // ラベル/HUD更新は従来通り下で行うため continue せずに通過する選択もあるが、
            // ここでは枠描画のみ優先して早期continueする。
            continue;
        }
        // 長さによる枠色決定（非選択グレー枠の色変更要件を反映）
        double Lm = std::numeric_limits<double>::quiet_NaN();
        auto itL = last_length_by_id_.find(item.id);
        if (itL != last_length_by_id_.end()) Lm = itL->second;
        bool valid_as_aspara = isValidAsAsparagus(Lm);
        bool harvestable = isHarvestable(Lm);
        // 有効でない個体はスキップ（リストから除外）
        if (!std::isnan(Lm) && !valid_as_aspara) {
            continue;
        }
        // 色決定（非選択は収穫可否に基づく 1px 半透明ライン）
        cv::Scalar c = color_uns;
        bool harvest_decided = false;
        if (!std::isnan(Lm)) {
            c = harvestable ? cv::Scalar(0,255,0) : cv::Scalar(0,165,255); // 収穫可:緑 / 不可:オレンジ
            harvest_decided = true;
        } else {
            // StemDetection に長さがあればそこから判断
            auto itd2 = last_det_by_id_.find(item.id);
            if (itd2 != last_det_by_id_.end() && std::isfinite(itd2->second.length_m)) {
                double Ld = itd2->second.length_m;
                bool hv = (Ld >= harvest_min_length_m_ && Ld <= harvest_max_length_m_);
                c = hv ? cv::Scalar(0,255,0) : cv::Scalar(0,165,255);
                harvest_decided = true;
            } else {
                c = is_sel ? color_sel : color_uns; // 判定不可時は従来色
            }
        }

        if (is_sel) {
            // 選択中は従来通りの実線描画
            int th = thick_sel;
            cv::rectangle(out, item.bbox, c, th);
        } else {
            if (harvest_decided) {
                // 非選択: 収穫可否が決まっているときは 1px・透過40% で描画
                int thin_px = std::max(1, thick_uns);
                cv::rectangle(overlay_lines, item.bbox, c, thin_px);
                overlay_lines_drawn = true;
            } else {
                // 判定不可: 従来の非選択スタイル（グレー）で1px実線
                cv::rectangle(out, item.bbox, color_uns, std::max(1, thick_uns));
            }
        }
        // ラベル描画の信頼度はここでは使用しない（右情報ウィンドウに集約）
        //std::string label_text = std::string("アスパラ#") + std::to_string(item.id) + " " + std::to_string(conf_pct) + "%";
        // 日本語対応のfluentテキストを利用
        //fluent::text::drawShadow(out, label_text, {item.bbox.x, std::max(0, item.bbox.y-4)}, {255,255,255}, {0,0,0}, 0.6, 2, 0);

        // 選択中のHUD更新
        if (item.id == selected_id_) {
            any_selected_drawn = true;
            // 選択ID変化/ bbox変化が大きい時のみレイアウト更新
            if (last_windows_id_ != item.id || cv::norm(last_windows_bbox_.tl() - item.bbox.tl()) > 2 || std::abs(last_windows_bbox_.area() - item.bbox.area()) > 4) {
                // 右テキスト（黒80%、白1px枠）
                bool info_fixed = false;
                try { info_fixed = this->get_parameter("info.fixed.enabled").as_bool(); } catch (...) {}
                if (!info_fixed) {
                    right_info_window_.setSide(fluent_ui::AutoSizeInfoWindow::Side::Right);
                    right_info_window_.setAnchorBBox(item.bbox);
                }
                // YAMLで色を指定可能に（未設定時は従来色）
                cv::Scalar info_bg(0,0,0), info_border(255,255,255), info_text(255,255,255), info_shadow(0,0,0);
                try {
                    auto v = this->get_parameter("info.color.bg_bgr").as_integer_array(); if (v.size()==3) info_bg=cv::Scalar((int)v[0],(int)v[1],(int)v[2]);
                } catch (...) {}
                try {
                    auto v = this->get_parameter("info.color.border_bgr").as_integer_array(); if (v.size()==3) info_border=cv::Scalar((int)v[0],(int)v[1],(int)v[2]);
                } catch (...) {}
                try {
                    auto v = this->get_parameter("info.color.text_bgr").as_integer_array(); if (v.size()==3) info_text=cv::Scalar((int)v[0],(int)v[1],(int)v[2]);
                } catch (...) {}
                try {
                    auto v = this->get_parameter("info.color.shadow_bgr").as_integer_array(); if (v.size()==3) info_shadow=cv::Scalar((int)v[0],(int)v[1],(int)v[2]);
                } catch (...) {}
                right_info_window_.setBackground(info_bg, true);
                right_info_window_.setBorder(info_border, 1);
                float info_alpha = 0.8f; try { info_alpha = static_cast<float>(this->get_parameter("info.window_alpha").as_double()); } catch (...) {}
                right_info_window_.setAlpha(std::clamp(info_alpha, 0.0f, 1.0f));
                {
                    // 右側情報ウィンドウに詳細を表示（スタイル/幅/行間を先に設定）
                    right_info_window_.setMaxTextWidthPx(340);
                    right_info_window_.setTextStyle(0.7, 2, info_text, info_shadow);
                    int lh = static_cast<int>(std::round(20 * 0.7)) + 3;
                    right_info_window_.setLineHeightPx(lh);

                    auto fmt_cm = [](float m)->std::string{ if (!std::isfinite(m)) return std::string("--"); char b[32]; snprintf(b, sizeof(b), "%.1f", m*100.0f); return std::string(b); };
                    auto fmt_ms = [](double ms)->std::string{ if (!std::isfinite(ms)) return std::string("--"); char b[32]; snprintf(b, sizeof(b), "%.1f", ms); return std::string(b); };
                    std::vector<std::string> lines;
                    lines.emplace_back(std::string("アスパラガス#") + std::to_string(item.id));
                    auto itd = last_det_by_id_.find(item.id);
                    if (itd != last_det_by_id_.end()) {
                        const auto &d = itd->second;
                        // 1行1項目に分割
                        lines.emplace_back(std::string("2D信頼度: ") + (std::isfinite(d.yolo_score)? std::to_string((int)std::round(d.yolo_score*100.0f)) : std::string("--")) + "%");
                        lines.emplace_back(std::string("3D信頼度: ") + (std::isfinite(d.pca_score)? std::to_string((int)std::round(d.pca_score*100.0f)) : std::string("--")) + "%");
                        // 要望により距離表記を削除し、座標出力に専念（右ウィンドウは簡潔化のため距離行を非表示）
                        lines.emplace_back(std::string("長さ: ") + (std::isfinite(d.length_m)? fmt_cm(d.length_m) : std::string("--")) + " cm");
                        //lines.emplace_back(std::string("曲率: ") + (std::isfinite(d.curvature)? std::to_string((int)std::round(d.curvature*100.0f)) : std::string("--")) + "%");
                        lines.emplace_back(std::string("分析時間: ") + fmt_ms(d.roi_ms) + " ms");
                        lines.emplace_back(std::string("点群数: ") + std::to_string(d.point_count));

                    } else {
                        lines.emplace_back(std::string("詳細: --"));
                    }
                    // 行配列を結合（後から追加行を容易にするため分割構造を維持）
                    std::string text;
                    for (size_t i = 0; i < lines.size(); ++i) {
                        text += lines[i];
                        if (i + 1 < lines.size()) text += "\n";
                    }
                    right_info_window_.setText(text, true);
                }
                right_info_window_.tick(0.0);
                // 固定配置のときは (fx,fy) に移動し、幅/高さが指定されていれば固定
                {
                    bool info_fixed2 = false; int fx2=0, fy2=0; int fw=0, fh=0;
                    try { info_fixed2 = this->get_parameter("info.fixed.enabled").as_bool(); } catch (...) {}
                    try { fx2 = this->get_parameter("info.fixed.x").as_int(); } catch (...) {}
                    try { fy2 = this->get_parameter("info.fixed.y").as_int(); } catch (...) {}
                    try { fw = this->get_parameter("info.fixed.width").as_int(); } catch (...) {}
                    try { fh = this->get_parameter("info.fixed.height").as_int(); } catch (...) {}
                    if (info_fixed2) {
                        right_info_window_.setLocked(true);
                        if (fw > 0) { right_info_window_.setForcedWidthPx(fw); }
                        cv::Rect cur = right_info_window_.currentRect();
                        int use_w = (fw > 0) ? fw : std::max(1, cur.width);
                        int use_h = (fh > 0) ? fh : std::max(1, cur.height);
                        cv::Rect fixed(fx2, fy2, use_w, use_h);
                        right_info_window_.setImmediateRect(fixed);
                    }
                }
                last_windows_id_ = item.id;
                last_windows_bbox_ = item.bbox;
            } else {
                // 位置のみ追従（右情報ウィンドウ）
                bool info_fixed = false; try { info_fixed = this->get_parameter("info.fixed.enabled").as_bool(); } catch (...) {}
                if (!info_fixed) {
                    right_info_window_.setAnchorBBox(item.bbox);
                }
            }
            // 追加オーバーレイ（アプローチ/根本/先端/軸）
            {
                auto itd = last_det_by_id_.find(item.id);
                if (itd != last_det_by_id_.end()) {
                    const auto &d = itd->second;
                    if (d.approach_px_x >= 0 && d.approach_px_y >= 0) cv::circle(out, cv::Point(d.approach_px_x, d.approach_px_y), 6, color_approach_, 2);
                    if (d.root_px_x >= 0 && d.root_px_y >= 0) cv::circle(out, cv::Point(d.root_px_x, d.root_px_y), 6, color_root_, 2);
                    // tipは未算出時(0,0)を誤描画しない
                    if (d.tip_px_x >= 0 && d.tip_px_y >= 0 && (d.tip_px_x != 0 || d.tip_px_y != 0)) {
                        cv::circle(out, cv::Point(d.tip_px_x, d.tip_px_y), 6, color_tip_, 2);
                    }
                    if (d.root_px_x >= 0 && d.root_px_y >= 0 && d.tip_px_x >= 0 && d.tip_px_y >= 0 && (d.tip_px_x != 0 || d.tip_px_y != 0)) {
                        cv::line(out, cv::Point(d.root_px_x, d.root_px_y), cv::Point(d.tip_px_x, d.tip_px_y), color_axis_, 1, cv::LINE_AA);
                    }
                }
            }

            // 右テキスト（黒80%、白1px枠）
            if (info_enabled) right_info_window_.show(); else right_info_window_.hide();

            // 選択矩形ROIの点群出力（停止中）
            // NOTE: ユーザー要望により、一旦コメントアウト。将来復帰時は computeApproachDistanceM→publishRoiPointcloudIfReady を再接続。
            // double appr_z = -1.0;
            // if (computeApproachDistanceM(approach_px_, appr_z)) {
            //     publishRoiPointcloudIfReady(appr_z);
            // }
        }
    }
    // 選択なし（または描画スキップで出ていない）ならウィンドウは隠す
    if (!any_selected_drawn) {
        right_info_window_.hide();
        approach_window_.hide();
    }
    // 非選択ラインの半透明合成（40%）
    if (overlay_lines_drawn) {
        double ns_alpha = 0.4; try { ns_alpha = this->get_parameter("overlay.nonselected_lines_alpha").as_double(); } catch (...) {}
        cv::addWeighted(out, 1.0, overlay_lines, std::clamp(ns_alpha, 0.0, 1.0), 0.0, out);
    }

    // カーソル描画
    if (cursor_.IsShown()) {
        cv::Point c = cursor_.GetPosition();
        cv::Scalar cc = cursor_.Color(); int sz = cursor_.SizePx(); int th = cursor_.ThicknessPx();
        int x1 = std::max(0, c.x - sz), x2 = std::min(out.cols-1, c.x + sz);
        int y1 = std::max(0, c.y - sz), y2 = std::min(out.rows-1, c.y + sz);
        cv::line(out, cv::Point(x1, c.y), cv::Point(x2, c.y), cc, th);
        cv::line(out, cv::Point(c.x, y1), cv::Point(c.x, y2), cc, th);
    }
    // 固定表示が有効なら、毎フレーム 強制位置/サイズ に固定する
    {
        bool info_fixed = false; int fx=0, fy=0; int fw=0, fh=0;
        try { info_fixed = this->get_parameter("info.fixed.enabled").as_bool(); } catch (...) {}
        try { fx = this->get_parameter("info.fixed.x").as_int(); } catch (...) {}
        try { fy = this->get_parameter("info.fixed.y").as_int(); } catch (...) {}
        try { fw = this->get_parameter("info.fixed.width").as_int(); } catch (...) {}
        try { fh = this->get_parameter("info.fixed.height").as_int(); } catch (...) {}
        if (info_fixed) {
            if (fw > 0) { right_info_window_.setForcedWidthPx(fw); }
            cv::Rect cur = right_info_window_.currentRect();
            int use_w = (fw > 0) ? fw : std::max(1, cur.width);
            int use_h = (fh > 0) ? fh : std::max(1, cur.height);
            right_info_window_.setImmediateRect(cv::Rect(fx, fy, use_w, use_h));
        }
    }
    // HUD合成（最後）
    {
        bool info_enabled_end = true; try { info_enabled_end = this->get_parameter("info_window_enabled").as_bool(); } catch (...) {}
        if (info_enabled_end) { right_info_window_.render(out); }
    }

    // フォールバックは待機画面を使用する方針のため、直近画像の保持は行わない
    auto hdr = latest_color_ ? latest_color_->header : std_msgs::msg::Header();
    // 生画像は購読者がいる場合のみ公開
    if (annotated_image_pub_ && annotated_image_pub_->get_subscription_count() > 0) {
        auto msg = cv_bridge::CvImage(hdr, sensor_msgs::image_encodings::BGR8, out).toImageMsg();
        annotated_image_pub_->publish(*msg);
    }
    // 圧縮は購読者がいる場合のみ生成/公開（高コスト回避）
    if (annotated_compressed_pub_ && annotated_compressed_pub_->get_subscription_count() > 0) {
        try {
            auto cimg = cv_bridge::CvImage(hdr, sensor_msgs::image_encodings::BGR8, out);
            sensor_msgs::msg::CompressedImage cmsg; cmsg.header = hdr; cmsg.format = "jpeg"; cimg.toCompressedImageMsg(cmsg);
            annotated_compressed_pub_->publish(cmsg);
        } catch (...) {}
    }
}
// ROI点群出力は廃止

} // namespace fv_aspara_analyzer

// エントリポイント（最小）
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fv_aspara_analyzer::FvAsparaAnalyzerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


