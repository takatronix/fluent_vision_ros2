# ポイントクラウド最適化ガイド - トピック公開前の削減

## なぜトピック公開前に減らすべきか？

### ネットワーク負荷の比較
```
640x480 @ 6fps の場合：
- 生データ: 307,200点 × 6fps × 32bytes/点 = 59MB/秒
- 削減後: 10,000点 × 6fps × 32bytes/点 = 1.9MB/秒（97%削減！）
```

### CPU負荷
- シリアライズ/デシリアライズのコストが激減
- トピック購読側の処理も軽くなる

## 削減戦略

### 1. **ダウンサンプリング（最も簡単）**
```cpp
// 公開前に削減
auto cloud = generatePointCloud(depth, color, camera_info);

// 方法1: ボクセルグリッド
pcl::VoxelGrid<pcl::PointXYZRGB> vg;
vg.setInputCloud(cloud);
vg.setLeafSize(0.01f, 0.01f, 0.01f);  // 1cmボクセル
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
vg.filter(*downsampled);

// 307,200点 → 約15,000点に削減
publish(downsampled);  // これを公開
```

### 2. **関心領域（ROI）のみ公開**
```cpp
// アスパラガスがありそうな範囲だけ
auto filtered = FluentCloud::from(cloud)
    .cropBox(Eigen::Vector3f(-0.5, -0.5, 0.2),    // min
             Eigen::Vector3f(0.5, 0.5, 1.5))       // max
    .downsample(0.005)
    .toPointCloud();
    
// 全体の20%程度に削減
publish(filtered);
```

### 3. **適応的削減（スマート）**
```cpp
class AdaptivePointCloudPublisher {
public:
    void publishOptimized(const PointCloudPtr& cloud) {
        size_t original_size = cloud->size();
        PointCloudPtr to_publish;
        
        if (original_size > 100000) {
            // 非常に多い → 積極的に削減
            to_publish = downsample(cloud, 0.02);  // 2cmボクセル
        } else if (original_size > 50000) {
            // やや多い → 適度に削減
            to_publish = downsample(cloud, 0.01);  // 1cmボクセル
        } else if (original_size > 20000) {
            // 普通 → 軽く削減
            to_publish = downsample(cloud, 0.005); // 5mmボクセル
        } else {
            // 少ない → そのまま
            to_publish = cloud;
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "点群削減: %zu → %zu点 (%.1f%%)",
            original_size, to_publish->size(),
            100.0 * to_publish->size() / original_size);
            
        cloud_pub_->publish(toROSMsg(to_publish));
    }
};
```

### 4. **段階的処理パイプライン**
```cpp
// 効率的なパイプライン設計
class EfficientPipeline {
    // ステップ1: 生成時に既に削減
    PointCloudPtr generateReduced(const cv::Mat& depth, 
                                 const cv::Mat& color,
                                 int skip = 2) {
        // 2ピクセルおきに処理（75%削減）
        for (int v = 0; v < depth.rows; v += skip) {
            for (int u = 0; u < depth.cols; u += skip) {
                // 3D変換
            }
        }
    }
    
    // ステップ2: フィルタリング
    PointCloudPtr filterNoise(const PointCloudPtr& cloud) {
        return FluentCloud::from(cloud)
            .removeOutliers(20, 2.0)  // 外れ値除去
            .filterByDistance(0.2, 2.0)  // 距離フィルタ
            .toPointCloud();
    }
    
    // ステップ3: 最終削減
    PointCloudPtr finalReduction(const PointCloudPtr& cloud) {
        // 目標: 10,000点以下
        if (cloud->size() > 10000) {
            float leaf_size = std::sqrt(cloud->size() / 10000.0) * 0.005;
            return downsample(cloud, leaf_size);
        }
        return cloud;
    }
};
```

## 実装例：完全な最適化ノード

```cpp
class OptimizedPointCloudNode : public rclcpp::Node {
public:
    OptimizedPointCloudNode() : Node("optimized_pointcloud") {
        // パラメータ
        this->declare_parameter<int>("target_points", 10000);
        this->declare_parameter<bool>("publish_full", false);
        this->declare_parameter<bool>("publish_roi", true);
        this->declare_parameter<bool>("publish_downsampled", true);
        
        target_points_ = this->get_parameter("target_points").as_int();
        
        // 複数の最適化版を公開
        if (this->get_parameter("publish_full").as_bool()) {
            full_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/points/full", 1);  // QoS=1（大きいデータ）
        }
        
        if (this->get_parameter("publish_roi").as_bool()) {
            roi_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/points/roi", 10);
        }
        
        if (this->get_parameter("publish_downsampled").as_bool()) {
            downsampled_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/points/downsampled", 10);
        }
    }
    
    void processAndPublish(const PointCloudPtr& cloud) {
        auto start = std::chrono::high_resolution_clock::now();
        
        // 1. フル版（デバッグ用、通常は無効）
        if (full_pub_) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*cloud, msg);
            full_pub_->publish(msg);
        }
        
        // 2. ROI版（アスパラガス領域）
        if (roi_pub_) {
            auto roi_cloud = extractROI(cloud);
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*roi_cloud, msg);
            roi_pub_->publish(msg);
        }
        
        // 3. ダウンサンプル版（可視化用）
        if (downsampled_pub_) {
            auto down_cloud = adaptiveDownsample(cloud, target_points_);
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*down_cloud, msg);
            downsampled_pub_->publish(msg);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "公開処理時間: %ld μs", duration.count());
    }
    
private:
    int target_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr full_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_pub_;
};
```

## 推奨設定

### アスパラガス検出用
```yaml
# 生成時
generation:
  skip_pixels: 2      # 2ピクセルおき（75%削減）
  min_distance: 0.2   # 20cm以上
  max_distance: 1.5   # 1.5m以下

# 公開前
pre_publish:
  voxel_size: 0.01   # 1cmボクセル
  target_points: 10000  # 最大1万点
  remove_outliers: true
```

### 可視化用
```yaml
visualization:
  voxel_size: 0.02   # 2cmボクセル（粗い）
  target_points: 5000   # 5千点で十分
  colorize: true     # 色付き
```

## まとめ

**必ず公開前に削減すべき理由：**
1. ネットワーク帯域を97%節約
2. 購読側のCPU負荷も軽減
3. 録画時のストレージも節約
4. 遅延の削減

**削減しても問題ない理由：**
- アスパラガス解析には高密度不要
- 可視化には5,000点で十分
- 必要なら購読側で補間可能