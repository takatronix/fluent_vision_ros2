#ifndef FLUENT_CLOUD_UTILS_POINT_TYPES_HPP
#define FLUENT_CLOUD_UTILS_POINT_TYPES_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace fluent_cloud {
namespace utils {

/**
 * PCLの主要なポイントタイプの説明
 */

// ========== 基本的なポイントタイプ ==========

// pcl::PointXYZ - 最も基本的な3D座標のみ
// struct {
//   float x, y, z;
//   float padding;  // SSE最適化のため
// }

// pcl::PointXYZI - 3D座標 + 輝度
// struct {
//   float x, y, z;
//   float intensity;  // LiDARの反射強度など
// }

// pcl::PointXYZRGB - 3D座標 + RGB色
// struct {
//   float x, y, z;
//   union {
//     struct { uint8_t b, g, r, a; };
//     float rgb;
//   };
// }

// pcl::PointXYZRGBA - 3D座標 + RGBA色（透明度付き）
// struct {
//   float x, y, z;
//   union {
//     struct { uint8_t b, g, r, a; };
//     uint32_t rgba;
//   };
// }

// pcl::PointXYZRGBL - 3D座標 + RGB色 + ラベル
// struct {
//   float x, y, z;
//   union {
//     struct { uint8_t b, g, r, a; };
//     float rgb;
//   };
//   uint32_t label;  // セグメンテーション用ラベル
// }

// ========== 法線付きポイントタイプ ==========

// pcl::PointNormal - 3D座標 + 法線ベクトル
// struct {
//   float x, y, z;
//   float normal_x, normal_y, normal_z;
//   float curvature;  // 曲率
// }

// pcl::PointXYZINormal - 3D座標 + 輝度 + 法線
// struct {
//   float x, y, z;
//   float intensity;
//   float normal_x, normal_y, normal_z;
//   float curvature;
// }

// pcl::PointXYZRGBNormal - 3D座標 + RGB色 + 法線
// struct {
//   float x, y, z;
//   float normal_x, normal_y, normal_z;
//   union {
//     struct { uint8_t b, g, r, a; };
//     float rgb;
//   };
//   float curvature;
// }

// ========== カスタムポイントタイプの定義例 ==========

// セマンティック情報付きポイント
struct PointXYZRGBSemantic {
    PCL_ADD_POINT4D;  // x, y, z, padding を追加
    PCL_ADD_RGB;      // rgb を追加
    uint32_t semantic_label;  // 意味ラベル（車、歩行者、建物など）
    float confidence;         // 分類の信頼度
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 時刻情報付きポイント（4D LiDAR用）
struct PointXYZIT {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;  // 各点の取得時刻
    uint16_t ring;     // LiDARのリング番号
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 詳細特徴付きポイント
struct PointXYZRGBFeatures {
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_NORMAL4D;  // normal_x, normal_y, normal_z, curvature
    float descriptor[33];  // FPFH特徴量（33次元）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// ========== ポイントタイプ変換ユーティリティ ==========

template<typename PointInT, typename PointOutT>
class PointTypeConverter {
public:
    static void convert(const pcl::PointCloud<PointInT>& input,
                       pcl::PointCloud<PointOutT>& output) {
        output.clear();
        output.header = input.header;
        output.width = input.width;
        output.height = input.height;
        output.is_dense = input.is_dense;
        output.sensor_origin_ = input.sensor_origin_;
        output.sensor_orientation_ = input.sensor_orientation_;
        
        output.points.resize(input.points.size());
        
        for (size_t i = 0; i < input.points.size(); ++i) {
            copyPoint(input.points[i], output.points[i]);
        }
    }
    
private:
    // 基本的なコピー（XYZのみ）
    template<typename T1, typename T2>
    static void copyPoint(const T1& src, T2& dst) {
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
    }
    
    // 特殊化：色情報のコピー
    template<>
    static void copyPoint(const pcl::PointXYZRGB& src, pcl::PointXYZRGB& dst) {
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.rgb = src.rgb;
    }
    
    // 特殊化：輝度情報のコピー
    template<>
    static void copyPoint(const pcl::PointXYZI& src, pcl::PointXYZI& dst) {
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
    }
};

// ========== ポイント情報の取得ヘルパー ==========

template<typename PointT>
class PointInfo {
public:
    static std::string getInfo(const PointT& point) {
        std::stringstream ss;
        ss << "Point(" << point.x << ", " << point.y << ", " << point.z << ")";
        return ss.str();
    }
    
    static bool hasRGB() { return false; }
    static bool hasIntensity() { return false; }
    static bool hasNormal() { return false; }
};

// 特殊化：RGB付き
template<>
class PointInfo<pcl::PointXYZRGB> {
public:
    static std::string getInfo(const pcl::PointXYZRGB& point) {
        std::stringstream ss;
        ss << "Point(" << point.x << ", " << point.y << ", " << point.z 
           << ") RGB(" << (int)point.r << ", " << (int)point.g << ", " << (int)point.b << ")";
        return ss.str();
    }
    
    static bool hasRGB() { return true; }
    static bool hasIntensity() { return false; }
    static bool hasNormal() { return false; }
};

// 特殊化：法線付き
template<>
class PointInfo<pcl::PointNormal> {
public:
    static std::string getInfo(const pcl::PointNormal& point) {
        std::stringstream ss;
        ss << "Point(" << point.x << ", " << point.y << ", " << point.z 
           << ") Normal(" << point.normal_x << ", " << point.normal_y << ", " << point.normal_z 
           << ") Curvature(" << point.curvature << ")";
        return ss.str();
    }
    
    static bool hasRGB() { return false; }
    static bool hasIntensity() { return false; }
    static bool hasNormal() { return true; }
};

} // namespace utils
} // namespace fluent_cloud

// PCLにカスタムポイントタイプを登録
POINT_CLOUD_REGISTER_POINT_STRUCT(fluent_cloud::utils::PointXYZRGBSemantic,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (uint32_t, semantic_label, semantic_label)
    (float, confidence, confidence)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(fluent_cloud::utils::PointXYZIT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (uint16_t, ring, ring)
)

#endif // FLUENT_CLOUD_UTILS_POINT_TYPES_HPP