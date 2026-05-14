/**
 * @file speed_common.h
 * @brief 速度规划的公共头文件
 * @details 包含了速度规划的公共头文件，主要包含了一些常用的结构体和函数
 */
#pragma once

#include "obstacle/obstacle.h"
#include "path/path_data.h"
#include "speed/speed_result.h"
#include "base/log.h"

namespace gpal::pnc::planning {

struct SpeedState {
  /// @brief 纵向位移（单位：米）
  double s{0.0};
  /// @brief 纵向速度（单位：m/s）
  double v{0.0};
  /// @brief 纵向加速度（单位：m/s²）
  double a{0.0};
};

struct BoxInfo {
  /// @brief 默认构造函数
  BoxInfo() = default;
  /// @brief 构造矩形框信息对象
  /// @param[in] x 中心点x坐标（单位：米）
  /// @param[in] y 中心点y坐标（单位：米）
  /// @param[in] theta 朝向角（单位：弧度，取值范围[-π, π]）
  /// @param[in] length 长度（单位：米，必须>0）
  /// @param[in] width 宽度（单位：米，必须>0）
  BoxInfo(const double& x, const double& y, const double& theta, const double& length, const double& width)
      : x(x), y(y), theta(theta), length(length), width(width) {
    cos_heading = std::cos(theta);
    sin_heading = std::sin(theta);
  }

  /// @brief 中心点x坐标（单位：米）
  double x = 0.0;
  /// @brief 中心点y坐标（单位：米）
  double y = 0.0;
  /// @brief 朝向角（单位：弧度，逆时针为正）
  double theta = 0.0;
  /// @brief 长度（单位：米，沿朝向方向）
  double length = 0.0;
  /// @brief 宽度（单位：米，垂直朝向方向）
  double width = 0.0;
  /// @brief 朝向角余弦值
  double cos_heading = 0.0;
  /// @brief 朝向角正弦值
  double sin_heading = 0.0;
};

struct BoxProjectInfo {
  /// @brief 默认构造函数
  BoxProjectInfo() = default;
  /// @brief 构造函数，直接初始化投影信息
  /// @param i 对应的路径段索引
  /// @param t 投影时间戳
  /// @param s 在路径上的纵向投影坐标
  /// @param l 在路径上的横向投影坐标
  BoxProjectInfo(const size_t& i, const double& t, const double& s, const double& l)
      : index(i), time(t), project_s(s), project_l(l) {}
  /// @brief 构造函数，直接初始化投影信息
  /// @param i 对应的路径段索引
  /// @param t 投影时间戳
  /// @param sl_pair 在路径上的纵向和横向投影坐标
  BoxProjectInfo(const size_t& i, const double& t, const std::pair<double, double>& sl_pair)
      : index(i), time(t), project_s(sl_pair.first), project_l(sl_pair.second) {}

  /// @brief 路径段索引（从0开始计数）
  size_t index = 0;

  /// @brief 投影时间戳（单位：秒）
  double time = 0.0;

  /// @brief 纵向投影坐标（单位：米，沿路径方向）
  double project_s = 0.0;

  /// @brief 横向投影坐标（单位：米，左正右负）
  double project_l = 0.0;
};

struct PathGroup {
  /// @brief 原始离散化路径
  DiscretizedPath origin_path_;

  /// @brief 扩展后的多分辨率路径
  DiscretizedPath extand_interval_path_;

  /// @brief 用于路径分段的原始路径点集合
  std::vector<PathPt> path_points_for_segmentation_;

  /// @brief 扩展后的路径分段点集合
  std::vector<PathPt> path_points_for_segmentation_after_extand_;

  /// @brief 路径分段后的线段集合
  std::vector<math::LineSegment2d> line_segments_for_path_segmentation_;

  /// @brief 清空所有路径数据
  void clear() {
    origin_path_.clear();
    extand_interval_path_.clear();
    path_points_for_segmentation_.clear();
    path_points_for_segmentation_after_extand_.clear();
    line_segments_for_path_segmentation_.clear();
  }
};

struct LateralPath {
  /// @brief 横向轨迹路径组
  PathGroup discretized_path_group_;
  /// @brief 局部路径组
  PathGroup local_path_group_;
};

DiscretizedPath intervalPath(const DiscretizedPath& path, const double& step);
std::pair<size_t, PathPt> evaluatePath(const DiscretizedPath& path, size_t start_index, const double s);
// std::pair<double, double> CalcSLCoordinatesToInfinitPath(const PathGroup& path_group, double init_s,
//                                                          const math::Vec2d& pt, int& collision_segment_index);
bool isObstacleStatic(const Decision::ObjectType& obstacle_type,
                      const std::vector<proto::TrajectoryPoint>& obs_pred_trajectory, const bool& is_static);

}  // namespace gpal::pnc::planning
