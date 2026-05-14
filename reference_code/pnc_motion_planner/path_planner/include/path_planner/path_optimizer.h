/**
 * @file path_optimizer.h
 * @brief 路径优化器实现文件
 * @details 该文件实现了路径优化器的核心功能，包括路径优化、范围判断等。路径优化器用于在路径规划过程中对路径进行平滑和优化，确保路径的连续性和可行性。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include "base/status.h"
#include "speed/speed_data.h"
#include "reference_line/reference_line.h"
#include "path/path_boundary.h"
#include "path/path_decision.h"
#include "path/path_data.h"
#include "config/vehicle_model/vehicle_config.pb.h"

namespace gpal::pnc::planning {

/**
 * @brief 路径优化器类,实现了路径优化器的核心功能，用于在路径规划过程中对路径进行平滑和优化，确保路径的连续性和可行性。
 */
class PathOptimizer {
 public:
  /**
   * @brief 默认构造函数
   */
  PathOptimizer() = default;
  /**
   * @brief 默认析构函数
   */
  ~PathOptimizer() = default;

  /**
   * @brief 初始化路径优化器
   * @return 返回初始化是否成功
   */
  virtual bool init();

  /**
   * @brief 获取路径优化器的名称
   * @return 返回路径优化器的名称
   */
  virtual std::string name() const = 0;

  /**
   * @brief 路径优化处理函数
   * @param reference_line 参考线
   * @param start_pt 起始点
   * @param boundary 路径边界
   * @param final_path_data 最终路径数据
   * @return 返回处理状态
   */
  virtual Status proc(const ReferenceLine& reference_line, const TrajectoryPt& start_pt,
                      const PathBoundary& boundary, PathData* const final_path_data) = 0;
  void refinePath(const TrajectoryPt& start_pt, PathData* const path_data);

  static bool isInRange(const std::vector<std::pair<float, float>>& ranges, const float& s, float s_tolerance = 0.0);
  static bool isInRange(const std::vector<std::tuple<std::string, float, float>>& special_tags_range, const std::string tag,
                        const float& s, float s_tolerance = 0.0);
  static bool isInIgnoreRange(const std::vector<IgnoreRangeInfo>& ranges, const double& s);

 protected:
  std::shared_ptr<VehicleConfig> vehicle_config_ = nullptr; ///< 车辆配置参数
  std::shared_ptr<VehicleParam> vehicle_param_ = nullptr;  ///< 车辆参数
};

}
