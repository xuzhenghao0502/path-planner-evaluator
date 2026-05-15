
/**
 * @file path_bound_filter.h
 * @brief 路径边界过滤器实现文件
 * @details 该文件实现了路径边界过滤器的核心功能，包括路径边界的更新、过滤、重置等操作。路径边界过滤器用于在路径规划过程中对路径边界进行平滑处理，确保路径的连续性和可行性。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "config_manager/config_manager.h"
#include "util/table_interpolate.h"

#include "config/path_bound_parser/path_bound_parser_config.pb.h"

namespace gpal::pnc::planning  {

/**
 * @class PathBoundFilter
 * @brief 路径边界过滤器类，用于对路径边界信息进行过滤和更新。
 * @details 该类提供了对路径边界信息的过滤功能，支持根据速度、类型等条件对边界信息进行动态调整。
 */
class PathBoundFilter {
 public:
  /**
   * @brief 默认构造函数
   */
  PathBoundFilter() = default;
  PathBoundFilter(const PathBoundFilterConfig& config);
  /**
   * @brief 默认析构函数
   */
  ~PathBoundFilter() = default;

  /**
   * @brief 获取当前路径边界信息
   * @return 返回当前路径边界信息的引用
   */
  const util::AbstractTable1d<double, double, double>& bound() const { return bound_; }

  void reset();

  /**
   * @brief 设置速度过滤输出系数
   * @param filter_out_speed 速度过滤输出系数，用于平滑处理当前边界与前一帧边界的关系
   */
  void setFilterOutSpeed(const double filter_out_speed){ filter_out_speed_ = filter_out_speed; }
  void update(const std::vector<std::tuple<double, double, double>>& bound);
  void updateWithSpeed(const std::vector<std::tuple<double, double, double>>& bound);
  void updateWithSpeed(const std::vector<std::tuple<double, double, double>>& bound, const double& cur_s, const std::string type = "barrier");

 protected:
  void updateFilterBoundary(const double& filter_in, const double& filter_out,
                            std::tuple<double, double, double>& current_bound);

 protected:
  PathBoundFilterConfig config_;  ///< 路径边界过滤器配置，包含过滤器的各项参数
  double filter_out_speed_ = 0.3; ///< 速度过滤输出系数，用于平滑处理当前边界与前一帧边界的关系
  util::AbstractTable1d<double, double, double> bound_; ///< 存储当前路径边界信息的容器
};

} 
