/**
 * @file reference_line_provider.h
 * @brief 参考线提供器实现文件
 * @details 该文件实现了ReferenceLineProvider类，负责为规划模块提供平滑后的参考线。
 *          主要功能包括：初始化参考线平滑器、平滑不同类型的路径（如记忆路线、感知车道线、历史路径等）、
 *          以及将原始路径转换为参考线。
 */

#pragma once

#include <chrono>
#include <cstdio>
#include <future>
#include <list>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "base/log.h"
#include "config/vehicle_model/vehicle_config.pb.h"
#include "config_manager/config_manager.h"
#include "point/path_pt.h"
#include "proto/vehicle_state/vehicle_state.pb.h"
#include "reference_line.h"
#include "reference_line_smoother.h"

namespace gpal::pnc::planning {

/**
 * @class ReferenceLineProvider
 * @brief 参考线提供器类，负责为规划模块提供平滑后的参考线
 */
class ReferenceLineProvider {
 public:
  ReferenceLineProvider();
  /**
   * @brief 析构函数，用于释放ReferenceLineProvider类的资源
   */
  ~ReferenceLineProvider() {}

  bool smoothReferenceLines(std::shared_ptr<SlicedRoute> route, std::vector<ReferenceLine>* smoothed_res);
  bool smoothReferenceLines(const std::vector<PerceptionLane>& lanes, std::vector<ReferenceLine>* smoothed_res);
  bool smoothReferenceLines(const MainPathHistory& path_history, std::vector<ReferenceLine>* smoothed_res);
  bool smoothReferenceLines(const std::map<int, LocalRoute>& local_routes, std::vector<ReferenceLine>* smoothed_res);

  bool addRawReferenceLines(const std::shared_ptr<SlicedRoute>& route, std::vector<ReferenceLine>* raw_res);
  bool addRawReferenceLines(const std::vector<PerceptionLane>& lanes, std::vector<ReferenceLine>* raw_res);
  bool addRawReferenceLines(const std::map<int, LocalRoute>& local_routes, std::vector<ReferenceLine>* raw_res);

  bool smoothReferenceLine(const ReferenceLine& raw_reference_line, ReferenceLine* reference_line,
                           bool is_async = false);
  bool smoothSlicedRoute(const std::shared_ptr<SlicedRoute> route, std::vector<ReferenceLine>* reference_lines);
  bool smoothPerceptionLanes(const std::vector<PerceptionLane>& lanes, std::vector<ReferenceLine>* reference_lines);
  bool smoothPathHistory(const MainPathHistory& path_history, std::vector<ReferenceLine>* reference_lines);
  bool smoothLocalRoutes(const std::map<int, LocalRoute>& local_routes, std::vector<ReferenceLine>* reference_lines);

  bool smoothOcpReferenceLine(const ReferenceLine& raw_reference_line, ReferenceLine* reference_line,
                              bool is_async = false);

  /**
   * @brief 设置参考线平滑功能使能标志
   * @details 该函数用于控制是否启用参考线平滑功能。当标志为true时，生成的参考线会经过平滑处理；
   *          当标志为false时，将直接使用原始参考线。
   *
   * @param[in] enable_smooth_ref_line 平滑功能使能标志
   *  - 变量类型: bool
   *  - 取值范围:
   *    - true: 启用参考线平滑
   *    - false: 禁用参考线平滑
   *  - 默认值: 由类初始化时决定（默认为true）
   *
   * @par 成员变量说明:
   * - enable_smooth_ref_line_: 布尔类型成员变量，存储当前平滑功能状态
   */
  void setEnableSmoothRefLine(const bool& enable_smooth_ref_line) { enable_smooth_ref_line_ = enable_smooth_ref_line; }

 private:
  planning::ConfigManager config_;                         ///< 配置管理器
  planning::ReferenceLineSmootherConfig smoother_config_;  ///< 参考线平滑器配置
  bool is_initialized_ = false;                            ///< 是否已初始化
  double last_calculation_time_ = 0.0;                     ///< 上次计算时间
  bool enable_smooth_ref_line_ = true;                     ///< 是否启用参考线平滑
};
}  // namespace gpal::pnc::planning
