/**
 * @file reference_line_smoother.h
 * @brief 参考线平滑器头文件
 * @details 该文件定义了参考线平滑器的基类ReferenceLineSmoother，用于对参考线进行平滑处理。
 *          主要功能包括：初始化平滑器、平滑参考线、定义锚点结构体等。
 */

#pragma once

#include <vector>

#include "config/reference_line_smoother_config.pb.h"
#include "reference_line.h"

namespace gpal::pnc::planning {

/**
 * @brief 锚点结构体，用于参考线平滑
 */
struct AnchorPoint {
  gpal::proto::PathPoint path_point;  ///< 路径点信息
  double lateral_bound = 0.0;         ///< 横向边界约束
  double longitudinal_bound = 0.0;    ///< 纵向边界约束
  // enforce smoother to strictly follow this reference point
  bool enforced = false;  ///< 是否强制平滑器严格遵循此参考点
};

/**
 * @brief 参考线平滑器基类
 */
class ReferenceLineSmoother {
 public:
  /**
   * @brief 构造函数
   * @param config 参考线平滑器配置
   */
  explicit ReferenceLineSmoother(const planning::ReferenceLineSmootherConfig& config) : config_(config) {}

  /**
   * @brief 平滑参考线
   * @param input 输入的参考线
   * @param output 输出的平滑后的参考线
   * @param is_async 是否异步执行
   * @return 如果平滑成功返回true，否则返回false
   */
  virtual bool smooth(const ReferenceLine&, ReferenceLine* const, bool is_async = false) = 0;

  /**
   * @brief 虚析构函数
   */
  virtual ~ReferenceLineSmoother() = default;

 protected:
  planning::ReferenceLineSmootherConfig config_;  ///< 参考线平滑器配置
};

}  // namespace gpal::pnc::planning
