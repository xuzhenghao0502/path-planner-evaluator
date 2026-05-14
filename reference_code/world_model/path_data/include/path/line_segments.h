/**
 * @file line_segments.h
 * @brief 线段集合数据结构定义
 * @details 提供线段集合数据结构，用于存储特定类型的连续线段集合
 */

#pragma once
#include "math/line_segment2d.h"

namespace gpal::pnc::planning {

/**
 * @brief 线段集合数据结构
 * @details 存储特定类型的连续线段集合，用于道路边界、障碍物边界等场景
 }
 */
struct LineSegments {
  /**
   * @brief 线段类型枚举
   * @details 用于标识线段集合的类型，如道路边界、障碍物边界等
   * @note 可以根据实际需求扩展更多类型
   */
  enum class Type {
    LEFT_ROAD_BORADER,   ///< 左侧道路边界
    RIGHT_ROAD_BORADER,  ///< 右侧道路边界
    LEFT_OB,             ///< 左侧障碍物边界
    RIGHT_OB,            ///< 右侧障碍物边界
  };

  Type type{Type::LEFT_ROAD_BORADER};         ///< 线段类型标识
  double start_s{0.0};                        ///< 起始s坐标
  double end_s{0.0};                          ///< 结束s坐标
  std::vector<math::LineSegment2d> segments;  ///< 线段集合
};

}  // namespace gpal::pnc::planning
