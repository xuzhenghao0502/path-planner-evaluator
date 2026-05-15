/**
 * @file common_utils.h
 * @brief 包含决策数据相关的通用工具函数，为决策过程提供基础功能支持
 * @author 智能驾驶决策规划团队
 * @date 2024-03-01
 * @version 1.0
 * @copyright Copyright (C) 2024 Shanghai Geometrical Perception and Learning Co., Ltd. All rights reserved.
 * @note 模块功能：
 * - 提供一系列通用工具函数，辅助决策数据的处理和计算
 */
#include <cmath>

#include "decision_data/decision_result.h"

namespace gpal::pnc::planning {
namespace Decision {
std::tuple<double, double, BoundaryPointTypeInfo, BoundaryPointTypeInfo> LateralBoundConsInterpolate(
    double s, const BoundPointList& points);
}  // namespace Decision
}  // namespace gpal::pnc::planning
