/**
 * @file utils.h
 * @brief 包含决策数据处理的实用工具函数，辅助决策逻辑的实现
 * @author 智能驾驶决策规划团队
 * @date 2024-03-01
 * @version 1.0
 * @copyright Copyright (C) 2024 Shanghai Geometrical Perception and Learning Co., Ltd. All rights reserved.
 * @note 模块功能：
 * - 提供时间计算、角度处理、随机数生成等实用工具函数
 */

#ifndef _PNC_POLICY_MAKER_UTILS_
#define _PNC_POLICY_MAKER_UTILS_

#include <chrono>
#include <cmath>
#include <utility>
namespace gpal::pnc::planning {
namespace Decision {
namespace Utils {

double GetTimeInSec(uint64_t sec, uint64_t nsec);

double NormalizeAngle(const double angle);

double AngleDiff(const double from, const double to);

int RandomInt(const int s, const int t, unsigned int rand_seed);

double RandomDouble(const double s, const double t, unsigned int rand_seed);

double Gaussian(const double u, const double std, const double x);

}  // namespace Utils
}  // namespace Decision
}  // namespace gpal::pnc::planning
#endif