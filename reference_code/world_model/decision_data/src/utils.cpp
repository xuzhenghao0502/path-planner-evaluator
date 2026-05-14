/**
 * @file utils.cpp
 * @brief 包含决策数据相关的通用工具函数，如时间转换、角度归一化、随机数生成等
 * @author 智能驾驶决策规划团队
 * @date 2024-03-01
 * @version 1.0
 * @copyright Copyright (C) 2024 Shanghai Geometrical Perception and Learning Co., Ltd. All rights reserved.
 * @note 模块功能：
 * - 将秒和纳秒转换为秒数
 * - 归一化角度到 [-π, π] 范围
 * - 计算两个角度之间的差值
 * - 生成指定范围的随机整数和随机浮点数
 * - 计算高斯分布概率密度值
 * - 确保数值不为零
 */
#include "decision_data/utils.h"

#include <cmath>

namespace gpal::pnc::planning {
namespace Decision {
namespace Utils {

/**
 * @brief 将秒和纳秒转换为秒数
 * @param[in] sec 秒数
 * @param[in] nsec 纳秒数
 * @par 输入参数说明:
 * - sec: 类型 `uint64_t`，取值范围为 [0, +∞)
 * - nsec: 类型 `uint64_t`，取值范围为 [0, 1000000000)
 * @return double 转换后的秒数
 * @par 处理流程：
 * @startuml
 * start
 * :将 sec 转换为 double 类型;
 * :将 nsec 转换为 double 类型并乘以 1.0E-9;
 * :将两者相加;
 * :返回相加结果;
 * stop
 * @enduml
 */
double GetTimeInSec(uint64_t sec, uint64_t nsec) {
  return (static_cast<double>(sec) + static_cast<double>(nsec * 1.0E-9));
}

/**
 * @brief 将角度归一化到 [-π, π] 范围
 * @param[in] angle 输入角度，单位为弧度
 * @par 输入参数说明:
 * - angle: 类型 `const double`，取值范围为 (-∞, +∞)
 * @return double 归一化后的角度，范围为 [-π, π]
 * @par 处理流程：
 * @startuml
 * start
 * :计算 angle + M_PI 对 2.0 * M_PI 取模的结果 a;
 * if (a 小于 0.0) then (是)
 *   :a 加上 2.0 * M_PI;
 * endif
 * :返回 a - M_PI;
 * stop
 * @enduml
 */
double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return (a - M_PI);
}

/**
 * @brief 计算两个角度之间的差值，结果归一化到 [-π, π] 范围
 * @param[in] from 起始角度，单位为弧度
 * @param[in] to 结束角度，单位为弧度
 * @par 输入参数说明:
 * - from: 类型 `const double`，取值范围为 (-∞, +∞)
 * - to: 类型 `const double`，取值范围为 (-∞, +∞)
 * @return double 两个角度之间的差值，范围为 [-π, π]
 * @par 处理流程：
 * @startuml
 * start
 * :计算 to - from 的差值;
 * :调用 NormalizeAngle 函数对差值进行归一化;
 * :返回归一化后的差值;
 * stop
 * @enduml
 */
double AngleDiff(const double from, const double to) { return NormalizeAngle(to - from); }

/**
 * @brief 生成指定范围的随机整数
 * @param[in] s 随机数范围的下限
 * @param[in] t 随机数范围的上限
 * @param[in] rand_seed 随机数种子
 * @par 输入参数说明:
 * - s: 类型 `const int`，取值范围为 (-∞, +∞)
 * - t: 类型 `const int`，取值范围为 (-∞, +∞)
 * - rand_seed: 类型 `unsigned int`，取值范围为 [0, +∞)
 * @return int 生成的随机整数，范围为 [s, t]
 * @par 处理流程：
 * @startuml
 * start
 * if (s 大于等于 t) then (是)
 *   :返回 s;
 *   stop
 * else (否)
 *   :使用 rand_r 函数生成随机数;
 *   :计算随机数对 (t - s + 1) 取模的结果;
 *   :将结果加上 s;
 *   :返回计算结果;
 *   stop
 * endif
 * @enduml
 */
int RandomInt(const int s, const int t, unsigned int rand_seed) {
  if (s >= t) {
    return s;
  }
  return s + rand_r(&rand_seed) % (t - s + 1);
}

/**
 * @brief 生成指定范围的随机浮点数
 * @param[in] s 随机数范围的下限
 * @param[in] t 随机数范围的上限
 * @param[in] rand_seed 随机数种子
 * @par 输入参数说明:
 * - s: 类型 `const double`，取值范围为 (-∞, +∞)
 * - t: 类型 `const double`，取值范围为 (-∞, +∞)
 * - rand_seed: 类型 `unsigned int`，取值范围为 [0, +∞)
 * @return double 生成的随机浮点数，范围为 [s, t)
 * @par 处理流程：
 * @startuml
 * start
 * :使用 rand_r 函数生成随机数并与 16383 按位与;
 * :计算 (t - s) / 16383.0 乘以按位与结果;
 * :将结果加上 s;
 * :返回计算结果;
 * stop
 * @enduml
 */

double RandomDouble(const double s, const double t, unsigned int rand_seed) {
  return s + (t - s) / 16383.0 * (rand_r(&rand_seed) & 16383);
}

/**
 * @brief 计算高斯分布的概率密度值
 * @param[in] u 高斯分布的均值
 * @param[in] std 高斯分布的标准差
 * @param[in] x 要计算概率密度的值
 * @par 输入参数说明:
 * - u: 类型 `const double`，取值范围为 (-∞, +∞)
 * - std: 类型 `const double`，取值范围为 (0, +∞)
 * - x: 类型 `const double`，取值范围为 (-∞, +∞)
 * @return double 高斯分布在 x 处的概率密度值
 * @par 处理流程：
 * @startuml
 * start
 * :计算 1.0 / sqrt(2 * M_PI * std * std);
 * :计算 -(x - u) * (x - u) / (2 * std * std);
 * :计算指数函数 exp 结果;
 * :将两部分结果相乘;
 * :返回相乘结果;
 * stop
 * @enduml
 */
// Gaussian
double Gaussian(const double u, const double std, const double x) {
  return (1.0 / std::sqrt(2 * M_PI * std * std)) * std::exp(-(x - u) * (x - u) / (2 * std * std));
}

}  // namespace Utils
}  // namespace Decision
}  // namespace gpal::pnc::planning