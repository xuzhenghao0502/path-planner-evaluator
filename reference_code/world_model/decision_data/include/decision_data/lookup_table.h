/**
 * @file lookup_table.h
 * @brief 包含查找表相关的定义和功能，用于快速查找决策所需信息
 * @author 智能驾驶决策规划团队
 * @date 2024-03-01
 * @version 1.0
 * @copyright Copyright (C) 2024 Shanghai Geometrical Perception and Learning Co., Ltd. All rights reserved.
 * @note 模块功能：
 * - 定义查找表结构和相关操作，提高决策过程中的查找效率
 */

#ifndef _PNC_POLICY_MAKER_LOOKUP_TABLE_
#define _PNC_POLICY_MAKER_LOOKUP_TABLE_

#include <algorithm>
#include <iostream>
#include <vector>

namespace gpal::pnc::planning {
namespace Decision {

// 一维查找表类
class LookupTable1D {
 public:
  /**
   * @brief 构造函数，默认初始化一维查找表对象。
   */
  LookupTable1D() = default;

  /**
   * @brief 默认析构函数，用于销毁一维查找表对象。
   */
  ~LookupTable1D() = default;

  void Initialize(const std::vector<double> &input_x, const std::vector<double> &input_y);

  double Lookup(const double target_x) const;

 private:
  /**
   * @brief 线性插值函数，根据给定的 x 值和插值区间计算对应的 y 值。
   * @param[in] x 待插值的 x 值。
   * @param[in] x0 插值区间左端点的 x 值。
   * @param[in] x1 插值区间右端点的 x 值。
   * @param[in] y0 插值区间左端点的 y 值。
   * @param[in] y1 插值区间右端点的 y 值。
   * @par 输入参数说明:
   * - x: 类型 `const double`，取值范围为 [x0, x1]。
   * - x0: 类型 `const double`，取值范围为 (-∞, +∞)。
   * - x1: 类型 `const double`，取值范围为 (-∞, +∞)，且 x1 > x0。
   * - y0: 类型 `const double`，取值范围为 (-∞, +∞)。
   * - y1: 类型 `const double`，取值范围为 (-∞, +∞)。
   * @return double 插值得到的 y 值。
   * @par 处理流程：
   * @startuml
   * start
   * :计算插值比例 (x - x0) / (x1 - x0);
   * :计算插值结果 y0 + (y1 - y0) * 插值比例;
   * :返回插值结果;
   * stop
   * @enduml
   */
  double LinearInterpolation(const double x, const double x0, const double x1, const double y0, const double y1) const {
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
  }

 private:
  bool is_initialized_ = false; ///< @brief 是否初始化成功
  std::vector<double> x_values_;  ///< @brief X轴的值
  std::vector<double> y_values_;  ///< @brief Y轴对应的值
};

/**
 * @class LookupTable2D
 * @brief 二维查找表类，支持双线性插值查询
 * @details 本类用于存储二维网格数据，提供基于双线性插值的快速查询功能。
 */
class LookupTable2D {
public:
  /**
   * @brief 默认构造函数
   * @post 创建未初始化的查找表对象
   */
  LookupTable2D() = default;

  /**
   * @brief 默认析构函数
   */
  ~LookupTable2D() = default;

  void Initialize(const std::vector<double>& x_axis,
                  const std::vector<double>& y_axis,
                  const std::vector<std::vector<double>>& z_table);

  double Lookup(const double xq, const double yq) const;

private:
  bool is_init_ = false; ///< @brief 初始化状态标志
  std::vector<double> x_; ///< @brief X轴坐标序列（严格递增）
  std::vector<double> y_; ///< @brief Y轴坐标序列（严格递增）
  std::vector<std::vector<double>> z_; ///< @brief 二维数据表，z_[i][j]对应x_[i]和y_[j]

  /**
   * @brief 边界索引裁剪函数
   * @param[in] lo 计算得到的下限索引
   * @param[in] hi 计算得到的上限索引
   * @param[in] n 坐标序列总长度
   * @param[out] out_lo 修正后的下限索引（范围：[0, n-1]）
   * @param[out] out_hi 修正后的上限索引（范围：[0, n-1]）
   */
  static void clipIndex(long lo, long hi, long n, long& out_lo, long& out_hi)
  {
    out_lo = std::max(0L, lo);
    out_hi = std::min(n - 1, hi);
  }
};

}  // namespace Decision
}  // namespace gpal::pnc::planning
#endif