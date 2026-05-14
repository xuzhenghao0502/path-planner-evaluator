/**
 * @file lookup_table.cpp
 * @brief 包含一维和二维查找表的初始化和查找功能实现
 * @author 智能驾驶决策规划团队
 * @date 2024-03-01
 * @version 1.0
 * @copyright Copyright (C) 2024 Shanghai Geometrical Perception and Learning Co., Ltd. All rights reserved.
 * @note 模块功能：
 * - 初始化一维和二维查找表
 * - 使用线性插值在一维查找表中查找目标值
 * - 使用双线性插值在二维查找表中查找目标值
 */

#include "decision_data/lookup_table.h"
namespace gpal::pnc::planning {
namespace Decision {

/**
 * @brief 初始化一维查找表数据结构
 * @param[in] input_x 输入的X轴坐标向量
 * @param[in] input_y 输入的Y轴对应值向量
 * 
 * @par 输入参数说明:
 * - input_x: 类型 `const std::vector<double>&`，必须满足：
 *   - 非空向量
 *   - 元素按升序排列
 *   - 大小与input_y一致
 *   - 取值范围: (-∞, +∞)
 * - input_y: 类型 `const std::vector<double>&`，必须满足：
 *   - 非空向量
 *   - 大小与input_x一致
 *   - 取值范围: (-∞, +∞)
 *
 * @par 处理流程：
 * @startuml
 * start
 * if (input_x为空 || input_y为空) then (是)
 *   :设置is_initialized_=false;
 *   stop
 * else if (input_x大小 != input_y大小) then (是)
 *   :设置is_initialized_=false;
 *   stop
 * else (否)
 *   :深拷贝input_x到x_values_;
 *   :深拷贝input_y到y_values_;
 *   :设置is_initialized_=true;
 *   stop
 * endif
 * @enduml
 */
void LookupTable1D::Initialize(const std::vector<double> &input_x, const std::vector<double> &input_y) {
  // 检查输入向量长度是否一致
  if ((input_x.size() != input_y.size()) || input_x.empty()) {
    is_initialized_ = false;
    return;
  }

  // 复制数据以确保内部数据一致性
  x_values_ = input_x;
  y_values_ = input_y;

  is_initialized_ = true;
}

/**
 * @brief 在一维查找表中执行插值查询
 * @param[in] target_x 需要查询的X坐标值
 * 
 * @par 输入参数说明:
 * - target_x: 类型 `const double`，取值范围为:
 *   - (-∞, +∞) 任意实数
 *   - 实际有效范围 [x_values_.front(), x_values_.back()]
 *
 * @return double 插值计算结果，返回条件：
 * - 未初始化时返回0.0
 * - target_x小于最小值时返回y_values_.front()
 * - target_x大于最大值时返回y_values_.back()
 * - 区间内时返回线性插值结果
 *
 * @par 处理流程：
 * @startuml
 * start
 * if (is_initialized_ == false) then (是)
 *   :返回0.0;
 *   stop
 * else (否)
 *   :使用std::lower_bound查找target_x;
 *   if (target_x < x_values_.front()) then (是)
 *     :返回y_values_.front();
 *   else if (target_x > x_values_.back()) then (是)
 *     :返回y_values_.back();
 *   else (区间内)
 *     :计算indexBelow和indexAbove;
 *     :执行LinearInterpolation;
 *     :返回插值结果;
 *   endif
 * endif
 * stop
 * @enduml
 */
double LookupTable1D::Lookup(const double target_x) const {
  if (!is_initialized_) {
    return 0.0;
  }
  // 二分查找目标X值在x_values_中的位置
  auto lower = std::lower_bound(x_values_.begin(), x_values_.end(), target_x);

  // 如果目标X小于最小的X值或大于最大的X值，返回边界值
  if (lower == x_values_.begin()) {
    return y_values_.front();
  } else if (lower == x_values_.end()) {
    return y_values_.back();
  }

  // 获取目标X值两边的索引
  size_t indexBelow = std::distance(x_values_.begin(), lower) - 1;
  size_t indexAbove = indexBelow + 1;

  // 使用线性插值计算目标Y值
  return LinearInterpolation(target_x, x_values_[indexBelow], x_values_[indexAbove], y_values_[indexBelow],
                             y_values_[indexAbove]);
}

/**
 * @brief 初始化二维查找表
 * @param[in] x_axis X轴坐标序列，要求：
 * - 类型：const std::vector<double>&
 * - 大小 ≥2
 * - 元素严格递增
 * @param[in] y_axis Y轴坐标序列，要求：
 * - 类型：const std::vector<double>&
 * - 大小 ≥2
 * - 元素严格递增
 * @param[in] z_table 二维数据表，要求：
 * - 类型：const std::vector<std::vector<double>>&
 * - 外层vector大小 == x_axis.size()
 * - 内层vector大小 == y_axis.size()
 * @par 初始化流程图：
 * @startuml
 * start
 * :检查输入维度;
 * if (x_axis.size < 2) then (无效)
 *   :标记初始化失败;
 *   stop;
 * endif
 * if (y_axis.size < 2) then (无效)
 *   :标记初始化失败;
 *   stop;
 * endif
 * if (z_table维度不匹配) then (无效)
 *   :标记初始化失败;
 *   stop;
 * endif
 * :深拷贝输入数据;
 * :标记初始化成功;
 * stop
 * @enduml
 */
void LookupTable2D::Initialize(const std::vector<double>& x_axis,
                               const std::vector<double>& y_axis,
                               const std::vector<std::vector<double>>& z_table)
{
    if (x_axis.empty() || y_axis.empty()) {
        is_init_ = false;
        return;
    }
    if (!std::is_sorted(x_axis.begin(), x_axis.end())) {
        is_init_ = false;
        return;
    }
    if (!std::is_sorted(y_axis.begin(), y_axis.end())) {
        is_init_ = false;
        return;
    }
    if ((z_table.size() != y_axis.size()) ||
        std::any_of(z_table.begin(), z_table.end(),
                    [&](const auto& row) { return row.size() != x_axis.size(); })) {
        is_init_ = false;
        return;
    }
    x_ = x_axis;
    y_ = y_axis;
    z_ = z_table;
    is_init_ = true;
}

/**
 * @brief 执行二维查找操作
 * @param[in] xq 查询点X坐标，实际有效范围：[x_axis.front(), x_axis.back()]
 * @param[in] yq 查询点Y坐标，实际有效范围：[y_axis.front(), y_axis.back()]
 * @return double 插值结果，当未初始化时返回0.0
 * @par 查询流程图：
 * @startuml
 * start
 * if (未初始化) then (是)
 *   :返回0.0;
 *   stop;
 * endif
 * :查找xq相邻索引;
 * :查找yq相邻索引;
 * :执行双线性插值;
 * :返回插值结果;
 * stop
 * @enduml
 */
double LookupTable2D::Lookup(const double xq, const double yq) const
{   
    if (!is_init_) {
        return 0.0;
    }
    // 1. x 方向
    auto ix = static_cast<long>(std::lower_bound(x_.begin(), x_.end(), xq) - x_.begin());
    long x0;
    long x1;
    clipIndex((ix - 1), ix, static_cast<long>(x_.size()), x0, x1);
    if (x0 == x1) {
      x1 = std::min((x0 + 1), static_cast<long>(x_.size()) - 1);
    }
    // 2. y 方向
    auto iy = static_cast<long>(std::lower_bound(y_.begin(), y_.end(), yq) - y_.begin());
    long y0;
    long y1;
    clipIndex((iy - 1), iy, static_cast<long>(y_.size()), y0, y1);
    if (y0 == y1) {
      y1 = std::min((y0 + 1), static_cast<long>(y_.size()) - 1);
    }
    // 3. 四个角点
    double x0v = x_[x0];
    double x1v = x_[x1];
    double y0v = y_[y0];
    double y1v = y_[y1];
    double z00 = z_[y0][x0];
    double z01 = z_[y0][x1];
    double z10 = z_[y1][x0];
    double z11 = z_[y1][x1];
    // 4. 双线性插值
    double tx = (x1v == x0v) ? 0.0 : ((xq - x0v) / (x1v - x0v));
    double ty = (y1v == y0v) ? 0.0 : ((yq - y0v) / (y1v - y0v));
    return ((1 - tx) * (1 - ty) * z00 + tx * (1 - ty) * z01 +
           (1 - tx) * ty * z10 + tx * ty * z11);
}


}  // namespace Decision
}  // namespace gpal::pnc::planning