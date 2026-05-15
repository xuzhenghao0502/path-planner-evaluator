/**
 * @file speed_data.cpp
 * @brief 速度数据容器
 * @details 本类提供速度规划结果的存储与查询接口
 */

#include "speed/speed_data.h"

#include <algorithm>
#include <utility>

#include "math/linear_interpolation.h"
#include "util/string_util.h"
#include "util/util.h"

namespace gpal::pnc::planning {

/**
 * @brief 速度数据容器构造函数（带数据初始化）
 * @param[in] speed_points 速度点集合（所有权转移）
 *
 * @par 处理流程:
 * 1. 通过移动语义接管输入数据所有权
 * 2. 按时间戳升序排序（lambda 比较器使用 SpeedPoint::t()）
 *
 * @note 特性:
 * - 强时序性：强制保证数据按时间顺序存储
 * - 零拷贝：使用移动语义避免vector数据复制
 *
 * @warning 注意:
 * - 输入参数会被清空：调用后原始speed_points将失效
 * - 时间戳要求：输入数据必须包含有效t()值
 */
SpeedData::SpeedData(std::vector<SpeedPoint> speed_points) : std::vector<SpeedPoint>(std::move(speed_points)) {
  std::sort(begin(), end(), [](const SpeedPoint& p1, const SpeedPoint& p2) { return p1.t() < p2.t(); });
}
/**
 * @brief 追加速度点并维护时间有序性
 * @param[in] s     路径累积距离（单位：米）
 * @param[in] time  相对时间戳（单位：秒）
 * @param[in] v     瞬时速度（单位：m/s）
 * @param[in] a     瞬时加速度（单位：m/s²）
 * @param[in] da    瞬时加加速度（单位：m/s³）
 *
 * @par 处理流程:
 * 1. 时间顺序校验：当容器非空时验证时间戳递增
 * 2. 速度点构造：调用 util::MakeSpeedPoint 生成协议格式数据
 * 3. 数据追加：通过 vector::push_back 添加新节点
 *
 * @note 特性:
 * - 强时序保障：通过 CHECK 保证 t(n) > t(n-1)
 * - 全参数构造：支持 s/t/v/a/da 完整运动状态记录
 * - 高效追加：时间复杂度 O(1)
 *
 * @warning 注意:
 * - 中断风险：当 time <= 最后时间戳时触发 CHECK 失败
 * - 单位统一：所有参数需使用国际单位制
 * - 数值范围：da 参数应来自可靠的轨迹规划结果
 */
void SpeedData::AppendSpeedPoint(const double s, const double time, const double v, const double a, const double da) {
  if (!empty()) {
    CHECK(back().t() < time);
  }
  push_back(util::MakeSpeedPoint(s, time, v, a, da));
}
/**
 * @brief 基于相对时间戳的速度点插值查询
 * @param[in] t 相对时间戳（单位：秒，相对于基准时间）
 * @param[out] speed_point 输出速度点指针
 *
 * @par 处理流程:
 * 1. 数据校验：容器大小 ≥2 && 时间在 [t_begin, t_end] 范围内
 * 2. 二分查找：定位目标时间所在区间 [t_prev, t_next]
 * 3. 线性插值：对 s/v/a/da 进行分段线性插值
 * 4. 边界处理：超界时返回最近端点
 *
 * @note 特性:
 * - 安全容错：1e-6 秒的时间校验容差
 * - 条件插值：仅当相邻点都有对应字段时才进行插值
 * - 高效查找：O(log n) 时间复杂度
 *
 * @return 查询状态
 * - true  成功获取有效速度点
 * - false 输入时间越界或数据不足
 *
 * @warning 注意:
 * - 时间基准：t 参数必须基于相同基准时间计算
 * - 数据完整：要求所有 SpeedPoint 包含 s 字段
 * - 内存安全：调用方需保证 speed_point 指针有效
 */
bool SpeedData::EvaluateByTime(const double t, pnc::SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().t() < t + 1.0e-6 && t - 1.0e-6 < back().t())) {
    return false;
  }

  auto comp = [](const SpeedPoint& sp, const double t) { return sp.t() < t; };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.t();
    double t1 = p1.t();

    SpeedPoint res;
    res.set_t(t);

    double s = math::lerp(p0.s(), t0, p1.s(), t1, t);
    res.set_s(s);

    if (p0.has_v() && p1.has_v()) {
      double v = math::lerp(p0.v(), t0, p1.v(), t1, t);
      res.set_v(v);
    }

    if (p0.has_a() && p1.has_a()) {
      double a = math::lerp(p0.a(), t0, p1.a(), t1, t);
      res.set_a(a);
    }

    if (p0.has_da() && p1.has_da()) {
      double da = math::lerp(p0.da(), t0, p1.da(), t1, t);
      res.set_da(da);
    }

    *speed_point = res;
  }
  return true;
}
/**
 * @brief 获取速度数据总时间跨度
 * @return 时间跨度值（单位：秒），空数据时返回0.0
 *
 * @note 特性:
 * - 实时计算：基于首尾元素时间差计算
 * - 空数据保护：自动处理空容器情况
 * - 高效访问：O(1) 时间复杂度
 *
 * @warning 注意:
 * - 非单调风险：当数据未排序时可能返回负值
 * - 精度限制：依赖 SpeedPoint::t() 的存储精度
 */
double SpeedData::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t() - front().t();
}
/// @brief 获取速度数据的调试字符串表示
/// @return 字符串表示
std::string SpeedData::DebugString() const {
  // const auto limit = min(size(), static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  // return util::StrCat("[\n", uto::common::util::PrintDebugStringIter(begin(), begin() + limit, ",\n"),
  //                                  "]\n");
  return "";
}

}  // namespace gpal::pnc::planning
