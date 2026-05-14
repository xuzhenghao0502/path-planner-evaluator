/**
 * @file speed_data.h
 * @brief 速度数据容器
 * @details 本类提供速度规划结果的存储与查询接口
 */
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "proto/common/pnc_point.pb.h"

namespace gpal::pnc::planning {

struct SpatialSpeedLimit {
  /// @brief 默认构造函数
  SpatialSpeedLimit() = default;
  /**  @brief 构造函数
   * @param[in] s 路径累积距离（相对于参考线起点）
   * @param[in] speed_limit 速度限制值
   * @param[in] id 限制标识符（用于区分限制来源）
   */
  SpatialSpeedLimit(const double& s, const double& speed_limit, const std::string& id)
      : s(s), speed_limit(speed_limit), id(id) {}
  /// @brief 路径累积距离（单位：米）
  double s = 0.0;
  /// @brief 限速（单位：米/秒）
  double speed_limit = 0.0;
  /// @brief 限速类型ID
  std::string id = "map";
};

class SpeedData : public std::vector<pnc::SpeedPoint> {
 public:
  /**
   * @brief 默认构造函数
   */
  SpeedData() = default;
  /**
   * @brief 析构函数
   */
  virtual ~SpeedData() = default;

  explicit SpeedData(std::vector<pnc::SpeedPoint> speed_points);

  void AppendSpeedPoint(const double s, const double time, const double v, const double a, const double da);

  bool EvaluateByTime(const double time, pnc::SpeedPoint* const speed_point) const;

  /**
   * @brief 基于绝对时间戳的速度点查询
   * @tparam _Rep 时间计数类型（chrono duration 的表示类型）
   * @tparam _Period 时间单位类型（chrono duration 的周期单位）
   * @param[in] duration 绝对时间戳（基于chrono的时间间隔表示）
   * @param[out] speed_point 速度点输出容器
   *
   * @par 输入/输出参数说明:
   * | 参数          | 类型                                     | 取值范围        | 说明                     |
   * |---------------|------------------------------------------|----------------|--------------------------|
   * | duration      | std::chrono::duration<_Rep, _Period>&   | >=0            | 绝对时间戳输入           |
   * | speed_point   | pnc::SpeedPoint*                        | 非空指针        | 速度点输出容器           |
   *
   * @par 处理流程:
   * @startuml
   :将输入时间转换为微秒精度;
   :计算相对于基准时间的时间差;
   :调用EvaluateByTime进行速度点查询;
   @enduml
   *
   * @note 功能特性:
   * - 时间精度：内部使用微秒级时间戳计算
   * - 时间基准：依赖setStamp设置的基准时间点
   * - 单位安全：自动处理chrono不同时间单位的转换
   *
   * @return 查询结果状态
   * - true  查询成功
   * - false 查询失败（时间超出范围）
   *
   * @warning 实现差异:
   * - 时间同步：需确保基准时间(stamp_)已正确设置
   * - 精度限制：微秒精度在连续运行30天后可能产生累积误差
   */
  template <typename _Rep, typename _Period>
  bool EvaluateByAbsoluteTime(const std::chrono::duration<_Rep, _Period>& duration,
                              pnc::SpeedPoint* const speed_point) const {
    auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(duration);
    const double time = std::chrono::duration<double>(stamp - stamp_).count();
    return EvaluateByTime(time, speed_point);
  }

  double TotalTime() const;
  /**
   * @brief 设置基准时间戳
   * @tparam _Rep 时间计数类型（chrono duration 的表示类型）
   * @tparam _Period 时间单位类型（chrono duration 的周期单位）
   * @param[in] duration 基准时间戳（基于chrono的时间间隔表示）
   */
  template <typename _Rep, typename _Period>
  void setStamp(const std::chrono::duration<_Rep, _Period>& duration) {
    stamp_ = std::chrono::duration_cast<std::chrono::microseconds>(duration);
  };

  virtual std::string DebugString() const;

 protected:
  /**
   * @brief 基准时间戳（微秒精度）
   */
  std::chrono::microseconds stamp_{0};
};

}  // namespace gpal::pnc::planning
