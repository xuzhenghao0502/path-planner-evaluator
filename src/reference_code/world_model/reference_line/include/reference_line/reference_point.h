/**
 * @file reference_point.h
 * @brief 参考点实现文件
 * @details 该文件实现了ReferencePoint类，用于表示参考线中的参考点信息。
 *          主要功能包括：初始化参考点、设置驾驶管道缓冲区、将参考点转换为路径点、
 *          横向和纵向偏移参考点、生成调试信息等。
 */

#pragma once

#include <string>
#include <vector>

#include "gpal-interface/planning/trajectory.pb.h"
#include "math/vec2d.h"
#include "proto/common/pnc_point.pb.h"

namespace gpal::pnc::planning {

/**
 * @brief ReferencePoint类
 * @details 该类用于表示参考点，继承自math::Vec3d，提供了参考点的各种属性和操作函数
 */
class ReferencePoint : public math::Vec3d {
 public:
  /**
   * @brief 默认构造函数
   * @details 创建一个空的ReferencePoint对象
   */
  ReferencePoint() = default;
  ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa, const double dkappa);
  ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa, const double dkappa,
                 const double left_bound, const double right_bound, const double road_left_bound,
                 const double road_right_bound);
  ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa, const double dkappa,
                 const double offset);
  ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa, const double dkappa,
                 const double left_bound, const double right_bound, const double road_left_bound,
                 const double road_right_bound, const double offset);
  ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa, const double dkappa,
                 const double offset, const double local_s);
  ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa, const double dkappa,
                 const double left_bound, const double right_bound, const double road_left_bound,
                 const double road_right_bound, const double offset, const double local_s);

  /**
   * @brief 获取航向角
   * @return 当前航向角值
   */
  double heading() const { return heading_; }
  /**
   * @brief 设置航向角
   * @param heading 要设置的航向角值
   */
  void setHeading(double heading) { heading_ = heading; }

  /**
   * @brief 获取坡度
   * @return 当前坡度值
   */
  double slope() const { return slope_; }
  /**
   * @brief 设置坡度
   * @param slope 要设置的坡度值
   */
  void setSlope(double slope) { slope_ = slope; }

  /**
   * @brief 获取曲率
   * @return 当前曲率值，单位：1/m
   */
  double kappa() const { return kappa_; }
  /**
   * @brief 设置曲率
   * @param kappa 要设置的曲率值，单位：1/m
   */
  void setKappa(const double kappa) { kappa_ = kappa; }

  /**
   * @brief 获取曲率变化率
   * @return 当前曲率变化率值，单位：1/m²
   */
  double dkappa() const { return dkappa_; }
  /**
   * @brief 设置曲率变化率
   * @param dkappa 要设置的曲率变化率值，单位：1/m²
   */
  void setDkappa(const double dkappa) { dkappa_ = dkappa; }

  /**
   * @brief 获取横向偏移量
   * @return 当前横向偏移量值，单位：米
   */
  double offset() const { return offset_; }
  /**
   * @brief 设置横向偏移量
   * @param offset 要设置的横向偏移量值，单位：米
   */
  void setOffset(const double offset) { offset_ = offset; }

  /**
   * @brief 获取局部路径长度
   * @return 当前局部路径长度值，单位：米
   */
  double local_s() const { return local_s_; }
  /**
   * @brief 设置参考点在参考线上的s
   * @param local_s 要设置的参考点在参考线上的s，单位：米
   */
  void setLocalS(const double local_s) { local_s_ = local_s; }

  /**
   * @brief 获取左侧边界距离
   * @return 当前左侧边界距离值，单位：米
   */
  double left_bound() const { return left_bound_; }
  /**
   * @brief 设置左侧边界距离
   * @param left_bound 要设置的左侧边界距离值，单位：米
   */
  void setLeftBound(double left_bound) { left_bound_ = left_bound; }

  /**
   * @brief 获取右侧边界距离
   * @return 当前右侧边界距离值，单位：米
   */
  double right_bound() const { return right_bound_; }
  /**
   * @brief 设置右侧边界距离
   * @param right_bound 要设置的右侧边界距离值，单位：米
   */
  void setRightBound(double right_bound) { right_bound_ = right_bound; }

  /**
   * @brief 获取道路左侧边界距离
   * @return 当前道路左侧边界距离值，单位：米
   */
  double road_left_bound() const { return road_left_bound_; }
  /**
   * @brief 设置道路左侧边界距离
   * @param road_left_bound 要设置的道路左侧边界距离值，单位：米
   */
  void setRoadLeftBound(double road_left_bound) { road_left_bound_ = road_left_bound; }

  /**
   * @brief 获取道路右侧边界距离
   * @return 当前道路右侧边界距离值，单位：米
   */
  double road_right_bound() const { return road_right_bound_; }
  /**
   * @brief 设置道路右侧边界距离
   * @param road_right_bound 要设置的道路右侧边界距离值，单位：米
   */
  void setRoadRightBound(double road_right_bound) { road_right_bound_ = road_right_bound; }

  /**
   * @brief 获取行驶空间左侧缓冲区
   * @return 当前行驶空间左侧缓冲区值，单位：米
   */
  double driving_tube_left_buffer() const { return driving_tube_left_buffer_; }
  /**
   * @brief 获取行驶空间右侧缓冲区
   * @return 当前行驶空间右侧缓冲区值，单位：米
   */
  double driving_tube_right_buffer() const { return driving_tube_right_buffer_; }
  void setDrivingTubeBuffer(const double left, const double right);

  gpal::proto::PathPoint ToPathPoint(double s) const;
  ReferencePoint lateralShift(const double l) const;
  ReferencePoint longitudinalShift(const double s) const;

  std::string DebugString() const;

 private:
  double heading_ = 0.0;                    ///< 航向角，单位：弧度
  float slope_ = 0.0;                       ///< 坡度，单位：百分比
  double kappa_ = 0.0;                      ///< 曲率，单位：1/m
  double dkappa_ = 0.0;                     ///< 曲率变化率，单位：1/m²
  double offset_ = 0.0;                     ///< 横向偏移量，单位：米
  double local_s_ = 0.0;                    ///< 局部路径长度，单位：米
  double left_bound_ = 0.0;                 ///< 左侧边界距离，单位：米
  double right_bound_ = 0.0;                ///< 右侧边界距离，单位：米
  double road_left_bound_ = 0.0;            ///< 道路左侧边界距离，单位：米
  double road_right_bound_ = 0.0;           ///< 道路右侧边界距离，单位：米
  double driving_tube_left_buffer_ = 0.0;   ///< 行驶空间左侧缓冲区，单位：米
  double driving_tube_right_buffer_ = 0.0;  ///< 行驶空间右侧缓冲区，单位：米
};

/**
 * @brief InterpolatedIndex类
 */
class InterpolatedIndex {
 public:
  /**
   * @brief 构造函数
   * @param id 索引ID
   * @param offset 偏移量
   */
  InterpolatedIndex(int id, double offset) : id(id), offset(offset) {}
  int id = 0;           ///< 索引ID
  double offset = 0.0;  ///< 偏移量，单位：米
};

}  // namespace gpal::pnc::planning
