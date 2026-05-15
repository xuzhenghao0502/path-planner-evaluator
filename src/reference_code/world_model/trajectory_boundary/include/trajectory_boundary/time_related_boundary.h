#pragma once

#include <set>

#include "trajectory_boundary/base_boundary.h"
namespace gpal::pnc::planning {
/**
 * @brief 时间相关边界
 * @details 用于管理时间相关的参考点边界
 */
class TimeRelatedBoundary : public BoundaryManagerBase<double, TimeRelatedBoundary> {
 public:
  using BoundaryManagerBase<double, TimeRelatedBoundary>::BoundaryManagerBase;
  TimeRelatedBoundary() = default;

  // ==================== 第一层：决策速度墙边界接口 ====================

  /**
   * @brief 设置决策速度墙边界
   */
  void setWallConstraintsBound(const math::IntervalData<Boundary>& wall_bound) {
    s_wall_constraints_bound_ = wall_bound;
  }

  /**
   * @brief 获取决策速度墙边界
   */
  const math::IntervalData<Boundary>& getWallConstraintsBound() const { return s_wall_constraints_bound_; }

  /**
   * @brief 获取可修改的决策速度墙边界
   */
  math::IntervalData<Boundary>& mutableWallConstraintsBound() { return s_wall_constraints_bound_; }

  /**
   * @brief 更新决策速度墙边界
   */
  void updateWallConstraintsBound(const std::vector<Boundary>& wall_constraints);
  void updateWallConstraintsBound(std::vector<Boundary>&& wall_constraints);

  // ==================== 第二层：静态障碍物接口 ====================

  /**
   * @brief 设置静态障碍物边界
   */
  void setStaticObjBound(const math::IntervalData<Boundary>& static_bound) { s_static_obj_bound_ = static_bound; }

  /**
   * @brief 获取静态障碍物边界
   */
  const math::IntervalData<Boundary>& getStaticObjBound() const { return s_static_obj_bound_; }

  math::IntervalData<Boundary>& mutableStaticObjBound() { return s_static_obj_bound_; }

  /**
   * @brief 更新静态障碍物边界
   */
  void updateStaticObjBound(const std::vector<Boundary>& static_bound);
  void updateStaticObjBound(std::vector<Boundary>&& static_bound);
  /**
   * @brief 获取静态障碍物ID集合
   */
  const std::set<std::string>& getStaticObjIds() const { return static_obj_ids_; }
  void insertStaticObjId(std::string obj_id) { static_obj_ids_.insert(obj_id); }
  /**
   * @brief 检查是否包含特定静态障碍物
   */
  bool hasStaticObj(const std::string& obj_id) const { return static_obj_ids_.count(obj_id) > 0; }

  // ==================== 第三层：动态障碍物接口 ====================

  /**
   * @brief 设置动态障碍物边界
   */
  void setDynamicObjBound(const math::IntervalData<Boundary>& dynamic_bound) { s_dynamic_obj_bound_ = dynamic_bound; }

  /**
   * @brief 获取动态障碍物边界
   */
  const math::IntervalData<Boundary>& getDynamicObjBound() const { return s_dynamic_obj_bound_; }
  math::IntervalData<Boundary>& mutableDynamicObjBound() { return s_dynamic_obj_bound_; }

  /**
   * @brief 更新动态障碍物边界
   */
  void updateDynamicObjBound(const std::vector<Boundary>& dynamic_bound);
  void updateDynamicObjBound(std::vector<Boundary>&& dynamic_bound);

  /**
   * @brief 获取动态障碍物ID集合
   */
  const std::set<std::string>& getDynamicObjIds() const { return dynamic_obj_ids_; }
  void insertDynamicObjId(std::string obj_id) { dynamic_obj_ids_.insert(obj_id); }

  /**
   * @brief 检查是否包含特定动态障碍物
   */
  bool hasDynamicObj(const std::string& obj_id) const { return dynamic_obj_ids_.count(obj_id) > 0; }

  // ==================== 综合静态边界接口 ====================

  const math::IntervalData<Boundary>& sStaticSoftBound() const { return s_static_soft_bound_; }
  math::IntervalData<Boundary>& mutableSStaticSoftBound() { return s_static_soft_bound_; }
  void setSStaticSoftBound(const math::IntervalData<Boundary>& s_static_soft_bound) {
    s_static_soft_bound_ = s_static_soft_bound;
  }
  void updateSStaticSoftBound(const std::vector<Boundary>& s_constraints);
  void updateSStaticSoftBound(std::vector<Boundary>&& s_constraints);

  const math::IntervalData<Boundary>& sStaticHardBound() const { return s_static_hard_bound_; }
  math::IntervalData<Boundary>& mutableSStaticHardBound() { return s_static_hard_bound_; }
  void setSStaticHardBound(const math::IntervalData<Boundary>& s_static_hard_bound) {
    s_static_hard_bound_ = s_static_hard_bound;
  }
  void updateSStaticHardBound(const std::vector<Boundary>& s_constraints);
  void updateSStaticHardBound(std::vector<Boundary>&& s_constraints);

  // ==================== 综合边界接口 ====================
  const math::IntervalData<Boundary>& sSoftBound() const { return s_soft_bound_; }
  math::IntervalData<Boundary>& mutableSSoftBound() { return s_soft_bound_; }
  void setSSoftBound(const math::IntervalData<Boundary>& s_soft_bound) { s_soft_bound_ = s_soft_bound; }
  void updateSSoftBound(const std::vector<Boundary>& s_constraints);
  void updateSSoftBound(std::vector<Boundary>&& s_constraints);

  const math::IntervalData<Boundary>& sHardBound() const { return s_hard_bound_; }
  math::IntervalData<Boundary>& mutableSHardBound() { return s_hard_bound_; }
  void setSHardBound(const math::IntervalData<Boundary>& s_hard_bound) { s_hard_bound_ = s_hard_bound; }
  void updateSHardBound(const std::vector<Boundary>& s_constraints);
  void updateSHardBound(std::vector<Boundary>&& s_constraints);

 private:
  friend class BoundaryManagerBase<double, TimeRelatedBoundary>;
  // --- 实现基类定义的接口 ---
  void initializeImpl(const std::vector<double>& indices);
  void createInterpolators();
  void resetImpl();

 private:
  // --- 边界数据 ---
  // 第一层：决策速度墙
  math::IntervalData<Boundary> s_wall_constraints_bound_;

  // 第二层：静态障碍物
  math::IntervalData<Boundary> s_static_obj_bound_;
  std::set<std::string> static_obj_ids_;

  // 第三层：动态障碍物
  math::IntervalData<Boundary> s_dynamic_obj_bound_;
  std::set<std::string> dynamic_obj_ids_;

  // 综合静态边界： 决策速度墙 + 静态障碍物
  math::IntervalData<Boundary> s_static_soft_bound_;
  math::IntervalData<Boundary> s_static_hard_bound_;

  // 综合边界： 决策速度墙 + 静态障碍物 + 动态障碍物
  math::IntervalData<Boundary> s_soft_bound_;
  math::IntervalData<Boundary> s_hard_bound_;
};

}  // namespace gpal::pnc::planning