/**
 * @file path_planner_base.h
 * @brief 路径规划器基类实现文件
 * @details 该文件实现了路径规划器的核心功能，包括路径初始化、路径优化、参考线加载、碰撞检测等。
 */

#pragma once

#include "local_view/local_view.h"
#include "reference_line_info/reference_line_info.h"
#include "decision_data/decision_result.h"
#include "speed/speed_data.h"
#include "path/path_data.h"

namespace gpal::pnc::planning {

/**
 * @brief 路径规划器基类
 */
class PathPlannerBase {
 public:
  /**
   * @brief 默认构造函数
   */
  PathPlannerBase() = default;
  /**
   * @brief 默认析构函数
   */
  ~PathPlannerBase() = default;

  /**
   * @brief 初始化路径规划器
   * @return 返回初始化是否成功
   */
  virtual bool init();

  /**
   * @brief 重置路径规划器
   * @return 返回重置是否成功
   */
  virtual bool reset() = 0;

  /**
   * @brief 单次运行路径规划
   * @param target_reference_line_info 目标参考线信息
   * @param current_reference_line_info 当前参考线信息
   * @param local_view 局部视图
   * @param decision_output 决策结果
   * @param prev_speed_data 前一次速度数据
   * @param stage_state 阶段状态
   * @param time_stamp 时间戳
   * @param path_boundary 路径边界
   * @param path_data 路径数据
   * @return 返回路径规划状态
   */
  virtual PathData::StatusType runOnce(const ReferenceLineInfo& target_reference_line_info,
                                       const ReferenceLineInfo& current_reference_line_info, 
                                       const LocalView& local_view,
                                       const DecisionResult& decision_output, const SpeedData& prev_speed_data,
                                       const int& stage_state,
                                       const int64_t& time_stamp,
                                       PathBoundary* path_boundary,
                                       PathData* path_data) {
    return PathData::StatusType::INVALID;
  };

  static void runRefinePath(const std::shared_ptr<PathData> prev_path_data, const TrajectoryPt& start_pt,
                            PathData* path_data);
  static void runLoadRefLine(const ReferenceLine& reference_line, const TrajectoryPt& start_pt,
                             PathData* path_data);

  PathData::BlockFSInfo collisionCheck(const Freespace& freespace, const std::vector<PathPt>& path,
                                       const double& collision_check_buffer, const double& corner_width,
                                       const bool& enable_curve_decide_process, const double& curve_look_ahead_distance,
                                       const double& curve_decide_kappa_thrd, const double& side_box_length,
                                       const double& side_box_width);
  PathData::BlockFSInfo collisionCheck(const std::vector<math::LineSegment2d>& boundary_segs,
                                       const std::vector<PathPt>& path, const double& collision_check_buffer,
                                       const bool& enable_curve_decide_process, const double& curve_look_ahead_distance,
                                       const double& curve_kappa_thresold, const double& side_box_length,
                                       const double& side_box_width);

  /**
   * @brief 获取时间消耗
   * @return 返回时间消耗
   */
  const double& timeConsumption() const { return time_consumption_; }

  /**
   * @brief 设置时间消耗
   * @param time_consumption 时间消耗
   */
  void setTimeConsumption(const double& time_consumption) { time_consumption_ = time_consumption; }

  /**
   * @brief 获取参考线自车SL信息
   * @return 返回参考线自车SL信息
   */
  const std::pair<std::array<double, 3>, std::array<double, 3>>& getRefLineAdcSlInfo() const { return adc_sl_info_; }

  PathData::BlockPointDirection getBlockFsPointDirection(const PathPt& check_point, const math::Vec2d& collision_point);

 protected:
  void decideInCurve(const std::vector<PathPt>& path, 
                    const double& curve_decide_look_ahead_dis,
                    const double& curve_decide_kappa_thrd,
                    bool& is_left_turn, bool& is_right_turn);
  math::Box2d generateSideCheckBox(const math::Box2d& box, const bool& is_right, 
                                   const double& length, const double& width);
  static void loadKappa(DiscretizedPath* path);

  std::shared_ptr<VehicleConfig> vehicle_config_; ///< 车辆配置参数
  double time_consumption_ = 0.0; ///< 时间消耗
  std::pair<std::array<double, 3>, std::array<double, 3>> adc_sl_info_;  ///< 自车SL信息
};

}
