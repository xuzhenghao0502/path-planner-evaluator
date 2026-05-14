/**
 * @copyright Copyright(C): Geometrical-Pal, 2024-2026, All Rights Reserved
 *
 * @file    base_parking_planner.h
 * @brief   The base class for all parking planners, providing common interfaces and utilities
 *          for parking maneuvers including parking-in, parking-out, and escape scenarios..
 * @details This abstract class inherits from PathPlannerBase and defines the essential
 *          parking planning workflow for various scenarios:
 *          1. Parking-in: Planning trajectory to enter a parking spot
 *          2. Parking-out: Planning trajectory to exit a parking spot
 *          3. Escape: Planning recovery trajectory from stuck or complex situations
 *
 *          The class provides:
 *          - Common preprocessing for parking slot detection data
 *          - Collision-free trajectory generation via search algorithms (e.g., Hybrid A*)
 *          - Trajectory optimization for smoothness and kinematic feasibility
 *          - Unified interfaces for different parking scenarios
 *
 *          Derived classes must implement parking-type-specific logic (e.g. different
 *          geometric constraints for vertical/parallel parking and escape maneuvers).
 *
 * @author  WeiTang(weitang@geometricalpal.com)
 * @version 3.0
 * @date    2025-12-05
 */

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <path/path_data.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "local_view/local_view.h"
#include "openspace_path_planner/adapter/parking_profile_manager.h"
#include "openspace_path_planner/core/generator/geometry_path_generator.h"
#include "openspace_path_planner/core/generator/openspace_roi_decider.h"
#include "openspace_path_planner/core/manager/openspace_core_manager.h"
#include "openspace_path_planner/utils/geometry_utils.h"
#include "openspace_path_planner/utils/openspace_common.h"
#include "openspace_path_planner/utils/openspace_tools.h"
#include "openspace_path_planner/utils/slot.h"
#include "path_planner/path_planner_base.h"
#include "speed/speed_result.h"

using std::pair;
using std::queue;
using std::vector;

namespace gpal::pnc::planning {

struct SlotParam {
  Slot slot;
  PathPt destination_point;  // 泊车停靠终点

  SlotParam() = default;
  SlotParam(const std::vector<PathPt>& _slot_corners, const Slot::SlotType& _slot_type = Slot::SlotType::VERTICAL,
            const Slot::SlotDirec& _slot_direc = Slot::SlotDirec::LEFT, const std::string& _slot_id = "")
      : slot(Slot(_slot_corners, _slot_type, _slot_direc, _slot_id)) {
    destination_point = slot.bottomEdgeCenter();
  }
};

struct SearchElements {
  PathPt start_pose;
  PathPt end_pose;
  std::vector<double> search_boundary;

  bool is_region_search = false;
  math::Box2d goal_region;
  double goal_region_heading = 0.0;
  double goal_region_heading_tolerance = 0.0;

  SearchElements(const PathPt& _start_pose = PathPt(), const PathPt& _end_pose = PathPt(),
                 const std::vector<double>& _search_boundary = {-1e3, 1e3, -1e3, 1e3})
      : start_pose(_start_pose), end_pose(_end_pose), search_boundary(_search_boundary) {}
};

using ParkingMode = ParkingPathPlannerProfile::ParkingMode;
using ParkingType = ParkingPathPlannerConfig::ParkingType;
using ParkingPlannerKey = std::pair<ParkingMode, ParkingType>;
class BaseParkingPlanner : public PathPlannerBase {
 protected:
#define PARKING_LOG(level, ...) OPENSPACE_LOG(level, "[", name_, "] ", __VA_ARGS__)

 public:
  enum class CalStatus : uint8_t { DEFAULT = 0, SEARCH, GEO_CONNECT, OPTIMIZE, POST_PROCESS, FINISH };
  enum class UnitStatus : uint8_t { INVALID = 0, WAITING, FAILED, SUCCEEDED };
  enum class PlannerStatus : uint8_t { NORMAL = 0, REPLAN, FAILED, FINISHED, HOLD_ON };
  enum class ParkEnvType : uint8_t { INVALID = 0, GENERAL, DEAD_END };

 public:
  BaseParkingPlanner() = default;
  explicit BaseParkingPlanner(const ParkingPlannerKey& key) : parking_planner_key_(key) {}
  ~BaseParkingPlanner() = default;

  bool init() override;
  bool reset() override;
  void clear();

  /**
   * @brief 主函数，求解泊车轨迹
   */
  PathData::StatusType getPath(const ReferenceLineInfo& target_reference_line_info,
                               const DiscretizedPath& driving_discretized_path, const LocalView& local_view,
                               const DecisionResult& decision_result, const bool destination_stop_flag,
                               const StopReason& stop_reason);
  /**
   * @brief 接口函数，获取轨迹剩余距离
   */
  double getRemainDis() { return remain_dis_; };

  /**
   * @brief 接口函数，获取泊车计算debug信息
   */
  std::string getDebugInfo() { return debug_info_; };

  /**
   * @brief 接口函数，获取泊车当前轨迹
   */
  DiscretizedPath getCurrentPath() { return current_flu_path_; };

  /**
   * @brief 接口函数，获取泊车完整轨迹
   */
  std::vector<DiscretizedPath> getFullPath() { return realtime_optimizer_segmented_flu_paths_; };

  /**
   * @brief 接口函数，反馈当前剩余轨迹段数目（含当前轨迹）
   */
  int getRemainTrajNum() { return realtime_optimizer_segmented_flu_paths_.size(); };

  /**
   * @brief 接口函数，获取泊车计算的边界信息
   */
  std::vector<PathData::BoundsVec3dWithId> getFullOptimizerBoundary() { return optimizer_boundary_vec_; };

  /**
   * @brief 接口函数，获取修正后的库位角点
   */
  std::pair<bool, std::vector<PathPt>> getCorrectedSlot() { return corrected_slot_; };

  /**
   * @brief 接口函数，获取不停车规划的剩余距离
   */
  double getNonStopPlanningDist() const { return non_stop_planning_dist_; }

 public:
  /**
   * @brief 对上游信息的预处理
   */
  virtual void slotProcess(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                           const std::vector<math::LineSegment2d>& boundary_seg) {
    PARKING_LOG(D, "default slotProcess");
  };

  /**
   * @brief 根据预处理参量，生成搜索要素（搜索起点、终点），传入roi模块
   */
  virtual bool generateSearchElements(const LocalView& local_view,
                                      const std::vector<Decision::DecisionObject>& select_ods,
                                      const std::vector<math::LineSegment2d>& boundary_seg,
                                      SearchElements& search_elements) {
    PARKING_LOG(D, "default generateSearchElements");
    return false;
  };

  /**
   * @brief 自定义约束边界
   */
  virtual void customBoundary(const LocalView& local_view, std::vector<Decision::DecisionObject>& select_ods) {
    PARKING_LOG(D, "default customBoundary");
  };

  /**
   * @brief 对泊车搜索结果的几何拼接策略
   */
  virtual void geometricConnect(const LocalView& local_view, const RoiDecideResult& roi) {
    PARKING_LOG(D, "default geometricConnect");
  };

  /**
   * @brief 对泊车优化结果进行后处理，如在换挡处增加直线过渡
   */
  virtual void trajPostProcess(const LocalView& local_view, const RoiDecideResult& roi) {
    PARKING_LOG(D, "default trajPostProcess");
  };

  /**
   * @brief 计算自车在多个方位上与库位边线的间隙距离
   */
  virtual void calVehicleSlotClearance() { PARKING_LOG(D, "default calVehicleSlotClearance"); };

  /**
   * @brief 泊车环境识别
   */
  virtual void parkEnvRecognizer(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                                 const std::vector<math::LineSegment2d>& boundary_seg) {
    PARKING_LOG(D, "default parkEnvRecognizer");
  };

  /**
   * @brief 判断泊车是否需要微调
   */
  virtual bool needsFineTune(const PathPt& curr_traj_point) {
    PARKING_LOG(D, "default needsFineTune");
    return false;
  };

  /**
   * @brief 产生微调轨迹
   */
  virtual void generateFineTunePath(const LocalView& local_view) { PARKING_LOG(D, "default generateFineTunePath"); };

  /**
   * @brief 判断当前重规划是否需要采用几何规划器
   */
  virtual bool isNeedGeometricReplan() {
    PARKING_LOG(D, "default isNeedGeometricReplan");
    return false;
  };

  /**
   * @brief 产生几何重规划轨迹
   */
  virtual bool generateGeometricFineTunePath(const LocalView& local_view, const RoiDecideResult& roi,
                                             const double& radius = 6, const int max_shift_num = 8,
                                             const double short_path_length = 0.5,
                                             const double short_path_extend_max_length = 2.0) {
    PARKING_LOG(D, "default generateGeometricFineTunePath");
    return false;
  };

  /**
   * @brief 判断泊车是否完成
   */
  virtual bool isParkingCompleted(const LocalView& local_view, const bool destination_stop_flag) {
    PARKING_LOG(D, "default isParkingCompleted");
    return false;
  };

  /**
   * @brief 判断是否是U-Turn
   */
  virtual bool uTurnJudge(const ReferenceLine& ref, const double s, const double forward_distance_thrd,
                          const double backward_dist_thrd, double& u_turn_kappa) {
    PARKING_LOG(D, "default uTurnJudge");
    return false;
  }

  /**
   * @brief 定义ROI（Region of Interest）
   */
  UnitStatus defineROI(const LocalView& local_view, const DecisionResult& decision_result, RoiDecideResult& roi);

  /**
   * @brief 搜索泊车轨迹
   */
  UnitStatus originalPathSearch(const LocalView& local_view, const RoiDecideResult& roi);

  /**
   * @brief 轨迹优化
   */
  UnitStatus optimizer(const LocalView& local_view, const DecisionResult& decision_result, const RoiDecideResult& roi);

  /**
   * @brief 选择当前轨迹并对轨迹进行裁剪
   */
  void selectAndClipCurrentTrajectory(const LocalView& local_view);

  /**
   * @brief 障碍物选择
   */
  std::vector<Decision::DecisionObject> obstacleSelector(const LocalView& local_view,
                                                         const DecisionResult& decision_result);

  /**
   * @brief 库位稳定性分析
   */
  void calculateSlotAndTrackingStability(const LocalView& local_view);

  /**
   * @brief 轨迹点信息打印
   */
  void debugPrintTraj(DiscretizedPath traj, string name);
  void debugPrintTraj(vector<DiscretizedPath> trajs, string name);

  /**
   * @brief 轨迹坐标系转换
   */
  vector<DiscretizedPath> local2GlobalPaths(const LocalView& local_view,
                                            const std::vector<DiscretizedPath>& local_paths);
  std::vector<DiscretizedPath> global2LocalPaths(const LocalView& local_view,
                                                 const std::vector<DiscretizedPath>& global_paths);

  /**
   * @brief 计算轨迹block fs
   */
  PathData::BlockFSInfo calPathBlockFS(const LocalView& local_view, const DiscretizedPath& path_map);

  /**
   * @brief 判断是否需要重规划
   */
  bool isNeedReplan(const LocalView& local_view, const bool destination_stop, const StopReason& stop_reason);

  double getParkingTotalDurationLimit() {
    return planner_profile_->parking_failure_indicators().parking_max_time_consuming();
  };

  /**
   * @brief 对单段轨迹进行重优化
   * @param local_view 当前的局部视图，提供环境信息和车辆状态
   * @param decision_result 决策结果，提供障碍物信息
   * @param segment_to_optimize 需要被优化的轨迹段（map坐标系）
   * @return UnitStatus 返回重优化的状态
   */
  UnitStatus reOptimizer(const LocalView& local_view, const DecisionResult& decision_result,
                         const DiscretizedPath& segment_to_optimize);
  bool handleShiftAndReoptimization(const LocalView& local_view, const DecisionResult& decision_result,
                                    bool& destination_stop);
  void mapBoundary(const LocalView& local_view, std::vector<math::LineSegment2d>& boundary_seg);

  bool endPointMoveToSafePosition(PathPt& end_pose, const LocalView& local_view,
                                  const std::vector<Decision::DecisionObject>& select_ods,
                                  const std::vector<math::LineSegment2d>& boundary_seg);
  void handleRealTimeRefinement(const LocalView& local_view, const DecisionResult& decision_result);
  void prepareRealTimeData(const LocalView& local_view, const DecisionResult& decision_result);

 protected:
  std::string name_ = "BaseParking";                                              // 泊车规划器名称
  std::string debug_info_ = "\n\n[" + name_ + "]:\n";                             // debug信息
  bool init_ = false;                                                             // 泊车规划器初始化标志位
  CalStatus cal_status_ = CalStatus::DEFAULT;                                     // 泊车规划器计算状态
  ParkEnvType park_env_type_ = ParkEnvType::INVALID;                              // 泊车环境类型
  RoiDeciderBaseHAStar park_roi_decider_;                                         // ROI确定计算
  GeometryPathGenerator geometry_path_generator_;                                 // 几何路径生成器
  SlotParam slot_;                                                                // 库位参数
  PathPt initial_slot_center_pos_;                                                // 初始车位中心位置
  double remain_dis_ = 0.0;                                                       // 当前轨迹剩余距离，m
  vector<PathData::BoundsVec3dWithId> optimizer_boundary_vec_;                    // 优化边界信息
  bool is_last_traj_ = false;                                                     // 是否是最后一段轨迹标志位
  PlannerStatus planner_status_ = PlannerStatus::NORMAL;                          // 泊车规划器状态
  bool is_tuning_ = false;                                                        // 是否正在微调标志位
  std::pair<double, double> road_error_ = std::make_pair(0.0, 0.0);               // 泊车跟踪横向误差，m
  std::string cal_info_ = "";                                                     // 实时的轨迹信息，最终传给debug_info_
  int shift_replan_num_count_ = 0;                                                // 换挡处重规划次数
  int forced_stop_replan_num_count_ = 0;                                          // 逼停重规划次数
  int slot_update_replan_num_count_ = 0;                                          // 库位更新重规划次数
  int forced_stop_time_count_ = 0;                                                // 逼停时间计数
  int forced_stop_cancel_time_count_ = 0;                                         // 逼停取消时间计数
  std::pair<bool, PathPt> end_pose_map_bak_ = std::make_pair(false, PathPt());    // 泊车结束位姿备份(全局坐标下)
  std::pair<bool, PathPt> start_pose_map_bak_ = std::make_pair(false, PathPt());  // 泊车开始位姿备份(全局坐标下)
  double current_fine_tune_count_ = 0.0;                                          // 泊车规划内当前微调次数
  double slot_lateral_error_ = 0.0;                                               // 库位横向误差，m
  double slot_longti_error_ = 0.0;                                                // 库位纵向误差，m
  double slot_heading_error_ = 0.0;                                               // 库位航向误差，rad
  PathPt::Direction replan_direction_ = PathPt::Direction::FORWARD;               // 重规划推荐搜索起点方向
  std::vector<PathPt> corrected_slot_corner_;                                     // 被修正的库位角点
  std::pair<bool, std::vector<PathPt>> corrected_slot_ = std::make_pair(false, std::vector<PathPt>());  // 被修正的库位

  Eigen::Matrix4d tf_map_2_ego_search_ = Eigen::Matrix4d::Identity();  // 搜索时刻map坐标系到ego坐标系的变换矩阵
  Eigen::Matrix4d tf_ego_2_map_search_ = Eigen::Matrix4d::Identity();  // 搜索时刻ego坐标系到map坐标系的变换矩阵
  DiscretizedPath driving_phase_splice_path_;                          // 行车轨迹拼接路径

  bool is_ignore_od_and_fs_ = false;                                           // 是否忽略障碍物检测和 freespace 计算
  std::shared_ptr<Freespace> null_freespace_ = std::make_shared<Freespace>();  // 忽略fs时输入空值
  double non_stop_planning_dist_ = 10000.0;                                    // 不停车规划剩余距离，m
  std::pair<bool, math::LineSegment2d> gate_wall_ = std::make_pair(false, math::LineSegment2d());  // 闸机约束墙

  SearchElements search_elements_;

 protected:
  ConfigManager* config_manager_ = nullptr;                         // 配参管理器
  std::unique_ptr<PlannerProfile> planner_profile_;                 // 泊车规划器参数
  ParkingPlannerKey parking_planner_key_;                           // 泊车场景（子场景）分类
  OpenspaceCoreManager manager_;                                    // 子模块运行及数据管理器
  std::shared_ptr<OpenspaceOptimizerData> optimizer_data_ptr_;      // 优化相关的数据集合
  std::shared_ptr<OpenspaceSearchData> search_data_ptr_;            // 搜索相关的数据集合
  DiscretizedPath orin_combined_flu_path_;                          // 原始搜索后未分段的初解轨迹
  vector<DiscretizedPath> optimizer_segmented_flu_path_;            // 优化后分段（按方向）的泊车轨迹，ego坐标系
  vector<DiscretizedPath> optimizer_segmented_map_paths_;           // 优化后分段（按方向）的泊车轨迹，map坐标系
  vector<DiscretizedPath> optimizer_segmented_map_paths_backup_;    // 上一次泊车结果备份
  vector<DiscretizedPath> realtime_optimizer_segmented_flu_paths_;  // 实时泊车轨迹（多段），ego坐标系
  DiscretizedPath current_flu_path_;                                // 当前泊车轨迹，ego坐标系
  bool is_reoptimizing_ = false;                                    ///< 标志位，指示当前是否正处于重优化流程中
  std::shared_ptr<OpenspaceOptimizerData> reopt_data_ptr_;          ///< 重优化数据
  std::unique_ptr<OpenspaceCoreManager> reopt_manager_;             ///< 用于重优化的独立执行管理器
  planning::ReferenceLineInfo target_reference_line_info_;          ///< 目标参考线信息
  DiscretizedPath driving_discretized_path_;                        ///< 当前行车轨迹

 private:
  // 实时重优化专用的管理器和数据指针，与主流程区分开
  std::shared_ptr<OpenspaceOptimizerData> realtime_data_ptr_;
  std::unique_ptr<OpenspaceCoreManager> realtime_manager_;

  // 标记当前是否有一个后台优化任务正在跑
  bool is_realtime_optimizing_ = false;
};

}  // namespace gpal::pnc::planning
