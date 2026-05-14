
/**
 * @file path_bound_parser.h
 * @brief 路径边界解析器实现文件
 * @details 该文件实现了路径边界解析器的核心功能，包括路径边界的初始化、静态和动态障碍物的边界信息获取等。路径边界解析器用于在路径规划过程中处理障碍物对路径边界的影响，确保路径的安全性和可行性。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "proto/common/pnc_point.pb.h"
#include "base/indexed_list.h"
#include "config_manager/config_manager.h"
#include "util/table_interpolate.h"
#include "util/hysteresis_flag.h"
#include "reference_line/reference_line.h"
#include "config/path_bound_parser/path_bound_parser_config.pb.h"
#include "path/path_decision.h"
#include "local_view/Freespace.h"
#include "decision_data/decision_result.h"

namespace gpal::pnc::planning {

/**
 *  @brief obs边界表征
 */
struct ObstacleBoundaryInfo {
  util::AbstractTable1d<double, double, double> bound;  ///< s, lower, upper
  std::vector<const Decision::DecisionObject*> right_bound_obstacles;   ///< right obs, nullptr if none
  std::vector<const Decision::DecisionObject*> left_bound_obstacles;    ///< left obs, nullptr if none
};

/**
 * @brief freespace边界表征
 */
struct FreespaceBoundaryInfo {
  util::AbstractTable1d<double, double, double> bound;  ///< s, lower, upper
  std::vector<int> right_types;                         ///< right types
  std::vector<int> left_types;                          ///< left types
};

/**
 * @brief 用于向 getBoundaryFromPolyline 函数传递抽象化障碍物线段信息的结构体。
 * @details 该结构体包含了计算边界所需的所有几何和语义信息，但与 DecisionObject 完全解耦。
 */
struct PolylineInput {
  std::string id = "";  // 用于在处理过程中进行唯一标识和映射的ID

  std::vector<math::LineSegment2d> segments;  // 从原始障碍物包围盒提取出的几何线段集合

  // 从原始障碍物的SL边界信息中提取
  double start_s = 0.0;  // SL边界的起始s值
  double end_s = 0.0;    // SL边界的结束s值
  double start_l = 0.0;  // SL边界的起始l值
  double end_l = 0.0;    // SL边界的结束l值

  // 定义一个独立的绕行方向枚举，以避免依赖 decision_define.h
  enum class NudgeType { LEFT_BYPASS, RIGHT_BYPASS, UNKNOWN };
  NudgeType nudge_type = NudgeType::UNKNOWN;  // 绕行方向，左侧绕行或右侧绕行
};

using PolylineRange = std::tuple<bool, double, const PolylineInput*, int>;
using PolylineBoundRange = std::tuple<bool, double, double, std::string>;
using PolylineInfoMap = std::map<std::string, std::pair<const std::vector<math::LineSegment2d>*, double>>;

/**
 * @brief 路径边界解析器类,用于在路径规划过程中处理障碍物等对路径边界的影响，确保路径的安全性和可行性。
 */
class PathBoundParser {
 public:
  /**
   * @brief 默认构造函数
   */
  PathBoundParser() = default;
  PathBoundParser(const PathBoundParserConfig& config);
  /**
   * @brief 默认析构函数
   */
  ~PathBoundParser() = default;

  /**
   * @brief 获取配置对象的引用
   * @return 返回配置对象的引用
   */
  PathBoundParserConfig& config() { return config_; }

  /**
   * @brief 获取配置对象的常量引用
   * @return 返回配置对象的常量引用
   */
  const PathBoundParserConfig& config() const { return config_; }
  
  /**
   * @brief 设置自车的Frenet坐标系下的s值
   * @param adc_frenet_s 自车的Frenet坐标系下的s值
   * @return 返回设置后的s值
   */
  double setAdcFrenetS(const double adc_frenet_s) { return adc_frenet_s_ = adc_frenet_s; };

  /**
   * @brief 设置自车的Frenet坐标系下的速度
   * @param adc_frenet_v 自车的Frenet坐标系下的速度
   * @return 返回设置后的速度
   */
  double setAdcFrenetSpeed(const double adc_frenet_v) { return adc_frenet_sd_ = adc_frenet_v; };

  /**
   * @brief 设置自车的时间-空间范围
   * @param adc_ts_ranges 自车的时间-空间范围，包含起始时间、结束时间、起始s值和结束s值
   * @return 返回设置后的时间-空间范围
   */
  std::vector<std::tuple<double, double, double, double>> setAdcTSRange(const std::vector<std::tuple<double, double, double, double>> adc_ts_ranges) {
    return adc_ts_ranges_ = adc_ts_ranges;
  };

  /**
   * @brief 设置自车的速度轨迹
   * @param adc_speed_traj 自车的速度轨迹，包含时间和s值的对
   */
  void setAdcSpeedTraj(const std::vector<std::pair<double, double>>& adc_speed_traj) { adc_speed_traj_ = adc_speed_traj; };

  /**
   * @brief 获取自车的速度轨迹
   * @return 返回自车的速度轨迹
   */
  std::vector<std::pair<double, double>> getAdcSpeedTraj() const { return adc_speed_traj_; };

  /**
   * @brief 设置自车的前部长度
   * @param adc_front_length 自车的前部长度
   */
  void setAdcFrontLength(const double adc_front_length) { adc_front_length_ = adc_front_length; };

   /**
   * @brief 设置自车的后部长度
   * @param adc_rear_length 自车的后部长度
   */
  void setAdcRearLength(const double adc_rear_length) { adc_rear_length_ = adc_rear_length; };

  /**
   * @brief 设置目标参考线
   * @param ref_line 目标参考线
   */
  void setTargetReferenceLine(const ReferenceLine& ref_line) { target_reference_line_ = ref_line; }

   /**
   * @brief 获取采样点
   * @return 返回采样点的引用
   */
  const std::vector<ReferencePoint>& sample_points() const { return sample_points_; }

   /**
   * @brief 获取自车系下的采样点
   * @return 返回自车系下的采样点的引用
   */
  const std::vector<ReferencePoint>& ego_sample_points() const { return ego_sample_points_; }

  bool init(const ReferenceLine& ref_line, const Eigen::Matrix4d& tf_map_2_ego, const double& start_s,
            const double& end_s);
  bool init(const ReferenceLine& ref_line, const Eigen::Matrix4d& tf_map_2_ego, const double& start_s,
            const double& end_s, const std::vector<std::tuple<double, double, double>>& max_range);
 
  ObstacleBoundaryInfo getBoundaryFromStaticObstacles(const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles);
  ObstacleBoundaryInfo getBoundaryFromStaticObstacles(const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
                                                      const std::vector<std::tuple<double, double, double>>& buffer);

  ObstacleBoundaryInfo getBoundaryFromDynamicObstacles(const ReferenceLine& ref_line,
                                                       const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
                                                       const std::vector<std::tuple<double, double, double, double, double, double, double>>& buffer);
  
  FreespaceBoundaryInfo getBoundInfoFromFreespace(const Freespace& freespace, const double buffer = 0.0);
  FreespaceBoundaryInfo getBoundInfoFromFreespace(const Freespace& freespace, const std::vector<double>& sample_offsets,
                                                  const double buffer = 0.0);
  FreespaceBoundaryInfo getBoundInfoFromFreespace(const Freespace& freespace,
                                                  const std::vector<IgnoreRangeInfo>& ignore_ranges,
                                                  const std::vector<double>& sample_offsets,
                                                  const double buffer = 0.0);
  FreespaceBoundaryInfo getBoundInfoFromFreespace(const Freespace& freespace, const std::vector<double>& sample_offsets,
                                                  const double buffer, const double start_buffer,
                                                  const double end_buffer);
  FreespaceBoundaryInfo getFreespaceBoundInfoWithLongiBuffer(const FreespaceBoundaryInfo& boundary,
                                                             const double start_buffer, const double end_buffer);
  ObstacleBoundaryInfo getBoundaryFromPolyline(const std::vector<PolylineInput>& polylines,
                                               const std::vector<std::tuple<double, double, double>>& buffer);

 protected:
  using StaticObstacleRange = std::tuple<int, double, const Decision::DecisionObject*, int>;    ///< is in range, ob_start_s, obstacle_ptr, index
  using StaticObstacleInfo = std::tuple<std::vector<math::LineSegment2d>, double>; ///< lineSegments, lat_buffer
  using StaticObstacleInfoMap = std::map<const Decision::DecisionObject*, StaticObstacleInfo>;  ///< obstacle_ptr, obstacle_info
  using ObstacleBoundRange = std::tuple<int, double, double, const Decision::DecisionObject*>; ///< is in range, ob_start_s, offset, obstacle_ptr
  using DynamicObstacleRange = std::tuple<int, double, double, double, const Decision::DecisionObject*, int>;  ///< is in bound, s, l, offset, obstacle_ptr, index
  using DynamicObstacleInfo = std::tuple<std::vector<math::LineSegment2d>, double, double>; ///< lineSegments, obstacle_l, lat_buffer
  using DynamicObstacleInfoMap = std::map<const Decision::DecisionObject*, DynamicObstacleInfo>; ///< obstacle_ptr, obstacle_info


  std::pair<std::vector<StaticObstacleRange>, std::vector<StaticObstacleRange>> sortStaticObstacleRange(
      const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles, const double resolution_thrd = 0.0,
      std::vector<std::shared_ptr<Decision::DecisionObject>>* ptr_small = nullptr);
  std::pair<std::vector<ObstacleBoundRange>, std::vector<ObstacleBoundRange>> sortStaticObstacleBoundRange(
      const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
      const std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>>& buffer_map);
  StaticObstacleInfo getStaticObstacleInfo(const Decision::DecisionObject& static_obstacle, const double lat_buffer = 0.0);
  
  std::tuple<const Decision::DecisionObject*, double> getLeftOffsetInfoFromStaticObstacle(
      const math::LineSegment2d& center, const StaticObstacleInfoMap& obstacle_borders);
  std::tuple<const Decision::DecisionObject*, double> getRightOffsetInfoFromStaticObstacle(
      const math::LineSegment2d& center, const StaticObstacleInfoMap& obstacle_borders);
  std::tuple<double, double, bool> getOffset(const math::LineSegment2d& segment,
                                             const math::LineSegment2d& bound_segment);

  std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>> getObstacleBufferMap(
      const std::vector<std::shared_ptr<Decision::DecisionObject>>& obstacles,
      const std::vector<std::tuple<double, double, double>>& buffer);

  bool isReverse(const ReferenceLine& ref_line, const Decision::DecisionObject* npc);

  std::pair<std::vector<PathBoundParser::DynamicObstacleRange>, std::vector<PathBoundParser::DynamicObstacleRange>> sortDynamicObstacleAndBoundaryRange(
      const ReferenceLine& ref_line,
      const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
      const std::vector<std::tuple<double, double, double, double, double, double, double>>& buffer);
  std::pair<std::vector<std::shared_ptr<Decision::DecisionObject>>, std::vector<std::tuple<double, double, double, double, double, double, double>>>
  pickDynamicObstaclesForDynamicNudge(const ReferenceLine& ref_line, const double& ego_veloc,
                                      const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
                                      const std::vector<std::tuple<double, double, double, double, double, double, double>>& buffer);
  std::unordered_map<std::string, std::tuple<double, double, double>> getPolylineBufferMap(
      const std::vector<PolylineInput>& polylines, const std::vector<std::tuple<double, double, double>>& buffer);

  std::pair<std::vector<PolylineRange>, std::vector<PolylineRange>> sortPolylineRange(
      const std::vector<PolylineInput>& polylines, double resolution_thrd, std::vector<PolylineInput>* ptr_small);

  std::pair<std::vector<PolylineBoundRange>, std::vector<PolylineBoundRange>> sortPolylineBoundRange(
      const std::vector<PolylineInput>& polylines,
      const std::unordered_map<std::string, std::tuple<double, double, double>>& buffer_map);

  std::tuple<std::string, double> getLeftOffsetInfoFromPolyline(const math::LineSegment2d& center,
                                                                const PolylineInfoMap& polyline_borders);

  std::tuple<std::string, double> getRightOffsetInfoFromPolyline(const math::LineSegment2d& center,
                                                                 const PolylineInfoMap& polyline_borders);

 protected:
  PathBoundParserConfig config_;  ///< 路径边界解析器配置，包含解析器的各项参数

 private:
  double adc_frenet_s_ = 0.0; ///< 自车的Frenet坐标系下的s值
  double adc_frenet_sd_ = 0.0; ///< 自车的Frenet坐标系下的速度
  std::vector<std::pair<double, double>> adc_speed_traj_; ///< 自车的速度轨迹，包含t和s值的对
  std::vector<std::tuple<double, double, double, double>> adc_ts_ranges_;  ///< 自车的时间-空间范围，包含起始时间、结束时间、起始s值和结束s值
  double adc_front_length_ = 0.0; ///< 自车的前部长度
  double adc_rear_length_ = 0.0; ///< 自车的后部长度
  ReferenceLine target_reference_line_; ///< 目标参考线
  std::vector<ReferencePoint> sample_points_; ///< 采样点
  std::vector<math::LineSegment2d> sample_segments_; ///< 采样线段
  std::vector<ReferencePoint> ego_sample_points_; ///< 自车系下的采样点
  util::AbstractTable1d<double, double, double> check_ranges_; ///< 检查范围
  std::unordered_map <std::string, util::HysteresisFlag<double>> hysteresis_delta_velocity_map_; ///< 速度差值的滞后标志映射
};

}  // namespace gpal::pnc::planning
