
/**
 * @file lateral_path_bound_parser.h
 * @brief 路径边界解析器实现文件
 * @details
 * 该文件实现了路径边界解析器的核心功能，包括路径边界的初始化、静态和动态障碍物的边界信息获取等。
 * 路径边界解析器用于在路径规划过程中处理障碍物对路径边界的影响，确保路径的安全性和可行性。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "base/indexed_list.h"
#include "config/spatiotemporal_planner/lateral_path_bound_parser_config.pb.h"
#include "config_manager/config_manager.h"
#include "manager/spatiotemporal_planner_scenario_manager.h"
#include "path/path_decision.h"
#include "proto/common/pnc_point.pb.h"
#include "reference_line/reference_line.h"
#include "trajectory_boundary/trajectory_boundary.h"
#include "util/hysteresis_flag.h"
#include "util/table_interpolate.h"

namespace gpal::pnc::planning {

class LateralPathBoundParser {
 public:
  LateralPathBoundParser() = default;
  ~LateralPathBoundParser() = default;

  bool init();
  bool reset(const ReferenceLine& ref_line, const double& start_s, const double& end_s, const double& resolution);

  bool getBoundaryFromStaticObjects(const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_object,
                                    const std::vector<std::tuple<double, double, double>>& buffer,
                                    std::vector<Boundary>& boundaries);

 protected:
  using StaticObjectRange =
      std::tuple<int, double, const Decision::DecisionObject*, int>;  ///< is in range, ob_start_s, object_ptr, index
  using StaticObjectInfo = std::tuple<std::vector<math::LineSegment2d>, double>;
  using StaticObjectInfoMap = std::map<const Decision::DecisionObject*, StaticObjectInfo>;
  using ObjectBoundRange =
      std::tuple<int, double, double,
                 const Decision::DecisionObject*>;  ///< is in range, ob_start_s, offset, object_ptr

  std::pair<std::vector<StaticObjectRange>, std::vector<StaticObjectRange>> sortStaticObjectRange(
      const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_object, const double resolution_thrd = 0.0,
      std::vector<std::shared_ptr<Decision::DecisionObject>>* ptr_small = nullptr);
  std::pair<std::vector<ObjectBoundRange>, std::vector<ObjectBoundRange>> sortStaticObjectBoundRange(
      const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_object,
      const std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>>& buffer_map);
  StaticObjectInfo getStaticObjectInfo(const Decision::DecisionObject& static_object, const double lat_buffer = 0.0);

  std::tuple<const Decision::DecisionObject*, double> getLeftOffsetInfoFromStaticObject(
      const math::LineSegment2d& center, const StaticObjectInfoMap& object_borders);
  std::tuple<const Decision::DecisionObject*, double> getRightOffsetInfoFromStaticObject(
      const math::LineSegment2d& center, const StaticObjectInfoMap& object_borders);
  std::tuple<double, double, bool> getOffset(const math::LineSegment2d& segment,
                                             const math::LineSegment2d& bound_segment);

  std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>> getObjectBufferMap(
      const std::vector<std::shared_ptr<Decision::DecisionObject>>& objects,
      const std::vector<std::tuple<double, double, double>>& buffer);

 private:
  std::vector<ReferencePoint> sample_points_;         ///< 采样点
  std::vector<math::LineSegment2d> sample_segments_;  ///< 采样线段
  double resolution_ = 1.0;                           ///< 采样分辨率

 protected:
  // 配置查找表
  std::unordered_map<SpatiotemporalPlannerScenarioManager::ManagerKey, std::unique_ptr<LateralPathBoundParserProfile>,
                     SpatiotemporalPlannerScenarioManager::ManagerKeyHash>
      config_map_;

  std::unique_ptr<SpatiotemporalPlannerScenarioManager::ConfigGetter<LateralPathBoundParserProfile>> config_getter_;
  LateralPathBoundParserProfile profile_;
};

}  // namespace gpal::pnc::planning