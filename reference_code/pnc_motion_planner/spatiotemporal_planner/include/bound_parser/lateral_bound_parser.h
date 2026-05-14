#pragma once

#include "bound_parser/base_bound_parser.h"
#include "bound_parser/lateral_path_bound_parser.h"
#include "config/spatiotemporal_planner/lateral_path_bound_parser_config.pb.h"
#include "decision_data/common_utils.h"
#include "decision_data/decision_result.h"
#include "manager/spatiotemporal_planner_data_manager.h"

namespace gpal::pnc::planning {

class LateralBoundParser : public BaseBoundParser {
 public:
  LateralBoundParser() = default;
  ~LateralBoundParser() override = default;

  bool init() override;
  void reset() override;
  std::string id() const override;

  /**
   * @brief 执行横向边界解析
   * @param data_manager 由调度器传入的数据管理器
   * @details 该方法解析横向边界信息，并将静态和动态障碍物转换为决策对象
   * @return bool 返回是否成功执行
   */
  bool run(DataManager& data_manager) override;

 private:
  bool update(SpatiotemporalPlannerDataManager& data_manager);
  bool initBoundary(DataManager& data_manager);
  bool calcBoundaryFromStaticObjects(DataManager& data_manager);
  std::pair<double, double> getObjectsLaterSafeBuffer(const std::shared_ptr<Decision::DecisionObject>& obs,
                                                      const double& adc_frenet_s_speed);
  std::pair<double, double> getObsLaterSafeBufferFromType(const Decision::ObjectType& obs_type,
                                                          const double& default_barrier_lateral_buffer,
                                                          const double& default_soft_lateral_buffer);
  bool updateBoundary(const double& lat_buffer, Boundary& boundary);
  void trimBoundary(const int blocked_idx, const double& lat_buffer, std::vector<Boundary>& boundary);
  void trimBoundary(const int start_idx, const int end_idx, const double& lat_buffer, std::vector<Boundary>& boundary);
  bool refineBoundaryUnderSpecialScene(SpatiotemporalPlannerDataManager& data_manager);
  std::pair<double, double> interpolateDecisionBoundary(const LateralBoundDecision& boundary_points, double s);
  bool ignoreBoundary(const LongitudinalBoundDecision& longitudinal_bound_decision,
                                        double& ignore_start_s, double& ignore_end_s);

  DataManager::VehicleInfo* vehicle_info_ = nullptr;  ///< 车辆信息指针

  ConfigManager* config_manager_ = nullptr;  ///< 配置管理器
  LateralPathBoundParserProfile* profile_;   ///< 横向边界解析配置

  LateralPathBoundParser bound_parser_;               ///< 横向边界解析器
  math::IntervalData<Boundary> trajectory_boundary_;  ///< 路径边界数据
  std::unordered_map<int, BaseLateralPathBoundParserProfile::ObjectInfo>
      object_type_lateral_distance_map_;  ///< 基于障碍物类型的横向安全距离map表
  FsmState behavior_ = FsmState::KEEP;
  bool l_offset_behavior_valid_ = false;  ///< 决策横向offset信息有效性标志
  int crossing_line_behavior_ = 0;                   ///< 跨线行为 0 不跨线， 1 左侧跨线， 2 右侧跨线
};

}  // namespace gpal::pnc::planning
