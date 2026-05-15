#pragma once

#include "manager/spatiotemporal_planner_data_manager.h"
#include "spatiotemporal_abstract_module.h"

namespace gpal::pnc::planning {

class DecisionResultPreprocess : public SpatiotemporalAbstractModule {
 public:
  DecisionResultPreprocess() = default;
  ~DecisionResultPreprocess() = default;
  /**
   * @brief 获取模块 ID
   * @return std::string 模块 ID
   * @details 该方法返回当前模块的唯一标识符
   */
  std::string id() const override;

  bool init() override;
  void reset() override;
  bool run(SpatiotemporalPlannerDataManager& data_manager) override;

 private:
  bool decisionObjectPreprocess(SpatiotemporalPlannerDataManager& data_manager);
  bool decisionTrajectoryPreprocess(SpatiotemporalPlannerDataManager& data_manager);
  bool decisionBoundaryPreprocess(SpatiotemporalPlannerDataManager& data_manager);

  bool isValidDecisionObject(const Decision::DecisionObject& object);
  void setObstacleDefaultPrediction(const size_t& traj_point_count, const Decision::DecisionObject& object,
                                    std::vector<proto::TrajectoryPoint>& predicted_trajectory);
  void setObstaclePrediction(const size_t& traj_point_count, const Decision::DecisionObject& object,
                             std::vector<proto::TrajectoryPoint>& predicted_trajectory);
};

}  // namespace gpal::pnc::planning