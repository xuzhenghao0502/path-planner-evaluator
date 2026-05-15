#pragma once
#include <fmt/core.h>

#include "common/spatiotemporal_config_registry.h"
#include "manager/spatiotemporal_planner_data_manager.h"
#include "spatiotemporal_abstract_module.h"

namespace gpal::pnc::planning {

class BaseDataPreprocess : public SpatiotemporalAbstractModule {
 public:
  BaseDataPreprocess() = default;
  ~BaseDataPreprocess() = default;

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
  bool referenceLinePreprocess(SpatiotemporalPlannerDataManager& data_manager);
  bool vehicleInfoPreprocess(SpatiotemporalPlannerDataManager& data_manager);
  bool scenarioConfigPreprocess(SpatiotemporalPlannerDataManager& data_manager);
  bool timeRelatedBoundaryPreprocess(SpatiotemporalPlannerDataManager& data_manager);
  bool velocityRelatedBoundaryPreprocess(SpatiotemporalPlannerDataManager& data_manager);

  bool updateTrajectoryBoundary(SpatiotemporalPlannerDataManager& data_manager);
  bool updateTimeRelatedBoundary(SpatiotemporalPlannerDataManager& data_manager);
  bool updateVelocityRelatedBoundary(SpatiotemporalPlannerDataManager& data_manager);

  TrajectoryPt getPlanningStartPoint(SpatiotemporalPlannerDataManager& data_manager);

 protected:
  static constexpr double kSpeedEpsilon = 0.1;                                 ///< 速度容差
  std::unique_ptr<SpatiotemporalPlannerScenarioManager> scenario_manager_;     ///< 场景管理器
  mutable SpatiotemporalPlannerScenarioManager::ManagerKey last_manager_key_;  ///< 场景管理键

  ConfigManager* config_manager_ = nullptr;  ///< 配置管理器
};

}  // namespace gpal::pnc::planning
