#include "common/spatiotemporal_config_registry.h"

// 包含所有需要的配置类型头文件
#include "config/spatiotemporal_planner/decision_object_parser_config.pb.h"
#include "config/spatiotemporal_planner/lateral_path_bound_parser_config.pb.h"
#include "config/spatiotemporal_planner/longitudinal_bound_parser_config.pb.h"
#include "config/spatiotemporal_planner/spatiotemporal_optimizer_config.pb.h"
#include "config/spatiotemporal_planner/spatiotemporal_preprocess_config.pb.h"

namespace gpal::pnc::planning {

void registerAllSpatiotemporalConfigs() {
  STLOG(D, "[registerAllSpatiotemporalConfigs] Starting configuration registration");

  auto& registry = ConfigRegistry::getInstance();

  // 清空之前的注册（如果有）
  registry.clearRegistry();

  // 1. 注册决策障碍物解析器配置
  registry.registerConfig<DecisionObjectParserConfigList, DecisionObjectParserProfile>(
      "DecisionObjectParserConfigList", "DecisionObjectParserProfile",
      &SpatiotemporalPlannerDataManager::ConfigInfo::decision_object_parser_profile);

  // 2. 注册横向路径边界解析器配置
  registry.registerConfig<LateralPathBoundParserConfigList, LateralPathBoundParserProfile>(
      "LateralPathBoundParserConfigList", "LateralPathBoundParserProfile",
      &SpatiotemporalPlannerDataManager::ConfigInfo::lateral_path_bound_parser_profile);

  // 3. 注册纵向边界解析器配置
  registry.registerConfig<LongitudinalBoundParserConfigList, LongitudinalBoundParserProfile>(
      "LongitudinalBoundParserConfigList", "LongitudinalBoundParserProfile",
      &SpatiotemporalPlannerDataManager::ConfigInfo::longitudinal_bound_parser_profile);

  // 4. 注册时空优化器配置
  registry.registerConfig<SpatiotemporalOptimizerConfigList, SpatiotemporalOptimizerProfile>(
      "SpatiotemporalOptimizerConfigList", "SpatiotemporalOptimizerProfile",
      &SpatiotemporalPlannerDataManager::ConfigInfo::spatiotemporal_optimizer_profile);

  registry.registerConfig<SpatiotemporalPreprocessConfigList, SpatiotemporalPreprocessProfile>(
      "SpatiotemporalPreprocessConfigList", "SpatiotemporalPreprocessProfile",
      &SpatiotemporalPlannerDataManager::ConfigInfo::spatiotemporal_preprocess_profile);

  STLOG(D, "[registerAllSpatiotemporalConfigs] Registered ", registry.getRegisteredConfigCount(), " configurations");
}

}  // namespace gpal::pnc::planning