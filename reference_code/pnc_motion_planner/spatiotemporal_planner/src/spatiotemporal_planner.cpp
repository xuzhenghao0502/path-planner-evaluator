#include "spatiotemporal_planner.h"

namespace gpal::pnc::planning {

using ModuleFactory = AbstractFactory<SpatiotemporalAbstractModule>;

bool SpatiotemporalPlanner::init() {
  pipeline_.clear();
  // 1. 初始化数据管理器
  data_manager_ = std::make_unique<SpatiotemporalPlannerDataManager>();
  data_manager_->init();

  // 2. 从配置管理器获取各个子模块列表
  std::vector<std::string> pipeline_modules;
  if (!PipelineConfigManager::Instance().getPipeline("yeah", pipeline_modules)) {
    STLOG(E, "[SpatiotemporalPlanner] Failed to get pipeline config for name: ", "yeah");
    return false;
  }
  // 3. 遍历模块列表，创建子模块的实例
  for (const auto& module_name : pipeline_modules) {
    auto module = ModuleFactory::instance().create(module_name);
    if (!module) {
      STLOG(E, "[SpatiotemporalPlanner] Failed to create module from factory: ", module_name);
      return false;
    }
    // 4. 初始化模块
    if (!module->init()) {
      STLOG(E, "[SpatiotemporalPlanner] Module initialization failed: ", module->id());
      return false;
    }
    STLOG(I, "[SpatiotemporalPlanner] Module initialized successfully: ", module->id());
    pipeline_.emplace_back(std::move(module));
  }
  return true;
}

bool SpatiotemporalPlanner::reset() {
  // 重置逻辑
  data_manager_->reset();
  for (const auto& module : pipeline_) {
    module->reset();
  }
  return true;
}

PathData::StatusType SpatiotemporalPlanner::run(
    const std::shared_ptr<ReferenceLineInfo>& target_reference_line_info,
    const std::shared_ptr<ReferenceLineInfo>& current_reference_line_info, const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<DecisionResult>& decision_result, const StageState& stage_state,
    std::shared_ptr<SpatiotemporalPlannerDataManager::OutputData> spatiotemporal_planner_result,
    const std::pair<bool, double>& remain_dis_info) {
  STLOG(I,
        "[SpatiotemporalPlanner] ----------------- \U0001F9AD Spatiotemporal Path planning Start. \U0001F320 "
        "-----------------");
  string module_time_cost = "";
  // 1. 数据准备：将外部输入填充到 DataManager的input_data 中
  auto t1 = std::chrono::steady_clock::now();
  populateInputData(target_reference_line_info, current_reference_line_info, local_view, stage_state, decision_result,
                    remain_dis_info);
  auto t2 = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration<float, std::milli>(t2 - t1);
  STLOG(I, fmt::format("[SpatiotemporalPlanner] [populateInputData] {} ms", duration.count()));
  module_time_cost += fmt::format("[Input:{:.2g}], ", duration.count());

  // 2. 按顺序执行流水线中的所有模块
  for (const auto& module : pipeline_) {
    auto t3 = std::chrono::steady_clock::now();
    if (!module->run(*data_manager_)) {
      STLOG(E, fmt::format("[SpatiotemporalPlanner] Module {} failed in pipeline.", module->id()));
      spatiotemporal_planner_result->debug_info = fmt::format("[ST] Module {} failed in pipeline.", module->id());
      return PathData::StatusType::FAILED;  // 假设有 FAILED 状态
    }
    STLOG(I, fmt::format("[SpatiotemporalPlanner] Module {} completed successfully.", module->id()));
    auto t4 = std::chrono::steady_clock::now();
    duration = std::chrono::duration<float, std::milli>(t4 - t3);
    STLOG(I, fmt::format("[SpatiotemporalPlanner] [{}] {} ms", module->id(), duration.count()));
    std::string module_abbr = getModuleAbbreviation(module->id());
    module_time_cost += fmt::format("[{}:{:.2g}], ", module_abbr, duration.count());
  }

  // local path check
  localPathCollosionCheck();
  // 3. 数据后处理与输出
  auto t5 = std::chrono::steady_clock::now();
  extractOutputData(spatiotemporal_planner_result);
  auto t6 = std::chrono::steady_clock::now();
  duration = std::chrono::duration<float, std::milli>(t6 - t5);
  STLOG(I, fmt::format("[SpatiotemporalPlanner] [extractOutputData] {} ms", duration.count()));
  module_time_cost += fmt::format("[Output:{:.2g}], ", duration.count());
  duration = std::chrono::duration<float, std::milli>(t6 - t1);
  STLOG(I, fmt::format("[SpatiotemporalPlanner] [SpatPlanner] {} ms", duration.count()));
  module_time_cost += fmt::format("[SPT:{:.2g}], ", duration.count());
  spatiotemporal_planner_result->debug_info += module_time_cost;

  return PathData::StatusType::RUNNING;
}

void SpatiotemporalPlanner::populateInputData(const std::shared_ptr<ReferenceLineInfo>& target_reference_line_info,
                                              const std::shared_ptr<ReferenceLineInfo>& current_reference_line_info,
                                              const std::shared_ptr<LocalView>& local_view,
                                              const StageState& stage_state,
                                              const std::shared_ptr<DecisionResult>& decision_result,
                                              const std::pair<bool, double>& remain_dis_info) {
  auto& input_data = data_manager_->mutableInputData();
  input_data.target_ref_line_info = target_reference_line_info.get();
  input_data.current_ref_line_info = current_reference_line_info.get();
  if (target_reference_line_info == nullptr) {
    STLOG(I, "[SpatiotemporalPlanner] target_reference_line_info is null");
  }
  if (current_reference_line_info == nullptr) {
    STLOG(I, "[SpatiotemporalPlanner] current_reference_line_info is null");
  }
  if (!target_reference_line_info->isValid()) {
    STLOG(I, "[SpatiotemporalPlanner] target_reference_line_info is invalid");
  }
  if (!current_reference_line_info->isValid()) {
    STLOG(I, "[SpatiotemporalPlanner] current_reference_line_info is invalid");
  }
  input_data.stage_state = stage_state;
  input_data.decision_result = decision_result.get();
  input_data.console = local_view->getConsolePtr().get();
  input_data.chassis = local_view->getChassisPtr().get();
  input_data.localization = local_view->getLocalizationPtr().get();
  input_data.memorized_route = local_view->getMemorizedRoutePtr().get();
  input_data.env_road_cognition = local_view->getEnvRoadCognitionPtr().get();
  input_data.vehicle_state = local_view->getVehicleStatePtr().get();
  input_data.freespace = local_view->getFreespacePtr().get();
  input_data.destination_remain_dis_info = remain_dis_info;
}
/*释放output data*/
void SpatiotemporalPlanner::extractOutputData(
    std::shared_ptr<SpatiotemporalPlannerDataManager::OutputData> spatiotemporal_planner_result) {
  auto& output_data = data_manager_->mutableOutputData();
  auto& bound_info = data_manager_->boundaryInfo();
  // TODO：增加可视化&debug相关输出
  spatiotemporal_planner_result->planning_trajectory = output_data.planning_trajectory;
  spatiotemporal_planner_result->debug_info = output_data.debug_info;
  spatiotemporal_planner_result->soft_lateral_bound = bound_info.trajectory_boundary->softLateralBound();
  spatiotemporal_planner_result->hard_lateral_bound = bound_info.trajectory_boundary->hardLateralBound();
  spatiotemporal_planner_result->s_soft_bound = bound_info.time_related_boundary->sSoftBound();
  spatiotemporal_planner_result->max_speed_limit = output_data.max_speed_limit;
  spatiotemporal_planner_result->coarse_trajectory = output_data.coarse_trajectory;
  spatiotemporal_planner_result->interaction_infos = output_data.interaction_infos;
  spatiotemporal_planner_result->is_valid = output_data.is_valid;
  spatiotemporal_planner_result->stop_at_destination = getDestinationStopFlag(
      data_manager_->inputData().destination_remain_dis_info, data_manager_->inputData().chassis);
  spatiotemporal_planner_result->nearest_obs_info = output_data.nearest_obs_info;
  spatiotemporal_planner_result->need_stop = output_data.need_stop;
  spatiotemporal_planner_result->block_fs_info = output_data.block_fs_info;
  spatiotemporal_planner_result->local_path = output_data.local_path;
}

void SpatiotemporalPlanner::localPathCollosionCheck() {
  const auto& input_data = data_manager_->inputData();
  const auto& profile = data_manager_->configInfo().longitudinal_bound_parser_profile;
  LocalPathProcess local_path_process;
  if (!local_path_process.init(*profile)) {
    STLOG(E, "[SpatiotemporalPlanner] local path process init failed");
    return;
  }
  auto local_path_collision_dis =
      local_path_process.getLocalPathBlockDis(input_data.chassis, input_data.vehicle_state, input_data.localization,
                                              input_data.freespace, *input_data.decision_result);
  data_manager_->mutableOutputData().local_path = local_path_process.getLocalPath();
  if (local_path_process.getBlockFSInfo().s < local_path_process.getBlockObsInfo().s) {
    data_manager_->mutableOutputData().block_fs_info = local_path_process.getBlockFSInfo();
  } else {
    data_manager_->mutableOutputData().block_fs_info = local_path_process.getBlockObsInfo();
  }
} 

bool SpatiotemporalPlanner::getDestinationStopFlag(const pair<bool, double>& remain_dis_info, const Chassis* chassis) {
  bool stop_at_destination = false;
  STLOG(I, fmt::format("  remain_dis_info.second: {}", remain_dis_info.second));
  if (!remain_dis_info.first) {
    return stop_at_destination;
  }
  double stop_distance_threshold = 2.0;
  if (remain_dis_info.second < stop_distance_threshold && chassis->Speed() < 1.0 * KMH_MS) {
    stop_at_destination = true;
  }
  return stop_at_destination;
}

 std::string SpatiotemporalPlanner::getModuleAbbreviation(const std::string& module_id) {
    static const std::unordered_map<std::string, std::string> abbr_map = {
      {"BaseDataPreprocess", "Base"},
      {"DecisionResultPreprocess", "Dec"},
      {"DecisionObjectsParser", "Obj"},
      {"LateralBoundParser", "Lat"},
      {"LongitudinalBoundParser", "Lon"},
      {"SpatiotemporalOptimizerLoader", "Load"},
      {"SpatiotemporalOptimizer", "Opt"}
    };
    
    auto it = abbr_map.find(module_id);
    if (it != abbr_map.end()) {
      return it->second;
    }
    
    // 如果没有找到映射，使用前3个字符作为缩写
    if (module_id.length() > 3) {
      return module_id.substr(0, 3);
    }
    return module_id;
  }

}  // namespace gpal::pnc::planning
