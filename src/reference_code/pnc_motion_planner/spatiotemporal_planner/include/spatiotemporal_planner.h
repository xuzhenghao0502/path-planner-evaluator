#pragma once

#include <fmt/core.h>

#include "base/log.h"
#include "bound_parser/lateral_bound_parser.h"
#include "bound_parser/longitudinal_bound_parser.h"
#include "config/vehicle_model/vehicle_config.pb.h"
#include "config_manager/config_manager.h"
#include "decision_data/decision_result.h"
#include "decision_objects_parser/decision_objects_parser.h"
#include "local_view/local_view.h"
#include "manager/spatiotemoral_planner_pipeline_manager.h"
#include "manager/spatiotemporal_planner_data_manager.h"
#include "manager/spatiotemporal_planner_scenario_manager.h"
#include "path/path_data.h"
#include "preprocess/base_data_preprocess.h"
#include "preprocess/decision_result_preprocess.h"
#include "reference_line_info/reference_line_info.h"
#include "spatiotemporal_optimizer/spatiotemporal_optimizer.h"
#include "spatiotemporal_optimizer/spatiotemporal_optimizer_loader.h"
#include "speed/speed_data.h"

namespace gpal::pnc::planning {
class SpatiotemporalPlanner {
 public:
  SpatiotemporalPlanner() = default;
  ~SpatiotemporalPlanner() = default;

  /**
   * @brief 初始化时空规划器
   * @return bool 初始化是否成功
   */
  bool init();

  /**
   * @brief 重置时空规划器
   * @return bool 重置是否成功
   */
  bool reset();

  /**
   * @brief 执行时空规划
   * @param target_reference_line_info 目标参考线信息
   * @param current_reference_line_info 当前参考线信息
   * @param local_view 本地视图信息
   * @param decision_result 决策结果
   * @param prev_speed_data 上一次速度数据
   * @param stage_state 当前阶段状态
   * @param time_stamp 时间戳
   * @param path_boundary 路径边界信息
   * @param path_data 输出的路径数据
   * @details 该方法执行时空规划的主要逻辑，包括路径生成、边界解析、障碍物处理等
   * @note 该方法会根据输入的参考线信息、决策结果和本地视图等数据生成规划路径，并更新路径边界和障碍物信息
   * @return PathData::StatusType 执行是否成功
   */

  PathData::StatusType run(const std::shared_ptr<ReferenceLineInfo>& target_reference_line_info,
                           const std::shared_ptr<ReferenceLineInfo>& current_reference_line_info,
                           const std::shared_ptr<LocalView>& local_view,
                           const std::shared_ptr<DecisionResult>& decision_result, const StageState& stage_state,
                           std::shared_ptr<SpatiotemporalPlannerDataManager::OutputData> spatiotemporal_planner_result,
                           const std::pair<bool, double>& remain_dis_info);

  /**
   * @brief 将 run 方法的输入参数填充到 data_manager_ 中
   */
  void populateInputData(const std::shared_ptr<ReferenceLineInfo>& target_reference_line_info,
                         const std::shared_ptr<ReferenceLineInfo>& current_reference_line_info,
                         const std::shared_ptr<LocalView>& local_view, const StageState& stage_state,
                         const std::shared_ptr<DecisionResult>& decision_result,
                         const std::pair<bool, double>& remain_dis_info);

  /**
   * @brief 将 run 方法的输出结果填充到 spatiotemporal_planner_result 中
   */
  void extractOutputData(std::shared_ptr<SpatiotemporalPlannerDataManager::OutputData> spatiotemporal_planner_result);

  /**
   * @brief 本地路径碰撞检查
   */
  void localPathCollosionCheck();

 private:
  std::unique_ptr<SpatiotemporalPlannerDataManager> data_manager_;       ///< 数据管理器，用于存储输入输出数据
  std::vector<std::unique_ptr<SpatiotemporalAbstractModule>> pipeline_;  ///< 流水线模块列表

  bool getDestinationStopFlag(const pair<bool, double>& remain_dis_info, const Chassis* chassis);
  std::string getModuleAbbreviation(const std::string& module_id);
};

}  // namespace gpal::pnc::planning