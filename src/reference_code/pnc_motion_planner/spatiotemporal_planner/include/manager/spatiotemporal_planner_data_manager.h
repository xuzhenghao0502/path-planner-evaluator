#pragma once

#include <atomic>
#include <filesystem>
#include <fmt/chrono.h>
#include <memory>
#include <string>
#include <unordered_set>

#include "common/spatiotemporal_common.h"
#include "config/spatiotemporal_planner/decision_object_parser_config.pb.h"
#include "config/spatiotemporal_planner/lateral_path_bound_parser_config.pb.h"
#include "config/spatiotemporal_planner/longitudinal_bound_parser_config.pb.h"
#include "config/spatiotemporal_planner/spatiotemporal_optimizer_config.pb.h"
#include "config/spatiotemporal_planner/spatiotemporal_preprocess_config.pb.h"
#include "decision_data/decision_result.h"
#include "local_view/local_view.h"
#include "manager/spatiotemporal_planner_scenario_manager.h"
#include "math/math_utils.h"
#include "ocp/ocp_model.h"
#include "path/discretized_path.h"
#include "path/path_data.h"
#include "point/path_pt.h"
#include "reference_line_info/reference_line_info.h"
#include "speed/speed_data.h"
#include "trajectory_boundary/time_related_boundary.h"
#include "trajectory_data/trajectory.h"
#include "trajectory_boundary/trajectory_boundary.h"
#include "trajectory_boundary/velocity_related_boundary.h"
#include "ellipse_object_info/ellipse_object_info.h"
#include "util/abstract_factory.h"
#include "util/task_handler.h"
#include "util/timer.h"

namespace gpal::pnc::planning {

/**
 * @brief 时空联合规划数据管理器
 * @note 方法：
 * 提供返回引用的访问器 (T& 和 const T&)
 */
 namespace DrivingScenarios {
    const std::string END = "END";
    const std::string JUNCTION = "JUNCTION";
    const std::string JUNCTION_FORWARD = "JUNCTION_FORWARD";  // 直行通过路口
    const std::string MERGING = "MERGING";
    const std::string LANE_CHANGE = "LANE_CHANGE";
    const std::string EMERGENCY_BRAKE = "EMERGENCY_BRAKE";
    const std::string VEHICLE_START = "VEHICLE_START";
    const std::string GATE = "GATE";
}
class SpatiotemporalPlannerDataManager {
 public:
  /**
   * @brief 构造函数
   */
  SpatiotemporalPlannerDataManager() = default;

  /**
   * @brief 析构函数
   */
  ~SpatiotemporalPlannerDataManager() = default;

  // 禁用拷贝构造和拷贝赋值。
  SpatiotemporalPlannerDataManager(const SpatiotemporalPlannerDataManager&) = delete;
  SpatiotemporalPlannerDataManager& operator=(const SpatiotemporalPlannerDataManager&) = delete;

  struct VehicleInfo {
    // 车辆状态信息
    TrajectoryPt start_point;                                                ///< 规划起始点
    std::pair<std::array<double, 3>, std::array<double, 3>> sl_info;         ///< 车辆状态信息 (s, l) 对应的 (x, y) 坐标
    std::pair<std::array<double, 3>, std::array<double, 3>> driven_sl_info;  ///< 车辆状态信息 (s, l) 对应的 (x, y) 坐标

    double curr_right_bound = -0.75;  ///< 目标参考线规划起始点右边界
    double curr_left_bound = 0.75;    ///< 目标参考线规划起始点左边界
    double frenet_end_s = 0.0;        ///< 目标参考线终点 Frenet 坐标系下的 s 坐标

    void reset() {
      // 重置车辆信息
      start_point = TrajectoryPt();
      sl_info = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
      driven_sl_info = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
      curr_right_bound = -0.75;
      curr_left_bound = 0.75;
      frenet_end_s = 0.0;
    }
  };

  struct InputData {
    StageState stage_state = StageState::IdleStage;  ///< 当前阶段状态

    const Chassis* chassis = nullptr;                 ///< 底盘状态信息
    const Console* console = nullptr;                 ///< 控制台信息
    const Localization* localization = nullptr;       ///< 定位信息
    const MemorizedRoute* memorized_route = nullptr;  ///< 记忆路线信息
    const EnvRoadCognition* env_road_cognition = nullptr;  ///< 环境道路认知信息
    const VehicleState* vehicle_state = nullptr; ///<车辆状态信息
    const Freespace* freespace = nullptr;  ///<fs信息

    const ReferenceLineInfo* target_ref_line_info = nullptr;   ///< 目标参考线信息
    const ReferenceLineInfo* current_ref_line_info = nullptr;  ///< 当前参考线信息

    std::string pre_ref_id_ = "";                                                ///< 历史参考ID
    std::vector<std::pair<std::string, std::vector<std::string>>> pre_lane_id_;  ///< 历史车道ID
    bool change_ref_ = false;                                                    ///< 是否切换参考线
    const DecisionResult* decision_result = nullptr;                             ///< 决策结果信息

    std::unique_ptr<VehicleInfo> vehicle_info = nullptr;  ///< 车辆信息

    std::pair<bool, double> destination_remain_dis_info = {false, 10000.0};  ///< FIRST: 是否有终点信息, SECOND: 剩余距离信息

    TrajectoryPt destination_point = TrajectoryPt();  ///< 目的地参考点

    void reset() {
      // 重置输入信息
      stage_state = StageState::IdleStage;
      chassis = nullptr;
      console = nullptr;
      localization = nullptr;
      memorized_route = nullptr;
      env_road_cognition = nullptr;
      vehicle_state = nullptr;
      freespace = nullptr;
      target_ref_line_info = nullptr;
      current_ref_line_info = nullptr;
      decision_result = nullptr;
      vehicle_info.reset();
      destination_remain_dis_info = {false, 10000.0};
      destination_point = TrajectoryPt();
    }
  };

  struct OutputData {
    std::string debug_info = "";                                 ///< 调试信息输出
    std::unordered_set<PathData::DebugStatusType> debug_status;  ///< 调试状态集合
    Trajectory planning_trajectory;

    math::IntervalData<Boundary> soft_lateral_bound;
    math::IntervalData<Boundary> hard_lateral_bound;

    math::IntervalData<Boundary> s_soft_bound;

    double max_speed_limit = kMaxSpeedMS;
    Trajectory coarse_trajectory;

    bool is_valid = false;  ///< 是否有效

    bool stop_at_destination = false;

    std::vector<InteractionInfo> interaction_infos; // size = N_

    std::tuple<string, double, double > nearest_obs_info = std::make_tuple("", 0.0, 0.0);  // obs_id , obs_s, obs_v
    bool need_stop = false;

    PathData::BlockFSInfo block_fs_info;
    DiscretizedPath local_path;

    void reset() {
      // 重置输出信息
      debug_info.clear();
      debug_status.clear();
      planning_trajectory.clear();
      coarse_trajectory.clear();
      is_valid = false;
      soft_lateral_bound = math::IntervalData<Boundary>();
      hard_lateral_bound = math::IntervalData<Boundary>();
      s_soft_bound = math::IntervalData<Boundary>();

      max_speed_limit = kMaxSpeedMS;
      stop_at_destination = false;
      interaction_infos.clear();
      nearest_obs_info = std::make_tuple("", 0.0, 0.0);
      need_stop = false;
      local_path.clear();
      block_fs_info = PathData::BlockFSInfo();
    }
  };

  struct BoundaryInfo {
    std::vector<std::tuple<string, float, float>> special_scene_s_range_sets;

    // 边界解析的输出
    std::unique_ptr<TrajectoryBoundary> trajectory_boundary = nullptr;             ///< 路径边界信息
    std::unique_ptr<TimeRelatedBoundary> time_related_boundary = nullptr;          ///< 时间相关边界信息
    std::unique_ptr<VelocityRelatedBoundary> velocity_related_boundary = nullptr;  ///< 速度相关边界信息

    void reset() {
      // 重置边界信息
      special_scene_s_range_sets.clear();
      trajectory_boundary.reset();
      time_related_boundary.reset();
      velocity_related_boundary.reset();
    }
  };

  struct ObjectsInfo {
    std::vector<std::unique_ptr<Decision::DecisionObject>> target_decision_objects;  ///< 目标障碍物,前处理结果
    std::vector<std::shared_ptr<Decision::DecisionObject>> static_objects;           ///< 静态障碍物
    std::vector<std::vector<ObjectInfo>> target_objects_info;                        ///< 目标物信息，优化输入

    void reset() {
      // 重置目标物信息
      target_decision_objects.clear();
      target_objects_info.clear();
      static_objects.clear();
    }
  };

  struct ScenarioInfo {
    SpatiotemporalPlannerScenarioManager::ManagerKey manager_key;
    bool has_scenario_changed = true;  ///< 场景是否已更改
    void reset() {
      // 重置场景信息
      manager_key = SpatiotemporalPlannerScenarioManager::ManagerKey();
      has_scenario_changed = true;
    }
  };

  struct ConfigInfo {
    std::unique_ptr<DecisionObjectParserProfile> decision_object_parser_profile;         ///< 决策对象解析配置
    std::unique_ptr<LateralPathBoundParserProfile> lateral_path_bound_parser_profile;    ///< 横向向路径边界解析配置
    std::unique_ptr<LongitudinalBoundParserProfile> longitudinal_bound_parser_profile;   ///< 纵向路径边界解析配置
    std::unique_ptr<SpatiotemporalOptimizerProfile> spatiotemporal_optimizer_profile;    ///< 时空优化器配置
    std::unique_ptr<SpatiotemporalPreprocessProfile> spatiotemporal_preprocess_profile;  ///< 前处理配置
    void reset() {
      // 重置配置
      decision_object_parser_profile.reset();
      lateral_path_bound_parser_profile.reset();
      longitudinal_bound_parser_profile.reset();
      spatiotemporal_optimizer_profile.reset();
      spatiotemporal_preprocess_profile.reset();
    }
  };

  struct OptimizerInfo {
    enum AsyncStatus {
      COLLISION_CHEAK_FAILED = -3,
      INFEASIBLED = -2,
      ASYNC_FAILED = -1,
      ASYNC_DO_NOTHING_YET = 0,
      ASYNC_UNDERLOCKING = 1,
      ASYNC_SOLVED = 2,
    };
    struct SolverData {
      std::atomic<bool> is_finished = false;
      std::mutex mutex;
      std::condition_variable_any cond;
      std::string ref_id = "";
      std::string context = "";
      SolveStatus status = SolveStatus::SOLVER_UNINIT;
      std::shared_ptr<OptimalControlProblem> model = nullptr;
      std::vector<PathData::DebugStatusType> debug_status = {};
    };
    AsyncStatus async_status = AsyncStatus::ASYNC_DO_NOTHING_YET;
    double lane_keep_start_s = 0.0;
    std::string profile = "";
    std::vector<SpatiotemporalState> init_guess = {};
    std::vector<std::unordered_map<std::string, std::pair<bool, double>>> optimizer_parameters = {};
    std::string async_optimizer_debug_info = "";
    SolveInfo solve_info;
    std::unordered_map<std::string, bool> current_driving_scenario = {
        {DrivingScenarios::END, false},
        {DrivingScenarios::JUNCTION, false},
        {DrivingScenarios::JUNCTION_FORWARD, false},
        {DrivingScenarios::MERGING, false},
        {DrivingScenarios::LANE_CHANGE, false},
        {DrivingScenarios::EMERGENCY_BRAKE, false},
        {DrivingScenarios::VEHICLE_START, false},
        {DrivingScenarios::GATE, false},
    };
    void reset() {
      // 重置优化器信息
      // TODO: 实现重置逻辑
      AsyncStatus async_status = AsyncStatus::ASYNC_DO_NOTHING_YET;
      lane_keep_start_s = 0.0;
      init_guess.clear();
      optimizer_parameters.clear();
      profile.clear();
      async_optimizer_debug_info.clear();
      current_driving_scenario = {
          {DrivingScenarios::END, false},
          {DrivingScenarios::JUNCTION, false},
          {DrivingScenarios::JUNCTION_FORWARD, false},
          {DrivingScenarios::MERGING, false},
          {DrivingScenarios::LANE_CHANGE, false},
          {DrivingScenarios::EMERGENCY_BRAKE, false},
          {DrivingScenarios::VEHICLE_START, false},
          {DrivingScenarios::GATE, false},
      };
      solve_info.reset();
    }
  };

  /**
   * @brief 初始化数据管理器
   */
  void init() {
    // 初始化所有数据成员
    input_data_.reset();
    output_data_.reset();
    boundary_info_.reset();
    objects_info_.reset();
    scenario_info_.reset();
    optimizer_info_.reset();
    grids_info_.reset();
  }

  /**
   * @brief 重置所有数据成员到默认状态
   * @note 用于清空数据。
   */
  void reset() { init(); }

  /**
   * @brief 获取输入数据 (返回常量引用)
   * @return const InputData& 不可修改的输入数据
   */
  const InputData& inputData() const { return input_data_; }
  /**
   * @brief 获取可修改的输入数据 (返回可修改的引用)
   * @return InputData& 可修改的输入数据
   */
  InputData& mutableInputData() { return input_data_; }

  /**
   * @brief 获取输出数据 (返回常量引用)
   */
  const OutputData& outputData() const { return output_data_; }
  /**
   * @brief 获取可修改的输出数据 (返回可修改的引用)
   */
  OutputData& mutableOutputData() { return output_data_; }

  /**
   * @brief 获取边界信息 (返回常量引用)
   */
  const BoundaryInfo& boundaryInfo() const { return boundary_info_; }
  /**
   * @brief 获取可修改的边界信息 (返回可修改的引用)
   */
  BoundaryInfo& mutableBoundaryInfo() { return boundary_info_; }

  /**
   * @brief 获取目标物信息 (返回常量引用)
   */
  const ObjectsInfo& objectsInfo() const { return objects_info_; }
  /**
   * @brief 获取可修改的目标物信息 (返回可修改的引用)
   */
  ObjectsInfo& mutableObjectsInfo() { return objects_info_; }

  /**
   * @brief 获取场景信息 (返回常量引用)
   */
  const ScenarioInfo& scenarioInfo() const { return scenario_info_; }
  /**
   * @brief 获取可修改的场景信息 (返回可修改的引用)
   */
  ScenarioInfo& mutableScenarioInfo() { return scenario_info_; }

  /**
   * @brief 获取优化器信息 (返回常量引用)
   */
  const OptimizerInfo& optimizerInfo() const { return optimizer_info_; }
  /**
   * @brief 获取可修改的优化器信息 (返回可修改的引用)
   */
  OptimizerInfo& mutableOptimizerInfo() { return optimizer_info_; }

  /**
   * @brief 获取配置数据 (返回常量引用)
   */
  const ConfigInfo& configInfo() const { return config_info_; }
  /**
   * @brief 获取可修改的配置数据 (返回可修改的引用)
   */
  ConfigInfo& mutableConfigInfo() { return config_info_; }

  /**
   * @brief 获取网格信息 (返回常量引用)
   * @return const GridsInfo& 不可修改的网格信息
   * @warning 原则上网格信息不可修改，不要返回可修改的引用！
   * @note 网格信息包含时间网格和速度网格等。
   */
  const GridsInfo& gridsInfo() const { return grids_info_; }

 private:
  InputData input_data_;
  OutputData output_data_;
  BoundaryInfo boundary_info_;
  ObjectsInfo objects_info_;
  ScenarioInfo scenario_info_;
  OptimizerInfo optimizer_info_;
  ConfigInfo config_info_;
  GridsInfo grids_info_;
};
}  // namespace gpal::pnc::planning
