/**
 * @file ocp_path_optimizer.cpp
 * @brief OCP路径优化器实现文件
 * @details
 * 该文件实现了基于最优控制问题（OCP）的路径优化器，用于在路径规划过程中对路径进行平滑和优化，确保路径的连续性和可行性。
 */

#include "path_planner/ocp_path_optimizer.h"
#include "ocp/ocp_model.h"
#include "base/singleton.h"
#include "util/timer.h"
#include "config_manager/config_manager.h"
#include <filesystem>
#include <fmt/chrono.h>

// #define LOG_OCP_DATA
// #define LOG_SUCCESS_STAGE
// #define LOG_FAILED_STAGE

namespace gpal::pnc::planning {

namespace {

/**
 * @brief 符号函数（带平滑处理）
 * @details 该函数用于计算带平滑处理的符号函数值。通过引入一个极小值eps来避免分母为零的情况，使得函数在零值附近平滑过渡。
 * 
 * @param[in] value 输入值，类型为double
 * @param[in] eps 平滑系数，用于避免分母为零，类型为double
 * 
 * @par 输入参数说明:
 * - value: 需要计算符号的值
 * - eps: 平滑系数，通常为一个极小的正数
 * 
 * @par 关键变量说明:
 * - 无显式关键变量
 * 
 * @par 计算公式:
 * - sgn(value) = value / (|value| + eps)
 * 
 * @par 流程图:
 * @startuml
 * start
 * :计算分母: |value| + eps;
 * :计算符号函数值: value / 分母;
 * stop
 * @enduml
 * 
 * @return double 带平滑处理的符号函数值
 * 
 * @note 该函数主要用于优化控制问题中，避免在零值附近出现不连续的情况
 * 
 * @warning eps值不宜过大，否则会影响符号函数的准确性
 */
double sgn(const double value, const double eps) { return value / (std::abs(value) + eps); }
}  // namespace

#ifdef LOG_OCP_DATA
static int debug_count_ = 0;
static OcpDataField ocp_data_field_;
#endif

/**
 * @brief 初始化OCP路径优化器
 * @details 该函数用于初始化OCP路径优化器，主要完成车辆配置、车辆参数和优化器配置的加载。通过单例模式获取配置管理器，并从中加载相关配置信息。
 * 
 * @par 输入参数说明:
 * - 无显式输入参数
 * 
 * @par 关键变量说明:
 * - config_manager (ConfigManager*): 配置管理器的单例实例
 * - vehicle_config_ (std::shared_ptr<VehicleConfig>): 车辆配置信息的智能指针
 * - vehicle_param_ (std::shared_ptr<VehicleParam>): 车辆参数信息的智能指针
 * - optimizer_config_ (OcpPathOptimizerConfig): OCP路径优化器配置
 * - curr_profile_type_ (std::string): 当前配置文件类型
 * - priority_name_map_ (std::map<std::string, int32_t>): 场景优先级映射表
 * - fallback_result_ (std::shared_ptr<PathData>): 备用路径数据
 * 
 * @par 初始化流程:
 * 1. 获取配置管理器的单例实例
 * 2. 从配置管理器中加载车辆配置信息
 * 3. 从配置管理器中加载车辆参数信息
 * 4. 从配置管理器中加载OCP路径优化器配置
 * 5. 初始化当前配置文件类型为空
 * 6. 调用reset()函数重置状态
 * 7. 清空场景优先级映射表
 * 8. 创建备用路径数据对象
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取配置管理器单例实例;
 * :加载车辆配置信息;
 * :加载车辆参数信息;
 * :加载OCP路径优化器配置;
 * :初始化当前配置文件类型;
 * :调用reset()重置状态;
 * :清空场景优先级映射表;
 * :创建备用路径数据对象;
 * :返回初始化成功状态;
 * stop
 * @enduml
 * 
 * @return bool 初始化是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在OCP路径优化器启动时调用，确保所有配置信息正确加载
 * 
 * @warning 需确保配置管理器已正确初始化，且包含有效的配置信息
 */
bool OcpPathOptimizer::ensureProfileType(const std::string& preferred_label) {
  // 如果已有有效的 profile，直接复用，不重置
  if (!curr_profile_type_.empty() &&
      optimizer_config_.profiles().find(curr_profile_type_) != optimizer_config_.profiles().end()) {
    return true;
  }
  // 尝试 preferred_label
  if (optimizer_config_.profiles().find(preferred_label) != optimizer_config_.profiles().end()) {
    curr_profile_type_ = preferred_label;
    return true;
  }
  // 兜底：取第一个可用 key
  if (!optimizer_config_.profiles().empty()) {
    curr_profile_type_ = optimizer_config_.profiles().begin()->first;
    ERT_PLOG_W << "[OcpPathOptimizer::ensureProfileType] preferred_label=\"" << preferred_label
               << "\" not found, falling back to \"" << curr_profile_type_ << "\"";
    return true;
  }
  ERT_PLOG_E << "[OcpPathOptimizer::ensureProfileType] profiles map is empty, cannot initialize profile type!";
  return false;
}

bool OcpPathOptimizer::init() {
  auto config_manager = Singleton<ConfigManager>::get_instance();
  vehicle_config_ = std::make_shared<VehicleConfig>(config_manager->vehicle_config());
  vehicle_param_ = std::make_shared<VehicleParam>(config_manager->vehicle_config().vehicle_param());
  optimizer_config_ = config_manager->getConfig<OcpPathOptimizerConfig>("OcpPathOptimizerConfig");
  
  curr_profile_type_ = "";
  reset();
  priority_name_map_.clear();
  fallback_result_ = std::make_shared<PathData>();
  return true;
}

/**
 * @brief 重置OCP路径优化器状态
 * @details 该函数用于重置OCP路径优化器的内部状态，主要清除各类路径边界信息，为下一次路径规划做准备。
 * 
 * @par 输入参数说明:
 * - 无显式输入参数
 * 
 * @par 关键变量说明:
 * - decision_path_boundary_ (std::vector<PathBoundary>): 决策路径边界信息
 * - barrier_path_boundary_ (std::vector<PathBoundary>): 障碍物路径边界信息
 * - soft_path_boundary_ (std::vector<PathBoundary>): 软约束路径边界信息
 * 
 * @par 重置流程:
 * 1. 清空决策路径边界信息
 * 2. 清空障碍物路径边界信息
 * 3. 清空软约束路径边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :清空决策路径边界信息;
 * :清空障碍物路径边界信息;
 * :清空软约束路径边界信息;
 * :返回重置成功状态;
 * stop
 * @enduml
 * 
 * @return bool 重置是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在每次路径规划开始前调用，确保优化器状态正确重置
 * 
 * @warning 需确保在调用该函数时，没有正在进行中的异步计算任务
 */
bool OcpPathOptimizer::reset() {
  decision_path_boundary_.clear();
  barrier_path_boundary_.clear();
  soft_path_boundary_.clear();
  return true;
}

/**
 * @brief 判断是否存在异步计算任务
 * @details 该函数用于判断当前是否存在正在进行的异步计算任务。通过检查async_data_指针是否为空来判断是否有异步任务。
 * 
 * @par 输入参数说明:
 * - 无显式输入参数
 * 
 * @par 关键变量说明:
 * - async_data_ (std::shared_ptr<SolverData>): 异步计算任务的数据指针
 * 
 * @par 判断逻辑:
 * - 如果async_data_不为空，则存在异步计算任务
 * - 如果async_data_为空，则不存在异步计算任务
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查async_data_是否为空;
 * if (async_data_ != nullptr?) then (是)
 *   :返回true;
 * else (否)
 *   :返回false;
 * endif
 * stop
 * @enduml
 * 
 * @return bool 是否存在异步计算任务，true表示存在，false表示不存在
 * 
 * @note 该函数应在需要判断是否有异步任务时调用，例如在开始新的计算任务之前
 * 
 * @warning 需确保async_data_指针的正确性，避免空指针异常
 */
bool OcpPathOptimizer::hasAsyncProcess() const { return async_data_ != nullptr; }

/**
 * @brief 丢弃异步计算任务
 * @details 该函数用于丢弃当前正在进行的异步计算任务。如果存在异步任务，会检查该任务是否与之前保存的模型信息一致，若一致则重置相关模型信息，最后释放异步任务数据。
 * 
 * @par 输入参数说明:
 * - 无显式输入参数
 * 
 * @par 关键变量说明:
 * - async_data_ (std::shared_ptr<SolverData>): 异步计算任务的数据指针
 * - prev_model_info_ (std::tuple<std::string, std::string, UpdateParamsMethod, std::shared_ptr<OptimalControlProblem>>): 保存的模型信息
 * 
 * @par 丢弃流程:
 * 1. 检查是否存在异步任务
 * 2. 如果存在异步任务，检查是否与之前保存的模型信息一致
 * 3. 若一致，则重置相关模型信息
 * 4. 释放异步任务数据
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查是否存在异步任务;
 * if (存在异步任务?) then (是)
 *   :检查是否与之前保存的模型信息一致;
 *   if (一致?) then (是)
 *     :重置相关模型信息;
 *   endif
 *   :释放异步任务数据;
 * endif
 * stop
 * @enduml
 * 
 * @return void
 * 
 * @note 该函数应在需要取消当前异步任务时调用，例如在开始新的计算任务之前
 * 
 * @warning 需确保在调用该函数时，没有其他线程正在访问异步任务数据
 */
void OcpPathOptimizer::dropAsyncProcess() {
  if (hasAsyncProcess()) {
    if (prev_model_info_.model == async_data_->model) {
      prev_model_info_.model.reset();
    }
    async_data_.reset();
  }
}

/**
 * @brief 预处理函数，为路径优化做准备
 * @details 该函数用于在路径优化前进行必要的预处理工作，包括初始化参考线、起点信息、路径边界等，并计算相关参数。
 * 
 * @param[in] reference_line 参考线信息
 * @param[in] start_point 规划起点信息
 * @param[in] boundary 路径边界信息
 * @param[in] prev_path 前一次规划的路径
 * 
 * @par 输入参数说明:
 * - reference_line: 参考线对象，包含参考线的基本信息
 * - start_point: 规划起点，包含位置、速度等信息
 * - boundary: 路径边界信息，包含决策边界、障碍物边界等
 * - prev_path: 前一次规划的路径，用于初始化优化器
 * 
 * @par 关键变量说明:
 * - target_ref_line_: 目标参考线
 * - xy_planning_start_point_: 规划起点的XY坐标
 * - sl_planning_start_point_: 规划起点的SL坐标
 * - decision_path_boundary_: 决策路径边界
 * - barrier_path_boundary_: 障碍物路径边界
 * - soft_path_boundary_: 软约束路径边界
 * - N_: 路径点数量
 * - ds_: 路径点间距
 * - accumulated_s_: 累积的s坐标
 * - curr_profile_type_: 当前配置文件类型
 * - lane_keep_start_s_: 车道保持起始s坐标
 * - prev_path_info_: 前一次路径信息
 * - lat_a_limit_: 横向加速度限制
 * - priority_name_map_: 场景优先级映射表
 * 
 * @par 预处理流程:
 * 1. 检查边界信息是否有效
 * 2. 初始化目标参考线和规划起点
 * 3. 重置并初始化各类路径边界
 * 4. 计算路径点数量和间距
 * 5. 计算累积的s坐标
 * 6. 根据边界标签获取当前配置文件类型
 * 7. 获取车道保持起始s坐标
 * 8. 计算前一次路径信息
 * 9. 初始化横向加速度限制
 * 10. 导入场景优先级信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查边界信息是否有效;
 * :初始化目标参考线和规划起点;
 * :重置并初始化各类路径边界;
 * :计算路径点数量和间距;
 * :计算累积的s坐标;
 * :根据边界标签获取当前配置文件类型;
 * :获取车道保持起始s坐标;
 * :计算前一次路径信息;
 * :初始化横向加速度限制;
 * :导入场景优先级信息;
 * :返回预处理结果;
 * stop
 * @enduml
 * 
 * @return bool 预处理是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在每次路径优化前调用，确保优化器状态正确初始化
 * 
 * @warning 需确保输入参数的有效性，特别是边界信息和前一次路径信息
 */
bool OcpPathOptimizer::preProcess(const ReferenceLine& reference_line, const TrajectoryPt& start_point,
                                  const PathBoundary& boundary, const DiscretizedPath& prev_path) {
  if (boundary.size() < 2 || boundary.delta_s() < 0.1) {
    ERT_PLOG_I << "[OcpPathOptimizer::preProcess]: bound_length is too short" ;
    return false;
  }

  target_ref_line_.reset(new ReferenceLine(reference_line));
  xy_planning_start_point_ = start_point;
  sl_planning_start_point_ = target_ref_line_->toFrenetFrame(start_point);
  
  // 获取各类边界(车尾起始，已去除半车宽）.
  reset();
  decision_path_boundary_.assign(boundary.decision_boundary().begin(), boundary.decision_boundary().end());
  barrier_path_boundary_.assign(boundary.barrier_boundary().begin(), boundary.barrier_boundary().end());
  soft_path_boundary_.assign(boundary.soft_boundary().begin(), boundary.soft_boundary().end());

  N_ = boundary.size() - 1;
  ds_ = boundary.delta_s();

  // 以自车当前位置s为起始，计算accumulated_s_
  accumulated_s_.clear();
  for (int i = 0; i <= N_; ++i) {
    accumulated_s_.emplace_back(sl_planning_start_point_.first[0] + i * ds_);
  }
  
  // 从边界label获取curr_profile_type_
  if (curr_profile_type_ != boundary.label()) {
    curr_profile_type_ = boundary.label();
    auto iter = optimizer_config_.profiles().find(curr_profile_type_);
    if (iter == optimizer_config_.profiles().end()) {
      curr_profile_type_ = "";
      return false;
    }
  }
  
  // 获取lane_keep_start_s_
  lane_keep_start_s_ = boundary.lane_keep_start_s();
  if (optimizer_config_.enable_use_aim_horizon_time()) {
    cost_weight_max_decay_length_ = std::max(xy_planning_start_point_.v() * optimizer_config_.aim_horizon_time(), optimizer_config_.cost_weight_max_decay_length());
  } else {
    cost_weight_max_decay_length_ = optimizer_config_.cost_weight_max_decay_length();
  }
  // 计算prev_path_info_信息: std::tuple<bool, double, PathPt>
  auto& [prev_path_valid, prev_ref_s, prev_start_pt] = prev_path_info_; 
  prev_path_valid = false;
  if (!prev_path.empty() && prev_model_info_.status != SolveStatus::INFEASIBLE_DETECTED) {
    constexpr double max_prev_path_err = 0.7;
    double min_dist = std::numeric_limits<double>::max();
    prev_start_pt = prev_path.getNearestPoint(start_point.path_pt(), min_dist);
    SLPoint sl_pt;
    if (min_dist < max_prev_path_err && target_ref_line_->xy2sl(prev_start_pt, &sl_pt)) {
      prev_ref_s = sl_pt.s();
      constexpr double max_tolerence_diff_s = 0.01;  // 防止小的投影误差，造成历史轨迹不可用
      if (accumulated_s_.front() - max_tolerence_diff_s < prev_ref_s && prev_ref_s < accumulated_s_.back() + max_tolerence_diff_s) {
        prev_path_valid = true;
      }
    };
  }

  lat_a_limit_ = {optimizer_config_.lateral_acc_limit_for_radius().lateral_acc_limit_for_radius_1000(),
                  optimizer_config_.lateral_acc_limit_for_radius().lateral_acc_limit_for_radius_500(),
                  optimizer_config_.lateral_acc_limit_for_radius().lateral_acc_limit_for_radius_250(),
                  optimizer_config_.lateral_acc_limit_for_radius().lateral_acc_limit_for_radius_100(),
                  optimizer_config_.lateral_acc_limit_for_radius().lateral_acc_limit_for_radius_50(),
                  optimizer_config_.lateral_acc_limit_for_radius().lateral_acc_limit_for_radius_20(),
                  optimizer_config_.lateral_acc_limit_for_radius().lateral_acc_limit_for_radius_10()};

  // 场景优先级信息导入.
  priority_name_map_.clear();
  for (const auto& tag : optimizer_config_.tag_priority()) {
    priority_name_map_.insert(tag);
  }
  // ERT_PLOG_I<<optimizer_config_.DebugString();
  return true;
}

/**
 * @brief 同步路径优化处理函数
 * @details 该函数用于执行同步路径优化计算，根据输入的参考线、起点和边界信息，生成优化后的路径数据。
 *
 * @param[in] reference_line 参考线信息
 * @param[in] start_point 规划起点信息
 * @param[in] boundary 路径边界信息
 * @param[out] path_data 输出的路径数据
 *
 * @par 输入参数说明:
 * - reference_line: 参考线对象，包含参考线的基本信息
 * - start_point: 规划起点，包含位置、速度等信息
 * - boundary: 路径边界信息，包含决策边界、障碍物边界等
 * - path_data: 输出的路径数据指针
 *
 * @par 关键变量说明:
 * - 无显式关键变量
 *
 * @par 处理流程:
 * 1. 直接返回成功状态
 *
 * @par 流程图:
 * @startuml
 * start
 * :返回成功状态;
 * stop
 * @enduml
 *
 * @return Status 处理状态，返回Status::OK表示成功
 *
 * @note 该函数目前是一个空实现，直接返回成功状态
 *
 * @warning 该函数需要根据具体需求实现路径优化逻辑
 */
Status OcpPathOptimizer::proc(const ReferenceLine& reference_line, const TrajectoryPt& start_point,
                              const PathBoundary& boundary, PathData* const path_data) {
  util::TimerLogger<std::milli> timer(
      "proc", [](const std::string& str) { ERT_PLOG_I << "[OcpPathOptimizer::proc]: " << str; });

  // 1) 前处理，失败则返回PREPROCESS_FAILED.
  {
    auto tap_timer = timer.tap("preProcess");
    if (!preProcess(reference_line, start_point, boundary, path_data->discretized_path())) {
      return Status(ErrorCode::PLANNING_ERROR, "preProcess faield");
    }
  }

  // 2.  get the ocp model, set the stage
  // auto tap_timer = timer.tap("solver");
  std::shared_ptr<OptimalControlProblem> model = nullptr;
  {
    auto init_tap_timer = timer.tap("initModel");
    model = initModel(profile(), path_data->discretized_path());
    tranStatesToPathData(model, reference_line, fallback_result_);
  }
#ifdef LOG_OCP_DATA
  // log data if need to debug solver
  if (model->logData(&ocp_data_field_) < 100) {
    std::filesystem::path buffer = std::filesystem::path(__FILE__).parent_path();
    std::string direction = buffer;
    std::ofstream of(fmt::format("{}_data_{}.bin", ocp_data_field_.model(), debug_count_++));
    if (of) {
      google::protobuf::io::OstreamOutputStream ofs(&of);
      ocp_data_field_.SerializeToOstream(&of);
      ocp_data_field_.clear_data();
      of.close();
    }
  }
#endif

  // 3. solve the problem
  auto solve_tap_timer = timer.tap("solve");
  auto status = model->solve();
  ERT_PLOG_D << "[OcpPathOptimizer::proc] profile: " << curr_profile_type_ << " status: " << status << " iter: " << model->solver()->iteration_number()
            << " computation_time: " << model->solver()->getSolveInfo().computation_time * 1000.0 << " first_order: " << model->solver()->getSolveInfo().first_order_condition
            << " complementary: " << model->solver()->getSolveInfo().complementary_condtion << " equality: " << model->solver()->getSolveInfo().equality_condition
            << " inequality: " << model->solver()->getSolveInfo().inequality_condition;
  {
    auto finish_tap_timer = timer.tap("postProcess");
    prev_model_info_ = {reference_line.id(), curr_profile_type_, curr_update_params_method_, model, status};
    solver_info_ = model->solver()->getSolveInfo();
    if (status == SolveStatus::SOLVED) {
      if (!tranStatesToPathData(model, reference_line, boundary, path_data)) {
        return Status(ErrorCode::PLANNING_ERROR, "tranStatesToPathData failed");
      }
    } else {
#ifdef LOG_FAILED_STAGE
      auto states = model->getX();
      auto params = model->getParam();
      for (int i = 0; i < states.size(); i++) {
        ERT_PLOG_D << "[OcpPathOptimizer::proc]: " << states[i].debugString() << ", " << params[i].debugString();
      }
#endif
      return Status(ErrorCode::PLANNING_ERROR, "ipm solver failed");
    }
  }
  return Status::OK();
}

/**
 * @brief 异步路径优化处理函数
 * @details 该函数用于执行异步路径优化计算，根据输入的参考线、起点和边界信息，生成优化后的路径数据。该函数采用异步方式执行优化计算，并支持超时机制。
 * 
 * @param[in] timeout 超时时间，单位为毫秒
 * @param[in] reference_line 参考线信息
 * @param[in] start_point 规划起点信息
 * @param[in] boundary 路径边界信息
 * @param[out] path_data 输出的路径数据
 * 
 * @par 输入参数说明:
 * - timeout: 异步计算的最大等待时间
 * - reference_line: 参考线对象，包含参考线的基本信息
 * - start_point: 规划起点，包含位置、速度等信息
 * - boundary: 路径边界信息，包含决策边界、障碍物边界等
 * - path_data: 输出的路径数据指针
 * 
 * @par 关键变量说明:
 * - async_data_: 异步计算任务的数据指针
 * - async_planner_debug_info_: 异步计算的调试信息
 * - prev_model_info_: 保存的模型信息
 * - fallback_result_: 备用路径数据
 * 
 * @par 处理流程:
 * 1. 初始化计时器
 * 2. 执行预处理，若失败则返回PREPROCESS_FAILED状态
 * 3. 准备异步计算数据
 * 4. 初始化模型
 * 5. 启动异步计算任务
 * 6. 等待异步计算完成或超时
 * 7. 执行后处理，包括结果验证和路径数据转换
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化计时器;
 * :执行预处理;
 * if (预处理成功?) then (是)
 *   :准备异步计算数据;
 *   :初始化模型;
 *   :启动异步计算任务;
 *   :等待异步计算完成或超时;
 *   :执行后处理;
 * else (否)
 *   :返回PREPROCESS_FAILED状态;
 * endif
 * stop
 * @enduml
 * 
 * @return AsyncStatus 异步处理状态，包括ASYNC_SOLVED、PREPROCESS_FAILED等
 * 
 * @note 该函数应在需要异步路径优化时调用，确保优化器状态正确初始化
 * 
 * @warning 需确保输入参数的有效性，特别是边界信息和前一次路径信息
 */
OcpPathOptimizer::AsyncStatus OcpPathOptimizer::asyncProc(const std::chrono::milliseconds& timeout, 
                                                          const ReferenceLine& reference_line, 
                                                          const TrajectoryPt& start_point,
                                                          const PathBoundary& boundary, 
                                                          PathData* const path_data) {
  util::TimerLogger<std::milli> timer("asyncProc", [](const std::string& str) { ERT_PLOG_I << "[OcpPathOptimizer::asyncProc]: " << str; });
  
  AsyncStatus status = AsyncStatus::ASYNC_SOLVED;
  async_planner_debug_info_ = "";
  
  // 1) 前处理，失败则返回PREPROCESS_FAILED.
  {
    auto tap_timer = timer.tap("preProcess");
    if (!preProcess(reference_line, start_point, boundary, path_data->discretized_path())) {
      status = AsyncStatus::PREPROCESS_FAILED;
      return status;
    }
  }

  std::vector<OcpVariable> init_guess;
  if (async_data_ == nullptr) {
    auto async_timer = timer.tap("prepareData");
    auto async_data = std::make_shared<SolverData>();
    async_data->ref_id = reference_line.id();
    async_data->profile = curr_profile_type_; // 来源于边界label类型信息
    // 2) initModel.
    {
      auto tap_timer = async_timer.tap("initModel");
      async_data->model = initModel(profile(), path_data->discretized_path());
    }
    init_guess = async_data->model->getX();
    tranStatesToPathData(async_data->model, reference_line, fallback_result_);
#ifdef LOG_OCP_DATA
    if (async_data->model->logData(&ocp_data_field_) > 10) {
      std::filesystem::path buffer = std::filesystem::path(__FILE__).parent_path();
      std::string direction = buffer;
      std::ofstream of(direction + fmt::format("{}_data_{}.bin", ocp_data_field_.model(), debug_count_++));
      if (of)
      {
        google::protobuf::io::OstreamOutputStream ofs(&of);
        ocp_data_field_.SerializeToOstream(&of);
        ocp_data_field_.clear_data();
        of.close();
      }
    }
#endif
    
    // 3) ocp实时规划(独立线程).
    {
      auto tap_timer = async_timer.tap("importTask");
      task_handler_.importTask([=] {
        util::TimerLogger<std::milli> timer(async_data->model->name(), [=](const std::string& str) {
        ERT_PLOG_I << "[bypassaaa][OcpPathOptimizer::asyncProc]: profile = " << async_data->profile
                  << "  ipm_solver status = " << async_data->status 
                  << "  iter = " << async_data->model->solver()->iteration_number()
                  << "  computation_ms = " << async_data->model->solver()->getSolveInfo().computation_time * 1000.0
                  << "  first_order = " << async_data->model->solver()->getSolveInfo().first_order_condition
                  << "  complementary = " << async_data->model->solver()->getSolveInfo().complementary_condtion
                  << "  equality = " << async_data->model->solver()->getSolveInfo().equality_condition
                  << "  inequality = " << async_data->model->solver()->getSolveInfo().inequality_condition
                  ;
        });
        async_data->status = async_data->model->solve();
        async_data->is_finished = true;
        // 通知所有等待的线程.
        async_data->cond.notify_all();
       });
      async_data_ = async_data;
    }
  }
  
  // 4）仍在求解中.
  {
    auto tap_timer = timer.tap("underlockSolve");
    std::unique_lock<std::mutex> lck(async_data_->mutex);
    // 阻塞当前线程，直到条件变量被唤醒，或到制定时限时长后.
    async_data_->cond.wait_for(lck, std::chrono::milliseconds(timeout), [=]() {
      bool is_finished = async_data_->is_finished;
      return is_finished;
    });
  }
  
  // 5) 后处理.
  {
    auto tap_timer = timer.tap("postProcess");
    async_planner_debug_info_ = fmt::format(
        "\nprofile:{}, ipm_solver:{}, iter:{}, time:{:.2f}, first_order:{:.2e}, complementary:{:.2e}, equality:{:.2e}, inequality:{:.2e}, finish:{}",
        async_data_->profile, static_cast<int>(async_data_->status), async_data_->model->solver()->iteration_number(),
        async_data_->model->solver()->getSolveInfo().computation_time * 1000.0,
        async_data_->model->solver()->getSolveInfo().first_order_condition,
        async_data_->model->solver()->getSolveInfo().complementary_condtion,
        async_data_->model->solver()->getSolveInfo().equality_condition,
        async_data_->model->solver()->getSolveInfo().inequality_condition,
        static_cast<int>(async_data_->is_finished));
    // 明确哪些约束不满足:
    // refer to: ipm_ocp_solver.cpp->calcOptimalityCondition()->infeasible_index_(满足stages_[i].g(j) > 1e-4 &&
    // !stages_[i].isConstraintsHasSlack(j)) 1) LateralGeneralConstraints
    //   values(0) = -l + ll;
    //      values(1) = l - lu;
    //      values(2) = -blf + lfl;
    //      values(3) = blf - lfu;
    //      values(4) = -blr + lrl;
    //      values(5) = blr - lru;
    //      values(6) = KappaLowerBound - kappa;
    //      values(7) = -KappaUpperBound + kappa;
    //      values(8) = -cos_anonymous_0;
    //      values(9) = DKappaLowerBound - dkappa;
    //      values(10) = -DKappaUpperBound + dkappa;
    //      values(11) = -l - slack_offset + sll;
    //      values(12) = l - slack_offset - slu;
    //      values(13) = -blf - slack_offset + slfl;
    //      values(14) = blf - slack_offset - slfu;
    //      values(15) = -blr - slack_offset + slrl;
    //      values(16) = blr - slack_offset - slru;

    // 2) LateralGeneralConstraints<StageType::INITIAL>
    //      values(0) = DKappaLowerBound - dkappa;
    //      values(1) = -DKappaUpperBound + dkappa;

    //  3) LateralGeneralConstraints<StageType::TERMINAL>
    //      values(0) = -l + ll;
    //      values(1) = l - lu;
    //      values(2) = -blf + lfl;
    //      values(3) = blf - lfu;
    //      values(4) = -blr + lrl;
    //      values(5) = blr - lru;
    //      values(6) = KappaLowerBound - kappa;
    //      values(7) = -KappaUpperBound + kappa;
    //      values(8) = -cos_anonymous_0;

    //  4) LateralGeneralStateOnlyEqualities<StageType::INITIAL>
    //      values(0) = s;
    //      values(1) = x;
    //      values(2) = y;
    //      values(3) = theta;
    //      values(4) = kappa;
    for (auto& index : async_data_->model->solver()->getSolveInfo().infeasible_index) {
      async_planner_debug_info_ += "\ninfeasible:{" + std::to_string(std::get<0>(index)) + ", " +
                                   std::to_string(std::get<1>(index)) + ", " + std::to_string(std::get<2>(index)) +
                                   ", " + std::to_string(std::get<3>(index)) + ", " +
                                   fmt::format("{:.3e}", std::get<4>(index)) + "}";
    }

    if (async_data_->is_finished) {
      prev_model_info_ = {async_data_->ref_id, async_data_->profile, curr_update_params_method_, async_data_->model, async_data_->status};
      checkValid(async_data_);

      for (const auto& it : async_data_->debug_status) {
        path_data->mutablePlannerDebugStatus()->push_back(it);
      }      
      fillOcpPathData(async_data_->model, boundary, path_data);
      if (async_data_->status == SolveStatus::SOLVED) {
#ifdef LOG_SUCCESS_STAGE
        auto states = async_data_->model->getX();
        auto params = async_data_->model->getParam();
        for (int i = 0; i < states.size(); ++i) {
          ERT_PLOG_I << "[OcpPathOptimizer::asyncProc]: " << states[i].debugString() << "\n"
                    << params[i].debugString() ;
        }
#endif

        if (!tranStatesToPathData(async_data_->model, reference_line, boundary, path_data)) {
          return AsyncStatus::ASYNC_ERROR_SOLVED;
        }
      } else {
#ifdef LOG_FAILED_STAGE
        auto states = async_data_->model->getX();
        auto params = async_data_->model->getParam();
        for (int i = 0; i < states.size(); ++i) {
          ERT_PLOG_I << "[OcpPathOptimizer::asyncProc]: " << states[i].debugString()
                    << "\n" << params[i].debugString() ;
        }
#endif
        if (async_data_->status == SolveStatus::INFEASIBLE_DETECTED) {
          status = AsyncStatus::INFEASIBLED;
        } else {
          status = AsyncStatus::ASYNC_FAILED;
        }
      }
      
      setSolverInfo(async_data_->model->solver()->getSolveInfo());
      async_data_.reset();
    } else {
      status = AsyncStatus::ASYNC_UNDERLOCKING;
    }
  }

  return status;
}

/**
 * @brief 初始化优化控制模型
 * @details 该函数用于初始化优化控制模型，根据配置文件和前一次路径信息创建或重用模型，并设置相关参数。
 * 
 * @param[in] profile 优化器配置文件
 * @param[in] prev_path 前一次规划的路径
 * 
 * @par 输入参数说明:
 * - profile: 优化器配置文件，包含模型类型、默认参数等信息
 * - prev_path: 前一次规划的路径，用于模型初始化
 * 
 * @par 关键变量说明:
 * - model: 优化控制模型指针
 * - curr_update_params_method_: 当前参数更新方法
 * - prev_model_info_: 保存的模型信息
 * - ds_: 路径点间距
 * - N_: 路径点数量
 * - optimizer_config_: 优化器配置
 * 
 * @par 初始化流程:
 * 1. 检查是否存在有效的先前模型
 * 2. 如果不存在，则创建新模型并初始化
 * 3. 如果存在，则重用先前模型并根据需要重置
 * 4. 设置模型参数
 * 5. 初始化模型
 * 6. 更新模型状态
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查是否存在有效的先前模型;
 * if (存在有效模型?) then (是)
 *   :重用先前模型;
 *   :检查是否需要重置;
 *   if (需要重置?) then (是)
 *     :重置模型;
 *   endif
 * else (否)
 *   :创建新模型;
 *   :初始化模型;
 * endif
 * :设置模型参数;
 * :初始化模型;
 * :更新模型状态;
 * stop
 * @enduml
 * 
 * @return std::shared_ptr<OptimalControlProblem> 初始化后的优化控制模型
 * 
 * @note 该函数应在每次路径优化前调用，确保模型状态正确初始化
 * 
 * @warning 需确保输入参数的有效性，特别是配置文件和前一次路径信息
 */
std::shared_ptr<OptimalControlProblem> OcpPathOptimizer::initModel(const OcpPathOptimizerProfile& profile,
                                                                   const DiscretizedPath& prev_path) {
  std::shared_ptr<OptimalControlProblem> model;
  curr_update_params_method_ = UpdateParamsMethod::LATERAL_GENERAL;
  if (!hasValidPrevModel()) {
    model = OptimalControlProblem::create(profile.model());
    if (model != nullptr) {
      model->mutable_config()->set_integrator_type(OcpConfig::ERK4);
      model->mutable_config()->set_dt(ds_);
      model->mutable_config()->set_horizon_length(N_);
      for (const auto& param : profile.default_params()) {
        model->setParam(param.key(), param.value());
      }
      if (profile.has_ipm_config()) {
        model->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(profile.ipm_config());
      } else {
        model->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(optimizer_config_.ipm_config());
      }
    }
    if (model->name() == "LateralGeneral") {
      initLateralGeneral(model);
    } 
    model->init();
  } else {
    ERT_PLOG_I << "[OcpPathOptimizer::initModel]: warm start" ;
    auto prev_profile = prev_model_info_.profile;
    model = prev_model_info_.model;
    for (const auto& param : profile.default_params()) {
      model->setParam(param.key(), param.value());
    }
    if (prev_profile != curr_profile_type_ || model->N() != N_ || target_ref_line_->has_change_reference()) {
      ERT_PLOG_I <<"[OcpPathOptimizer::initModel]: warm start reset, profile: " << prev_profile
                << "->" << curr_profile_type_
                << "  N: " << model->N()
                << "->"<< N_
                << "  has_change_reference = "<< target_ref_line_->has_change_reference() ;
      if (profile.has_ipm_config()) {
        model->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(profile.ipm_config());
      }
      model->mutable_config()->set_dt(ds_);
      model->mutable_config()->set_horizon_length(N_);
      model->init();
    } else if (std::abs(model->config().dt() - ds_) > 1e-6) {
      ERT_PLOG_I <<"[OcpPathOptimizer::initModel]: warm start reset dt, dt: " << model->config().dt() << "->" << ds_ ;
      model->setDt(ds_);
    }
  }

  if (model->name() == "LateralGeneral") {
    updateLateralGeneral(model, prev_path);
  }

  return model;
}

/**
 * @brief 初始化横向通用模型
 * @details 该函数用于初始化横向通用模型，设置车辆参数和模型参数，并为每个路径点计算相关约束条件。
 * 
 * @param[in] model 优化控制模型指针
 * 
 * @par 输入参数说明:
 * - model: 优化控制模型指针，用于设置模型参数
 * 
 * @par 关键变量说明:
 * - vehicle_param_: 车辆参数，包含前后轴到质心的距离等信息
 * - sl_planning_start_point_: 规划起点的SL坐标
 * - xy_planning_start_point_: 规划起点的XY坐标
 * - profile(): 优化器配置文件
 * 
 * @par 初始化流程:
 * 1. 设置车辆参数，包括前后轴到质心的距离和轴距
 * 2. 定义模型参数计算函数，为每个路径点计算相关约束条件
 * 3. 计算参考点、前轴参考点和后轴参考点
 * 4. 设置横向约束条件，包括软约束和硬约束
 * 5. 设置转向角约束和转向角变化率约束
 * 6. 根据场景优先级设置相关参数
 * 
 * @par 流程图:
 * @startuml
 * start
 * :设置车辆参数;
 * :定义模型参数计算函数;
 * :计算参考点、前轴参考点和后轴参考点;
 * :设置横向约束条件;
 * :设置转向角约束和转向角变化率约束;
 * :根据场景优先级设置相关参数;
 * stop
 * @enduml
 * 
 * @return bool 初始化是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在每次路径优化前调用，确保模型参数正确初始化
 * 
 * @warning 需确保输入参数的有效性，特别是车辆参数和规划起点信息
 */
bool OcpPathOptimizer::initLateralGeneral(std::shared_ptr<OptimalControlProblem> model) {
  model->setParam("lf", vehicle_param_->front_edge_to_ego());
  model->setParam("lr", vehicle_param_->rear_edge_to_ego());
  model->setParam("wheelbase", vehicle_param_->wheel_base());
  const double constaint_ignore_buffer = vehicle_param_->front_edge_to_ego();
  auto linearInterpolation = [](double i, double N, double value_lower, double value_upper) -> double {
    if (std::abs(N) < 1e-9) return value_lower;

    const double step = (value_upper - value_lower) / N;
    double result = value_lower + i * step;

    // 边界截断功能
    return std::clamp(result, min(value_lower, value_upper), max(value_lower, value_upper));
  };
  model->setParam(
      [=](const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double& t, const int& k, OcpVariable* ptr) {
        double ego_s = sl_planning_start_point_.first[0];
        double curr_s = x(0);
        auto ref_point = getInitPoint(curr_s);  // according to s
        auto ref_point_direction = getInitPointDrivingDirection(curr_s);
        auto kf_ref_point = getInitPoint(ref_point.x() + vehicle_param_->front_edge_to_ego() * cos(ref_point.heading()), ref_point.y() + vehicle_param_->front_edge_to_ego() * sin(ref_point.heading()));
        auto kr_ref_point = getInitPoint(ref_point.x() - vehicle_param_->rear_edge_to_ego() * cos(ref_point.heading()), ref_point.y() - vehicle_param_->rear_edge_to_ego() * sin(ref_point.heading()));
        double kf = kf_ref_point.local_s();
        double kr = kr_ref_point.local_s();
        auto kf_direction = target_ref_line_->getDirectionFromS(kf).direction;
        auto kr_direction = target_ref_line_->getDirectionFromS(kr).direction;
        ptr->set("xr", ref_point.x());
        ptr->set("yr", ref_point.y());
        ptr->set("thetar", getUnifySpaceHeading(x(3), ref_point.heading()));
        ptr->set("kr", ref_point.kappa());
        ptr->set("xrf", kf_ref_point.x());
        ptr->set("yrf", kf_ref_point.y());
        ptr->set("thetarf", getUnifySpaceHeading(x(3), kf_ref_point.heading()));
        ptr->set("xrr", kr_ref_point.x());
        ptr->set("yrr", kr_ref_point.y());
        ptr->set("thetarr", getUnifySpaceHeading(x(3), kr_ref_point.heading()));
        if (k > 0) {
          // barrier
          auto barrier_res = getBarrierCenterBound(ref_point);
          auto& [barrier_bound, barrier_bound_valid] = barrier_res;
          if (curr_s > ego_s + constaint_ignore_buffer && barrier_bound_valid) {
            auto [s, lower, upper] = barrier_bound;
            ptr->set("ll", lower);
            ptr->set("lu", upper);
          } else {
            ptr->set("ll", -20.0);
            ptr->set("lu", 20.0);
          }
          // soft
          auto soft_res = getSoftCenterBound(curr_s);
          auto& [soft_bound, soft_bound_valid] = soft_res;
          if (curr_s > ego_s + constaint_ignore_buffer && soft_bound_valid) {
            auto [s, lower, upper] = soft_bound;
            ptr->set("sll", lower);
            ptr->set("slu", upper);
          } else {
            ptr->set("sll", -20.0);
            ptr->set("slu", 20.0);
          }


          if (profile().enb_edge_barrier_bound()) {
            auto f_barrier_res = getBarrierEdgeBound(kf_ref_point);
            auto& [f_barrier_bound, f_barrier_bound_valid] = f_barrier_res;
            if (curr_s > ego_s + constaint_ignore_buffer && f_barrier_bound_valid 
                && kf_direction != DrivingDirection::kDirectionUTurnOnly
                && kf_direction != DrivingDirection::kDirectionLeftOnly
                && kf_direction != DrivingDirection::kDirectionRightOnly
                && !isInRange(getSpecialSceneRange(), "Narrow", kf, 10.0)) {
              auto [s, lower, upper] = f_barrier_bound;
              ptr->set("lfl", lower);
              ptr->set("lfu", upper);
            } else {
              ptr->set("lfl", -20.0);
              ptr->set("lfu", 20.0);
            }
            auto r_barrier_res = getBarrierEdgeBound(kr_ref_point);
            auto& [r_barrier_bound, r_barrier_bound_valid] = r_barrier_res;
            if (curr_s > ego_s + constaint_ignore_buffer && r_barrier_bound_valid
               && kr_direction != DrivingDirection::kDirectionUTurnOnly
               && kr_direction != DrivingDirection::kDirectionLeftOnly
               && kr_direction != DrivingDirection::kDirectionRightOnly
               && !isInRange(getSpecialSceneRange(), "Narrow", kr, 10.0)) {
              auto [s, lower, upper] = r_barrier_bound;
              ptr->set("lrl", lower);
              ptr->set("lru", upper);
            } else {
              ptr->set("lrl", -20.0);
              ptr->set("lru", 20.0);
            }
          }
          if (profile().enb_edge_soft_bound()) {           
            auto f_soft_res = getSoftEdgeBound(kf);
            auto& [f_soft_bound, f_soft_bound_valid] = f_soft_res;
            if (curr_s > ego_s + constaint_ignore_buffer && f_soft_bound_valid) {
              auto [s, lower, upper] = f_soft_bound;
              ptr->set("slfl", lower);
              ptr->set("slfu", upper);
            } else {
              ptr->set("slfl", -20.0);
              ptr->set("slfu", 20.0);
            }
            auto r_soft_res = getSoftEdgeBound(kr);
            auto& [r_soft_bound, r_soft_bound_valid] = r_soft_res;
            if (curr_s > ego_s + constaint_ignore_buffer && r_soft_bound_valid) {
              auto [s, lower, upper] = r_soft_bound;
              ptr->set("slrl", lower);
              ptr->set("slru", upper);
            } else {
              ptr->set("slrl", -20.0);
              ptr->set("slru", 20.0);
            }
          }
        }

        auto [ref_offset, ref_offset_valid] = getRefOffset(ref_point);
        if (ref_offset_valid) {
          auto [s, l_offset] = ref_offset;
          ptr->set("l_offset", l_offset);
        }

        if (curr_s >= lane_keep_start_s_) {
          ptr->set("l_weight", linearInterpolation(k, cost_weight_max_decay_length_, 1e-3, profile().weights().l_ref_weight_piecewise()));
        }

        double const_a_speed = std::sqrt(xy_planning_start_point_.v() * xy_planning_start_point_.v() + 2.0 * starting_acc_ * k);
        double speed_limit = getSpeedLimit(curr_s,ref_point.kappa());
        const double steer_angle_bound = getMaxSteerAngleBound(t, std::min(const_a_speed, speed_limit), ref_point_direction);
        ptr->set("SteerLowerBound", -steer_angle_bound);
        ptr->set("SteerUpperBound", steer_angle_bound);
        const double dsteer_angle_bound = getMaxDSteerAngleBound(t, std::min(const_a_speed, speed_limit), ref_point_direction);
        ptr->set("DSteerLowerBound", -dsteer_angle_bound);
        ptr->set("DSteerUpperBound", dsteer_angle_bound);
        // ERT_PLOG_I<<"   s    "<<curr_s<<"   speed    "<<  std::min(const_a_speed, speed_limit) * MS_KMH  <<"    steer_angle_bound  = " << steer_angle_bound << " dsteer_angle_bound = " << dsteer_angle_bound;
        string valid_tag = "";
        for (auto& [tag, start_s, end_s] : getSpecialSceneRange()) {
          if (x(0) >= start_s && x(0) <= end_s) {
            int32_t priority_level = 0;
            if (priority_name_map_.count(tag) > 0) {
              priority_level = priority_name_map_[tag];
              if (valid_tag == "") {
                valid_tag = tag;
              } else {
                if (priority_level < priority_name_map_[valid_tag]) {
                  valid_tag = tag;
                }
              }
            }
          }
        }
        for (auto& param : profile().ranged_params()) {
          if (param.range_key() == valid_tag) {
            ptr->set(param.key(), param.value());
          }
        }
      });
  return true;
}

/**
 * @brief 更新横向通用模型
 * @details 该函数用于更新横向通用模型的状态和参数，根据前一次路径信息和当前规划起点，设置模型的初始状态和猜测值。
 * 
 * @param[in] model 优化控制模型指针
 * @param[in] prev_path 前一次规划的路径
 * 
 * @par 输入参数说明:
 * - model: 优化控制模型指针，用于更新模型状态和参数
 * - prev_path: 前一次规划的路径，用于模型更新
 * 
 * @par 关键变量说明:
 * - sl_planning_start_point_: 规划起点的SL坐标
 * - xy_planning_start_point_: 规划起点的XY坐标
 * - prev_path_info_: 前一次路径信息
 * - profile(): 优化器配置文件
 * 
 * @par 更新流程:
 * 1. 设置模型的初始状态
 * 2. 如果存在有效的前一次路径，则根据前一次路径更新模型猜测值
 * 3. 如果不存在有效的前一次路径，则根据初始点信息更新模型猜测值
 * 4. 应用自行车横向控制策略
 * 
 * @par 流程图:
 * @startuml
 * start
 * :设置模型的初始状态;
 * if (存在有效的前一次路径?) then (是)
 *   :根据前一次路径更新模型猜测值;
 * else (否)
 *   :根据初始点信息更新模型猜测值;
 * endif
 * :应用自行车横向控制策略;
 * stop
 * @enduml
 * 
 * @return bool 更新是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在每次路径优化前调用，确保模型状态正确更新
 * 
 * @warning 需确保输入参数的有效性，特别是前一次路径信息和规划起点信息
 */
bool OcpPathOptimizer::updateLateralGeneral(std::shared_ptr<OptimalControlProblem> model,
                                            const DiscretizedPath& prev_path) {
  Eigen::VectorXd start_state = Eigen::VectorXd::Zero(5);
  start_state << sl_planning_start_point_.first.at(0), xy_planning_start_point_.path_pt().x(),
      xy_planning_start_point_.path_pt().y(), xy_planning_start_point_.path_pt().theta(),
      xy_planning_start_point_.path_pt().front_steer();
  model->setX0(start_state);
  double prev_heading = xy_planning_start_point_.path_pt().theta();
  if (hasValidPrevPath()) {
    double weight_pre_steer = profile().weights().prev_steer();
    double gamma_weight_prev_steer = profile().weights().gamma_weight_prev_steer();
    auto& [prev_path_valid, prev_ref_s, prev_start_pt] = prev_path_info_;
    for (int i = 1; i <= model->N(); ++i) {
      model->updateParams(i - 1);
      double prev_s = prev_start_pt.s() + model->t(i);
      if (prev_s > prev_path.back().s()) {
        applyBicycleLateralCtrlPolycy(model, i);
      } else {
        auto prev_pt = prev_path.evaluate(prev_s);
        weight_pre_steer *= gamma_weight_prev_steer;
        model->setParam("prev_steer", prev_pt.front_steer(), i);
        model->setParam("weight_prev_steer", weight_pre_steer, i);
        if (target_ref_line_->has_change_reference()) {
          applyBicycleLateralCtrlPolycy(model, i);
        } else{
          double ref_s = prev_pt.ref_s() - prev_start_pt.ref_s() + sl_planning_start_point_.first.at(0);
          model->setXGuess("s", ref_s, i);
          model->setXGuess("x", prev_pt.x(), i);
          model->setXGuess("y", prev_pt.y(), i);
          model->setXGuess("theta", getUnifySpaceHeading(prev_heading, prev_pt.theta()), i);
          model->setXGuess("steer", prev_pt.front_steer(), i);
        }
      }
      prev_heading = model->x(i, "theta");
    }
    model->updateParams(model->N());
  } else {
    // no valid prev path
    auto ptr0 = getInitPoint(accumulated_s_[0]);
    auto ptr1 = getInitPoint(accumulated_s_[1]);
    double v0x = start_state(1) - ptr0.x();
    double v0y = start_state(2) - ptr0.y();
    double v1x = ptr1.x() - ptr0.x();
    double v1y = ptr1.y() - ptr0.y();
    constexpr double angle = 1.0 * ANG2RAD;

    for (size_t i = 1; i <= model->N(); i++) {
      model->updateParams(i - 1);
      applyBicycleLateralCtrlPolycy(model, i);
    }
    model->updateParams(model->N());
  }
  return true;
}

/**
 * @brief 应用自行车横向控制策略
 * @details 该函数用于应用自行车横向控制策略，根据当前状态和参考点信息，计算并设置模型的猜测值和控制输入。
 * 
 * @param[in] model 优化控制模型指针
 * @param[in] idx 当前路径点的索引
 * 
 * @par 输入参数说明:
 * - model: 优化控制模型指针，用于更新模型状态和参数
 * - idx: 当前路径点的索引，用于设置模型的猜测值和控制输入
 * 
 * @par 关键变量说明:
 * - x0: 当前路径点的状态
 * - p0: 当前路径点的参数
 * - t0: 当前路径点的时间
 * - lat_err_thrd: 横向误差阈值
 * - heading_err_thrd: 航向误差阈值
 * - steer_bound: 转向角限制
 * - dsteer_bound: 转向角变化率限制
 * - kappa_bound: 曲率限制
 * 
 * @par 控制策略流程:
 * 1. 计算当前状态与参考点之间的横向误差和航向误差
 * 2. 如果误差小于阈值，则直接设置模型的猜测值
 * 3. 如果误差大于阈值，则计算目标曲率和转向角变化率
 * 4. 设置模型的猜测值和控制输入
 * 
 * @par 流程图:
 * @startuml
 * start
 * :计算横向误差和航向误差;
 * if (误差小于阈值?) then (是)
 *   :设置模型的猜测值;
 * else (否)
 *   :计算目标曲率和转向角变化率;
 *   :设置模型的猜测值和控制输入;
 * endif
 * stop
 * @enduml
 * 
 * @return bool 控制策略是否成功应用，true表示成功，false表示失败
 * 
 * @note 该函数应在每次路径优化前调用，确保模型状态正确更新
 * 
 * @warning 需确保输入参数的有效性，特别是模型状态和参考点信息
 */
bool OcpPathOptimizer::applyBicycleLateralCtrlPolycy(std::shared_ptr<OptimalControlProblem> model,
                                                     const size_t idx) {
  // todo: change dynamic model @jiangxueliang
  constexpr double lat_err_thrd = 0.1;
  constexpr double heading_err_thrd = 1.0 * ANG2RAD;

  auto& x0 = model->x(idx - 1);
  auto& p0 = model->p(idx - 1);
  double t0 = model->t(idx - 1);

  const double dx = x0(1) - p0("xr");
  const double dy = x0(2) - p0("yr");
  const double l = -sin(p0("thetar")) * dx + cos(p0("thetar")) * dy;
  const double heading_err = math::NormalizeAngle(x0(3) - p0("thetar"));
  if (std::abs(l) < lat_err_thrd && std::abs(heading_err) < heading_err_thrd) {
    model->setXGuess("s", accumulated_s_[idx], idx);
    model->setXGuess("x", p0("xr"), idx);
    model->setXGuess("y", p0("yr"), idx);
    model->setXGuess("theta", getUnifySpaceHeading(x0(3), p0("thetar")), idx);
  } else {
    DrivingDirection ref_point_direction = getInitPointDrivingDirection(x0(0));
    double steer_bound = getMaxSteerAngleBound(t0, xy_planning_start_point_.v(), ref_point_direction);
    double dsteer_bound = getMaxDSteerAngleBound(t0, xy_planning_start_point_.v(), ref_point_direction);
    const double kappa_bound = tan(steer_bound) / vehicle_param_->wheel_base();
    double cur_psif = x0("steer");
    const double temp = 1.0f - p0("kr") * l;
    double target_kappa = 0;
    const double tau = 1.0;
    if (std::abs(temp) > 1e-6) {
      constexpr double max_heading_err = 1.0 * ANG2RAD;
      const double sin_theta_err = sin(heading_err);
      const double cos_theta_err = cos(heading_err);
      // 公式推导: lateral_general_model.py模型中的dynamics定义:
      // l = -sin(thetar) * (x - xr) + cos(thetar) * (y - yr), s_dot = cos(theta - thetar) / (1 - kr * l), x_dot = cos(theta), y_dot = sin(theta)
      // 因此l' = dl / ds = dl / dt * (1 / (ds / dt)) = (-sin(thetar) * dx / dt + cos(thetar) * dy / dt) * (1 / (ds / dt)) = tan(theta - thetar) * (1 - kr * l)
      const double l_prime = tan(heading_err) * temp;
      double eta = tan(max_heading_err), alpha = kappa_bound, eps = 0.5;
      // 前馈控制: 用于预测所需的曲率调整.
      // 反馈控制: 用于消除横向误差.
      const double feedforward = p0("kr") * cos_theta_err * (1 + sin_theta_err * sin_theta_err) / temp;
      const double sigma = l_prime + eta * std::copysign(std::sqrt(std::abs(l)), l);
      const double feedback = -alpha * sgn(sigma, eps);
      target_kappa = std::clamp(feedforward + feedback, -kappa_bound, kappa_bound);
    }
    const double target_dkappa = -tau * (tan (cur_psif) / vehicle_param_->wheel_base() - target_kappa);
    const double target_psi_f = std::clamp(target_dkappa * vehicle_param_->wheel_base() / (1 + std::pow(std::tan(cur_psif), 2)),-dsteer_bound,dsteer_bound);
    model->setUGuess("dsteer", target_psi_f, idx - 1);
    model->rollOut(idx);
  }
  return true;
}

/**
 * @brief 将优化器状态转换为路径数据
 * @details 该函数用于将优化器计算得到的状态转换为路径数据，包括路径点的位置、航向、曲率等信息。
 * 
 * @param[in] model 优化控制模型指针
 * @param[in] reference_line 参考线信息
 * @param[in] boundary 路径边界信息
 * @param[out] path_data 输出的路径数据
 * 
 * @par 输入参数说明:
 * - model: 优化控制模型指针，包含优化后的状态信息
 * - reference_line: 参考线对象，用于获取参考点信息
 * - boundary: 路径边界信息，包含决策边界、障碍物边界等
 * - path_data: 输出的路径数据指针
 * 
 * @par 关键变量说明:
 * - globals: 优化器计算的全局变量，包含曲率等信息
 * - states: 优化器计算的状态变量，包含位置、航向等信息
 * - controls: 优化器计算的控制输入，包含曲率变化率等信息
 * 
 * @par 转换流程:
 * 1. 检查输入参数的有效性
 * 2. 遍历优化器的每个状态点
 * 3. 获取每个状态点的位置、航向、曲率等信息
 * 4. 根据参考线获取参考点的高度和坡度信息
 * 5. 将转换后的路径点信息存储到路径数据中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查输入参数有效性;
 * :遍历优化器状态点;
 * :获取状态点信息;
 * :获取参考点高度和坡度;
 * :存储路径点信息;
 * stop
 * @enduml
 * 
 * @return bool 转换是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在优化器计算完成后调用，确保路径数据正确生成
 * 
 * @warning 需确保输入参数的有效性，特别是优化器状态和参考线信息
 */
bool OcpPathOptimizer::tranStatesToPathData(std::shared_ptr<OptimalControlProblem> model,
                                            const ReferenceLine& reference_line, const PathBoundary& boundary,
                                            PathData* const path_data) {
  if (path_data == nullptr) {
    ERT_PLOG_I << "[OcpPathOptimizer::tranStatesToPathData]: path_data is NULL" ;
    return false;
  }
  vector<PathPt> path_points;
  auto globals = model->getGlobals();
  //auto params = model->getParam();
  for (size_t i = 0; i <= model->N(); ++i) {
    auto& state = model->x(i);  // ref_s x y theta kappa
    auto& global = globals[i];
    auto control = model->u(i);

    double kappa = global(0);
    double d_kappa = control(0) * (1 + tan(state(4)) * tan(state(4))) / vehicle_param_->wheel_base();
    auto ref_point = reference_line.getReferencePoint(state(0));
    path_points.emplace_back(state(1), state(2), ref_point.z(), ref_point.slope(), state(3), kappa, model->t(i), d_kappa, 0.0);  // x y z slope heading kappa s dkappa ddkappa
    path_points.back().set_l(global(1));
    path_points.back().set_ref_s(state(0));   
    path_points.back().set_front_steer(state(4));
    // ERT_PLOG_I<<" lateral traj   "<<"x= "<<path_points.back().x()<<" y= "<<path_points.back().y()<<" theta= "<<path_points.back().theta()<<" kappa= "<<path_points.back().kappa()<<" wheel_steer= "<<path_points.back().front_steer()<<" wheel_steer_rate= "<<control(0);
  }
  path_data->setPathLabel(boundary.label());
  path_data->setReferenceLine(&reference_line);
  path_data->setDiscretizedPath(DiscretizedPath(path_points));
  return true;
}

// for fallback
/**
 * @brief 将优化器状态转换为兜底路径数据
 * @details 该函数用于将优化器计算得到的状态转换为兜底路径数据，包括路径点的位置、航向、曲率等信息。
 * 
 * @param[in] model 优化控制模型指针
 * @param[in] reference_line 参考线信息
 * @param[out] fallback_result 输出的兜底路径数据
 * 
 * @par 输入参数说明:
 * - model: 优化控制模型指针，包含优化后的状态信息
 * - reference_line: 参考线对象，用于获取参考点信息
 * - fallback_result: 输出的兜底路径数据指针
 * 
 * @par 关键变量说明:
 * - globals: 优化器计算的全局变量，包含曲率等信息
 * - states: 优化器计算的状态变量，包含位置、航向等信息
 * - controls: 优化器计算的控制输入，包含曲率变化率等信息
 * 
 * @par 转换流程:
 * 1. 检查输入参数的有效性
 * 2. 遍历优化器的每个状态点
 * 3. 获取每个状态点的位置、航向、曲率等信息
 * 4. 根据参考线获取参考点的高度和坡度信息
 * 5. 将转换后的路径点信息存储到兜底路径数据中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查输入参数有效性;
 * :遍历优化器状态点;
 * :获取状态点信息;
 * :获取参考点高度和坡度;
 * :存储路径点信息;
 * stop
 * @enduml
 * 
 * @return bool 转换是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在优化器计算完成后调用，确保兜底路径数据正确生成
 * 
 * @warning 需确保输入参数的有效性，特别是优化器状态和参考线信息
 */
bool OcpPathOptimizer::tranStatesToPathData(std::shared_ptr<OptimalControlProblem> model,
                                            const ReferenceLine& reference_line, shared_ptr<PathData> fallback_result) {
  if (fallback_result == nullptr) {
    ERT_PLOG_I << "[OcpPathOptimizer::tranStatesToPathData]: path_data is NULL" ;
    return false;
  }
  fallback_result->clear();
  auto globals = model->getGlobals();
  vector<PathPt> path_points;
  for (size_t i = 0; i <= model->N(); ++i) {
    auto& state = model->x(i);  // ref_s x y theta steer
    auto& global = globals[i];
    auto control = model->u(i);

    double kappa = global(0);
    double d_kappa = control(0) * (1 + tan(state(4)) * tan(state(4))) / vehicle_param_->wheel_base();
    auto ref_point = reference_line.getReferencePoint(state(0));
    path_points.emplace_back(state(1), state(2), ref_point.z(), ref_point.slope(), state(3), kappa, model->t(i),d_kappa, 0.0);  // x y z slope heading kappa s dkappa ddkappa
    path_points.back().set_l(global(1));
    path_points.back().set_ref_s(state(0));   
    path_points.back().set_front_steer(state(4));
    // ERT_PLOG_D<<" init guess: s =  "<< path_points.back().s()<<"    l   = "<< path_points.back().l()<< " x   = "<<state(1)<<" y   = "<<state(2)<<"  steer  "<<state(4);

  }
  fallback_result->setDiscretizedPath(DiscretizedPath(path_points));
  return true;
}

/**
 * @brief 检查优化器结果的合法性
 * @details 该函数用于检查优化器计算结果的合法性，包括不等式约束是否满足，并在不满足时尝试重新求解。
 * 
 * @param[in] data 优化器求解数据，包含模型和求解状态等信息
 * 
 * @par 输入参数说明:
 * - data: 优化器求解数据指针，包含模型和求解状态等信息
 * 
 * @par 关键变量说明:
 * - model: 优化控制模型指针
 * - status: 优化器求解状态
 * - optimizer_config_: 优化器配置，包含合法性检查的容差和最大重解次数
 * - start_state: 规划起点的状态信息
 * 
 * @par 检查流程:
 * 1. 检查优化器是否成功求解
 * 2. 检查不等式约束是否满足合法性容差
 * 3. 如果不满足，则尝试重新求解，最多重解optimizer_config_.resolve_max_num()次
 * 4. 如果重解成功，则更新求解状态
 * 5. 如果重解失败，则标记求解状态为不可行
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查优化器是否成功求解;
 * if (求解成功?) then (是)
 *   :检查不等式约束是否满足;
 *   if (约束满足?) then (是)
 *     :返回;
 *   else (否)
 *     :尝试重新求解;
 *     if (重解成功?) then (是)
 *       :更新求解状态;
 *     else (否)
 *       :标记求解状态为不可行;
 *     endif
 *   endif
 * endif
 * stop
 * @enduml
 * 
 * @note 该函数应在优化器求解完成后调用，确保优化结果的合法性
 * 
 * @warning 需确保输入参数的有效性，特别是优化器求解数据
 */
void OcpPathOptimizer::checkValid(std::shared_ptr<SolverData> data) {
  data->debug_status.clear();
  auto& model = data->model;
  auto& status = data->status;
  if (status != SolveStatus::SOLVED) return;

  // auto myprint = [&]() {
  //   auto inEquals = model->solver()->getInequalities();
  //   std::string str = "";
  //   for (size_t i = 1; i + 1 <= model->N(); ++i) {
  //     auto& it = inEquals[i];
  //     for (size_t j = 0; j < 10; j++) {
  //       if (it(j) > optimizer_config_.valid_tol()) {
  //         str += fmt::format("{} {} >> {:.2f}; ", i, j, it(j));
  //       }
  //     }
  //   }
  //   if(! str.empty()) ERT_PLOG_D << "[OcpPathOptimizer::checkValid]: " << str  ;
  // };

  auto isValid = [&]() {
    auto inEquals = model->solver()->getInequalities();
    for (size_t i = 1; i + 1 <= model->N(); ++i) {
      auto& it = inEquals[i];
      for (size_t j = 0; j < 10; j++) {
        if (it(j) > optimizer_config_.valid_tol()) {
          return false;
        }
      }
    }
    return true;
  };

  Eigen::VectorXd start_state = Eigen::VectorXd::Zero(5);
  start_state << sl_planning_start_point_.first.at(0), xy_planning_start_point_.path_pt().x(),
      xy_planning_start_point_.path_pt().y(), xy_planning_start_point_.path_pt().theta(),
      xy_planning_start_point_.path_pt().front_steer();

  model->updateParams();
  if (isValid()) return;

  bool is_valid = false;
  size_t resolve_ind = 1;
  for(; resolve_ind <= optimizer_config_.resolve_max_num(); resolve_ind++){
    model->setX0(start_state);
    status = model->solve();
    if(status != SolveStatus::SOLVED) {
      ERT_PLOG_I << "[OcpPathOptimizer::checkValid]: try to resolve but failed: resolve_ind = " << resolve_ind ;
      return;
    }
    model->updateParams();
    if (isValid()) {
      is_valid = true;
      break;
    }
  }
  
  if(is_valid){
    ERT_PLOG_I << "[OcpPathOptimizer::checkValid]: resolve success: resolve_ind = " << resolve_ind ;
  } else {
    ERT_PLOG_I << "[OcpPathOptimizer::checkValid]: resolve failed" ;
    status = SolveStatus::INFEASIBLE_DETECTED;
    data->debug_status.push_back(PathData::DebugStatusType::POST_CHECK_INVALID);
  }
}

/**
 * @brief 填充优化器路径数据
 * @details 该函数用于将优化器计算得到的状态信息填充到路径数据结构中，包括路径点的横向偏差等信息。
 * 
 * @param[in] model 优化控制模型指针
 * @param[in] boundary 路径边界信息
 * @param[out] path_data 输出的路径数据
 * 
 * @par 输入参数说明:
 * - model: 优化控制模型指针，包含优化后的状态信息
 * - boundary: 路径边界信息，包含决策边界、障碍物边界等
 * - path_data: 输出的路径数据指针
 * 
 * @par 关键变量说明:
 * - ocp_path_info: 优化器路径信息，包含每个路径点的横向偏差等信息
 * - states: 优化器计算的状态变量，包含位置、航向等信息
 * - params: 优化器计算的参数，包含参考点信息
 * 
 * @par 填充流程:
 * 1. 计算当前状态点与参考点之间的横向偏差
 * 2. 根据路径边界信息获取障碍物ID
 * 3. 将计算得到的信息存储到路径数据结构中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :遍历优化器状态点;
 * :计算横向偏差;
 * :获取障碍物ID;
 * :存储路径点信息;
 * stop
 * @enduml
 * 
 * @note 该函数应在优化器计算完成后调用，确保路径数据正确填充
 * 
 * @warning 需确保输入参数的有效性，特别是优化器状态和路径边界信息
 */
void OcpPathOptimizer::fillOcpPathData(std::shared_ptr<OptimalControlProblem> model,
                                       const PathBoundary& boundary, PathData* const path_data) {
  auto calcLateralDis = [](double x, double y, double xr, double yr, double thetar) {
    return -sin(thetar) * (x - xr) + cos(thetar) * (y - yr);
  };

  auto ocp_path_info = path_data->mutableOcpPathInfo();
  ocp_path_info->clear();
  for (size_t i = 0; i <= model->N(); ++i) {
    ocp_path_info->emplace_back();
    auto& info = ocp_path_info->back();
    info.ind = i;
    auto states = model->x(i);
    auto curr_s = states("s");
    auto x = states("x");
    auto y = states("y");
    auto params = model->p(i);
    auto xr = params("xr");
    auto yr = params("yr");
    auto thetar = params("thetar");
    info.lateral_dis = calcLateralDis(x, y, xr, yr, thetar);
    auto map_it = boundary.findConstKeyObstacle(curr_s);
    if (map_it) {
      auto ob_info = map_it.value()->second;
      info.ob_id_left = ob_info.ob_id_left;
      info.ob_id_right = ob_info.ob_id_right;
    }
  }
}



// 无约束求解轨迹：模型及规划起点与有约束轨迹一致，但无横向硬边界约束，仅考虑软边界约束，利于横向控制跟踪；优先级最高.
/**
 * @brief 无约束求解轨迹
 * @details 该函数用于生成无约束的优化轨迹，模型及规划起点与有约束轨迹一致，但无横向硬边界约束，仅考虑软边界约束，利于横向控制跟踪；优先级最高。
 * 
 * @param[in] reference_line 参考线信息
 * @param[in] start_point 规划起点
 * @param[in] boundary 路径边界信息
 * @param[out] path_data 输出的路径数据
 * 
 * @par 输入参数说明:
 * - reference_line: 参考线对象，用于获取参考点信息
 * - start_point: 规划起点，包含位置、速度等信息
 * - boundary: 路径边界信息，包含决策边界、障碍物边界等
 * - path_data: 输出的路径数据指针
 * 
 * @par 关键变量说明:
 * - model: 优化控制模型指针
 * - status: 优化器求解状态
 * - prev_model_info_: 前一次模型信息，包含模型名称、配置等
 * - async_planner_debug_info_: 异步规划调试信息
 * 
 * @par 求解流程:
 * 1. 初始化无约束模型
 * 2. 求解优化问题
 * 3. 如果求解成功，则将状态转换为路径数据
 * 4. 如果求解失败，则返回错误状态
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化无约束模型;
 * :求解优化问题;
 * if (求解成功?) then (是)
 *   :将状态转换为路径数据;
 * else (否)
 *   :返回错误状态;
 * endif
 * stop
 * @enduml
 * 
 * @return Status 求解状态，OK表示成功，其他表示失败
 * 
 * @note 该函数应在需要生成无约束轨迹时调用，确保模型状态正确更新
 * 
 * @warning 需确保输入参数的有效性，特别是参考线和规划起点信息
 */
Status OcpPathOptimizer::unconstrainedProc(const ReferenceLine& reference_line, const TrajectoryPt& start_point,
                                           const PathBoundary& boundary, PathData* const path_data) {
  util::TimerLogger<std::milli> timer("unconstrainedproc", [](const std::string& str) { ERT_PLOG_I << "[OcpPathOptimizer::unconstrainedProc]: " << str ; });

  // 0）确保每帧都调 preProcess()，更新 target_ref_line_、N_、ds_、accumulated_s_ 等内部状态。
  {
    bool pre_ok = preProcess(reference_line, start_point, boundary, path_data->discretized_path());
    if (!pre_ok) {
      return Status(ErrorCode::PLANNING_ERROR, "unconstrainedProc: preProcess faield");
    }
  }

  // 1）get ocp model, set stage
  timer("initUnconstrainModel");
  auto model = initUnconstrainModel(profile(), path_data->discretized_path());
  timer("initUnconstrainModel");
#ifdef LOG_OCP_DATA
  // log data if need to debug solver
  if (model->logData(&ocp_data_field_) > 10) {
    std::filesystem::path buffer = std::filesystem::path(__FILE__).parent_path();
    std::string direction = buffer;
    std::ofstream of(direction + fmt::format("{}_data_{}.bin", ocp_data_field_.model(), debug_count_++));
    if (of) {
      google::protobuf::io::OstreamOutputStream ofs(&of);
      ocp_data_field_.SerializeToOstream(&of);
      ocp_data_field_.clear_data();
      of.close();
    }
  }
#endif

  // 2）solve problem
  timer(profile().model());
  auto status = model->solve();
  ERT_PLOG_I << "[OcpPathOptimizer::unconstrainedProc]: " << curr_profile_type_
            << "  " << profile().model()
            << "  ipm_solver status: " << status
            << "  iter = " << model->solver()->iteration_number() ;
  timer(profile().model());

  // 3）get path result
  timer("postProcess");
  prev_model_info_ = {target_ref_line_->id(), curr_profile_type_,curr_update_params_method_, model, status};

  async_planner_debug_info_ += fmt::format(
      "\n unconstrainted  ipm_solver:{}, iter:{}, time:{:.2f}",
      static_cast<int>(status), model->solver()->iteration_number(),
      model->solver()->getSolveInfo().computation_time * 1000.0);

  if (status == SolveStatus::SOLVED) {
#ifdef LOG_SUCCESS_STAGE
    auto states = model->getX();
    auto params = model->getParam();
    for (int i = 0; i < states.size(); ++i) {
      ERT_PLOG_I << "[OcpPathOptimizer::asyncProc]: " << states[i].debugString()
                    << "\n" << params[i].debugString() ;
    }
#endif
    
    tranStatesToPathData(model, reference_line, boundary, path_data);
  } else {
#ifdef LOG_FAILED_STAGE
    auto states = model->getX();
    auto params = model->getParam();
    for (int i = 0; i < states.size(); ++i) {
      ERT_PLOG_I << "[OcpPathOptimizer::asyncProc]: " << states[i].debugString()
                    << "\n" << params[i].debugString() ;
    }
#endif
    return Status(ErrorCode::PLANNING_ERROR, "Unconstrain ipm solver faield");
  }
  return Status::OK();
}

/**
 * @brief 初始化无约束模型
 * @details 该函数用于初始化无约束优化模型，根据配置文件和前一次路径信息设置模型参数和配置。
 * 
 * @param[in] profile 优化器配置文件
 * @param[in] prev_path 前一次规划的路径
 * 
 * @par 输入参数说明:
 * - profile: 优化器配置文件，包含模型名称、默认参数等信息
 * - prev_path: 前一次规划的路径，用于模型更新
 * 
 * @par 关键变量说明:
 * - model: 优化控制模型指针
 * - curr_update_params_method_: 当前参数更新方法
 * - profile(): 优化器配置文件
 * - optimizer_config_: 优化器配置
 * 
 * @par 初始化流程:
 * 1. 创建优化控制模型
 * 2. 设置模型配置，包括积分器类型、时间步长、预测步长等
 * 3. 设置模型默认参数
 * 4. 如果模型名称为"LateralGeneral"，则初始化无约束横向通用模型
 * 5. 初始化模型
 * 6. 如果模型名称为"LateralGeneral"，则更新横向通用模型
 * 
 * @par 流程图:
 * @startuml
 * start
 * :创建优化控制模型;
 * :设置模型配置;
 * :设置模型默认参数;
 * if (模型名称为"LateralGeneral"?) then (是)
 *   :初始化无约束横向通用模型;
 * endif
 * :初始化模型;
 * if (模型名称为"LateralGeneral"?) then (是)
 *   :更新横向通用模型;
 * endif
 * stop
 * @enduml
 * 
 * @return std::shared_ptr<OptimalControlProblem> 初始化后的优化控制模型指针
 * 
 * @note 该函数应在需要生成无约束轨迹时调用，确保模型正确初始化
 * 
 * @warning 需确保输入参数的有效性，特别是优化器配置文件和前一次路径信息
 */
std::shared_ptr<OptimalControlProblem> OcpPathOptimizer::initUnconstrainModel(const OcpPathOptimizerProfile& profile,
                                                                              const DiscretizedPath& prev_path) {
  std::shared_ptr<OptimalControlProblem> model;
  curr_update_params_method_ = UpdateParamsMethod::LATERAL_UNCONSTRAINED;
  model = OptimalControlProblem::create(profile.model());
  if (model != nullptr) {
    model->mutable_config()->set_integrator_type(OcpConfig::ERK4);
    model->mutable_config()->set_dt(ds_);
    model->mutable_config()->set_horizon_length(N_);
    for (const auto& param : profile.default_params()) {
      model->setParam(param.key(), param.value());
    }
    if (profile.has_ipm_config()) {
      model->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(profile.ipm_config());
    } else {
      model->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(optimizer_config_.ipm_config());
    }
  }
  if (model->name() == "LateralGeneral") {
    // 相比initLateralGeneral，initUnconstrainedLateralGeneral仅考虑软边界.
    initUnconstrainedLateralGeneral(model);
  }
  model->init();

  if (model->name() == "LateralGeneral") {
    updateLateralGeneral(model, prev_path);
  }

  return model;
}

// 相比initLateralGeneral，initUnconstrainedLateralGeneral仅考虑软边界.
/**
 * @brief 初始化无约束横向通用模型
 * @details 该函数用于初始化无约束横向通用模型，设置模型参数和配置，相比有约束模型仅考虑软边界约束。
 * 
 * @param[in] model 优化控制模型指针
 * 
 * @par 输入参数说明:
 * - model: 优化控制模型指针，用于设置模型参数和配置
 * 
 * @par 关键变量说明:
 * - vehicle_param_: 车辆参数，包含前后轴到车体中心的距离等信息
 * - sl_planning_start_point_: 规划起点的SL坐标
 * - xy_planning_start_point_: 规划起点的XY坐标
 * - profile(): 优化器配置文件
 * 
 * @par 初始化流程:
 * 1. 设置车辆前后轴到车体中心的距离
 * 2. 设置模型参数，包括参考点信息、软边界约束、转向角限制等
 * 3. 根据当前状态和参考点信息，计算并设置模型参数
 * 
 * @par 流程图:
 * @startuml
 * start
 * :设置车辆前后轴距离;
 * :遍历模型状态点;
 * :获取参考点信息;
 * :计算并设置软边界约束;
 * :计算并设置转向角限制;
 * :设置模型参数;
 * stop
 * @enduml
 * 
 * @return bool 初始化是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在初始化无约束模型时调用，确保模型参数正确设置
 * 
 * @warning 需确保输入参数的有效性，特别是优化控制模型指针
 */
bool OcpPathOptimizer::initUnconstrainedLateralGeneral(std::shared_ptr<OptimalControlProblem> model) {
  model->setParam("lf", vehicle_param_->front_edge_to_ego());
  model->setParam("lr", vehicle_param_->rear_edge_to_ego());
  model->setParam("wheelbase", vehicle_param_->wheel_base());
  const double constaint_ignore_buffer = vehicle_param_->front_edge_to_ego();
  auto linearInterpolation = [](double i, double N, double value_lower, double value_upper) -> double {
    if (std::abs(N) < 1e-9) return value_lower;

    const double step = (value_upper - value_lower) / N;
    double result = value_lower + i * step;

    // 边界截断功能
    return std::clamp(result, min(value_lower, value_upper), max(value_lower, value_upper));
  };
  model->setParam(
      [=](const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double t, const int k, OcpVariable* ptr) {
        double ego_s = sl_planning_start_point_.first[0];
        double curr_s = x(0);
        auto ref_point = getInitPoint(curr_s);  // according to s
        auto ref_point_direction = getInitPointDrivingDirection(curr_s);
        auto kf_ref_point = getInitPoint(ref_point.x() + vehicle_param_->front_edge_to_ego() * cos(ref_point.heading()), ref_point.y() + vehicle_param_->front_edge_to_ego() * sin(ref_point.heading()));
        auto kr_ref_point = getInitPoint(ref_point.x() - vehicle_param_->rear_edge_to_ego() * cos(ref_point.heading()), ref_point.y() - vehicle_param_->rear_edge_to_ego() * sin(ref_point.heading()));
        double kf = kf_ref_point.local_s();
        double kr = kr_ref_point.local_s();
        ptr->set("xr", ref_point.x());
        ptr->set("yr", ref_point.y());
        ptr->set("thetar", getUnifySpaceHeading(x(3), ref_point.heading()));
        ptr->set("kr", ref_point.kappa());
        ptr->set("xrf", kf_ref_point.x());
        ptr->set("yrf", kf_ref_point.y());
        ptr->set("thetarf", getUnifySpaceHeading(x(3), kf_ref_point.heading()));
        ptr->set("xrr", kr_ref_point.x());
        ptr->set("yrr", kr_ref_point.y());
        ptr->set("thetarr", getUnifySpaceHeading(x(3), kr_ref_point.heading()));
        if (k > 0) {
          // soft
          auto soft_res = getSoftCenterBound(curr_s);
          auto& [soft_bound, soft_bound_valid] = soft_res;
          if (curr_s > ego_s + constaint_ignore_buffer && soft_bound_valid) {
            auto [s, lower, upper] = soft_bound;
            ptr->set("sll", lower);
            ptr->set("slu", upper);
          } else {
            ptr->set("sll", -20.0);
            ptr->set("slu", 20.0);
          }

          if (profile().enb_edge_soft_bound()) {           
            auto f_soft_res = getSoftEdgeBound(kf);
            auto& [f_soft_bound, f_soft_bound_valid] = f_soft_res;
            if (curr_s > ego_s + constaint_ignore_buffer && f_soft_bound_valid) {
              auto [s, lower, upper] = f_soft_bound;
              ptr->set("slfl", lower);
              ptr->set("slfu", upper);
            } else {
              ptr->set("slfl", -20.0);
              ptr->set("slfu", 20.0);
            }
            auto r_soft_res = getSoftEdgeBound(kr);
            auto& [r_soft_bound, r_soft_bound_valid] = r_soft_res;
            if (curr_s > ego_s + constaint_ignore_buffer && r_soft_bound_valid) {
              auto [s, lower, upper] = r_soft_bound;
              ptr->set("slrl", lower);
              ptr->set("slru", upper);
            } else {
              ptr->set("slrl", -20.0);
              ptr->set("slru", 20.0);
            }
          }
        }
        auto [ref_offset, ref_offset_valid] = getRefOffset(ref_point);
        if (ref_offset_valid) {
          auto [s, l_offset] = ref_offset;
          ptr->set("l_offset", l_offset);
        }
        if (curr_s >= lane_keep_start_s_) {
          ptr->set("l_weight", linearInterpolation(k, N_, 1e-3, profile().weights().l_ref_weight_piecewise()));
        }
         double const_a_speed = std::sqrt(xy_planning_start_point_.v() * xy_planning_start_point_.v() + 2.0 * starting_acc_ * k);
         double speed_limit = getSpeedLimit(curr_s,ref_point.kappa());
         const double steer_angle_bound = getMaxSteerAngleBound(t, std::min(const_a_speed, speed_limit), ref_point_direction);
         ptr->set("SteerLowerBound", -steer_angle_bound);
         ptr->set("SteerUpperBound", steer_angle_bound);
         const double dsteer_angle_bound = getMaxDSteerAngleBound(t, std::min(const_a_speed, speed_limit), ref_point_direction);
         ptr->set("DSteerLowerBound", -dsteer_angle_bound);
         ptr->set("DSteerUpperBound", dsteer_angle_bound);

        string valid_tag = "";
        for (auto& [tag, start_s, end_s] : getSpecialSceneRange()) {
          if (x(0) >= start_s && x(0) <= end_s) {
            int32_t priority_level = 0;
            if (priority_name_map_.count(tag) > 0) {
              priority_level = priority_name_map_[tag];
              if (valid_tag == "") {
                valid_tag = tag;
              } else {
                if (priority_level < priority_name_map_[valid_tag]) {
                  valid_tag = tag;
                }
              }
            }
          }
        }
        for (auto& param : profile().ranged_params()) {
          if (param.range_key() == valid_tag) {
            ptr->set(param.key(), param.value());
          }
        }
      });
  return true;
}



// 兜底轨迹：采用BicycleTrajectoryTracker ocp模型，模拟横向控制的行驶轨迹，但规划起点在自车当前位置，不利于横向控制跟踪.
/**
 * @brief 生成兜底路径
 * @details 该函数用于生成兜底路径，采用BicycleTrajectoryTracker ocp模型，模拟横向控制的行驶轨迹，但规划起点在自车当前位置，不利于横向控制跟踪。
 * 
 * @param[in] curr_state 当前车辆状态
 * @param[in] reference_line 参考线信息
 * @param[in] prev_speed_data 前一次速度数据
 * @param[in] path 前一次规划的路径
 * @param[in] curr_stamp 当前时间戳
 * @param[in] max_length 最大路径长度
 * 
 * @par 输入参数说明:
 * - curr_state: 当前车辆状态，包含位置、速度等信息
 * - reference_line: 参考线对象，用于获取参考点信息
 * - prev_speed_data: 前一次速度数据，用于生成速度点
 * - path: 前一次规划的路径，用于模型初始化
 * - curr_stamp: 当前时间戳，用于速度点生成
 * - max_length: 最大路径长度，用于限制路径长度
 * 
 * @par 关键变量说明:
 * - speed_points: 速度点序列，用于模型求解
 * - model: 优化控制模型指针
 * - status: 优化器求解状态
 * - states: 优化器计算的状态变量，包含位置、航向等信息
 * - ctrls: 优化器计算的控制输入，包含加速度、曲率变化率等信息
 * 
 * @par 生成流程:
 * 1. 检查输入路径是否为空
 * 2. 根据前一次速度数据生成速度点序列
 * 3. 初始化兜底路径模型
 * 4. 求解优化问题
 * 5. 如果求解成功，则将状态转换为路径数据
 * 6. 如果求解失败，则记录日志信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查输入路径是否为空;
 * if (路径为空?) then (是)
 *   :返回空路径;
 * else (否)
 *   :生成速度点序列;
 *   :初始化兜底路径模型;
 *   :求解优化问题;
 *   if (求解成功?) then (是)
 *     :将状态转换为路径数据;
 *   else (否)
 *     :记录日志信息;
 *   endif
 * endif
 * stop
 * @enduml
 * 
 * @return DiscretizedPath 生成的兜底路径
 * 
 * @note 该函数应在需要生成兜底路径时调用，确保路径正确生成
 * 
 * @warning 需确保输入参数的有效性，特别是当前车辆状态和前一次速度数据
 */
DiscretizedPath OcpPathOptimizer::generateProtectPath(const VehicleState& curr_state,
                                                      const ReferenceLine& reference_line,
                                                      const SpeedData& prev_speed_data,
                                                      const DiscretizedPath& path, const int64_t curr_stamp,
                                                      const double max_length) {
  DiscretizedPath res;
  if (!path.empty()) {
    util::TimerLogger<std::milli> timer("protect_path_ocp", [](const std::string &str)
                                        { ERT_PLOG_I << "[OcpPathOptimizer::generateProtectPath]: " << str ; });
    std::vector<gpal::pnc::SpeedPoint> speed_points;
    if (optimizer_config_.force_cv_model() || !getSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, 1.0, 0.0, 20.0, 0.1,
                                                                               std::max(max_length, path.back().s()), &speed_points)) {
      getSpeedPointsFromCVModel(std::max(1.0, curr_state.linear_velocity()), 0.0, 20.0, 0.1,
                                std::max(max_length, path.back().s()), &speed_points);
    }
    // 采用BicycleTrajectoryTracker ocp模型.
    auto model = initProtectPath(profile().model(), speed_points, path, curr_state);
    auto status = model->solve();
    ERT_PLOG_I << "[OcpPathOptimizer::generateProtectPath]: " << model->name()
              << "  status = " << status
              << "  iteration = " << model->solver()->iteration_number() ;

#ifdef LOG_OCP_PROTECTPATH_DATA
    model->logData();
#endif
    if (status == SolveStatus::SOLVED) {
      auto states = model->getX();
      auto ctrls = model->getU();
      for (int i = 0; i + 1 < states.size(); ++i) {
        auto &state = states[i]; // x y heading v kappa
        const auto& ctrl = ctrls[i];   // a dkappa
        auto ref_point = reference_line.getReferencePoint(state(0), state(1));
        res.emplace_back(state(0), state(1), ref_point.z(), ref_point.slope(), state(2), state(4), speed_points[i].s(), ctrl(1), 0.0); // x y z slope heading kappa s dkappa ddkappa
        res.back().set_ref_s(ref_point.local_s());
      }
    } else {
#ifdef LOG_OCP_PROTECTPATH_DATA
      // log data if need to debug solver
      if (model->logDataSize() > 500) {
        auto ocp_data_field = model->getLogData();
        std::filesystem::path buffer = std::filesystem::path(__FILE__).parent_path();
        std::string direction = buffer;
        std::ofstream of(direction + fmt::format("{}_data_{}.bin", ocp_data_field.model(), debug_count_++));
        if (of) {
          google::protobuf::io::OstreamOutputStream ofs(&of);
          ocp_data_field.SerializeToOstream(&of);
          ocp_data_field.clear_data();
          of.close();
        }
      }
#endif
    }
  }

  return res;
}

/**
 * @brief 从前一次速度数据生成速度点序列
 * @details 该函数用于从前一次速度数据中生成速度点序列，用于优化器的输入。
 * 
 * @param[in] prev_speed_data 前一次速度数据
 * @param[in] curr_stamp 当前时间戳
 * @param[in] min_v 最小速度限制
 * @param[in] t0 起始时间
 * @param[in] t1 结束时间
 * @param[in] dt 时间步长
 * @param[in] max_s 最大路径长度
 * @param[out] speed_points 输出的速度点序列
 * 
 * @par 输入参数说明:
 * - prev_speed_data: 前一次速度数据，包含速度、加速度等信息
 * - curr_stamp: 当前时间戳，用于确定起始时间
 * - min_v: 最小速度限制，低于该值将被设置为min_v
 * - t0: 起始时间，用于生成速度点序列
 * - t1: 结束时间，用于生成速度点序列
 * - dt: 时间步长，用于生成速度点序列
 * - max_s: 最大路径长度，用于限制速度点序列的长度
 * - speed_points: 输出的速度点序列指针
 * 
 * @par 关键变量说明:
 * - speed_point: 单个速度点，包含时间、位置、速度、加速度等信息
 * - prev_start_t: 前一次速度数据的起始时间
 * - prev_start_s: 前一次速度数据的起始位置
 * - delta_t: 时间步长，用于计算加速度
 * 
 * @par 生成流程:
 * 1. 检查输入参数的有效性
 * 2. 根据当前时间戳获取起始速度点
 * 3. 遍历时间序列，生成速度点
 * 4. 如果速度低于最小速度限制，则设置为min_v
 * 5. 计算加速度并存储速度点
 * 6. 如果路径长度超过最大限制，则停止生成
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查输入参数有效性;
 * :获取起始速度点;
 * :遍历时间序列;
 * :检查速度是否低于最小限制;
 * :计算加速度;
 * :存储速度点;
 * :检查路径长度是否超过最大限制;
 * stop
 * @enduml
 * 
 * @return bool 生成是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在需要从前一次速度数据生成速度点序列时调用，确保速度点序列正确生成
 * 
 * @warning 需确保输入参数的有效性，特别是前一次速度数据和当前时间戳
 */
bool OcpPathOptimizer::getSpeedPointsFromPrevSpeedData(const SpeedData& prev_speed_data,
                                                       const int64_t curr_stamp, const double min_v,
                                                       const double t0, const double t1, const double dt,
                                                       const double max_s,
                                                       std::vector<gpal::pnc::SpeedPoint>* speed_points) {
  if (speed_points == nullptr) {
    return false;
  }
  gpal::pnc::SpeedPoint speed_point;
  if (!prev_speed_data.EvaluateByAbsoluteTime(std::chrono::microseconds(curr_stamp), &speed_point)) {
    return false;
  }
  double prev_start_t = speed_point.t();
  double prev_start_s = speed_point.s();
  speed_points->clear();
  for (double t = t0; t <= t1; t += dt) {
    if (!prev_speed_data.EvaluateByTime(prev_start_t + t, &speed_point)) {
      speed_point = prev_speed_data.back();
    }
    speed_point.set_t(t);
    double s = speed_point.s() - prev_start_s;
    if (speed_point.v() < min_v) {
      speed_point.set_v(min_v);
      speed_point.set_a(0.0);
    }
    if (!speed_points->empty()) {
      const double delta_t = speed_point.t() - speed_points->back().t();
      if (delta_t < 1e-6) {
        continue;
      }
      speed_points->back().set_a((speed_point.v() - speed_points->back().v()) / delta_t);
      s = speed_points->back().s() + speed_points->back().v() * delta_t +
          0.5 * speed_points->back().a() * delta_t * delta_t;
    }
    if (s > max_s) {
      break;
    }
    speed_point.set_s(s);
    speed_points->emplace_back(speed_point);
  }
  return speed_points->size() > 1;
}

/**
 * @brief 从恒定速度模型生成速度点序列
 * @details 该函数用于从恒定速度模型生成速度点序列，假设车辆以恒定速度行驶，用于优化器的输入。
 * 
 * @param[in] v0 初始速度
 * @param[in] t0 起始时间
 * @param[in] t1 结束时间
 * @param[in] dt 时间步长
 * @param[in] max_s 最大路径长度
 * @param[out] speed_points 输出的速度点序列
 * 
 * @par 输入参数说明:
 * - v0: 初始速度，单位为m/s
 * - t0: 起始时间，单位为s
 * - t1: 结束时间，单位为s
 * - dt: 时间步长，单位为s
 * - max_s: 最大路径长度，单位为m
 * - speed_points: 输出的速度点序列指针
 * 
 * @par 关键变量说明:
 * - end_s: 结束路径长度，取v0*(t1-t0)和max_s的最小值
 * - end_t: 结束时间，根据结束路径长度和初始速度计算
 * - N: 速度点序列的长度，根据时间步长计算
 * 
 * @par 生成流程:
 * 1. 检查输入参数的有效性
 * 2. 计算结束路径长度和结束时间
 * 3. 根据时间步长计算速度点序列的长度
 * 4. 遍历时间序列，生成速度点
 * 5. 将生成的速度点存储到速度点序列中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查输入参数有效性;
 * :计算结束路径长度和结束时间;
 * :计算速度点序列长度;
 * :遍历时间序列;
 * :生成速度点;
 * :存储速度点;
 * stop
 * @enduml
 * 
 * @return bool 生成是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在需要从恒定速度模型生成速度点序列时调用，确保速度点序列正确生成
 * 
 * @warning 需确保输入参数的有效性，特别是初始速度和最大路径长度
 */
bool OcpPathOptimizer::getSpeedPointsFromCVModel(const double v0, const double t0, const double t1,
                                                 const double dt, const double max_s,
                                                 std::vector<gpal::pnc::SpeedPoint>* speed_points) {
  if (speed_points == nullptr) {
    return false;
  }
  const double end_s = std::min(v0 * (t1 - t0), max_s);
  const double end_t = end_s / v0 + t0;
  const int N = (end_t - t0) / dt + 1;
  speed_points->clear();
  for (int i = 0; i <= N; ++i) {
    gpal::pnc::SpeedPoint speed_point;
    const double t = t0 + i * dt;
    const double s = v0 * t;
    speed_point.set_t(t);
    speed_point.set_s(s);
    speed_point.set_v(v0);
    speed_points->emplace_back(std::move(speed_point));
  }
  return speed_points->size() > 1;
}

/**
 * @brief 初始化兜底路径模型
 * @details 该函数用于初始化兜底路径模型，采用BicycleTrajectoryTracker ocp模型，用于生成兜底路径。
 * 
 * @param[in] profile 模型配置文件名称
 * @param[in] speed_points 速度点序列，用于模型求解
 * @param[in] path 前一次规划的路径，用于模型初始化
 * @param[in] curr_state 当前车辆状态
 * 
 * @par 输入参数说明:
 * - profile: 模型配置文件名称，用于创建优化控制模型
 * - speed_points: 速度点序列，包含时间、位置、速度等信息
 * - path: 前一次规划的路径，用于模型初始化
 * - curr_state: 当前车辆状态，包含位置、速度等信息
 * 
 * @par 关键变量说明:
 * - protect_path_model_: 兜底路径模型指针
 * - warm_start: 是否使用热启动，如果模型已存在且预测步长相同则使用热启动
 * - optimizer_config_: 优化器配置，包含模型默认参数和IPM配置
 * 
 * @par 初始化流程:
 * 1. 检查模型是否已存在且预测步长相同，决定是否使用热启动
 * 2. 如果模型不存在或模型名称不匹配，则创建新模型
 * 3. 设置模型配置，包括积分器类型、时间步长、预测步长等
 * 4. 设置模型默认参数和IPM配置
 * 5. 如果模型名称为"BicycleTrajectoryTracker"，则初始化自行车轨迹跟踪模型
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查模型是否已存在且预测步长相同;
 * if (模型存在且预测步长相同?) then (是)
 *   :使用热启动;
 * else (否)
 *   :创建新模型;
 * endif
 * :设置模型配置;
 * :设置模型默认参数和IPM配置;
 * if (模型名称为"BicycleTrajectoryTracker"?) then (是)
 *   :初始化自行车轨迹跟踪模型;
 * endif
 * stop
 * @enduml
 * 
 * @return std::shared_ptr<OptimalControlProblem> 初始化后的优化控制模型指针
 * 
 * @note 该函数应在需要生成兜底路径时调用，确保模型正确初始化
 * 
 * @warning 需确保输入参数的有效性，特别是速度点序列和前一次路径信息
 */
std::shared_ptr<OptimalControlProblem> OcpPathOptimizer::initProtectPath(const std::string& profile, 
                                                                         const std::vector<gpal::pnc::SpeedPoint>& speed_points, 
                                                                         const DiscretizedPath& path,
                                                                         const VehicleState& curr_state) {
  std::string protect_path_model = "BicycleTrajectoryTracker";

  bool warm_start = false;
  if (protect_path_model_ == nullptr || protect_path_model_->name() != protect_path_model) {
    protect_path_model_ = OptimalControlProblem::create(protect_path_model);
  } else if (protect_path_model_->N() == speed_points.size() - 1) {
    warm_start = true;
  }
  if (protect_path_model_ != nullptr) {
    if (!warm_start) {
      protect_path_model_->mutable_config()->set_integrator_type(OcpConfig::FORWARD_EULER);
      protect_path_model_->mutable_config()->set_dt(0.1);
      protect_path_model_->mutable_config()->set_horizon_length(speed_points.size() - 1);
      protect_path_model_->mutable_config()->set_max_iter_time(0.1);
      auto iter = optimizer_config_.protect_path_profiles().find(protect_path_model_->name());
      if (iter != optimizer_config_.protect_path_profiles().end()) {
        for (const auto& param : iter->second.default_params()) {
          protect_path_model_->setParam(param.key(), param.value());
        }
        if (iter->second.has_ipm_config()) {
          protect_path_model_->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(iter->second.ipm_config());
        } else if (optimizer_config_.has_protect_path_ipm_config()) {
          protect_path_model_->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(optimizer_config_.protect_path_ipm_config());
        }
      } else if (optimizer_config_.has_protect_path_ipm_config()) {
        protect_path_model_->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(optimizer_config_.protect_path_ipm_config());
      }
    }
    if (protect_path_model_->name() == "BicycleTrajectoryTracker") {
      initBicycleTrajectoryTracker(protect_path_model_, speed_points, path, curr_state, warm_start);
    }
  }
  return protect_path_model_;
}

/**
 * @brief 初始化自行车轨迹跟踪模型
 * @details 该函数用于初始化自行车轨迹跟踪模型，设置模型初始状态和参数，用于生成兜底路径。
 * 
 * @param[in] model 优化控制模型指针
 * @param[in] speed_points 速度点序列，用于模型求解
 * @param[in] path 前一次规划的路径，用于模型初始化
 * @param[in] curr_state 当前车辆状态
 * @param[in] warm_start 是否使用热启动
 * 
 * @par 输入参数说明:
 * - model: 优化控制模型指针，用于设置模型初始状态和参数
 * - speed_points: 速度点序列，包含时间、位置、速度等信息
 * - path: 前一次规划的路径，用于模型初始化
 * - curr_state: 当前车辆状态，包含位置、速度等信息
 * - warm_start: 是否使用热启动，true表示使用热启动，false表示不使用
 * 
 * @par 关键变量说明:
 * - start_state: 模型初始状态，包含位置、航向、速度、曲率等信息
 * - prev_heading: 前一次航向角，用于计算当前航向角
 * - path_state: 路径状态，包含路径点的位置、航向、曲率等信息
 * 
 * @par 初始化流程:
 * 1. 设置模型初始状态，包括位置、航向、速度、曲率等信息
 * 2. 如果不使用热启动，则初始化模型
 * 3. 遍历速度点序列，设置模型参数，包括参考点信息、速度限制等
 * 4. 如果当前状态点不是第一个点，则应用自行车轨迹跟踪控制策略
 * 
 * @par 流程图:
 * @startuml
 * start
 * :设置模型初始状态;
 * if (不使用热启动?) then (是)
 *   :初始化模型;
 * endif
 * :遍历速度点序列;
 * :设置模型参数;
 * if (不是第一个点?) then (是)
 *   :应用控制策略;
 * endif
 * stop
 * @enduml
 * 
 * @return bool 初始化是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在初始化兜底路径模型时调用，确保模型初始状态和参数正确设置
 * 
 * @warning 需确保输入参数的有效性，特别是优化控制模型指针和当前车辆状态
 */
bool OcpPathOptimizer::initBicycleTrajectoryTracker(std::shared_ptr<OptimalControlProblem> model,
                                                    const std::vector<gpal::pnc::SpeedPoint>& speed_points,
                                                    const DiscretizedPath& path, const VehicleState& curr_state,
                                                    const bool warm_start) {
  Eigen::VectorXd start_state = Eigen::VectorXd::Zero(5);
  start_state << curr_state.x(), curr_state.y(), curr_state.yaw(), std::max(1.0, curr_state.linear_velocity()), curr_state.kappa(); // x y theta v kappa
  model->setX0(start_state);
  if (!warm_start) {
    model->init();
  }
  double prev_heading = curr_state.yaw();
  for (int i = 0; i <= model->N(); ++i) {
    auto& speed_point = speed_points[i];
    auto path_state = path.evaluate(speed_point.s());
    if (i > 0) {
      applyBicycleTrajectoryTrackerCtrlPolycy(model, i, path_state.ref_s());
    }
    // model->setUGuess("a", speed_point.a(), i);
    model->setParam("xr", path_state.x(), i);
    model->setParam("yr", path_state.y(), i);
    model->setParam("thetar", getUnifySpaceHeading(prev_heading, path_state.theta()), i);
    model->setParam("kr", path_state.kappa(), i);
    model->setParam("vr", speed_point.v(), i);
    model->setParam("v_upper", speed_point.v() + 1.0, i);
    prev_heading = model->x(i, "theta");
  }
  return true;
}

/**
 * @brief 应用自行车轨迹跟踪控制策略
 * @details 该函数用于应用自行车轨迹跟踪控制策略，根据当前状态与参考点的偏差计算控制输入，并更新模型状态。
 * 
 * @param[in] model 优化控制模型指针
 * @param[in] idx 当前状态点索引
 * @param[in] ref_local_s 参考点的局部s坐标
 * 
 * @par 输入参数说明:
 * - model: 优化控制模型指针，用于获取当前状态和设置控制输入
 * - idx: 当前状态点索引，用于更新模型状态
 * - ref_local_s: 参考点的局部s坐标，用于计算控制策略
 * 
 * @par 关键变量说明:
 * - x0: 当前状态点，包含位置、航向等信息
 * - p0: 当前参数点，包含参考点信息
 * - xe: 当前状态点与参考点在x方向的偏差
 * - ye: 当前状态点与参考点在y方向的偏差
 * - thetae: 当前状态点与参考点的航向偏差
 * - position_tol: 位置偏差容差，用于判断是否需要更新模型状态
 * 
 * @par 控制策略流程:
 * 1. 计算当前状态点与参考点的位置偏差和航向偏差
 * 2. 如果位置偏差小于容差，则直接更新模型状态为参考点状态
 * 3. 否则，根据偏差计算目标曲率和加速度
 * 4. 根据目标曲率和加速度更新模型控制输入
 * 5. 根据控制输入更新模型状态
 * 
 * @par 流程图:
 * @startuml
 * start
 * :计算位置偏差和航向偏差;
 * if (位置偏差小于容差?) then (是)
 *   :更新模型状态为参考点状态;
 * else (否)
 *   :计算目标曲率和加速度;
 *   :更新模型控制输入;
 *   :更新模型状态;
 * endif
 * stop
 * @enduml
 * 
 * @return bool 控制策略应用是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在每个状态点调用，确保控制策略正确应用
 * 
 * @warning 需确保输入参数的有效性，特别是优化控制模型指针和当前状态点索引
 */
bool OcpPathOptimizer::applyBicycleTrajectoryTrackerCtrlPolycy(std::shared_ptr<OptimalControlProblem> model,
                                                               const size_t& idx, const double& ref_local_s) {
  constexpr double position_tol = 0.01;

  auto& x0 = model->x(idx - 1);
  auto& p0 = model->p(idx - 1);
  double t0 = model->t(idx - 1);

  const double dx = p0("xr") - x0("x");
  const double dy = p0("yr") - x0("y");
  const double cos_theta = cos(x0("theta"));
  const double sin_theta = sin(x0("theta"));
  const double xe = cos_theta * dx + sin_theta * dy;
  const double ye = -sin_theta * dx + cos_theta * dy;
  const double thetae = math::NormalizeAngle(p0("thetar") - x0("theta"));
  if (std::abs(xe) < position_tol && std::abs(ye) < position_tol) {
    model->setXGuess("x", p0("xr"), idx);
    model->setXGuess("y", p0("yr"), idx);
    model->setXGuess("theta", getUnifySpaceHeading(x0("theta"), p0("thetar")), idx);
    model->setXGuess("v", p0("vr"), idx);
    model->setXGuess("kappa", p0("kr"), idx);
  } else {
    constexpr double eta_xe = 0.1, eta_ye = 0.1, eta_thetae = 0.1;
    constexpr double alpha_a = 2.0;
    DrivingDirection ref_point_direction = getInitPointDrivingDirection(ref_local_s);
    const double kappa_bound = getMaxKappaBound(t0, xy_planning_start_point_.v(), ref_point_direction);
    const double dkappa_bound = getMaxDKappaBound(t0, xy_planning_start_point_.v(), ref_point_direction);
    const double slide_xe = eta_xe * std::copysign(std::sqrt(std::abs(xe)), xe);
    const double slide_ye = eta_ye * std::copysign(std::sqrt(std::abs(ye)), ye);
    const double sigma_v = x0("v") - p0("vr") * cos(thetae) - slide_xe;
    const double sigma_thetae = sin(thetae) + slide_ye;
    const double target_kappa =
        std::clamp(p0("kr") + eta_thetae * std::copysign(std::sqrt(std::abs(sigma_thetae)), sigma_thetae), -kappa_bound,
                   kappa_bound);
    const double target_a =
        std::clamp(-alpha_a * sgn(sigma_v, 0.1), std::max(-10 * x0("v"), p0("a_lower")), p0("a_upper"));
    const double target_dkappa = -dkappa_bound * sgn(x0("kappa") - target_kappa, 0.01);
    model->setUGuess("a", target_a, idx - 1);
    model->setUGuess("dkappa", target_dkappa, idx - 1);
    model->rollOut(idx);
  }
  return true;
}



/**
 * @brief 考虑车辆宽度对路径边界的影响
 * @details 该函数用于根据车辆宽度调整路径边界，确保车辆在边界内安全行驶。
 * 
 * @param[in] width 车辆宽度
 * @param[in,out] bound_res 路径边界结果，包含边界信息和有效性标志
 * 
 * @par 输入参数说明:
 * - width: 车辆宽度，单位为米
 * - bound_res: 路径边界结果指针，包含边界信息和有效性标志
 * 
 * @par 关键变量说明:
 * - bound: 路径边界，包含s坐标、左边界和右边界
 * - valid: 边界有效性标志，true表示有效，false表示无效
 * - lmin: 左边界值
 * - lmax: 右边界值
 * 
 * @par 处理流程:
 * 1. 检查边界有效性，如果无效则直接返回
 * 2. 根据车辆宽度调整左右边界
 * 3. 如果调整后的左边界大于右边界，则标记边界为无效
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查边界有效性;
 * if (边界无效?) then (是)
 *   :直接返回;
 * else (否)
 *   :根据车辆宽度调整左右边界;
 *   if (左边界 > 右边界?) then (是)
 *     :标记边界为无效;
 *   endif
 * endif
 * stop
 * @enduml
 * 
 * @note 该函数应在路径规划过程中调用，确保路径边界考虑车辆宽度
 * 
 * @warning 需确保输入参数的有效性，特别是车辆宽度和路径边界结果
 */
void OcpPathOptimizer::considerVehicleWidth(const double width, OcpPathOptimizer::BoundRes* bound_res) {
  // TODO BLOCK INFO
  auto& [bound, valid] = *bound_res;
  if (!valid) return;
  auto& [s, lmin, lmax] = bound;
  lmin += width / 2;
  lmax -= width / 2;
  if (lmin > lmax) {
    valid = false;
  }
}

/**
 * @brief 检查是否存在有效的前一次路径
 * @details 该函数用于检查是否存在有效的前一次路径信息，通过判断prev_path_info_的第一个元素是否为真来确定。
 * 
 * @par 关键变量说明:
 * - prev_path_info_: 前一次路径信息，包含路径有效性标志、路径数据等信息
 * 
 * @return bool 是否存在有效的前一次路径，true表示存在，false表示不存在
 * 
 * @note 该函数应在需要检查前一次路径有效性时调用，确保路径规划的正确性
 * 
 * @warning 需确保prev_path_info_已正确初始化
 */
bool OcpPathOptimizer::hasValidPrevPath() { return std::get<0>(prev_path_info_); }

/**
 * @brief 检查是否存在有效的前一次模型
 * @details 该函数用于检查是否存在有效的前一次模型信息，通过判断前一次模型是否为空、模型名称是否匹配以及参数更新方法是否一致来确定。
 * 
 * @par 关键变量说明:
 * - prev_model_info_: 前一次模型信息，包含模型ID、配置文件、参数更新方法和模型指针
 * - prev_model: 前一次模型指针
 * - prev_profile: 前一次模型配置文件名称
 * - prev_update_params_method: 前一次模型参数更新方法
 * - curr_update_params_method_: 当前模型参数更新方法
 * 
 * @par 检查流程:
 * 1. 检查前一次模型是否为空
 * 2. 检查前一次模型名称是否与当前模型名称匹配
 * 3. 检查前一次模型参数更新方法是否与当前方法一致
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查前一次模型是否为空;
 * if (模型为空?) then (是)
 *   :返回false;
 * else (否)
 *   :检查模型名称是否匹配;
 *   if (名称匹配?) then (是)
 *     :检查参数更新方法是否一致;
 *     if (方法一致?) then (是)
 *       :返回true;
 *     else (否)
 *       :返回false;
 *     endif
 *   else (否)
 *     :返回false;
 *   endif
 * endif
 * stop
 * @enduml
 * 
 * @return bool 是否存在有效的前一次模型，true表示存在，false表示不存在
 * 
 * @note 该函数应在需要检查前一次模型有效性时调用，确保模型规划的正确性
 * 
 * @warning 需确保prev_model_info_已正确初始化
 */
bool OcpPathOptimizer::hasValidPrevModel() {
  auto prev_update_params_method = prev_model_info_.update_method;
  auto prev_model = prev_model_info_.model;
  auto prev_solve_status = prev_model_info_.status;
  return prev_model != nullptr && prev_model->name() == profile().model() &&
         prev_update_params_method == curr_update_params_method_ &&
         prev_solve_status != SolveStatus::NUMERICAL_ERROR_INTERRUPTION;
}

/**
 * @brief 获取当前优化器配置文件
 * @details 该函数用于获取当前优化器的配置文件，配置文件包含模型名称、默认参数、约束条件等信息。
 * 
 * @par 关键变量说明:
 * - optimizer_config_: 优化器配置，包含所有配置文件
 * - curr_profile_type_: 当前配置文件类型，用于从optimizer_config_中获取对应的配置文件
 * 
 * @return const OcpPathOptimizerProfile& 当前优化器配置文件的常量引用
 * 
 * @note 该函数应在需要获取当前优化器配置时调用，确保使用正确的配置文件
 * 
 * @warning 需确保curr_profile_type_已正确设置，且optimizer_config_中包含对应的配置文件
 */
const OcpPathOptimizerProfile& OcpPathOptimizer::profile() const {
  return optimizer_config_.profiles().at(curr_profile_type_);
}

/**
 * @brief 根据s坐标获取参考点
 * @details 该函数用于根据给定的s坐标从参考线中获取对应的参考点信息。
 * 
 * @param[in] s 参考点的s坐标，单位为米
 * 
 * @par 关键变量说明:
 * - target_ref_line_: 目标参考线对象，包含参考点信息
 * 
 * @return const ReferencePoint 返回的参考点对象，包含位置、航向、曲率等信息
 * 
 * @note 该函数应在需要获取参考点信息时调用，确保参考点信息正确获取
 * 
 * @warning 需确保s坐标在参考线范围内，且target_ref_line_已正确初始化
 */
const ReferencePoint OcpPathOptimizer::getInitPoint(const double s) { return target_ref_line_->getReferencePoint(s); }

/**
 * @brief 根据x,y坐标获取参考点
 * @details 该函数用于根据给定的x,y坐标从参考线中获取对应的参考点信息。
 * 
 * @param[in] x 参考点的x坐标，单位为米
 * @param[in] y 参考点的y坐标，单位为米
 * 
 * @par 关键变量说明:
 * - target_ref_line_: 目标参考线对象，包含参考点信息
 * 
 * @return const ReferencePoint 返回的参考点对象，包含位置、航向、曲率等信息
 * 
 * @note 该函数应在需要根据x,y坐标获取参考点信息时调用，确保参考点信息正确获取
 * 
 * @warning 需确保x,y坐标在参考线范围内，且target_ref_line_已正确初始化
 */
const ReferencePoint OcpPathOptimizer::getInitPoint(const double x, const double y) { return target_ref_line_->getReferencePoint(x,y); }

/**
 * @brief 根据s坐标获取参考点的行驶方向
 * @details 该函数用于根据给定的s坐标从参考线中获取对应的行驶方向信息。如果在一定范围内检测到U型转弯标志，则返回U型转弯方向，否则返回当前s坐标对应的行驶方向。
 * 
 * @param[in] s 参考点的s坐标，单位为米
 * 
 * @par 关键变量说明:
 * - target_ref_line_: 目标参考线对象，包含参考点信息
 * - check_s: 检查的s坐标范围，从s-30到s+30，步长为1米
 * - kDirectionUTurnOnly: U型转弯方向标志
 * 
 * @par 处理流程:
 * 1. 在s坐标前后30米范围内遍历检查
 * 2. 如果检测到U型转弯标志，则返回U型转弯方向
 * 3. 否则返回当前s坐标对应的行驶方向
 * 
 * @par 流程图:
 * @startuml
 * start
 * :遍历s坐标前后30米范围;
 * if (检测到U型转弯标志?) then (是)
 *   :返回U型转弯方向;
 * else (否)
 *   :返回当前s坐标的行驶方向;
 * endif
 * stop
 * @enduml
 * 
 * @return DrivingDirection 返回的行驶方向，包含直行、左转、右转、U型转弯等信息
 * 
 * @note 该函数应在需要获取参考点行驶方向时调用，确保行驶方向正确获取
 * 
 * @warning 需确保s坐标在参考线范围内，且target_ref_line_已正确初始化
 */
const DrivingDirection OcpPathOptimizer::getInitPointDrivingDirection(const double s) { 
  for(float check_s = std::fmax(0.0, s - 30.0); check_s < std::fmin(target_ref_line_->length(), s + 30.0); check_s += 1.0) {
    if(target_ref_line_->getDirectionFromS(check_s).direction == DrivingDirection::kDirectionUTurnOnly) {
      return DrivingDirection::kDirectionUTurnOnly;
    }
  }
  return target_ref_line_->getDirectionFromS(s).direction; 
}

/**
 * @brief 统一空间航向角计算
 * @details 该函数用于计算统一空间下的航向角，通过将目标航向角与基准航向角的差值归一化到[-π, π]范围内，然后与基准航向角相加得到最终结果。
 * 
 * @param[in] heading_base 基准航向角，单位为弧度
 * @param[in] heading 目标航向角，单位为弧度
 * 
 * @par 关键变量说明:
 * - delta_heading: 目标航向角与基准航向角的差值，经过归一化处理
 * 
 * @par 处理流程:
 * 1. 计算目标航向角与基准航向角的差值
 * 2. 将差值归一化到[-π, π]范围内
 * 3. 将归一化后的差值与基准航向角相加，得到统一空间下的航向角
 * 
 * @par 流程图:
 * @startuml
 * start
 * :计算航向角差值;
 * :归一化差值到[-π, π];
 * :计算统一空间航向角;
 * stop
 * @enduml
 * 
 * @return double 统一空间下的航向角，单位为弧度
 * 
 * @note 该函数应在需要统一航向角空间时调用，确保航向角计算正确
 * 
 * @warning 需确保输入航向角的有效性，特别是基准航向角
 */
double OcpPathOptimizer::getUnifySpaceHeading(const double heading_base, const double heading) {
  double delta_heading = math::NormalizeAngle(heading - heading_base);
  return heading_base + delta_heading;
}

/**
 * @brief 获取障碍物中心边界信息
 * @details 该函数用于根据参考点从障碍物路径边界中插值获取中心边界信息，包含边界值和有效性标志。
 * 
 * @param[in] rpt 参考点，包含位置、航向等信息
 * 
 * @par 关键变量说明:
 * - barrier_path_boundary_: 障碍物路径边界对象，包含边界信息和插值方法
 * - rpt.local_s(): 参考点的局部s坐标，用于插值
 * 
 * @par 返回值说明:
 * - std::tuple<double, double, double>: 包含s坐标、左边界值和右边界值
 * - bool: 边界有效性标志，true表示有效，false表示无效
 * 
 * @par 处理流程:
 * 1. 根据参考点的局部s坐标从障碍物路径边界中插值
 * 2. 返回插值结果和有效性标志
 * 
 * @par 流程图:
 * @startuml
 * start
 * :根据局部s坐标插值;
 * :返回边界信息和有效性标志;
 * stop
 * @enduml
 * 
 * @return std::pair<std::tuple<double, double, double>, bool> 返回的边界信息和有效性标志
 * 
 * @note 该函数应在需要获取障碍物中心边界信息时调用，确保边界信息正确获取
 * 
 * @warning 需确保参考点的局部s坐标在障碍物路径边界范围内
 */
std::pair<std::tuple<double, double, double>, bool> OcpPathOptimizer::getBarrierCenterBound(const ReferencePoint& rpt) {
  return barrier_path_boundary_.interpolate(rpt.local_s());
}

/**
 * @brief 获取障碍物边缘边界信息
 * @details 该函数用于根据参考点从障碍物路径边界中插值获取边缘边界信息，包含边界值和有效性标志。与中心边界相比，边缘边界考虑了车辆边缘的额外缓冲区。
 * 
 * @param[in] rpt 参考点，包含位置、航向等信息
 * 
 * @par 关键变量说明:
 * - barrier_path_boundary_: 障碍物路径边界对象，包含边界信息和插值方法
 * - rpt.local_s(): 参考点的局部s坐标，用于插值
 * 
 * @par 返回值说明:
 * - std::tuple<double, double, double>: 包含s坐标、左边界值和右边界值
 * - bool: 边界有效性标志，true表示有效，false表示无效
 * 
 * @par 处理流程:
 * 1. 根据参考点的局部s坐标从障碍物路径边界中插值
 * 2. 返回插值结果和有效性标志
 * 
 * @par 流程图:
 * @startuml
 * start
 * :根据局部s坐标插值;
 * :返回边界信息和有效性标志;
 * stop
 * @enduml
 * 
 * @return std::pair<std::tuple<double, double, double>, bool> 返回的边界信息和有效性标志
 * 
 * @note 该函数应在需要获取障碍物边缘边界信息时调用，确保边界信息正确获取
 * 
 * @warning 需确保参考点的局部s坐标在障碍物路径边界范围内
 * 
 * @todo 基于edge增加角的buffer
 */
std::pair<std::tuple<double, double, double>, bool> OcpPathOptimizer::getBarrierEdgeBound(const ReferencePoint& rpt) {
  // TODO 基于edge增加角的buffer
  return barrier_path_boundary_.interpolate(rpt.local_s());
}

// std::pair<std::tuple<double, double, double>, bool> OcpPathOptimizer::getSoftCenterBound(const double s, const bool allow_cross) {
//   auto res = soft_path_boundary_.interpolate(s);
//   auto& [soft_bound, soft_bound_valid] = res;
//   if (soft_bound_valid) {
//     auto& [curr_s, lower, upper] = soft_bound;
//     if (!allow_cross && lower >= upper) {
//       double mid = 0.5 * (lower + upper);
//       lower = mid - 1e-6;
//       upper = mid + 1e-6;
//     }
//   }
//   return res;
// }

/**
 * @brief 获取软约束中心边界信息
 * @details 该函数用于根据s坐标从软约束路径边界中插值获取中心边界信息，包含边界值和有效性标志。同时考虑障碍物边界和软约束边界的交集。
 * 
 * @param[in] s 查询点的s坐标，单位为米
 * @param[in] allow_cross 是否允许边界交叉，true表示允许，false表示不允许
 * 
 * @par 关键变量说明:
 * - barrier_path_boundary_: 障碍物路径边界对象，包含边界信息和插值方法
 * - soft_path_boundary_: 软约束路径边界对象，包含边界信息和插值方法
 * - bound_res: 返回的边界结果，包含s坐标、左边界值和右边界值
 * - valid_res: 边界有效性标志，true表示有效，false表示无效
 * 
 * @par 处理流程:
 * 1. 从障碍物路径边界中插值获取障碍物边界信息
 * 2. 从软约束路径边界中插值获取软约束边界信息
 * 3. 初始化返回结果，默认边界为最大范围
 * 4. 如果两个边界都有效，则计算两者的交集
 * 5. 如果不允许边界交叉且左边界大于右边界，则调整边界值
 * 
 * @par 流程图:
 * @startuml
 * start
 * :从障碍物边界插值;
 * :从软约束边界插值;
 * :初始化返回结果;
 * if (两个边界都有效?) then (是)
 *   :计算边界交集;
 *   if (不允许交叉且左边界>右边界?) then (是)
 *     :调整边界值;
 *   endif
 * endif
 * stop
 * @enduml
 * 
 * @return std::pair<std::tuple<double, double, double>, bool> 返回的边界信息和有效性标志
 * 
 * @note 该函数应在需要获取软约束中心边界信息时调用，确保边界信息正确获取
 * 
 * @warning 需确保s坐标在路径边界范围内
 */
std::pair<std::tuple<double, double, double>, bool> OcpPathOptimizer::getSoftCenterBound(const double s, const bool allow_cross) {
  const auto& [barrier_bound, has_barrier_bound] = barrier_path_boundary_.interpolate(s);
  const auto& [soft_bound, has_soft_bound] = soft_path_boundary_.interpolate(s);
  std::pair<std::tuple<double, double, double>, bool> res{std::tuple<double, double, double>(s, std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()),
                                                          has_barrier_bound && has_soft_bound};
  auto& [bound_res, valid_res] = res;
  if (valid_res) {
    auto& [curr_s, lower, upper] = bound_res;
    lower = std::min(std::max(lower, std::get<1>(soft_bound)), std::get<2>(barrier_bound));
    upper = std::max(std::min(upper, std::get<2>(soft_bound)), std::get<1>(barrier_bound));
    if (!allow_cross && lower >= upper) {
      double mid = 0.5 * (lower + upper);
      lower = mid - 1e-6;
      upper = mid + 1e-6;
    }
  }
  return res;
}

/**
 * @brief 获取软约束边缘边界信息
 * @details 该函数用于根据s坐标从软约束路径边界中插值获取边缘边界信息，包含边界值和有效性标志。与中心边界相比，边缘边界考虑了车辆边缘的额外缓冲区。
 * 
 * @param[in] s 查询点的s坐标，单位为米
 * @param[in] allow_cross 是否允许边界交叉，true表示允许，false表示不允许
 * 
 * @par 关键变量说明:
 * - soft_path_boundary_: 软约束路径边界对象，包含边界信息和插值方法
 * - soft_bound: 插值得到的软约束边界，包含s坐标、左边界和右边界
 * - soft_bound_valid: 边界有效性标志，true表示有效，false表示无效
 * 
 * @par 处理流程:
 * 1. 从软约束路径边界中插值获取边界信息
 * 2. 如果边界有效，则检查是否允许边界交叉
 * 3. 如果不允许边界交叉且左边界大于右边界，则调整边界值
 * 
 * @par 流程图:
 * @startuml
 * start
 * :从软约束边界插值;
 * if (边界有效?) then (是)
 *   if (不允许交叉且左边界>右边界?) then (是)
 *     :调整边界值;
 *   endif
 * endif
 * stop
 * @enduml
 * 
 * @return std::pair<std::tuple<double, double, double>, bool> 返回的边界信息和有效性标志
 * 
 * @note 该函数应在需要获取软约束边缘边界信息时调用，确保边界信息正确获取
 * 
 * @warning 需确保s坐标在路径边界范围内
 */
std::pair<std::tuple<double, double, double>, bool> OcpPathOptimizer::getSoftEdgeBound(const double s, const bool allow_cross) {
  auto res = soft_path_boundary_.interpolate(s);
  auto& [soft_bound, soft_bound_valid] = res;
  if (soft_bound_valid) {
    auto& [curr_s, lower, upper] = soft_bound;
    if (!allow_cross && lower >= upper) {
      double mid = 0.5 * (lower + upper);
      lower = mid - 1e-6;
      upper = mid + 1e-6;
    }
  }
  return res;
}

/**
 * @brief 获取参考线偏移量信息
 * @details 该函数用于根据参考点从参考线偏移量信息中插值获取偏移量信息，包含偏移量值和有效性标志。
 * 
 * @param[in] rpt 参考点，包含位置、航向等信息
 * 
 * @par 关键变量说明:
 * - ref_offsets_info_: 参考线偏移量信息对象，包含偏移量信息和插值方法
 * - rpt.local_s(): 参考点的局部s坐标，用于插值
 * 
 * @par 返回值说明:
 * - std::tuple<double, double>: 包含s坐标和偏移量值
 * - bool: 偏移量有效性标志，true表示有效，false表示无效
 * 
 * @par 处理流程:
 * 1. 根据参考点的局部s坐标从参考线偏移量信息中插值
 * 2. 返回插值结果和有效性标志
 * 
 * @par 流程图:
 * @startuml
 * start
 * :根据局部s坐标插值;
 * :返回偏移量信息和有效性标志;
 * stop
 * @enduml
 * 
 * @return std::pair<std::tuple<double, double>, bool> 返回的偏移量信息和有效性标志
 * 
 * @note 该函数应在需要获取参考线偏移量信息时调用，确保偏移量信息正确获取
 * 
 * @warning 需确保参考点的局部s坐标在参考线偏移量信息范围内
 */
std::pair<std::tuple<double, double>, bool> OcpPathOptimizer::getRefOffset(const ReferencePoint& rpt) {
  return ref_offsets_info_.interpolate(rpt.local_s());
}

/**
 * @brief 获取最大曲率限制
 * @details 该函数用于根据当前路径长度、初始速度和行驶方向获取最大曲率限制值。通过查表获取不同速度下的曲率限制，并根据行驶方向进行调整。
 * 
 * @param[in] length 当前路径长度，单位为米
 * @param[in] init_v 初始速度，单位为米/秒
 * @param[in] ref_direction 行驶方向，包含直行、左转、右转、U型转弯等信息
 * 
 * @par 关键变量说明:
 * - speed_vec: 速度向量，包含不同速度值
 * - kappa_vec: 曲率限制向量，包含不同速度对应的曲率限制值
 * - optimizer_config_: 优化器配置，包含速度-曲率限制映射表
 * - kapp_bound_ctrl: 查表得到的曲率限制值
 * - MS_KMH: 速度单位转换系数，将米/秒转换为公里/小时
 * 
 * @par 处理流程:
 * 1. 遍历速度-曲率限制映射表，填充速度向量和曲率限制向量
 * 2. 根据初始速度查表获取曲率限制值
 * 3. 如果行驶方向为U型转弯，则将曲率限制设置为固定值0.2
 * 4. 返回曲率限制值
 * 
 * @par 流程图:
 * @startuml
 * start
 * :遍历速度-曲率限制映射表;
 * :根据初始速度查表获取曲率限制值;
 * if (行驶方向为U型转弯?) then (是)
 *   :设置曲率限制为0.2;
 * endif
 * :返回曲率限制值;
 * stop
 * @enduml
 * 
 * @return double 最大曲率限制值
 * 
 * @note 该函数应在需要获取曲率限制时调用，确保曲率限制正确获取
 * 
 * @warning 需确保输入参数的有效性，特别是初始速度和行驶方向
 */
double OcpPathOptimizer::getMaxKappaBound(const double length, const double init_v, DrivingDirection ref_direction) const {
  std::vector<double> speed_vec;
  std::vector<double> kappa_vec;
  for(const auto& ele: optimizer_config_.speed_kappa_bound_map().elements()) {
    speed_vec.emplace_back(ele.speed());
    kappa_vec.emplace_back(ele.kappa_bound());
  }
  double kapp_bound_ctrl = math::TableLookUp1D(speed_vec, kappa_vec, init_v * MS_KMH);
  if(ref_direction == DrivingDirection::kDirectionUTurnOnly) {
    kapp_bound_ctrl = 0.2;
  }

  // double min_v_square_ = profile().constraint().min_v() * profile().constraint().min_v();
  // const double init_v_2 = init_v * init_v;
  // const double delta_v_2 = 2.0 * profile().constraint().decel() * length;
  // double curr_v_2 = std::max(min_v_square_, init_v_2 - delta_v_2);
  // return std::min(kapp_bound_ctrl, std::min(profile().constraint().max_kappa(), profile().constraint().centr_acc() / curr_v_2));
  return kapp_bound_ctrl;
}

/**
 * @brief 获取最大转向角限制
 * @details 该函数用于根据当前路径长度、初始速度和行驶方向获取最大转向角限制值。通过查表获取不同速度下的转向角限制，并根据行驶方向进行调整。
 * 
 * @param[in] length 当前路径长度，单位为米
 * @param[in] init_v 初始速度，单位为米/秒
 * @param[in] ref_direction 行驶方向，包含直行、左转、右转、U型转弯等信息
 * 
 * @par 关键变量说明:
 * - speed_vec: 速度向量，包含不同速度值
 * - steer_angle_vec: 转向角限制向量，包含不同速度对应的转向角限制值
 * - optimizer_config_: 优化器配置，包含速度-转向角限制映射表
 * - steer_angle_bound_ctrl: 查表得到的转向角限制值
 * - MS_KMH: 速度单位转换系数，将米/秒转换为公里/小时
 * 
 * @par 处理流程:
 * 1. 遍历速度-转向角限制映射表，填充速度向量和转向角限制向量
 * 2. 根据初始速度查表获取转向角限制值
 * 3. 如果行驶方向为U型转弯，则将转向角限制设置为固定值0.48（假设转角打满28°）
 * 4. 返回转向角限制值
 * 
 * @par 流程图:
 * @startuml
 * start
 * :遍历速度-转向角限制映射表;
 * :根据初始速度查表获取转向角限制值;
 * if (行驶方向为U型转弯?) then (是)
 *   :设置转向角限制为0.48;
 * endif
 * :返回转向角限制值;
 * stop
 * @enduml
 * 
 * @return double 最大转向角限制值
 * 
 * @note 该函数应在需要获取转向角限制时调用，确保转向角限制正确获取
 * 
 * @warning 需确保输入参数的有效性，特别是初始速度和行驶方向
 */
double OcpPathOptimizer::getMaxSteerAngleBound(const double length, const double init_v, DrivingDirection ref_direction) const {
  std::vector<double> speed_vec;
  std::vector<double> steer_angle_vec;
  for(const auto& ele: optimizer_config_.steering_angle_bound_map().elements()) {
    speed_vec.emplace_back(ele.speed());
    steer_angle_vec.emplace_back(ele.steer_angle_bound());
  }
  double steer_angle_bound_ctrl = math::TableLookUp1D(speed_vec, steer_angle_vec, init_v * MS_KMH);
  if(ref_direction == DrivingDirection::kDirectionUTurnOnly) {
    steer_angle_bound_ctrl = 0.48; //假设转角打满28°
  }
  return steer_angle_bound_ctrl;
}

/**
 * @brief 获取最大曲率变化率限制
 * @details 该函数用于根据当前路径长度、初始速度和行驶方向获取最大曲率变化率限制值。通过查表获取不同速度下的曲率变化率限制，并根据行驶方向进行调整。
 * 
 * @param[in] curr_length 当前路径长度，单位为米
 * @param[in] init_v 初始速度，单位为米/秒
 * @param[in] ref_direction 行驶方向，包含直行、左转、右转、U型转弯等信息
 * 
 * @par 关键变量说明:
 * - speed_vec: 速度向量，包含不同速度值
 * - dkappa_vec: 曲率变化率限制向量，包含不同速度对应的曲率变化率限制值
 * - optimizer_config_: 优化器配置，包含速度-曲率变化率限制映射表
 * - dkapp_bound_ctrl: 查表得到的曲率变化率限制值
 * - MS_KMH: 速度单位转换系数，将米/秒转换为公里/小时
 * 
 * @par 处理流程:
 * 1. 遍历速度-曲率变化率限制映射表，填充速度向量和曲率变化率限制向量
 * 2. 根据初始速度查表获取曲率变化率限制值
 * 3. 如果行驶方向为U型转弯，则将曲率变化率限制设置为固定值0.05
 * 4. 返回曲率变化率限制值
 * 
 * @par 流程图:
 * @startuml
 * start
 * :遍历速度-曲率变化率限制映射表;
 * :根据初始速度查表获取曲率变化率限制值;
 * if (行驶方向为U型转弯?) then (是)
 *   :设置曲率变化率限制为0.05;
 * endif
 * :返回曲率变化率限制值;
 * stop
 * @enduml
 * 
 * @return double 最大曲率变化率限制值
 * 
 * @note 该函数应在需要获取曲率变化率限制时调用，确保曲率变化率限制正确获取
 * 
 * @warning 需确保输入参数的有效性，特别是初始速度和行驶方向
 */
double OcpPathOptimizer::getMaxDKappaBound(const double curr_length, const double init_v, DrivingDirection ref_direction) const {
  std::vector<double> speed_vec;
  std::vector<double> dkappa_vec;
  for(const auto& ele: optimizer_config_.speed_kappa_bound_map().elements()) {
    speed_vec.emplace_back(ele.speed());
    dkappa_vec.emplace_back(ele.dkappa_bound());
  }
  double dkapp_bound_ctrl = math::TableLookUp1D(speed_vec, dkappa_vec, init_v * MS_KMH);
  if(ref_direction == DrivingDirection::kDirectionUTurnOnly) {
    dkapp_bound_ctrl = 0.05;
  }

  // double min_v_square_ = profile().constraint().min_v() * profile().constraint().min_v();
  // const double init_v_2 = init_v * init_v;
  // const double delta_v_2 = 2 * profile().constraint().decel() * curr_length;
  // double curr_v = std::sqrt(std::max(min_v_square_, init_v_2 - delta_v_2));
  // return std::min(dkapp_bound_ctrl, profile().constraint().max_dkappa() / curr_v);
  return dkapp_bound_ctrl;
}

/**
 * @brief 获取最大转向角变化率限制
 * @details 该函数用于根据当前路径长度、初始速度和行驶方向获取最大转向角变化率限制值。通过查表获取不同速度下的转向角变化率限制，并根据行驶方向进行调整。
 * 
 * @param[in] curr_length 当前路径长度，单位为米
 * @param[in] init_v 初始速度，单位为米/秒
 * @param[in] ref_direction 行驶方向，包含直行、左转、右转、U型转弯等信息
 * 
 * @par 关键变量说明:
 * - speed_vec: 速度向量，包含不同速度值
 * - dsteer_angle_vec: 转向角变化率限制向量，包含不同速度对应的转向角变化率限制值
 * - optimizer_config_: 优化器配置，包含速度-转向角变化率限制映射表
 * - dsteer_angle_bound_ctrl: 查表得到的转向角变化率限制值
 * - MS_KMH: 速度单位转换系数，将米/秒转换为公里/小时
 * 
 * @par 处理流程:
 * 1. 遍历速度-转向角变化率限制映射表，填充速度向量和转向角变化率限制向量
 * 2. 根据初始速度查表获取转向角变化率限制值
 * 3. 如果行驶方向为U型转弯，则将转向角变化率限制设置为固定值0.48（假设一秒内打满28°）
 * 4. 返回转向角变化率限制值
 * 
 * @par 流程图:
 * @startuml
 * start
 * :遍历速度-转向角变化率限制映射表;
 * :根据初始速度查表获取转向角变化率限制值;
 * if (行驶方向为U型转弯?) then (是)
 *   :设置转向角变化率限制为0.48;
 * endif
 * :返回转向角变化率限制值;
 * stop
 * @enduml
 * 
 * @return double 最大转向角变化率限制值
 * 
 * @note 该函数应在需要获取转向角变化率限制时调用，确保转向角变化率限制正确获取
 * 
 * @warning 需确保输入参数的有效性，特别是初始速度和行驶方向
 */
double OcpPathOptimizer::getMaxDSteerAngleBound(const double curr_length, const double init_v, DrivingDirection ref_direction) const {
  std::vector<double> speed_vec;
  std::vector<double> dsteer_angle_vec;
  for(const auto& ele: optimizer_config_.steering_angle_bound_map().elements()) {
    speed_vec.emplace_back(ele.speed());
    dsteer_angle_vec.emplace_back(ele.dsteer_angle_bound());
  }
  double dsteer_angle_bound_ctrl = math::TableLookUp1D(speed_vec, dsteer_angle_vec, init_v * MS_KMH);
  if(ref_direction == DrivingDirection::kDirectionUTurnOnly) {
    dsteer_angle_bound_ctrl = 0.48;  //假设一秒 打满28°
  }
  return dsteer_angle_bound_ctrl;
}

/**
 * @brief 获取特殊场景范围信息
 * @details 该函数用于获取特殊场景的范围信息，包含场景标签、起始s坐标和结束s坐标。这些信息用于在路径规划过程中识别和处理特殊场景。
 * 
 * @par 关键变量说明:
 * - special_scene_range_: 特殊场景范围信息，包含场景标签、起始s坐标和结束s坐标
 * 
 * @par 返回值说明:
 * - const std::vector<std::tuple<std::string, float, float>>&: 返回的特殊场景范围信息，包含场景标签、起始s坐标和结束s坐标
 * 
 * @par 处理流程:
 * 1. 直接返回特殊场景范围信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :返回特殊场景范围信息;
 * stop
 * @enduml
 * 
 * @return const std::vector<std::tuple<std::string, float, float>>& 返回的特殊场景范围信息
 * 
 * @note 该函数应在需要获取特殊场景范围信息时调用，确保特殊场景信息正确获取
 * 
 * @warning 需确保special_scene_range_已正确初始化
 */
const std::vector<std::tuple<std::string, float, float>>& OcpPathOptimizer::getSpecialSceneRange() const {
    return special_scene_range_;
}

/**
 * @brief 设置特殊场景范围信息
 * @details 该函数用于设置特殊场景的范围信息，包含场景标签、起始s坐标和结束s坐标。这些信息用于在路径规划过程中识别和处理特殊场景。
 * 
 * @param[in] special_scene_range 特殊场景范围信息，包含场景标签、起始s坐标和结束s坐标
 * 
 * @par 关键变量说明:
 * - special_scene_range_: 特殊场景范围信息，包含场景标签、起始s坐标和结束s坐标
 * 
 * @par 处理流程:
 * 1. 清空当前特殊场景范围信息
 * 2. 遍历输入的特殊场景范围信息，逐个添加到special_scene_range_中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :清空当前特殊场景范围信息;
 * :遍历输入的特殊场景范围信息;
 * :逐个添加到special_scene_range_中;
 * stop
 * @enduml
 * 
 * @note 该函数应在需要更新特殊场景范围信息时调用，确保特殊场景信息正确设置
 * 
 * @warning 需确保输入参数的有效性，特别是场景标签和s坐标范围
 */
void OcpPathOptimizer::setSpecialSceneRange(std::vector<std::tuple<std::string, float, float>> special_scene_range) {
  special_scene_range_.clear();
  for (const auto& range : special_scene_range) {
    special_scene_range_.emplace_back(range);
  }
}

/**
 * @brief 设置参考线偏移量信息
 * @details 该函数用于设置参考线偏移量信息，包含s坐标和偏移量值。这些信息用于在路径规划过程中调整参考线的位置。
 * 
 * @param[in] ref_offsets_info 参考线偏移量信息，包含s坐标和偏移量值
 * 
 * @par 关键变量说明:
 * - ref_offsets_info_: 参考线偏移量信息，包含s坐标和偏移量值
 * 
 * @par 处理流程:
 * 1. 清空当前参考线偏移量信息
 * 2. 遍历输入的参考线偏移量信息，逐个添加到ref_offsets_info_中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :清空当前参考线偏移量信息;
 * :遍历输入的参考线偏移量信息;
 * :逐个添加到ref_offsets_info_中;
 * stop
 * @enduml
 * 
 * @note 该函数应在需要更新参考线偏移量信息时调用，确保参考线偏移量信息正确设置
 * 
 * @warning 需确保输入参数的有效性，特别是s坐标和偏移量值
 */
void OcpPathOptimizer::setRefOffsetsInfo(const std::vector<std::pair<double, double>> &ref_offsets_info){
  ref_offsets_info_.clear();
  for(auto ref_offset : ref_offsets_info){
    ref_offsets_info_.emplace_back(ref_offset.first, ref_offset.second);
  }
}

/**
 * @brief 获取速度限制
 * @details 该函数用于根据当前s坐标和参考线曲率获取速度限制值。速度限制由地图限速和参考线曲率限速共同决定，取两者中的较小值。
 * 
 * @param[in] curr_s 当前s坐标，单位为米
 * @param[in] ref_kappa 参考线曲率，单位为1/米
 * 
 * @par 关键变量说明:
 * - target_ref_line_: 目标参考线对象，包含地图限速信息
 * - kappa_table_: 曲率表，用于查表获取横向加速度限制
 * - lat_a_limit_: 横向加速度限制表，用于计算曲率限速
 * 
 * @par 处理流程:
 * 1. 从目标参考线中获取当前s坐标对应的地图限速
 * 2. 如果参考线曲率接近0，则设置为一个极小值
 * 3. 根据参考线曲率查表获取横向加速度限制
 * 4. 根据曲率和横向加速度限制计算曲率限速
 * 5. 返回地图限速和曲率限速中的较小值
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取地图限速;
 * :检查曲率是否接近0;
 * :根据曲率查表获取横向加速度限制;
 * :计算曲率限速;
 * :返回地图限速和曲率限速中的较小值;
 * stop
 * @enduml
 * 
 * @return double 速度限制值，单位为米/秒
 * 
 * @note 该函数应在需要获取速度限制时调用，确保速度限制正确获取
 * 
 * @warning 需确保输入参数的有效性，特别是当前s坐标和参考线曲率
 */
double OcpPathOptimizer::getSpeedLimit(const double curr_s, const double ref_kappa) {
  // 地图限速和参考线曲率限速取小
  double map_speed_limit = target_ref_line_->GetSpeedLimitFromS(curr_s);
  if(map_speed_limit < 1e-6) map_speed_limit = kMaxSpeedMS;
  double kappa = std::fabs(ref_kappa) < 1e-6 ? 1e-6 : ref_kappa;
  double kappa_lat_a_limit = gpal::pnc::planning::math::TableLookUp1D(kappa_table_, lat_a_limit_, std::fabs(kappa));
  double kappa_speed_limit = std::sqrt(std::fabs(kappa_lat_a_limit / kappa));
  return std::min(map_speed_limit, kappa_speed_limit);
}
}