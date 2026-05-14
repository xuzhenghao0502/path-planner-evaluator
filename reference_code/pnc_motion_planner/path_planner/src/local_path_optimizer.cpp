/**
 * @file local_path_optimizer.cpp
 * @brief 局部路径优化器实现文件
 * @details 该文件实现了局部路径优化器的核心功能，包括路径优化、速度点生成、模型初始化等。局部路径优化器用于在路径规划过程中对局部路径进行优化，确保路径的连续性和可行性。
 */

#include "path_planner/local_path_optimizer.h"
#include "ocp/ocp_model.h"
#include "base/singleton.h"
#include "util/timer.h"
#include "config_manager/config_manager.h"

// #define LOG_OCP_DATA
// #define LOG_SUCCESS_STAGE
// #define LOG_FAILED_STAGE

namespace gpal::pnc::planning {

namespace {

/**
 * @brief 符号函数
 * @details 该函数用于计算给定值的符号，同时避免除零错误。通过引入一个极小值eps，确保分母不为零。
 * 
 * @param[in] value 输入值
 * @param[in] eps 极小值，用于避免除零错误
 * 
 * @par 关键变量说明:
 * - value: 输入值，用于计算符号
 * - eps: 极小值，用于避免除零错误
 * 
 * @par 处理流程:
 * 1. 计算输入值的绝对值
 * 2. 将输入值除以其绝对值加上极小值eps，得到符号
 * 
 * @par 流程图:
 * @startuml
 * start
 * :计算输入值的绝对值;
 * :将输入值除以其绝对值加上极小值eps;
 * :返回符号;
 * stop
 * @enduml
 * 
 * @return double 符号值，范围为[-1, 1]
 * 
 * @note 该函数应在需要计算符号时调用，确保符号计算正确且避免除零错误
 * 
 * @warning 需确保输入参数的有效性，特别是极小值eps的选择
 */
double sgn(const double value, const double eps) { return value / (std::abs(value) + eps); }
}  // namespace

#ifdef LOG_OCP_DATA
static int debug_count_ = 0;
static OcpDataField ocp_data_field_;
#endif

/**
 * @brief 初始化函数
 * @details 该函数用于初始化LocalPathOptimizer对象，加载车辆配置和优化器配置，并设置默认的规划类型。
 * 
 * @par 关键变量说明:
 * - config_manager: 配置管理器单例，用于获取车辆配置和优化器配置
 * - vehicle_config_: 车辆配置对象，包含车辆的基本参数
 * - vehicle_param_: 车辆参数对象，包含车辆的详细参数
 * - optimizer_config_: 局部路径优化器配置对象，包含优化器的各项参数
 * - profile_type_: 规划类型，默认为"regular"
 * 
 * @par 处理流程:
 * 1. 获取配置管理器单例
 * 2. 从配置管理器中加载车辆配置，并初始化vehicle_config_和vehicle_param_
 * 3. 从配置管理器中加载局部路径优化器配置
 * 4. 设置默认的规划类型为"regular"
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取配置管理器单例;
 * :加载车辆配置;
 * :初始化vehicle_config_和vehicle_param_;
 * :加载局部路径优化器配置;
 * :设置默认规划类型为"regular";
 * stop
 * @enduml
 * 
 * @return bool 初始化结果，true表示成功，false表示失败
 * 
 * @note 该函数应在LocalPathOptimizer对象创建后立即调用，确保配置正确加载
 * 
 * @warning 需确保配置管理器已正确初始化，且相关配置存在
 */
bool LocalPathOptimizer::init() {
  auto config_manager = Singleton<ConfigManager>::get_instance();
  vehicle_config_ = std::make_shared<VehicleConfig>(config_manager->vehicle_config());
  vehicle_param_ = std::make_shared<VehicleParam>(config_manager->vehicle_config().vehicle_param());
  optimizer_config_ = config_manager->getConfig<LocalPathOptimizerConfig>("LocalPathOptimizerConfig");
  
  profile_type_ = "regular";
  return true;
}

/**
 * @brief 预处理函数
 * @details 该函数用于在局部路径规划前进行预处理，包括检查输入路径的有效性、设置规划起点和规划方向，并更新规划类型。
 * 
 * @param[in] start_point 规划起点，包含位置、航向等信息
 * @param[in] prev_path 前一路径，用于确定规划方向
 * @param[in] profile_type 规划类型，用于指定规划策略
 * 
 * @par 关键变量说明:
 * - xy_planning_start_point_: 规划起点，包含位置、航向等信息
 * - is_reverse_plan_: 规划方向标志，true表示反向规划，false表示正向规划
 * - profile_type_: 规划类型，用于指定规划策略
 * 
 * @par 处理流程:
 * 1. 检查前一路径的长度，如果小于2则返回失败
 * 2. 设置规划起点
 * 3. 根据前一路径的第一个点的方向确定规划方向
 * 4. 更新规划类型
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查前一路径长度;
 * if (路径长度 < 2?) then (是)
 *   :返回失败;
 * else (否)
 *   :设置规划起点;
 *   :确定规划方向;
 *   :更新规划类型;
 *   :返回成功;
 * endif
 * stop
 * @enduml
 * 
 * @return bool 预处理结果，true表示成功，false表示失败
 * 
 * @note 该函数应在局部路径规划前调用，确保规划起点、方向和类型正确设置
 * 
 * @warning 需确保输入参数的有效性，特别是前一路径的长度
 */
bool LocalPathOptimizer::preProcess(const TrajectoryPt& start_point, const DiscretizedPath& prev_path, const std::string& profile_type) {
  if (prev_path.size() < 2) {
    PINFO << "[LocalPathOptimizer::preProcess]: prev_path is too short";
    return false;
  }

  if(prev_path.back().s() < optimizer_config_.min_ref_path_length()){
    return false;
  }
  
  xy_planning_start_point_ = start_point;
  is_reverse_plan_ = prev_path.front().direction() == PathPt::Direction::BACKWARD ? true : false;
  profile_type_ = profile_type;
  return true;
}

// 主线规划：采用BicycleTrajectoryTracker ocp模型，模拟横向控制的行驶轨迹，规划起点在自车当前位置.
// 若主线规划失败，则进行基于滑模控制的local_path规划(getLocalPathInitGuess)
// 若local_path长度不足，或无local_path，或无实时横向轨迹，则进行fall back规划(GenerateFallBackLocalPath)，即基于CV运动学模型进行推演，且保证local_path长度.
/**
 * @brief 生成局部路径
 * @details 该函数用于生成局部路径，采用BicycleTrajectoryTracker OCP模型进行规划。如果主线规划失败，则进行基于滑模控制的local_path规划；如果local_path长度不足或无实时横向轨迹，则进行fall back规划。
 * 
 * @param[in] curr_state 当前车辆状态，包含位置、速度等信息
 * @param[in] prev_speed_data 前一次速度数据，用于生成速度点
 * @param[in] path 参考路径，用于规划局部路径
 * @param[in] curr_stamp 当前时间戳，用于速度数据插值
 * @param[in] max_length 最大路径长度，用于限制生成的局部路径长度
 * 
 * @par 关键变量说明:
 * - local_path_res_: 生成的局部路径结果
 * - local_path_guess_: 局部路径的初始猜测
 * - optimizer_config_: 优化器配置，包含是否强制使用CV模型等参数
 * - speed_points: 速度点序列，用于OCP模型求解
 * 
 * @par 处理流程:
 * 1. 清空局部路径结果和初始猜测
 * 2. 如果参考路径不为空，则生成速度点序列
 * 3. 初始化OCP模型并进行求解
 * 4. 如果求解成功，则提取状态和控制量生成局部路径
 * 5. 如果求解失败，则返回初始猜测作为局部路径
 * 
 * @par 流程图:
 * @startuml
 * start
 * :清空局部路径结果和初始猜测;
 * if (参考路径为空?) then (是)
 *   :返回失败;
 * else (否)
 *   :生成速度点序列;
 *   :初始化OCP模型;
 *   :求解OCP模型;
 *   if (求解成功?) then (是)
 *     :提取状态和控制量生成局部路径;
 *   else (否)
 *     :返回初始猜测作为局部路径;
 *   endif
 * endif
 * stop
 * @enduml
 * 
 * @return LocalPathStatus 局部路径生成状态，SOLVED表示成功，SOLVE_FAILD表示失败
 * 
 * @note 该函数应在需要生成局部路径时调用，确保局部路径正确生成
 * 
 * @warning 需确保输入参数的有效性，特别是当前车辆状态和参考路径
 */
LocalPathOptimizer::LocalPathStatus LocalPathOptimizer::generateLocalPathProc(const VehicleState& curr_state,
                                                                              const SpeedData& prev_speed_data,
                                                                              const DiscretizedPath& path,
                                                                              const int64_t curr_stamp,
                                                                              const double max_length) {
  LocalPathStatus res_status = LocalPathStatus::SOLVED;
  local_path_res_.clear();
  local_path_guess_.clear();

  if (!path.empty()) {
    util::TimerLogger<std::milli> timer("local_path_ocp", [](const std::string& str) {ERT_PLOG_I <<"[LocalPathOptimizer::generateLocalPathProc]: " << str ; });
    std::vector<gpal::pnc::SpeedPoint> speed_points;
    if (optimizer_config_.force_cv_model() ||
        !getSpeedPointsFromPrevSpeedData(prev_speed_data, curr_stamp, 1.0, 0.0, 10.0, 0.1,
                                         std::max(max_length, path.back().s()), &speed_points)) {
      getSpeedPointsFromCVModel(std::max(1.0, curr_state.linear_velocity()), 0.0, 10.0, 0.1,
                                std::max(max_length, path.back().s()), &speed_points);
    }
    auto model = initLocalPathModel(profile(), speed_points, path, curr_state);
    auto status = model->solve();
    ERT_PLOG_I <<"[LocalPathOptimizer::generateLocalPathProc]: " << model->name()
              << "  status = " << status
              << "  iteration = " << model->solver()->iteration_number() ;

#ifdef LOG_OCP_LOCALPATH_DATA
    model->logData();
#endif
    if (status == SolveStatus::SOLVED) {
      auto states = model->getX();
      auto ctrls = model->getU();
      double tmp_s = 0;
      double delta_t = 0.1;
      for (int i = 0; i + 1 < states.size(); ++i) {
        const auto& state = states[i]; // x y theta v kappa
        const auto& ctrl = ctrls[i];   // a dkappa
        if(i > 0 ){
          tmp_s += state(3) * delta_t + 0.5 * ctrl(0) * delta_t * delta_t;
        }
        local_path_res_.emplace_back(state(0), state(1), curr_state.z(), 0.0, state(2), state(4), abs(tmp_s), ctrl(1), 0.0); // x y z slope theta kappa s dkappa ddkappa
      }    
    } else {
      res_status = LocalPathStatus::SOLVE_FAILD;
      // return init guess
      local_path_res_ = local_path_guess_;
#ifdef LOG_OCP_LOCALPATH_DATA
      // log data if need to debug solver
      if (model->logDataSize() > 500) {
        auto ocp_data_field = model->getLogData();
        std::ofstream of(SpdlogWrapper::instance().getLogConfig().log_path() +
                         fmt::format("{}_data_{}.bin", ocp_data_field.model(), debug_count_++));
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
  return res_status;
}

/**
 * @brief 从前一次速度数据中获取速度点
 * @details 该函数用于从前一次速度数据中提取速度点序列，用于局部路径规划。通过插值方法获取指定时间范围内的速度点，并确保速度点的有效性。
 * 
 * @param[in] prev_speed_data 前一次速度数据，包含时间、速度、加速度等信息
 * @param[in] curr_stamp 当前时间戳，用于确定速度数据的起始时间
 * @param[in] min_v 最小速度限制，用于确保速度点的速度不低于该值
 * @param[in] t0 起始时间偏移量，相对于当前时间戳
 * @param[in] t1 结束时间偏移量，相对于当前时间戳
 * @param[in] dt 时间步长，用于生成速度点序列
 * @param[in] max_s 最大路径长度，用于限制生成的速度点序列的总长度
 * @param[out] speed_points 生成的速度点序列，包含时间、速度、加速度等信息
 * 
 * @par 关键变量说明:
 * - prev_start_t: 前一次速度数据的起始时间
 * - prev_start_s: 前一次速度数据的起始路径长度
 * - delta_t: 时间步长，用于计算加速度和路径长度
 * 
 * @par 处理流程:
 * 1. 检查输出指针是否为空，如果为空则返回失败
 * 2. 根据当前时间戳从前一次速度数据中获取起始速度点
 * 3. 清空速度点序列
 * 4. 遍历时间范围，通过插值获取速度点
 * 5. 如果速度低于最小速度限制，则设置为最小速度
 * 6. 计算加速度和路径长度，并确保路径长度不超过最大限制
 * 7. 将生成的速度点添加到序列中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查输出指针是否为空;
 * if (指针为空?) then (是)
 *   :返回失败;
 * else (否)
 *   :获取起始速度点;
 *   :清空速度点序列;
 *   :遍历时间范围;
 *   :插值获取速度点;
 *   :检查速度是否低于最小限制;
 *   :计算加速度和路径长度;
 *   :检查路径长度是否超过最大限制;
 *   :将速度点添加到序列中;
 *   :返回成功;
 * endif
 * stop
 * @enduml
 * 
 * @return bool 生成速度点序列的结果，true表示成功，false表示失败
 * 
 * @note 该函数应在需要从前一次速度数据中提取速度点时调用，确保速度点序列正确生成
 * 
 * @warning 需确保输入参数的有效性，特别是前一次速度数据和当前时间戳
 */
bool LocalPathOptimizer::getSpeedPointsFromPrevSpeedData(const SpeedData& prev_speed_data, const int64_t curr_stamp,
                                                         const double min_v, const double t0, const double t1,
                                                         const double dt, const double max_s,
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
 * @brief 从CV模型中获取速度点
 * @details 该函数用于根据恒定速度（CV）模型生成速度点序列，用于局部路径规划。通过恒定速度假设，生成指定时间范围内的速度点序列。
 * 
 * @param[in] v0 初始速度，用于生成速度点序列
 * @param[in] t0 起始时间，用于确定速度点序列的起始时间
 * @param[in] t1 结束时间，用于确定速度点序列的结束时间
 * @param[in] dt 时间步长，用于生成速度点序列
 * @param[in] max_s 最大路径长度，用于限制生成的速度点序列的总长度
 * @param[out] speed_points 生成的速度点序列，包含时间、速度、路径长度等信息
 * 
 * @par 关键变量说明:
 * - end_s: 结束路径长度，根据初始速度和时间范围计算
 * - end_t: 结束时间，根据结束路径长度和初始速度计算
 * - N: 速度点数量，根据时间范围和步长计算
 * 
 * @par 处理流程:
 * 1. 检查输出指针是否为空，如果为空则返回失败
 * 2. 计算结束路径长度和结束时间
 * 3. 计算速度点数量
 * 4. 清空速度点序列
 * 5. 遍历时间范围，生成速度点
 * 6. 将生成的速度点添加到序列中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查输出指针是否为空;
 * if (指针为空?) then (是)
 *   :返回失败;
 * else (否)
 *   :计算结束路径长度和结束时间;
 *   :计算速度点数量;
 *   :清空速度点序列;
 *   :遍历时间范围;
 *   :生成速度点;
 *   :将速度点添加到序列中;
 *   :返回成功;
 * endif
 * stop
 * @enduml
 * 
 * @return bool 生成速度点序列的结果，true表示成功，false表示失败
 * 
 * @note 该函数应在需要从CV模型中生成速度点时调用，确保速度点序列正确生成
 * 
 * @warning 需确保输入参数的有效性，特别是初始速度和时间范围
 */
bool LocalPathOptimizer::getSpeedPointsFromCVModel(const double v0, const double t0, const double t1, const double dt,
                                                   const double max_s, std::vector<gpal::pnc::SpeedPoint>* speed_points) {
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
 * @brief 初始化局部路径模型
 * @details 该函数用于初始化局部路径优化模型，根据配置文件和输入参数设置模型参数，并选择合适的模型进行初始化。
 * 
 * @param[in] profile 局部路径优化配置文件，包含模型名称、默认参数等信息
 * @param[in] speed_points 速度点序列，用于确定模型的时间步长和范围
 * @param[in] path 参考路径，用于模型初始化
 * @param[in] curr_state 当前车辆状态，用于模型初始化
 * 
 * @par 关键变量说明:
 * - local_path_model_: 局部路径优化模型，用于求解局部路径
 * - warm_start: 是否使用热启动标志，如果模型已存在且时间步长匹配，则使用热启动
 * - profile.model(): 模型名称，用于确定使用哪种模型进行优化
 * 
 * @par 处理流程:
 * 1. 检查是否需要创建新模型或使用热启动
 * 2. 如果不需要热启动，则设置模型的基本配置和参数
 * 3. 根据模型名称选择合适的初始化方法
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查是否需要创建新模型或使用热启动;
 * if (模型不存在或模型名称不匹配?) then (是)
 *   :创建新模型;
 * else (否)
 *   :使用热启动;
 * endif
 * :设置模型基本配置和参数;
 * if (模型名称 == "BicycleTrajectoryTracker") then (是)
 *   :初始化BicycleTrajectoryTracker模型;
 * endif
 * stop
 * @enduml
 * 
 * @return std::shared_ptr<OptimalControlProblem> 初始化后的局部路径优化模型
 * 
 * @note 该函数应在需要初始化局部路径优化模型时调用，确保模型正确配置和初始化
 * 
 * @warning 需确保输入参数的有效性，特别是配置文件和速度点序列
 */
std::shared_ptr<OptimalControlProblem> LocalPathOptimizer::initLocalPathModel(const LocalPathOptimizerProfile& profile,
                                                                              const std::vector<gpal::pnc::SpeedPoint>& speed_points,
                                                                              const DiscretizedPath& path, 
                                                                              const VehicleState& curr_state) {
  bool warm_start = false;
  if (local_path_model_ == nullptr || local_path_model_->name() != profile.model()) {
    local_path_model_ = OptimalControlProblem::create(profile.model());
  } else if (local_path_model_->N() == speed_points.size() - 1) {
    warm_start = true;
  }
  if (local_path_model_ != nullptr) {
    if (!warm_start) {
      local_path_model_->mutable_config()->set_integrator_type(OcpConfig::FORWARD_EULER);
      local_path_model_->mutable_config()->set_dt(0.1);
      local_path_model_->mutable_config()->set_max_iter_time(0.1);
      local_path_model_->mutable_config()->set_horizon_length(speed_points.size() - 1);

      for (auto& param : profile.default_params()) {
        local_path_model_->setParam(param.key(), param.value());
      }
      if (profile.has_ipm_config()) {
        local_path_model_->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(profile.ipm_config());
      } else {
        local_path_model_->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(optimizer_config_.local_path_ipm_config());
      }
    }

    if (local_path_model_->name() == "BicycleTrajectoryTracker") {
      initBicycleTrajectoryTracker(local_path_model_, speed_points, path, curr_state, warm_start);
    }
  }
  return local_path_model_;
}

/**
 * @brief 初始化自行车轨迹跟踪器模型
 * @details 该函数用于初始化自行车轨迹跟踪器模型，设置模型的初始状态和参数，并生成局部路径的初始猜测。
 * 
 * @param[in] model 最优控制问题模型，用于设置初始状态和参数
 * @param[in] speed_points 速度点序列，用于确定模型的时间步长和范围
 * @param[in] path 参考路径，用于模型初始化
 * @param[in] curr_state 当前车辆状态，用于模型初始化
 * @param[in] warm_start 是否使用热启动标志，如果为true，则跳过部分初始化步骤
 * 
 * @par 关键变量说明:
 * - start_state: 初始状态向量，包含位置、航向、速度等信息
 * - start_state_v: 初始速度，根据规划方向调整符号
 * - local_path_guess_: 局部路径的初始猜测，用于后续优化
 * - prev_heading: 前一时刻的航向角，用于计算当前航向角
 * 
 * @par 处理流程:
 * 1. 初始化状态向量，设置初始位置、航向、速度等信息
 * 2. 如果不需要热启动，则初始化模型
 * 3. 生成局部路径的初始猜测
 * 4. 遍历速度点序列，设置模型参数
 * 5. 根据规划方向调整速度上下限
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化状态向量;
 * if (不需要热启动?) then (是)
 *   :初始化模型;
 * endif
 * :生成局部路径初始猜测;
 * :遍历速度点序列;
 * :设置模型参数;
 * if (反向规划?) then (是)
 *   :调整速度上下限;
 * else (否)
 *   :设置正常速度上下限;
 * endif
 * stop
 * @enduml
 * 
 * @return bool 初始化结果，true表示成功，false表示失败
 * 
 * @note 该函数应在需要初始化自行车轨迹跟踪器模型时调用，确保模型正确配置和初始化
 * 
 * @warning 需确保输入参数的有效性，特别是当前车辆状态和参考路径
 */
bool LocalPathOptimizer::initBicycleTrajectoryTracker(std::shared_ptr<OptimalControlProblem> model,
                                                      const std::vector<gpal::pnc::SpeedPoint>& speed_points,
                                                      const DiscretizedPath& path, const VehicleState& curr_state,
                                                      const bool warm_start) {
  Eigen::VectorXd start_state = Eigen::VectorXd::Zero(5);
  double start_state_v = is_reverse_plan_ ? -1.0 * std::max(1.0, curr_state.linear_velocity())
                                          : std::max(1.0, curr_state.linear_velocity());
  start_state << curr_state.x(), curr_state.y(), curr_state.yaw(), start_state_v, curr_state.kappa(); // x y theta v kappa
  model->setX0(start_state);
  if (!warm_start) {
    model->init();
  }
  
  local_path_guess_.emplace_back(start_state(0), start_state(1), curr_state.z(), 0.0, start_state(2), start_state(4), 0.0, 0.0, 0.0); // x y z slope theta kappa s dkappa ddkappa

  double prev_heading = curr_state.yaw();
  for (int i = 0; i <= model->N(); ++i) {
    auto& speed_point = speed_points[i];
    auto path_state = path.evaluate(speed_point.s());
    if (i > 0) {
      applyBicycleTrajectoryTrackerCtrlPolycy(model, i);
    }
    // model->setUGuess("a", speed_point.a(), i);
    model->setParam("xr", path_state.x(), i);
    model->setParam("yr", path_state.y(), i);
    model->setParam("thetar", getUnifySpaceHeading(prev_heading, path_state.theta()), i);
    model->setParam("kr", path_state.kappa(), i);
    if (is_reverse_plan_) {
      model->setParam("vr", -1.0 * speed_point.v(), i);
      model->setParam("v_upper", 0.0, i);
      model->setParam("v_lower", -1.0 * speed_point.v() - 1.0, i);
    } else {
      model->setParam("vr", speed_point.v(), i);
      model->setParam("v_upper", speed_point.v() + 1.0, i);
      model->setParam("v_lower", 0.0, i);
    }
    prev_heading = model->x(i, "theta");
  }
  return true;
}

/**
 * @brief 应用自行车轨迹跟踪器控制策略
 * @details 该函数用于应用自行车轨迹跟踪器的控制策略，通过滑模控制方法调整车辆的加速度和曲率变化率，使车辆能够跟踪参考路径。
 * 
 * @param[in] model 最优控制问题模型，用于设置控制量和状态量
 * @param[in] idx 当前时间步的索引，用于确定当前控制量和状态量
 * 
 * @par 关键变量说明:
 * - position_tol: 位置误差容限，用于判断是否需要进行滑模控制
 * - x0: 当前时间步的状态向量
 * - p0: 当前时间步的参考路径参数
 * - dx, dy: 当前位置与参考位置的偏差
 * - xe, ye: 转换到车辆坐标系下的位置误差
 * - thetae: 航向角误差
 * - eta_xe, eta_ye, eta_thetae: 滑模控制增益参数
 * - kappa_bound, dkappa_bound: 曲率和曲率变化率的限制值
 * - slide_xe, slide_ye: 滑模控制中的滑动误差
 * - sigma_v, sigma_thetae: 滑模控制中的速度误差和航向角误差
 * - target_kappa, target_a, target_dkappa: 目标曲率、加速度和曲率变化率
 * 
 * @par 处理流程:
 * 1. 计算当前位置与参考位置的偏差
 * 2. 如果位置误差小于容限，则直接使用参考路径参数设置控制量和状态量
 * 3. 否则，应用滑模控制策略计算目标曲率、加速度和曲率变化率
 * 4. 设置控制量并更新状态量
 * 5. 保存初始猜测用于后续优化
 * 
 * @par 流程图:
 * @startuml
 * start
 * :计算当前位置与参考位置的偏差;
 * if (位置误差 < 容限?) then (是)
 *   :直接使用参考路径参数设置控制量和状态量;
 * else (否)
 *   :应用滑模控制策略计算目标曲率、加速度和曲率变化率;
 *   :设置控制量并更新状态量;
 * endif
 * :保存初始猜测用于后续优化;
 * stop
 * @enduml
 * 
 * @return bool 控制策略应用结果，true表示成功，false表示失败
 * 
 * @note 该函数应在每个时间步调用，确保车辆能够正确跟踪参考路径
 * 
 * @warning 需确保输入参数的有效性，特别是模型和索引
 */
bool LocalPathOptimizer::applyBicycleTrajectoryTrackerCtrlPolycy(std::shared_ptr<OptimalControlProblem> model,
                                                                 const size_t& idx) {
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
    // 滑模控制: 通过设置合适的滑动误差和趋近律,使系统状态快速地到达滑模面并保持在滑模面上,实现对系统的有效控制;其非线性调整和符号保留特性，使系统在大误差时能够快速响应，在小误差时能够平滑过渡，具有良好的鲁棒性和动态性能.
    constexpr double eta_xe = 0.1;
    double eta_ye = is_reverse_plan_ ? -0.1 : 0.1;
    double eta_thetae = is_reverse_plan_ ? -0.1 : 0.1;
    constexpr double alpha_a = 2.0;
    const double kappa_bound = getMaxKappaBound(xy_planning_start_point_.v());
    const double dkappa_bound = getMaxDKappaBound(xy_planning_start_point_.v());
    // std::copysign: 保留误差符号,确保滑动误差方向与原始误差一致,正确调整系统状态.
    // std::sqrt(std::abs(xe)): 对误差大小进行非线性调整,当误差较大时,此值较大,滑动误差slide_xe也较大,系统状态快速向滑模面靠拢;反之滑动误差较小,系统状态能更平滑地接近滑模面,避免过冲和振荡.
    // eta_xe和eta_ye: 控制增益参数,用于调整滑动误差的大小,可在保证系统稳定性的前提下,提高系统的响应速度和控制精度.   
    const double slide_xe = eta_xe * std::copysign(std::sqrt(std::abs(xe)), xe);
    const double slide_ye = eta_ye * std::copysign(std::sqrt(std::abs(ye)), ye);
    // sigma_v: 先计算v 与 vr在当前方向上的投影 之差,再减去沿x的滑动误差，以进一步调整速度误差;滑动误差的引入是滑模控制中常见的方法，用于提高系统的鲁棒性和响应速度.
    const double sigma_v = x0("v") - p0("vr") * cos(thetae) - slide_xe;
    // sigma_thetae: 先计算角度误差的正弦值,可反映角度误差的大小和方向,再加上沿y的滑动误差,以进一步调整角度误差.滑模控制中使用正弦函数可更好描述角度误差在不同方向上的影响;也同样通过引入滑动误差来调整角度误差,帮助系统更快地消除角度偏差,实现更精确的轨迹跟踪.
    const double sigma_thetae = sin(thetae) + slide_ye;
    const double target_kappa =
        std::clamp(p0("kr") + eta_thetae * std::copysign(std::sqrt(std::abs(sigma_thetae)), sigma_thetae), -kappa_bound,
                   kappa_bound);
    // max and min 是为了让\dot{v} = -kv, 通过改变平衡点为v=0防止出现开环反馈计算的位置出现向行驶方向反方向运动的情况
    const double k = -10.0;
    double target_a = std::clamp(-alpha_a * sgn(sigma_v, 0.1), std::max(k * x0("v"), p0("a_lower")), p0("a_upper"));
    if (is_reverse_plan_) {
      target_a = std::clamp(-alpha_a * sgn(sigma_v, 0.1), -p0("a_upper"), std::min(k * x0("v"), -p0("a_lower")));
    }

    const double target_dkappa = -dkappa_bound * sgn(x0("kappa") - target_kappa, 0.01);
    model->setUGuess("a", target_a, idx - 1);
    model->setUGuess("dkappa", target_dkappa, idx - 1);
    model->rollOut(idx);
  }
  // save initial guess
  auto& state = model->x(idx); // x y theta v kappa
  auto& control = model->u(idx); // a dkappa
  auto s = local_path_guess_.back().s() + state(3) * 0.1 + 0.5 * control(0) * 0.1 * 0.1;
  local_path_guess_.emplace_back(state(0), state(1), local_path_guess_.back().z(), 0.0, state(2), state(4), s, control(1), 0.0); // x y z slope theta kappa s dkappa ddkappa

  return true;
}

/**
 * @brief 生成后备局部路径
 * @details 该函数用于在主路径规划失败时生成后备局部路径，基于恒定速度（CV）模型进行路径推演，确保车辆能够继续行驶。
 * 
 * @param[in] direction 行驶方向，1表示正向，-1表示反向
 * @param[in] curr_state 当前车辆状态，包含位置、航向、速度等信息
 * @param[in] cur_speed 当前速度，用于确定推演路径的速度
 * @param[in] cur_s 当前路径长度，用于确定推演路径的起始位置
 * 
 * @par 关键变量说明:
 * - start_state: 初始状态向量，包含位置、航向、速度等信息
 * - v_lower, v_upper: 速度上下限，用于限制推演路径的速度
 * - cv: 推演路径的速度，根据方向和当前速度确定
 * - N_: 推演路径的步数，根据配置文件和速度计算
 * 
 * @par 处理流程:
 * 1. 初始化状态向量，设置初始位置、航向、速度等信息
 * 2. 生成推演路径的起始点
 * 3. 计算推演路径的速度和步数
 * 4. 遍历步数，推演路径的每个点
 * 5. 根据速度和方向计算每个点的位置、航向和路径长度
 * 6. 将推演路径的点添加到结果中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化状态向量;
 * :生成推演路径的起始点;
 * :计算推演路径的速度和步数;
 * :遍历步数;
 * :根据速度和方向计算每个点的位置、航向和路径长度;
 * :将推演路径的点添加到结果中;
 * stop
 * @enduml
 * 
 * @return DiscretizedPath 生成的后备局部路径，包含位置、航向、路径长度等信息
 * 
 * @note 该函数应在主路径规划失败时调用，确保车辆能够继续行驶
 * 
 * @warning 需确保输入参数的有效性，特别是当前车辆状态和速度
 */
DiscretizedPath LocalPathOptimizer::GenerateFallBackLocalPath(int direction, const VehicleState& curr_state,
                                                              float cur_speed, float cur_s) {
  Eigen::VectorXd start_state = Eigen::VectorXd::Zero(5);
  start_state << curr_state.x(), curr_state.y(), curr_state.yaw(), curr_state.linear_velocity(), curr_state.kappa();// x y theta v kappa

  DiscretizedPath res;
  res.emplace_back(start_state(0), start_state(1), curr_state.z(), 0.0, start_state(2), start_state(4), cur_s, 0.0, 0.0); // x y z slope theta kappa s dkappa ddkappa
  float v_lower = 1.0;
  float v_upper = 40.0;
  float step = 0.1;
  float cv = std::clamp(cur_speed, v_lower, v_upper);
  int N_ = std::fmax(optimizer_config_.force_tail_length(), optimizer_config_.keep_moving_time() * cv) / step;
  // cv *= direction;
  cv = step * direction;
  // 推演路径的1s内方向维持，随后成指数衰减
  double hold_range = optimizer_config_.cv_model_hold_time() * cur_speed;
  double coef = 1.0;
// 推演轨迹.
  for (int i = 1; i <= N_; ++i) {
    if (i * step > hold_range) {
      coef *= optimizer_config_.cv_model_decay_rate();
    }
    PathPt prev_state = res.back();
    float delta_s = std::abs(cv);
    float delta_theta = cv * start_state(4) * coef;
    float delta_x = cv * cos(prev_state.theta() + 0.5 * delta_theta);
    float delta_y = cv * sin(prev_state.theta() + 0.5 * delta_theta);   
    res.emplace_back(prev_state.x() + delta_x, prev_state.y() + delta_y, curr_state.z(), 0.0, prev_state.theta() + delta_theta, start_state(4),
                     prev_state.s() + delta_s, 0.0, 0.0); // x y z slope theta kappa s dkappa ddkappa
    if (res.back().s() > optimizer_config_.force_tail_length()) {
      break;
    }
  }
  
  return res;
}

/**
 * @brief 统一空间航向角
 * @details 该函数用于将给定的航向角与基准航向角统一到同一空间，确保航向角的变化在[-π, π]范围内。
 * 
 * @param[in] heading_base 基准航向角，用于确定航向角的参考方向
 * @param[in] heading 待统一的航向角，需要与基准航向角统一到同一空间
 * 
 * @par 关键变量说明:
 * - delta_heading: 航向角与基准航向角的差值，经过归一化处理
 * 
 * @par 处理流程:
 * 1. 计算航向角与基准航向角的差值
 * 2. 对差值进行归一化处理，确保其在[-π, π]范围内
 * 3. 返回统一后的航向角
 * 
 * @par 流程图:
 * @startuml
 * start
 * :计算航向角与基准航向角的差值;
 * :对差值进行归一化处理;
 * :返回统一后的航向角;
 * stop
 * @enduml
 * 
 * @return double 统一后的航向角，范围为[heading_base - π, heading_base + π]
 * 
 * @note 该函数应在需要将航向角统一到同一空间时调用，确保航向角的变化范围正确
 * 
 * @warning 需确保输入参数的有效性，特别是基准航向角和待统一的航向角
 */
double LocalPathOptimizer::getUnifySpaceHeading(const double heading_base, const double heading) {
  double delta_heading = math::NormalizeAngle(heading - heading_base);
  return heading_base + delta_heading;
}

/**
 * @brief 获取最大曲率限制
 * @details 该函数用于根据当前速度获取最大曲率限制值，通过查表的方式从配置文件中获取不同速度对应的曲率限制值。
 * 
 * @param[in] init_v 当前速度，用于查表获取对应的曲率限制值
 * 
 * @par 关键变量说明:
 * - speed_vec: 速度向量，包含配置文件中定义的不同速度值
 * - kappa_vec: 曲率限制向量，包含配置文件中定义的不同速度对应的曲率限制值
 * - kapp_bound_ctrl: 查表得到的曲率限制值
 * 
 * @par 处理流程:
 * 1. 从配置文件中加载速度向量和曲率限制向量
 * 2. 根据当前速度查表获取对应的曲率限制值
 * 3. 返回查表得到的曲率限制值
 * 
 * @par 流程图:
 * @startuml
 * start
 * :从配置文件中加载速度向量和曲率限制向量;
 * :根据当前速度查表获取对应的曲率限制值;
 * :返回查表得到的曲率限制值;
 * stop
 * @enduml
 * 
 * @return double 最大曲率限制值
 * 
 * @note 该函数应在需要获取最大曲率限制时调用，确保曲率限制值正确获取
 * 
 * @warning 需确保配置文件中的速度向量和曲率限制向量正确配置
 */
double LocalPathOptimizer::getMaxKappaBound(const double init_v) const {
  std::vector<double> speed_vec;
  std::vector<double> kappa_vec;
  for(const auto& ele: optimizer_config_.speed_kappa_bound_map().elements()) {
    speed_vec.emplace_back(ele.speed());
    kappa_vec.emplace_back(ele.kappa_bound());
  }
  double kapp_bound_ctrl = math::TableLookUp1D(speed_vec, kappa_vec, init_v * MS_KMH);
  return kapp_bound_ctrl;
}

/**
 * @brief 获取最大曲率变化率限制
 * @details 该函数用于根据当前速度获取最大曲率变化率限制值，通过查表的方式从配置文件中获取不同速度对应的曲率变化率限制值。
 * 
 * @param[in] init_v 当前速度，用于查表获取对应的曲率变化率限制值
 * 
 * @par 关键变量说明:
 * - speed_vec: 速度向量，包含配置文件中定义的不同速度值
 * - dkappa_vec: 曲率变化率限制向量，包含配置文件中定义的不同速度对应的曲率变化率限制值
 * - dkapp_bound_ctrl: 查表得到的曲率变化率限制值
 * 
 * @par 处理流程:
 * 1. 从配置文件中加载速度向量和曲率变化率限制向量
 * 2. 根据当前速度查表获取对应的曲率变化率限制值
 * 3. 返回查表得到的曲率变化率限制值
 * 
 * @par 流程图:
 * @startuml
 * start
 * :从配置文件中加载速度向量和曲率变化率限制向量;
 * :根据当前速度查表获取对应的曲率变化率限制值;
 * :返回查表得到的曲率变化率限制值;
 * stop
 * @enduml
 * 
 * @return double 最大曲率变化率限制值
 * 
 * @note 该函数应在需要获取最大曲率变化率限制时调用，确保曲率变化率限制值正确获取
 * 
 * @warning 需确保配置文件中的速度向量和曲率变化率限制向量正确配置
 */
double LocalPathOptimizer::getMaxDKappaBound(const double init_v) const {
  std::vector<double> speed_vec;
  std::vector<double> dkappa_vec;
  for(const auto& ele: optimizer_config_.speed_kappa_bound_map().elements()) {
    speed_vec.emplace_back(ele.speed());
    dkappa_vec.emplace_back(ele.dkappa_bound());
  }
  double dkapp_bound_ctrl = math::TableLookUp1D(speed_vec, dkappa_vec, init_v * MS_KMH);
  return dkapp_bound_ctrl;
}

/**
 * @brief 获取当前规划类型的配置文件
 * @details 该函数用于根据当前规划类型获取对应的配置文件。如果当前规划类型在配置文件中存在，则返回对应的配置文件；否则返回默认的"regular"配置文件。
 * 
 * @par 关键变量说明:
 * - profile_type_: 当前规划类型，用于确定返回哪个配置文件
 * - optimizer_config_.local_path_profiles(): 局部路径优化配置文件集合，包含所有规划类型的配置文件
 * 
 * @par 处理流程:
 * 1. 检查当前规划类型是否在配置文件中存在
 * 2. 如果存在，则返回对应的配置文件
 * 3. 如果不存在，则返回默认的"regular"配置文件
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查当前规划类型是否在配置文件中存在;
 * if (存在?) then (是)
 *   :返回对应的配置文件;
 * else (否)
 *   :返回默认的"regular"配置文件;
 * endif
 * stop
 * @enduml
 * 
 * @return const LocalPathOptimizerProfile& 当前规划类型的配置文件
 * 
 * @note 该函数应在需要获取当前规划类型的配置文件时调用，确保返回正确的配置文件
 * 
 * @warning 需确保配置文件中包含默认的"regular"配置文件
 */
const LocalPathOptimizerProfile& LocalPathOptimizer::profile() const {
  if (optimizer_config_.local_path_profiles().find(profile_type_) != optimizer_config_.local_path_profiles().end()) {
    return optimizer_config_.local_path_profiles().at(profile_type_);
  } else {
    return optimizer_config_.local_path_profiles().at("regular");
  }
}

}
