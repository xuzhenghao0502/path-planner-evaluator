#include "spatiotemporal_optimizer/spatiotemporal_optimizer.h"

// #define LOG_OCP_DATA
// #define LOG_init_OCP_DATA
// #define LOG_BAD_OCP_DATA

namespace gpal::pnc::planning {

#ifdef LOG_OCP_DATA
static int debug_count_ = 0;
static OcpDataField ocp_data_field_;
#endif

#ifdef LOG_init_OCP_DATA
static int debug_count_ = 0;
static OcpDataField ocp_data_field_;
#endif

#ifdef LOG_BAD_OCP_DATA
static int debug_count_ = 0;
static OcpDataField ocp_data_field_;
#endif

std::string SpatiotemporalOptimizer::id() const {
  return "SpatiotemporalOptimizer";
}
bool SpatiotemporalOptimizer::init() {
  // Initialization logic
  auto config_manager = Singleton<ConfigManager>::get_instance();
  vehicle_param_ = std::make_shared<VehicleParam>(config_manager->vehicle_config().vehicle_param());
  async_data_.reset();
  return true;
}

void SpatiotemporalOptimizer::reset() {
  async_data_.reset();
  // Reset logic
}

bool SpatiotemporalOptimizer::run(SpatiotemporalPlannerDataManager& data_manager) {
  // Running logic
  data_manager.mutableOptimizerInfo().async_status =
      asyncProc(std::chrono::milliseconds(optimizer_config_.timeout_ms()), data_manager);
  data_manager.mutableOptimizerInfo().solve_info = gtest_solver_info_;
  data_manager.mutableOutputData().debug_info.append(async_optimizer_debug_info_);

  return true;
}

SpatiotemporalPlannerDataManager::OptimizerInfo::AsyncStatus SpatiotemporalOptimizer::asyncProc(
    const std::chrono::milliseconds& timeout, SpatiotemporalPlannerDataManager& data_manager) {
  util::TimerLogger<std::milli> timer(
      "asyncProc", [](const std::string& str) { STLOG(W, "[SpatiotemporalOcpOptimizer::asyncProc]: " + str); });
  auto& trajectory_data = data_manager.mutableOutputData().planning_trajectory;
  auto& is_valid = data_manager.mutableOutputData().is_valid;
  is_valid = false;
  SpatiotemporalPlannerDataManager::OptimizerInfo::AsyncStatus status =
      SpatiotemporalPlannerDataManager::OptimizerInfo::AsyncStatus::ASYNC_SOLVED;

  // 1) 前处理，失败则返回PREPROCESS_FAILED.
  {
    auto tap_timer = timer.tap("preProcess");
    if (!preProcess(data_manager)) {
      status = SpatiotemporalPlannerDataManager::OptimizerInfo::AsyncStatus::ASYNC_FAILED;
      return status;
    }
  }
  double warm_start_time = 1e3;
  double solve_time = 1e3;
  async_data_.reset();
  if (async_data_ == nullptr) {
    auto async_timer = timer.tap("optimizer");
    auto async_data = std::make_shared<SpatiotemporalPlannerDataManager::OptimizerInfo::SolverData>();
    async_data->ref_id = target_ref_line_->id();
    async_data->context = curr_context_type_;  // 来源于边界label类型信息
    auto snapshot = std::make_shared<SolverInputSnapshot>();
    snapshot->vehicle_param = vehicle_param_;
    snapshot->sl_planning_start_point = sl_planning_start_point_;
    snapshot->lane_keep_start_s = lane_keep_start_s_;
    snapshot->cost_weight_max_decay_length = cost_weight_max_decay_length_;
    snapshot->target_ref_line = *target_ref_line_;  // Value copy
    snapshot->target_obstacle_data = target_obstacle_data_;
    snapshot->v_steer_bound = v_steer_bound_;
    snapshot->v_dsteer_bound = v_dsteer_bound_;
    snapshot->soft_speed_bound = soft_speed_bound_;
    snapshot->hard_speed_bound = hard_speed_bound_;
    snapshot->soft_lateral_bound = soft_lateral_bound_;
    snapshot->hard_lateral_bound = hard_lateral_bound_;
    snapshot->s_hard_bound = s_hard_bound_;
    snapshot->s_soft_bound = s_soft_bound_;
    snapshot->ref_line_data = ref_line_data_;
    snapshot->optimizer_parameters = optimizer_parameters_;
    snapshot->init_guess = init_guess_;
    snapshot->context = context();
    {
      auto tap_timer = async_timer.tap("initModel");
      // async_data->model = initModel(context(), trajectory_data);
      async_data->model = initModel(snapshot->context, trajectory_data, snapshot);
    }
#ifdef LOG_init_OCP_DATA
    auto& [prev_ref_id, prev_context, prev_update_params_method, prev_model, prev_solve_status] = prev_model_info_;
    if (async_data->model->logData(&ocp_data_field_) > 0) {
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

    // 3) ocp实时规划(独立线程).
    // 本帧内warm start
    async_data_ = async_data;
    SolveStatus solve_status;
    {
      auto tap_timer = async_timer.tap("warmstart");
      async_data_->model->mutable_config()->mutable_solver()->mutable_ipm()->set_max_iter_num(5);
      async_data_->model->mutable_config()->mutable_solver()->mutable_ipm()->set_max_iter_time(0.01);
      solve_status = async_data_->model->solve();
      warm_start_time = async_data_->model->solver()->getSolveInfo().computation_time * 1000.0;
    }
    //  STLOG(W, "[SpatiotemporalOcpOptimizer::asyncProc pred]: ", "context = ", async_data_->context, "  ",
    //       "ipm_solver status = ", async_data_->status, "  iter = ", async_data_->model->solver()->iteration_number(),
    //       "  computation_ms = ", async_data_->model->solver()->getSolveInfo().computation_time * 1000.0,
    //       "  first_order = ", async_data_->model->solver()->getSolveInfo().first_order_condition,
    //       "  complementary = ", async_data_->model->solver()->getSolveInfo().complementary_condtion,
    //       "  equality = ", async_data_->model->solver()->getSolveInfo().equality_condition,
    //       "  inequality = ", async_data_->model->solver()->getSolveInfo().inequality_condition);
    // start solve

    async_data_->model->mutable_config()->mutable_solver()->mutable_ipm()->set_max_iter_num(
        optimizer_config_.ipm_config().max_iter_num());
    async_data_->model->mutable_config()->mutable_solver()->mutable_ipm()->set_max_iter_time(
        optimizer_config_.ipm_config().max_iter_time());
    async_data_->model->mutable_config()->mutable_solver()->mutable_ipm()->set_auto_update_param(
        optimizer_config_.use_auto_update_param());

    async_data_->model->setX0(start_state_);
    async_data_->model->updateParams();

#ifdef LOG_OCP_DATA
    auto& [prev_ref_id, prev_context, prev_update_params_method, prev_model, prev_solve_status] = prev_model_info_;
    if (async_data->model->logData(&ocp_data_field_) > 0) {
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
    {
      auto tap_timer = async_timer.tap("solving");
      solve_status = async_data_->model->solve();
      solve_time = async_data_->model->solver()->getSolveInfo().computation_time * 1000.0;
      async_data_->status = solve_status;
      async_data_->is_finished = true;
    }
    STLOG(W, "[SpatiotemporalOcpOptimizer::asyncProc after]: ", "context = ", async_data_->context, "  ",
          "ipm_solver status = ", async_data_->status, "  iter = ", async_data_->model->solver()->iteration_number(),
          " warm_start_ms = ", warm_start_time, "  computation_ms = ",
          async_data_->model->solver()->getSolveInfo().computation_time * 1000.0 + warm_start_time,
          "  first_order = ", async_data_->model->solver()->getSolveInfo().first_order_condition,
          "  complementary = ", async_data_->model->solver()->getSolveInfo().complementary_condtion,
          "  equality = ", async_data_->model->solver()->getSolveInfo().equality_condition,
          "  inequality = ", async_data_->model->solver()->getSolveInfo().inequality_condition);
  }

#ifdef LOG_BAD_OCP_DATA
  if (async_data_->model->solver()->iteration_number() > 30
      || async_data_->status == SolveStatus::NUMERICAL_ERROR_INTERRUPTION
      || async_data_->status == SolveStatus::INFEASIBLE_DETECTED || 1) {
    if (async_data_->model->logData(&ocp_data_field_) > 0) {
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
  }

#endif
  // 5) 后处理.
  {
    auto tap_timer = timer.tap("postProcess");
    async_optimizer_debug_info_ +=
        fmt::format("\ncontext:{}, ipm_solver:{}, iter:{}, warm_time:{:.2f},solve time:{:.2f}", async_data_->context,
                    static_cast<int>(async_data_->status), async_data_->model->solver()->iteration_number(),
                    warm_start_time, async_data_->model->solver()->getSolveInfo().computation_time * 1000.0);

    for (auto& index : async_data_->model->solver()->getSolveInfo().infeasible_index) {
      async_optimizer_debug_info_ += "\ninfeasible:{" + std::to_string(std::get<0>(index)) + ", "
                                     + std::to_string(std::get<1>(index)) + ", " + std::to_string(std::get<2>(index))
                                     + ", " + std::to_string(std::get<3>(index)) + ", "
                                     + fmt::format("{:.3e}", std::get<4>(index)) + "}";
    }

    if (optimizer_config_.eable_resolve() || !optimizer_config_.use_auto_update_param()) {
      checkValid(async_data_);
    }
    if (async_data_->status == SolveStatus::MAX_ITERATION_REACHED
        || async_data_->status == SolveStatus::TIME_LIMIT_REACHED) {
      if (resultCheck(async_data_) && optimizer_config_.eable_result_check()) {
        async_optimizer_debug_info_ += "\n resultCheck success";
        async_data_->status = SolveStatus::SOLVED;
      } else {
        async_optimizer_debug_info_ += "\n resultCheck failed";
      }
    }
    prev_model_info_ =
        std::tuple<std::string, std::string, UpdateParamsMethod, std::shared_ptr<OptimalControlProblem>, SolveStatus>(
            async_data_->ref_id, async_data_->context, curr_update_params_method_, async_data_->model,
            async_data_->model->solver()->getSolveInfo().status);
    prev_iteration_ = async_data_->model->solver()->iteration_number();
    setSolverInfo(async_data_->model->solver()->getSolveInfo());
    if (async_data_->status == SolveStatus::SOLVED) {
      if (!trajNonCollisionCheck(async_data_->model)) {
        async_optimizer_debug_info_ += "\n collision check failed \n";
        return SpatiotemporalPlannerDataManager::OptimizerInfo::AsyncStatus::COLLISION_CHEAK_FAILED;
      }
      if (!tranStatesToPathData(async_data_->model, trajectory_data)) {
        async_optimizer_debug_info_ += "\n tran state to traj failed \n";
        return SpatiotemporalPlannerDataManager::OptimizerInfo::AsyncStatus::ASYNC_FAILED;
      }
      //=======================================================================
      // print obstacle
      if (0) {
        for (int i = 0; i < target_obstacle_data_.size(); i++) {
          for (int j = 0; j < 51; j++) {
            auto obs = target_obstacle_data_[i][j];
            STLOG(W, "obstacle_", i, " info:", " time=", j * 0.1, " x=", obs.x, " y=", obs.y, " theta=", obs.theta,
                  " width=", obs.width, " length=", obs.length, " a=", obs.a, " b=", obs.b);
          }
        }
        // print traj
        double shift_dis = vehicle_param_->length() / 2 - vehicle_param_->rear_overhang();
        for (int i = 0; i < trajectory_data.size(); i++) {
          auto pt = trajectory_data.at(i);
          STLOG(W, "trajectory info:", i, " time=", i * 0.1,
                " x=", pt.path_pt().x() + shift_dis * cos(pt.path_pt().theta()),
                " y=", pt.path_pt().y() + shift_dis * sin(pt.path_pt().theta()), " theta=", pt.path_pt().theta(),
                " width=", vehicle_param_->width(), " length=", vehicle_param_->length(), " speed=", pt.v() * 3.6);
        }

        for (auto& ref_pt : target_ref_line_.get()->reference_points()) {
          if (ref_pt.local_s() < sl_planning_start_point_.first.at(0) - 5
              || ref_pt.local_s() > sl_planning_start_point_.first.at(0) + 200) {
            continue;
          }
          STLOG(W, "reference line info:", " x=", ref_pt.x(), " y=", ref_pt.y());
        }
      }
      //=======================================================================
      is_valid = true;
    } else {
      status = SpatiotemporalPlannerDataManager::OptimizerInfo::AsyncStatus::ASYNC_FAILED;
    }
  }

  return status;
}

bool SpatiotemporalOptimizer::preProcess(SpatiotemporalPlannerDataManager& data_manager) {
  const auto& input_data = data_manager.inputData();
  const auto& scenario_info = data_manager.scenarioInfo();
  const auto& decision_result = input_data.decision_result;
  const auto& decision_objects = data_manager.objectsInfo();
  auto& optimizer_info = data_manager.mutableOptimizerInfo();
  auto& prev_trajectory = data_manager.mutableOutputData().planning_trajectory;

  hard_speed_bound_ = data_manager.boundaryInfo().trajectory_boundary->hardSpeedBound();
  soft_speed_bound_ = data_manager.boundaryInfo().trajectory_boundary->softSpeedBound();

  hard_lateral_bound_ = data_manager.boundaryInfo().trajectory_boundary->hardLateralBound();
  soft_lateral_bound_ = data_manager.boundaryInfo().trajectory_boundary->softLateralBound();

  v_steer_bound_ = data_manager.boundaryInfo().velocity_related_boundary->hardSteeringBound();
  v_dsteer_bound_ = data_manager.boundaryInfo().velocity_related_boundary->hardDsteeringBound();

  s_hard_bound_ = data_manager.boundaryInfo().time_related_boundary->sHardBound();
  s_soft_bound_ = data_manager.boundaryInfo().time_related_boundary->sSoftBound();

  ref_line_data_ = data_manager.boundaryInfo().trajectory_boundary->referencePointsData();

  // data clear
  target_obstacle_data_.clear();
  init_guess_.clear();
  accumulated_s_.clear();

  // data reset
  target_ref_line_.reset(new ReferenceLine(input_data.target_ref_line_info->ref_line()));
  xy_planning_start_point_ = input_data.vehicle_info->start_point;
  sl_planning_start_point_ = input_data.vehicle_info->sl_info;
  const auto time_grid = data_manager.gridsInfo().time_grid_info;
  dt_ = time_grid.step;
  horizon_ = time_grid.horizon;
  N_ = size_t(horizon_ / dt_);
  async_optimizer_debug_info_ = "[ST]: ";
  optimizer_config_ = *data_manager.configInfo().spatiotemporal_optimizer_profile;
  // std::cout<<"optimizer_config_  "<< optimizer_config_.DebugString()<<std::endl;

  // 处理参数表格
  steer_speed_vec_.clear();
  steer_coffient_vec_.clear();
  dsteer_speed_vec_.clear();
  dsteer_coffient_vec_.clear();
  lane_change_speed_vec_.clear();
  lane_change_params_vec_.clear();

  for (const auto& ele : optimizer_config_.speed_steer_cofficient_table().elements()) {
    steer_speed_vec_.emplace_back(ele.key());
    steer_coffient_vec_.emplace_back(ele.value());
    // std::cout<<"steer table: key "<< ele.key()<<" value "<< ele.value()<<std::endl;
  }

  for (const auto& ele : optimizer_config_.speed_dsteer_cofficient_table().elements()) {
    dsteer_speed_vec_.emplace_back(ele.key());
    dsteer_coffient_vec_.emplace_back(ele.value());
    // std::cout<<"dsteer table: key "<< ele.key()<<" value "<< ele.value()<<std::endl;
  }

  for(const auto& multi_ele : optimizer_config_.lane_change_params_table().multi_elements()) {
    lane_change_speed_vec_.emplace_back(multi_ele.key());
    std::vector<std::pair<string, double>> params;
    for (const auto& ele : multi_ele.elements()) {
      params.emplace_back(ele.name(), ele.value());
      // std::cout<<"lane change table: key "<< multi_ele.key()<<" name "<< ele.name()<<" value "<< ele.value()<<std::endl;
    }
    lane_change_params_vec_.emplace_back(params);
  }

  optimizer_parameters_ = data_manager.optimizerInfo().optimizer_parameters;
  start_state_ << sl_planning_start_point_.first.at(0), xy_planning_start_point_.path_pt().x(),
      xy_planning_start_point_.path_pt().y(), xy_planning_start_point_.path_pt().theta(),
      xy_planning_start_point_.path_pt().front_steer(), xy_planning_start_point_.v(), xy_planning_start_point_.a();
  STLOG(D, "[SpatiotemporalOcpOptimizer::updateSpatiotemporalPlanner]: s = ", start_state_(0), " x = ", start_state_(1),
        " y = ", start_state_(2), " theta = ", start_state_(3), " steer = ", start_state_(4), " v = ", start_state_(5),
        " a = ", start_state_(6));
  // todo:  add parameter check && bounds check
  if (optimizer_parameters_.empty()) {
    STLOG(W, "[SpatiotemporalOcpOptimizer::preProcess]: optimizer_parameters_ is empty!");
    return false;
  }

  // 以自车当前位置s为起始，计算accumulated_s_
  for (int i = 0; i <= N_; ++i) {
    accumulated_s_.emplace_back(sl_planning_start_point_.first[0] + i * dt_);
  }

  lane_keep_start_s_ = optimizer_info.lane_keep_start_s;
  cost_weight_max_decay_length_ = optimizer_config_.cost_weight_max_decay_length();
  target_obstacle_data_ = decision_objects.target_objects_info;
  current_driving_scenario_ = optimizer_info.current_driving_scenario;
  init_guess_ = optimizer_info.init_guess;
  curr_context_type_ = "regular";

  // 计算prev_traj_info_信息: std::tuple<bool, double, TrajectoryPt>
  auto& [prev_traj_valid, prev_ref_s, prev_start_pt] = prev_traj_info_;
  prev_traj_valid = false;
  if (!prev_trajectory.empty()) {
    constexpr double max_prev_traj_err = 0.7;
    double min_dist = std::numeric_limits<double>::max();
    prev_start_pt = prev_trajectory.getNearestPoint(xy_planning_start_point_.path_pt(), min_dist);
    SLPoint sl_pt;
    if (min_dist < max_prev_traj_err && target_ref_line_->xy2sl(prev_start_pt.path_pt(), &sl_pt)) {
      prev_ref_s = sl_pt.s();
      constexpr double max_tolerence_diff_s = 0.01;  // 防止小的投影误差，造成历史轨迹不可用
      if (accumulated_s_.front() - max_tolerence_diff_s < prev_ref_s
          && prev_ref_s < accumulated_s_.back() + max_tolerence_diff_s) {
        prev_traj_valid = true;
      }
    };
  }

  if (input_data.change_ref_) {
    prev_traj_valid = false;
    dropAsyncProcess();
  }

  return true;
}

bool SpatiotemporalOptimizer::hasValidPrevModel() {
  auto& [prev_ref_id, prev_context, prev_update_params_method, prev_model, prev_solve_status] = prev_model_info_;
  return prev_model != nullptr && prev_model->name() == context().model()
         && prev_solve_status != SolveStatus::NUMERICAL_ERROR_INTERRUPTION
         && prev_update_params_method == curr_update_params_method_ && prev_ref_id == target_ref_line_->id();
}

const SpatiotemporalOptimizerContext& SpatiotemporalOptimizer::context() const {
  return optimizer_config_.context().at(curr_context_type_);
}

std::shared_ptr<OptimalControlProblem> SpatiotemporalOptimizer::initModel(
    const SpatiotemporalOptimizerContext& context, const Trajectory& trajectory_data,
    std::shared_ptr<const SolverInputSnapshot> snapshot) {
  // std::cout << " context " << context.DebugString() << std::endl;
  std::shared_ptr<OptimalControlProblem> model;
  curr_update_params_method_ = UpdateParamsMethod::SPATIOTEMPORAL;
  if (!hasValidPrevModel()) {
    model = OptimalControlProblem::create(context.model());
    if (model != nullptr) {
      model->mutable_config()->set_integrator_type(OcpConfig::ERK4);
      model->mutable_config()->set_dt(dt_);
      model->mutable_config()->set_horizon_length(N_);

      for (const auto& param : context.default_params()) {
        model->setParam(param.key(), param.value());
      }
      model->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(optimizer_config_.ipm_config());
    }
    model->init();
  } else {
    STLOG(D, "[SpatiotemporalOcpOptimizer::initModel]: Using previous model ");
    auto& [prev_ref_id, prev_context, prev_update_method, prev_model, prev_solve_status] = prev_model_info_;
    model = prev_model;
    for (const auto& param : context.default_params()) {
      model->setParam(param.key(), param.value());
    }
  }
  initSpatiotemporalPlanner(model, snapshot);
  updateSpatiotemporalPlanner(model, snapshot, trajectory_data);

  return model;
}

bool SpatiotemporalOptimizer::initSpatiotemporalPlanner(std::shared_ptr<OptimalControlProblem> model,
                                                        std::shared_ptr<const SolverInputSnapshot> snapshot) {
  model->setParam("front_overhang", snapshot->vehicle_param->front_overhang());
  model->setParam("rear_overhang", snapshot->vehicle_param->rear_overhang());
  model->setParam("wheelbase", snapshot->vehicle_param->wheel_base());
  model->setParam("length", snapshot->vehicle_param->length());
  model->setParam("width", snapshot->vehicle_param->width());

  model->setParam("JHardLowerBound", -8.0);
  model->setParam("JHardUpperBound", 8.0);

  model->setParam("AHardLowerBound", -6.0);
  model->setParam("AHardUpperBound", 3.0);

  model->setParam("a_offset", 0.0);

  model->setParam("s_scale", context().scale_param().s_scale());
  model->setParam("v_scale", context().scale_param().v_scale());
  model->setParam("a_scale", context().scale_param().a_scale());
  model->setParam("jerk_scale", context().scale_param().jerk_scale());
  model->setParam("l_scale", context().scale_param().l_scale());
  model->setParam("theta_scale", context().scale_param().theta_scale());
  model->setParam("steer_scale", context().scale_param().steer_scale());
  model->setParam("dsteer_scale", context().scale_param().dsteer_scale());

  // std::cout << "context()" << context().DebugString() << std::endl;

  const double constaint_ignore_buffer = snapshot->vehicle_param->front_edge_to_ego();
  model->setParam([=](const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double& t, const int& k,
                      OcpVariable* ptr) mutable {
    // sr, xr, yr, thetar, kr, vr
    if (!param_cache_.initialized) {
      param_cache_.init(ptr);
    }
    double ego_s = snapshot->sl_planning_start_point.first[0];
    double curr_s = x(0);
    double curr_x = x(1);
    double curr_y = x(2);
    double curr_theta = x(3);
    double curr_v = x(5);
    double curr_a = x(6);
    double steer_lower_bound = snapshot->v_steer_bound.evaluate(curr_v).lower();
    double dsteer_lower_bound = snapshot->v_dsteer_bound.evaluate(curr_v).lower();
    auto ref_point = snapshot->ref_line_data.evaluate(curr_s);
    auto kf_ref_point = snapshot->ref_line_data.find(ReferencePoint(
        math::Vec3d(ref_point.x() + snapshot->vehicle_param->front_edge_to_ego() * cos(ref_point.heading()),
                    ref_point.y() + snapshot->vehicle_param->front_edge_to_ego() * sin(ref_point.heading()), 0.0),
        0.0, 0.0, 0.0, 0.0))->second;
    auto kr_ref_point = snapshot->ref_line_data.find(ReferencePoint(
        math::Vec3d(ref_point.x() - snapshot->vehicle_param->rear_edge_to_ego() * cos(ref_point.heading()),
                    ref_point.y() - snapshot->vehicle_param->rear_edge_to_ego() * sin(ref_point.heading()), 0.0),
        0.0, 0.0, 0.0, 0.0))->second;
    double steer_ref = std::clamp(std::atan(ref_point.kappa() * snapshot->vehicle_param->wheel_base()), steer_lower_bound, -steer_lower_bound);
    double kf = kf_ref_point.local_s();
    double kr = kr_ref_point.local_s();
    auto kf_direction = snapshot->target_ref_line.getDirectionFromS(kf).direction;
    auto kr_direction = snapshot->target_ref_line.getDirectionFromS(kr).direction;
    ptr->set(param_cache_.x_ref, ref_point.x());
    ptr->set(param_cache_.y_ref, ref_point.y());
    ptr->set(param_cache_.theta_ref, spatiotemporal_functions::getUnifySpaceHeading(x(3), ref_point.heading()));
    ptr->set(param_cache_.kappa_ref, ref_point.kappa());
    ptr->set(param_cache_.steer_ref, steer_ref);
    ptr->set(param_cache_.xf_ref, kf_ref_point.x());
    ptr->set(param_cache_.yf_ref, kf_ref_point.y());
    ptr->set(param_cache_.thetaf_ref, spatiotemporal_functions::getUnifySpaceHeading(x(3), kf_ref_point.heading()));
    ptr->set(param_cache_.xr_ref, kr_ref_point.x());
    ptr->set(param_cache_.yr_ref, kr_ref_point.y());
    ptr->set(param_cache_.thetar_ref, spatiotemporal_functions::getUnifySpaceHeading(x(3), kr_ref_point.heading()));

    //  set optimizer objective

    ptr->set(param_cache_.s_coarse, snapshot->optimizer_parameters[k].at("s_coarse").second);
    ptr->set(param_cache_.v_coarse, snapshot->optimizer_parameters[k].at("v_coarse").second);
    ptr->set(param_cache_.a_coarse, snapshot->optimizer_parameters[k].at("a_coarse").second);
    ptr->set(param_cache_.l_offset, snapshot->optimizer_parameters[k].at("l_offset").second);

    // lateral  bound
    // 避免自车处于边界不可行区域导致求解直接失败。
    if (k > 10) {
      // barrier

      ptr->set(param_cache_.LHardLowerBound, snapshot->hard_lateral_bound.evaluate(ref_point.local_s()).lower());
      ptr->set(param_cache_.LHardUpperBound, snapshot->hard_lateral_bound.evaluate(ref_point.local_s()).upper());

      ptr->set(param_cache_.LFHardLowerBound, snapshot->hard_lateral_bound.evaluate(kf_ref_point.local_s()).lower());
      ptr->set(param_cache_.LFHardUpperBound, snapshot->hard_lateral_bound.evaluate(kf_ref_point.local_s()).upper());
      ptr->set(param_cache_.LRHardLowerBound, snapshot->hard_lateral_bound.evaluate(kr_ref_point.local_s()).lower());
      ptr->set(param_cache_.LRHardUpperBound, snapshot->hard_lateral_bound.evaluate(kr_ref_point.local_s()).upper());
    }

    // soft

    ptr->set(param_cache_.LSoftLowerBound, snapshot->soft_lateral_bound.evaluate(ref_point.local_s()).lower());
    ptr->set(param_cache_.LSoftUpperBound, snapshot->soft_lateral_bound.evaluate(ref_point.local_s()).upper());

    ptr->set(param_cache_.LFSoftLowerBound, snapshot->soft_lateral_bound.evaluate(kf_ref_point.local_s()).lower());
    ptr->set(param_cache_.LFSoftUpperBound, snapshot->soft_lateral_bound.evaluate(kf_ref_point.local_s()).upper());
    ptr->set(param_cache_.LRSoftLowerBound, snapshot->soft_lateral_bound.evaluate(kr_ref_point.local_s()).lower());
    ptr->set(param_cache_.LRSoftUpperBound, snapshot->soft_lateral_bound.evaluate(kr_ref_point.local_s()).upper());

    // steer && dsteer bound

    ptr->set(param_cache_.SteerLowerBound, steer_lower_bound);
    ptr->set(param_cache_.SteerUpperBound, -steer_lower_bound);

    ptr->set(param_cache_.DSteerLowerBound, dsteer_lower_bound);
    ptr->set(param_cache_.DSteerUpperBound, -dsteer_lower_bound);

    // longitudinal bound
    // 只考虑未来的限速信息，避免当前的overtake等提速行为与限速行为冲突。
    if (k > 40) {
      ptr->set(param_cache_.VSoftUpperBound, snapshot->soft_speed_bound.evaluate(curr_s).upper());
      ptr->set(param_cache_.VHardUpperBound, snapshot->hard_speed_bound.evaluate(curr_s).upper());
    }




    // obstacle avoidance
    //  统一处理四个障碍物参数
    std::vector<ObjectInfo> relative_obs_data;
    for (int obs_index = 0; obs_index < snapshot->target_obstacle_data.size(); obs_index++) {
        relative_obs_data.emplace_back(snapshot->target_obstacle_data[obs_index][k]);
    }

    constexpr size_t num_obs_to_process = 8;
    for (size_t obs_idx = 0; obs_idx < num_obs_to_process; ++obs_idx) {
      if (obs_idx < relative_obs_data.size()) {
        auto& obs_data = relative_obs_data[obs_idx];
        obs_data.updateDynamicBuffers(curr_v, curr_a, k);
        obs_data.adjust(curr_theta, optimizer_config_.ellipse_adjust_param().ang_a_min_factor(),
                        optimizer_config_.ellipse_adjust_param().ang_a_max_factor(),
                        optimizer_config_.ellipse_adjust_param().ang_b_min_factor(),
                        optimizer_config_.ellipse_adjust_param().ang_b_max_factor());
        // 动态更新调试打印
        // if (k % 10 == 0) {
        //   STLOG(D, "[initSpatiotemporalPlanner] obs_", obs_idx, " info:", " time=", k * dt_, " x=", obs_data.x,
        //         " y=", obs_data.y, " theta=", obs_data.ellipse_angle, " a=", obs_data.a, " b=", obs_data.b,
        //         " length=", obs_data.length, " width=", obs_data.width, " v=", obs_data.v);
        //   STLOG(D, " [initSpatiotemporalPlanner] vehicle :", " x=", curr_x, " y=", curr_y, " theta=", curr_theta,
        //         " v=", curr_v, " length=", snapshot->vehicle_param->length(),
        //         " width=", snapshot->vehicle_param->width(),
        //         " front_edge_to_ego=", snapshot->vehicle_param->front_edge_to_ego(),
        //         " rear_edge_to_ego=", snapshot->vehicle_param->rear_edge_to_ego());
        // }

        // 设置位置参数
        ptr->set(param_cache_.obs_x[obs_idx], obs_data.x);
        ptr->set(param_cache_.obs_y[obs_idx], obs_data.y);
        ptr->set(param_cache_.obs_theta[obs_idx], obs_data.theta);

        // 设置三角函数参数
        ptr->set(param_cache_.obs_cos_theta[obs_idx], obs_data.cos_theta);
        ptr->set(param_cache_.obs_sin_theta[obs_idx], obs_data.sin_theta);

        // 设置避障相关参数
        if (obs_data.enable_optimized) {
          ptr->set(param_cache_.obs_a[obs_idx], obs_data.a);
          ptr->set(param_cache_.obs_b[obs_idx], obs_data.b);
          ptr->set(param_cache_.obs_a_square[obs_idx], obs_data.a_square);
          ptr->set(param_cache_.obs_b_square[obs_idx], obs_data.b_square);
          ptr->set(param_cache_.obs_v[obs_idx], obs_data.v * std::cos(obs_data.theta - curr_theta));
          ptr->set(param_cache_.obs_length[obs_idx], obs_data.length);
          ptr->set(param_cache_.obs_width[obs_idx], obs_data.width);
          ptr->set(param_cache_.obs_weight[obs_idx], context().weights().obs_weight());
          ptr->set(param_cache_.obs_avoided_bound[obs_idx], context().weights().obs_avoided_bound());
        } else {
          ptr->set(param_cache_.obs_a[obs_idx], 0.1);
          ptr->set(param_cache_.obs_b[obs_idx], 0.1);
          ptr->set(param_cache_.obs_a_square[obs_idx], 0.01);
          ptr->set(param_cache_.obs_b_square[obs_idx], 0.01);
          ptr->set(param_cache_.obs_v[obs_idx], 0.0);
          ptr->set(param_cache_.obs_length[obs_idx], 0.1);
          ptr->set(param_cache_.obs_width[obs_idx], 0.01);
          ptr->set(param_cache_.obs_weight[obs_idx], 0.01);
          ptr->set(param_cache_.obs_avoided_bound[obs_idx], 0.01);
        }
      } else {
        // 设置默认椭圆参数
        ptr->set(param_cache_.obs_a[obs_idx], 0.1);
        ptr->set(param_cache_.obs_b[obs_idx], 0.1);
        ptr->set(param_cache_.obs_a_square[obs_idx], 0.01);
        ptr->set(param_cache_.obs_b_square[obs_idx], 0.01);
        // 设置默认位置参数
        ptr->set(param_cache_.obs_x[obs_idx], xy_planning_start_point_.path_pt().x() + 999.0);
        ptr->set(param_cache_.obs_y[obs_idx], xy_planning_start_point_.path_pt().y() + 999.0);
        ptr->set(param_cache_.obs_theta[obs_idx], 0.0);

        // 设置默认三角函数参数
        ptr->set(param_cache_.obs_cos_theta[obs_idx], 1.0);
        ptr->set(param_cache_.obs_sin_theta[obs_idx], 0.0);
        // 关闭避障相关参数
        ptr->set(param_cache_.obs_weight[obs_idx], 0.1);
        ptr->set(param_cache_.obs_avoided_bound[obs_idx], 0.1);
      }
    }
  });
  return true;
}

bool SpatiotemporalOptimizer::updateSpatiotemporalPlanner(std::shared_ptr<OptimalControlProblem> model,
                                                          std::shared_ptr<const SolverInputSnapshot> snapshot,
                                                          const Trajectory& prev_trajectory) {
  model->setX0(start_state_);
  for (size_t i = 0; i <= model->N(); i++) {
    model->setParam("SHardLowerBound", s_hard_bound_.evaluate(i * dt_).lower(), i);
    model->setParam("SHardUpperBound", s_hard_bound_.evaluate(i * dt_).upper(), i);
    model->setParam("SSoftLowerBound", s_soft_bound_.evaluate(i * dt_).lower(), i);
    model->setParam("SSoftUpperBound", s_soft_bound_.evaluate(i * dt_).upper(), i);
  }
  double weight_pre_steer = context().weights().prev_steer();
  double gamma_weight_prev_steer = context().weights().gamma_weight_prev_steer();

  // s_coarse_weight & v_coarse_weight
  double s_coarse_weight = context().weights().s_coarse_weight();
  if (current_driving_scenario_[DrivingScenarios::VEHICLE_START]) {
    for (const auto& param : context().ranged_params()) {
      if (param.range_key() == "VEHICLE_START") {
        if (param.key() == "s_coarse_weight") {
          s_coarse_weight = param.value();
        }
        // std::cout << " param.key = " << param.key() << " value = " << param.value()
        //           << " range_key = " << param.range_key() << std::endl;
      }
    }
  }

  for (size_t i = 0; i <= model->N(); i++) {
    auto& state = model->x(i);
    if (i > 0) {
      model->setXGuess("s", init_guess_[i].s, i);
      model->setXGuess("x", init_guess_[i].x, i);
      model->setXGuess("y", init_guess_[i].y, i);
      model->setXGuess(
          "theta", spatiotemporal_functions::getUnifySpaceHeading(init_guess_[i - 1].theta, init_guess_[i].theta), i);
      model->setXGuess("steer", init_guess_[i].steer, i);
      model->setXGuess("v", init_guess_[i].v, i);
      model->setXGuess("a", init_guess_[i].a, i);
    }
    model->updateParams(i);

    double curr_s = state(0);
    if (!prev_trajectory.empty()) {
      weight_pre_steer *= gamma_weight_prev_steer;

      auto prev_point = prev_trajectory.evaluate(curr_s);

      model->setParam("prev_steer", prev_point.path_pt().front_steer(), i);
      model->setParam("weight_prev_steer", weight_pre_steer, i);
    } else {
      model->setParam("weight_prev_steer", 0.0, i);
    }


    model->setParam("s_coarse_weight",
                    (optimizer_parameters_[i]["s_coarse"].first ? s_coarse_weight : 0), i);
    model->setParam("v_coarse_weight",
                    (optimizer_parameters_[i]["v_coarse"].first ? context().weights().v_coarse_weight() : 0), i);


    if (curr_s >= lane_keep_start_s_) {
      model->setParam("l_ref_weight", context().weights().l_ref_weight(), i);
      model->setParam("theta_ref_weight", context().weights().theta_ref_weight(), i);
    }

    // 计算init_guess
    if (i > 0) {
      applyFrontWheelSteeringPolicy(model, init_guess_, snapshot, i);
    }
  }

  // steer && dsteer weight
  double steer_weight = math::TableLookUp1D(steer_speed_vec_, steer_coffient_vec_, start_state_(5) * MS_KMH);
  double dsteer_weight = math::TableLookUp1D(dsteer_speed_vec_, dsteer_coffient_vec_, start_state_(5) * MS_KMH);
  model->setParam("steer_weight", steer_weight);
  model->setParam("dsteer_weight", dsteer_weight);

  model->setParam("l_terminal", optimizer_parameters_.back()["l_terminal"].second);
  model->setParam("s_terminal", optimizer_parameters_.back()["s_terminal"].second);
  model->setParam("v_terminal", optimizer_parameters_.back()["v_terminal"].second);
  model->setParam("theta_terminal", optimizer_parameters_.back()["theta_terminal"].second);

  model->setParam("terminal_l_weight",
                  optimizer_parameters_.back()["l_terminal"].first ? context().weights().terminal_l_weight() : 0.0);
  model->setParam("terminal_s_weight",
                  optimizer_parameters_.back()["s_terminal"].first ? context().weights().terminal_s_weight() : 0.0);
  model->setParam("terminal_v_weight",
                  optimizer_parameters_.back()["v_terminal"].first ? context().weights().terminal_v_weight() : 0.0);
  model->setParam("terminal_theta_weight", optimizer_parameters_.back()["theta_terminal"].first
                                               ? context().weights().terminal_theta_weight()
                                               : 0.0);

  // 位置参数调整
  if (current_driving_scenario_[DrivingScenarios::JUNCTION_FORWARD]) {
    for (const auto& param : context().ranged_params()) {
      if (param.range_key() == "JUNCTION_FORWARD") {
        model->setParam(param.key(), param.value());
        // std::cout << " param.key = " << param.key() << " value = " << param.value()
        //           << " range_key = " << param.range_key() << std::endl;
      }
    }
  }

  // 行为参数调整
  if (current_driving_scenario_[DrivingScenarios::LANE_CHANGE]) {
    auto params = interpolateMultiParams(start_state_(5), lane_change_speed_vec_, lane_change_params_vec_);
    for (const auto& param : params) {
      model->setParam(param.first, param.second);
      // std::cout << " param.key = " << param.first << " value = " << param.second << std::endl;
    }
  }

  if (current_driving_scenario_[DrivingScenarios::VEHICLE_START]) {
    for (const auto& param : context().ranged_params()) {
      if (param.range_key() == "VEHICLE_START") {
        if (param.key() == "jerk_weight" || param.key() == "a_weight") {
          model->setParam(param.key(), param.value());
        }
        // std::cout << " param.key = " << param.key() << " value = " << param.value()
        //           << " range_key = " << param.range_key() << std::endl;
      }
    }
  } else if (current_driving_scenario_[DrivingScenarios::END]) {
    for (const auto& param : context().ranged_params()) {
      if (param.range_key() == "END") {
        model->setParam(param.key(), param.value());
        // std::cout << " param.key = " << param.key() << " value = " << param.value()
        //           << " range_key = " << param.range_key() << std::endl;
      }
    }
  }

  //紧急行为参数调整
  if(current_driving_scenario_[DrivingScenarios::EMERGENCY_BRAKE]){
    for (const auto& param : context().ranged_params()) {
      if (param.range_key() == "EMERGENCY_BRAKE") {
        model->setParam(param.key(), param.value());
        // std::cout << " param.key = " << param.key() << " value = " << param.value()
        //           << " range_key = " << param.range_key() << std::endl;
      }
    }
  }

  model->updateParams(model->N());
  return true;
}

bool SpatiotemporalOptimizer::applySpatiotemporalXGuess(std::shared_ptr<OptimalControlProblem> model,
                                                        const size_t idx) {
  model->setXGuess("s", init_guess_[idx].s, idx);
  model->setXGuess("x", init_guess_[idx].x, idx);
  model->setXGuess("y", init_guess_[idx].y, idx);
  model->setXGuess(
      "theta", spatiotemporal_functions::getUnifySpaceHeading(init_guess_[idx - 1].theta, init_guess_[idx].theta), idx);
  model->setXGuess("steer", init_guess_[idx].steer, idx);
  model->setXGuess("v", init_guess_[idx].v, idx);
  model->setXGuess("a", init_guess_[idx].a, idx);
  return true;
}

const DrivingDirection SpatiotemporalOptimizer::getInitPointDrivingDirection(const double s) {
  for (float check_s = std::fmax(0.0, s - 30.0); check_s < std::fmin(target_ref_line_->length(), s + 30.0);
       check_s += 1.0) {
    if (target_ref_line_->getDirectionFromS(check_s).direction == DrivingDirection::kDirectionUTurnOnly) {
      return DrivingDirection::kDirectionUTurnOnly;
    }
  }
  return target_ref_line_->getDirectionFromS(s).direction;
}

bool SpatiotemporalOptimizer::trajNonCollisionCheck(std::shared_ptr<OptimalControlProblem> model) {
  ObjectInfo ego_object;
  ObjectInfo target_object;
  for (size_t i = 0; i <= 30; ++i) { //关注3s内的碰撞关系
    auto& state = model->x(i);
    auto& control = model->u(i);
    auto& params = model->p(i);
    for (int obs_idx = 0; obs_idx < 8; ++obs_idx) {
      const std::string idx_str = std::to_string(obs_idx);

      if (control("obs_" + idx_str + "_slack") > 1e-2) {
        ego_object.x =
            state("x") + (vehicle_param_->length() / 2 - vehicle_param_->rear_overhang()) * cos(state("theta"));
        ego_object.y =
            state("y") + (vehicle_param_->length() / 2 - vehicle_param_->rear_overhang()) * sin(state("theta"));
        ego_object.theta = state("theta");
        ego_object.length = vehicle_param_->length() + optimizer_config_.collision_check_param().longitudinal_buffer();
        ego_object.width = vehicle_param_->width() + optimizer_config_.collision_check_param().lateral_buffer();
        target_object.x = params("x_" + idx_str);
        target_object.y = params("y_" + idx_str);
        target_object.theta = params("theta_" + idx_str);
        target_object.length = params("length_" + idx_str);
        target_object.width = params("width_" + idx_str);
        math::Vec2d nearest_pt;
        double alpha = calcAlphaBetweenBox(target_object, ego_object, false, &nearest_pt);
        // std::cout<<" index "<<i<<" obs_idx "<<obs_idx<<" alpha = "<<alpha<<endl;
        if (alpha < 1.0) {
          STLOG(W, "[SpatiotemporalOcpOptimizer::trajNonCollisionCheck]: collision detected at index ", i);
          return false;
        }
      }
    }
  }
  return true;
}

bool SpatiotemporalOptimizer::tranStatesToPathData(std::shared_ptr<OptimalControlProblem> model,
                                                   Trajectory& trajectory_data) {
  vector<TrajectoryPt> traj_points;
  auto globals = model->getGlobals();
  for (size_t i = 0; i <= model->N(); ++i) {
    auto& state = model->x(i);  // ref_s x y theta kappa
    auto& global = globals[i];  //
    auto control = model->u(i);
    double kappa = global(0);
    double d_kappa = control(0) * (1 + tan(state(4)) * tan(state(4))) / vehicle_param_->wheel_base();
    auto ref_point = target_ref_line_->getReferencePoint(state(0));
    traj_points.emplace_back(PathPt(state(1), state(2), ref_point.z(), ref_point.slope(), state(3), kappa, state(0),
                                    d_kappa, 0.0));           // x y z slope heading kappa s dkappa ddkappa
    traj_points.back().mutable_path_pt()->set_l(global(3));   // l = globals(3);
    traj_points.back().set_v(state(5));
    traj_points.back().set_a(state(6));
    traj_points.back().set_jerk(control(1));
    traj_points.back().mutable_path_pt()->set_front_steer(state(4));
    // cout<<" i = "<<i<<" s = "<<state(0)<<" x = "<<state(1)<<" y = "<<state(2)<<" theta = "<<state(3)<<" kappa="<<state(4)
    // <<" v = "<<state(5)<<" a = "<<state(6)<<" steer = "<<control(0)<<" jerk = "<<control(1)<<endl;
    // std::cout<<" traj info "<<" index  i = "<<i<<" d_steer = "<<control(0)<<" steer = "<<state(4)<<std::endl;
  }

  trajectory_data = Trajectory(traj_points);
  return true;
}

double SpatiotemporalOptimizer::calcAlphaBetweenBox(const ObjectInfo& obs_box, const ObjectInfo& ego_box,
                                                    const bool& is_obs_expanded, math::Vec2d* nearest_pt) {
  const double& x_ego = ego_box.x;
  const double& y_ego = ego_box.y;
  const double& theta_ego = ego_box.theta;
  const double& length_ego = ego_box.length;
  const double& width_ego = ego_box.width;
  const double& x_obs = obs_box.x;
  const double& y_obs = obs_box.y;
  const double& theta_obs = obs_box.theta;
  const double& length_obs = obs_box.length;
  const double& width_obs = obs_box.width;
  if (alpha_optimizer_ == nullptr) {
    alpha_optimizer_ = std::make_shared<QuadraticProgrammingSolver>(3, 0, 9);
    alpha_optimizer_->config()->set_first_order_tol(1e-3);
    alpha_optimizer_->config()->set_equality_constraint_tol(1e-3);
    alpha_optimizer_->config()->set_complementary_tol(1e-2);
    alpha_optimizer_->config()->set_inequality_constraint_tol(1e-2);
    alpha_optimizer_->config()->set_mu_min(1e-4);
    alpha_optimizer_->config()->set_barrier_strategy(IPMConfig::ADAPTIVE);
  }
  const double sin_theta_ego = sin(theta_ego);
  const double cos_theta_ego = cos(theta_ego);
  const double sin_theta_obs = sin(theta_obs);
  const double cos_theta_obs = cos(theta_obs);
  const double length_expan_centre_ego = length_ego / 2.0;  // 膨胀中心离车头
  const double width_expan_centre_ego = width_ego / 2.0;    // 膨胀中心离车左侧
  const double length_expan_centre_obs = length_obs / 2.0;
  const double width_expan_centre_obs = width_obs / 2.0;

  // std::cout<<" length_expan_centre_ego "<<length_expan_centre_ego<<"  width_expan_centre_ego
  // "<<width_expan_centre_ego<<endl; std::cout<<" length_expan_centre_obs "<<length_expan_centre_obs<<"
  // width_expan_centre_obs "<<width_expan_centre_obs<<endl;

  if (is_obs_expanded) {
    G_ << sin_theta_ego, -cos_theta_ego, -width_ego + width_expan_centre_ego, -sin_theta_ego, cos_theta_ego,
        -width_expan_centre_ego, -cos_theta_ego, -sin_theta_ego, -length_ego + length_expan_centre_ego, cos_theta_ego,
        sin_theta_ego, -length_expan_centre_ego, sin_theta_obs, -cos_theta_obs, -width_obs + width_expan_centre_obs,
        -sin_theta_obs, cos_theta_obs, -width_expan_centre_obs, -cos_theta_obs, -sin_theta_obs,
        -length_obs + length_expan_centre_obs, cos_theta_obs, sin_theta_obs, -length_expan_centre_obs, 0, 0, -1;
    h_ << -sin_theta_ego * x_ego + cos_theta_ego * y_ego, sin_theta_ego * x_ego - cos_theta_ego * y_ego,
        cos_theta_ego * x_ego + sin_theta_ego * y_ego, -cos_theta_ego * x_ego - sin_theta_ego * y_ego,
        -sin_theta_obs * x_obs + cos_theta_obs * y_obs, sin_theta_obs * x_obs - cos_theta_obs * y_obs,
        cos_theta_obs * x_obs + sin_theta_obs * y_obs, -cos_theta_obs * x_obs - sin_theta_obs * y_obs, 0;
  } else {
    G_ << sin_theta_ego, -cos_theta_ego, -width_ego + width_expan_centre_ego, -sin_theta_ego, cos_theta_ego,
        -width_expan_centre_ego, -cos_theta_ego, -sin_theta_ego, -length_ego + length_expan_centre_ego, cos_theta_ego,
        sin_theta_ego, -length_expan_centre_ego, sin_theta_obs, -cos_theta_obs, 0, -sin_theta_obs, cos_theta_obs, 0,
        -cos_theta_obs, -sin_theta_obs, 0, cos_theta_obs, sin_theta_obs, 0, 0, 0, -1;
    h_ << -sin_theta_ego * x_ego + cos_theta_ego * y_ego, sin_theta_ego * x_ego - cos_theta_ego * y_ego,
        cos_theta_ego * x_ego + sin_theta_ego * y_ego, -cos_theta_ego * x_ego - sin_theta_ego * y_ego,
        -sin_theta_obs * x_obs + cos_theta_obs * y_obs - width_obs + width_expan_centre_obs,
        sin_theta_obs * x_obs - cos_theta_obs * y_obs - width_expan_centre_obs,
        cos_theta_obs * x_obs + sin_theta_obs * y_obs - length_obs + length_expan_centre_obs,
        -cos_theta_obs * x_obs - sin_theta_obs * y_obs - length_expan_centre_obs, 0;
  }

  alpha_optimizer_->setParam(Q_, c_, G_, h_);
  alpha_optimizer_->solve();
  auto solution = alpha_optimizer_->getX();

  auto flag = alpha_optimizer_->getExitFlag();
  if (flag != SolveStatus::SOLVED) {
    STLOG(D, "flag:  ", flag, "  ego x :", x_ego, "  y:", y_ego, "  theta:", theta_ego, "  length:", length_ego,
          "  width:", width_ego, "  obs x :", x_obs, "  y:", y_obs, "  theta:", theta_obs, "  length:", length_obs,
          "  width:", width_obs);

    STLOG(D, "flag: ", flag, "  ego x :", x_ego, "  y:", y_ego, "  theta:", theta_ego, "  length:", length_ego,
          "  width:", width_ego, "  obs x :", x_obs, "  y:", y_obs, "  theta:", theta_obs, "  length:", length_obs,
          "  width:", width_obs);
    alpha_optimizer_->solve();
    auto solution = alpha_optimizer_->getX();
    auto re_flag = alpha_optimizer_->getExitFlag();
    if (re_flag != SolveStatus::SOLVED) {
      STLOG(D, "re_flag: ", re_flag);
      return 0.0;
    }
  }
  nearest_pt->set_x(solution(0));
  nearest_pt->set_y(solution(1));
  return solution(2);
}

void SpatiotemporalOptimizer::checkValid(
    std::shared_ptr<SpatiotemporalPlannerDataManager::OptimizerInfo::SolverData> data) {
  auto& model = data->model;
  auto& status = data->status;
  if (status != SolveStatus::SOLVED)
    return;

  auto isValid = [&]() {
    auto inEquals = model->solver()->getInequalities();
    for (size_t i = 1; i + 1 <= model->N(); ++i) {
      auto& it = inEquals[i];
      for (size_t j = 0; j < optimizer_config_.hard_bound_num(); j++) {
        if (it(j) > optimizer_config_.valid_tol()) {
          return false;
        }
      }
      // auto& control = model->u(i);
      // if (control("l_slack") > optimizer_config_.valid_tol_for_l_slack()) {
      //   STLOG(W, " model l_slack {} > valid_tol_for_l_slack",control("l_slack") );
      //     return false;
      // }
    }

    return true;
  };

  model->updateParams();
  if (isValid())
    return;

  bool is_valid = false;
  size_t resolve_ind = 1;
  for (; resolve_ind <= optimizer_config_.resolve_max_num(); resolve_ind++) {
    model->setX0(start_state_);
    status = model->solve();
    if (status != SolveStatus::SOLVED) {
      ERT_PLOG_I << "[SpatiotemporalOptimizer::checkValid]: try to resolve but failed: resolve_ind = " << resolve_ind;
      return;
    }
    model->updateParams();
    if (isValid()) {
      is_valid = true;
      async_optimizer_debug_info_ += fmt::format("\nresolve resolve {} times\n", resolve_ind);
      break;
    }
  }

  if (is_valid) {
    ERT_PLOG_I << "[SpatiotemporalOptimizer::checkValid]: resolve success: resolve_ind = " << resolve_ind;
  } else {
    ERT_PLOG_I << "[SpatiotemporalOptimizer::checkValid]: resolve failed";
    status = SolveStatus::INFEASIBLE_DETECTED;
  }
}

bool SpatiotemporalOptimizer::resultCheck(
    std::shared_ptr<SpatiotemporalPlannerDataManager::OptimizerInfo::SolverData> data) {
  auto& model = data->model;
  auto& status = data->status;
  if (model == nullptr)
    return false;
  auto isValid = [&]() {
    auto inEquals = model->solver()->getInequalities();
    for (size_t i = 1; i + 1 <= model->N(); ++i) {
      auto& it = inEquals[i];
      //hard bound 0 ~ 18
      for (size_t j = 0; j < optimizer_config_.hard_bound_num(); j++) {
        if (it(j) > optimizer_config_.valid_tol()) {
          STLOG(W, " model Inequalities {} > valid_tol", j);
          return false;
        }
      }
      // auto& control = model->u(i);
      // if (control("l_slack") > optimizer_config_.valid_tol_for_l_slack()) {
      //   STLOG(W, " model l_slack {} > valid_tol_for_l_slack",control("l_slack") );
      //     return false;
      // }
    }
    return true;
  };

  model->updateParams();
  if (!isValid()) {
    STLOG(D, "resultCheck: bad result");
    return false;
  }

  STLOG(D, "resultCheck: valid result");
  return true;
}

bool SpatiotemporalOptimizer::applyFrontWheelSteeringPolicy(std::shared_ptr<OptimalControlProblem> model,
                                                            std::vector<SpatiotemporalState> coarse_states,
                                                            std::shared_ptr<const SolverInputSnapshot> snapshot,
                                                            const size_t idx) {
  constexpr double lat_err_thrd = 0.3;
  constexpr double heading_err_thrd = 10.0 / 180.0 * M_PI;
  constexpr double d_t = 0.1;  // 控制周期

  auto& x0 = model->x(idx - 1);
  auto& p0 = model->p(idx - 1);
  double t0 = model->t(idx - 1);
  auto& coarse_state = coarse_states[idx - 1];
  // 计算横向误差和航向误差
  const double cur_v = x0("v");
  double current_steer = x0("steer");
  const double dx = x0("x") - coarse_state.x;
  const double dy = x0("y") - coarse_state.y;
  const double l = -sin(coarse_state.theta) * dx + cos(coarse_state.theta) * dy;
  const double thetae = math::NormalizeAngle(x0("theta") - coarse_state.theta);

  // 获取车辆参数
  const double wheelbase = vehicle_param_->wheel_base();

  // 获取转向限制（考虑控制周期）
  const double dsteer_lower_bound = snapshot->v_dsteer_bound.evaluate(cur_v).lower() * d_t;
  const double dsteer_upper_bound = snapshot->v_dsteer_bound.evaluate(cur_v).upper() * d_t;

  const double steer_lower_bound = snapshot->v_steer_bound.evaluate(cur_v).lower();
  const double steer_upper_bound = snapshot->v_steer_bound.evaluate(cur_v).upper();
  const double steer_bound = std::abs(steer_upper_bound);
  if (std::abs(l) < lat_err_thrd) {
    // 误差较小时直接使用参考轨迹
    model->setXGuess("s", coarse_state.s, idx);
    model->setXGuess("x", coarse_state.x, idx);
    model->setXGuess("y", coarse_state.y, idx);
    model->setXGuess("theta", spatiotemporal_functions::getUnifySpaceHeading(x0("theta"), coarse_state.theta), idx);
    model->setXGuess("steer", atan(p0("kappa_ref") * wheelbase), idx);
  } else {
    const double kappa_bound = tan(steer_bound) / wheelbase;
    double cur_psif = x0("steer");
    const double temp = 1.0f - p0("kappa_ref") * l;
    double target_kappa = 0;
    const double tau = 0.1;
    if (std::abs(temp) > 1e-6) {
      constexpr double max_heading_err = 1.0 * ANG2RAD;
      const double sin_theta_err = sin(thetae);
      const double cos_theta_err = cos(thetae);
      const double l_prime = tan(thetae) * temp;
      double eta = tan(max_heading_err), alpha = kappa_bound, eps = 0.5;
      const double feedforward = p0("kappa_ref") * cos_theta_err * (1 + sin_theta_err * sin_theta_err) / temp;
      const double sigma = l_prime + eta * std::copysign(std::sqrt(std::abs(l)), l);
      const double feedback = -alpha * sgn(sigma, eps);
      target_kappa = std::clamp(feedforward + 1.2 * feedback, -kappa_bound, kappa_bound);
    }
    const double target_dkappa = -tau * (tan(cur_psif) / wheelbase - target_kappa);
    const double target_dsteer = std::clamp(target_dkappa * wheelbase / (1 + std::pow(std::tan(cur_psif), 2)),
                                            dsteer_lower_bound, dsteer_upper_bound);
    model->setUGuess("dsteer", target_dsteer, idx - 1);
    model->rollOut(idx);
  }

  return true;
}

bool SpatiotemporalOptimizer::hasAsyncProcess() const {
  return async_data_ != nullptr;
}

void SpatiotemporalOptimizer::dropAsyncProcess() {
  if (hasAsyncProcess()) {
    if (std::get<3>(prev_model_info_) == async_data_->model) {
      std::get<3>(prev_model_info_).reset();
    }
    async_data_.reset();
  }
}

// 插值函数 从一个速度表插值，得到所有的params的线性插值结果
std::vector<std::pair<std::string, double>> SpatiotemporalOptimizer::interpolateMultiParams(
    double speed, const std::vector<double>& speed_vec,
    const std::vector<std::vector<std::pair<std::string, double>>>& params_vec) {
  std::vector<std::pair<std::string, double>> result;

  // 边界检查
  if (speed_vec.empty() || params_vec.empty() || speed_vec.size() != params_vec.size()) {
    return result;
  }

  // 如果速度小于等于最小值，直接返回第一个参数集
  if (speed <= speed_vec.front()) {
    return params_vec.front();
  }

  // 如果速度大于等于最大值，直接返回最后一个参数集
  if (speed >= speed_vec.back()) {
    return params_vec.back();
  }

  // 找到 speed 所在的区间 [i, i+1]
  size_t i = 0;
  for (; i < speed_vec.size() - 1; ++i) {
    if (speed >= speed_vec[i] && speed <= speed_vec[i + 1]) {
      break;
    }
  }


  // 计算插值比例
  double lower_speed = speed_vec[i];
  double upper_speed = speed_vec[i + 1];
  if (std::fabs(upper_speed - lower_speed) < 1e-9) {
    return params_vec[i];
  }
  double ratio = (speed - lower_speed) / (upper_speed - lower_speed);

  // 获取对应区间的参数集
  const auto& lower_params = params_vec[i];
  const auto& upper_params = params_vec[i + 1];

  // 确保两个参数集的参数数量和名称顺序一致
  if (lower_params.size() != upper_params.size()) {
    std::cerr << "Error: Parameter count mismatch!" << std::endl;
    return result;
  }

  // 对每个参数进行线性插值
  for (size_t j = 0; j < lower_params.size(); ++j) {
    // 验证参数名是否一致
    if (lower_params[j].first != upper_params[j].first) {
      std::cerr << "Error: Parameter name mismatch at index " << j << "!" << std::endl;
      continue;
    }

    double lower_value = lower_params[j].second;
    double upper_value = upper_params[j].second;
    double interpolated_value = lower_value + ratio * (upper_value - lower_value);

    result.emplace_back(lower_params[j].first, interpolated_value);
  }

  return result;
}
REGIST_MODULE(SpatiotemporalOptimizer);
}  // namespace gpal::pnc::planning
