#include "openspace_path_planner/core/optimizer/openspace_ocp_optimizer.h"

#include <fmt/chrono.h>

#include "base/singleton.h"
#include "ocp/ocp_model.h"
#include "util/timer.h"

// #define LOG_OCP_DATA
// #define LOG_SUCCESS_STAGE
// #define LOG_FAILED_STAGE

namespace gpal::pnc::planning {

#ifdef LOG_OCP_DATA
static int debug_count_ = 0;
static OcpDataField ocp_data_field_;
#endif

bool OpenspaceOcpOptimizer::init() {
  OPENSPACE_LOG(I, "[OCP] init!!!!! ");
  auto config_manager = Singleton<ConfigManager>::get_instance();
  vehicle_config_ = std::make_shared<VehicleConfig>(config_manager->vehicle_config());
  vehicle_param_ = std::make_shared<VehicleParam>(config_manager->vehicle_config().vehicle_param());
  config_ = config_manager->getConfig<OcpPathOptimizerConfig>("OpenspaceOcpPathOptimizerConfig");

  curr_profile_type_ = "";
  reset();
  has_inited_ = true;
  return has_inited_;
}

bool OpenspaceOcpOptimizer::reset() {
  accumulated_s_.clear();
  dropAsyncProcess();
  has_inited_ = false;
  debug_info_ = "";
  return true;
}

bool OpenspaceOcpOptimizer::hasAsyncProcess() const { return async_data_ != nullptr; }

void OpenspaceOcpOptimizer::dropAsyncProcess() {
  if (hasAsyncProcess()) {
    async_data_.reset();
  }
}

bool OpenspaceOcpOptimizer::preProcess(const DiscretizedPath& orin_path, const PathPt& init_point,
                                       const PathBoundary& boundary) {
  target_ref_path_ = &orin_path;
  xy_planning_start_point_ = init_point;
  planning_start_s_ = 0.0;
  decision_path_boundary_.assign(boundary.decision_boundary().begin(), boundary.decision_boundary().end());
  barrier_path_boundary_.assign(boundary.barrier_boundary().begin(), boundary.barrier_boundary().end());
  soft_path_boundary_.assign(boundary.soft_boundary().begin(), boundary.soft_boundary().end());

  if (curr_profile_type_ != boundary.label()) {
    curr_profile_type_ = boundary.label();
    auto iter = config_.profiles().find(curr_profile_type_);
    if (iter == config_.profiles().end()) {
      curr_profile_type_ = "";
      return false;
    }
  }
  OPENSPACE_LOG(D, "[OCP] curr_profile_type_ = ", curr_profile_type_);
  if (boundary.size() < 2 || boundary.delta_s() < 0.1) {
    OPENSPACE_LOG(E, "[OCP] bound_length is too short");
    return false;
  }

  N_ = boundary.size() - 1;
  ds_ = boundary.delta_s();
  min_v_square_ = profile().constraint().min_v() * profile().constraint().min_v();

  accumulated_s_.clear();
  for (int i = 0; i <= N_; i++) {
    accumulated_s_.emplace_back(planning_start_s_ + i * ds_);
  }

  priority_map_.clear();
  for (auto& tag : config_.tag_priority()) {
    priority_map_.insert(tag);
  }

  return true;
}

OpenspaceOcpOptimizer::AsyncStatus OpenspaceOcpOptimizer::asyncProc(const DiscretizedPath& orin_path,
                                                                    const PathPt& init_point,
                                                                    const PathBoundary& boundary,
                                                                    DiscretizedPath& final_path) {
  util::TimerLogger<std::milli> timer("asyncOptimizer",
                                      [](const std::string& str) { OPENSPACE_LOG(I, "[OCP][asyncProc]: ", str); });
  AsyncStatus status = AsyncStatus::ASYNC_SOLVED;
  {
    auto tap_timer = timer.tap("preProcess");
    if (!preProcess(orin_path, init_point, boundary)) {
      status = AsyncStatus::ASYNC_FAILED;
      reset();
      return status;
    }
  }
  if (async_data_ == nullptr) {
    OPENSPACE_LOG(D,
                  "[OCP] ######################  openspace ocp optimizer start !   "
                  "####################");
    start_stamp_ = std::chrono::steady_clock::now();
    auto async_timer = timer.tap("prepareData");
    auto async_data = std::make_shared<SolverData>();
    async_data->profile = curr_profile_type_;
    {
      auto tap_timer = async_timer.tap("initModel");
      async_data->model = initModel(profile());
    }
#ifdef LOG_OCP_DATA
    if (debug_count_ < 10) {
      async_data->model->logData(&ocp_data_field_);
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
      auto tap_timer = async_timer.tap("importTask");
      task_handler_.importTask([=] {
        util::TimerLogger<std::milli> timer(async_data->model->name(), [=](const std::string& str) {
          OPENSPACE_LOG(D, "[OCP][asyncProc]: async = ", async_data->profile,
                        "  ipm_solver status = ", async_data->status,
                        "  iter = ", async_data->model->solver()->iteration_number());
        });
        async_data->status = async_data->model->solve();
        async_data->is_finished = true;
        async_data->cond.notify_all();
        end_stamp_ = std::chrono::steady_clock::now();
      });
      async_data_ = async_data;
    }
  }

  {
    auto tap_timer = timer.tap("underlockSolve");
    std::unique_lock<std::mutex> lck(async_data_->mutex);
    int timeout = 20;
    async_data_->cond.wait_for(lck, std::chrono::milliseconds(timeout), [=]() {
      bool is_finished = async_data_->is_finished;
      return is_finished;
    });
  }
  auto tap_timer = timer.tap("postProcess");
  if (async_data_->is_finished) {
    debug_info_ = "";
    debug_info_ += fmt::format(
        "{} {} {:.4f}", static_cast<int>(async_data_->status), async_data_->model->solver()->iteration_number(),
        std::chrono::duration_cast<std::chrono::duration<double>>(end_stamp_ - start_stamp_).count());
    if (async_data_->status == SolveStatus::SOLVED) {
#ifdef LOG_SUCCESS_STAGE
      auto states = async_data_->model->getX();
      auto params = async_data_->model->getParam();
      for (int i = 0; i < states.size(); i++) {
        SFIELD(ocp, "{}\n{}", states[i].debugString(), params[i].debugString());
      }
#endif
      if (!tranStatesToDiscretizedPath(init_point, async_data_->model, final_path)) {
        status = AsyncStatus::ASYNC_ERROR_SOLVED;
        OPENSPACE_LOG(E,
                      "####################  openspace ocp optimizer data error !   "
                      "####################");
      } else {
        OPENSPACE_LOG(D,
                      "####################  openspace ocp optimizer success !  "
                      "####################");
        ;
      }
    } else {
#ifdef LOG_FAILED_STAGE
      auto states = async_data_->model->getX();
      auto params = async_data_->model->getParam();
      for (int i = 0; i < states.size(); i++) {
        SFIELD(ocp, "{}\n{}", states[i].debugString(), params[i].debugString());
      }
#endif
      status = AsyncStatus::ASYNC_FAILED;
      OPENSPACE_LOG(E,
                    "####################  openspace ocp optimizer failed !   "
                    "####################");
    }
    async_data_.reset();
  } else {
    status = AsyncStatus::ASYNC_UNDERLOCKING;
  }
  return status;
}

std::shared_ptr<OptimalControlProblem> OpenspaceOcpOptimizer::initModel(const OcpPathOptimizerProfile& profile) {
  OPENSPACE_LOG(D, "[OCP] initModel !!!");
  std::shared_ptr<OptimalControlProblem> model;
  model = OptimalControlProblem::create(profile.model());
  if (model != nullptr) {
    model->mutable_config()->set_integrator_type(OcpConfig::FORWARD_EULER);
    model->mutable_config()->set_dt(ds_);
    model->mutable_config()->set_horizon_length(N_);
    OPENSPACE_LOG(D, " model name  : ", profile.model());
    OPENSPACE_LOG(D, " param : ");
    for (auto& param : profile.default_params()) {
      OPENSPACE_LOG(D, " key : ", param.key(), " value : ", param.value());
      model->default_param().set(param.key(), param.value());
    }
    if (profile.has_ipm_config()) {
      model->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(profile.ipm_config());
    } else {
      model->mutable_config()->mutable_solver()->mutable_ipm()->CopyFrom(config_.ipm_config());
    }
  }
  if (model->name() == "ParkingGeneral" && (curr_profile_type_ == "parking_general" || curr_profile_type_ == "refine")) {
    initParkingGeneral(model);
  } else if (model->name() == "ParkingGeneral" && curr_profile_type_ == "fallback") {
    OPENSPACE_LOG(D, "[OCP] init fallback model");
    initFallback(model);
  } else {
    OPENSPACE_LOG(E, "[OCP] optimizer model name error : ", model->name());
  }

  if (model->name() == "ParkingGeneral") {
    updateParkingGeneral(model);
  }
  return model;
}

bool OpenspaceOcpOptimizer::initParkingGeneral(std::shared_ptr<OptimalControlProblem> model) {
  model->mutable_config()->set_integrator_type(OcpConfig::ERK4);
  model->mutable_config()->mutable_solver()->mutable_ipm()->set_barrier_strategy(IPMConfig::ADAPTIVE_SIGMA);
  model->default_param().set("lf", vehicle_param_->front_edge_to_ego());
  model->default_param().set("lr", vehicle_param_->rear_edge_to_ego());
  model->setParam("wheelbase", vehicle_param_->wheel_base());
  double max_bound = 20.0;
  model->setParam(
      [=](const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double t, const int k, OcpVariable* ptr) {
        double ego_s = 0;
        auto curr_s = x(0);
        double constaint_ignore_buffer = 0.0;  // 预留
        auto ref_point = getInitPoint(curr_s);
        double unit_v = ref_point.direction() == PathPt::Direction::FORWARD ? 1.0 : -1.0;
        if (unit_v > 0) {
          ptr->set("v_lower", 0.0);
        }
        if (unit_v < 0) {
          ptr->set("v_upper", 0.0);
        }
        double ref_v = unit_v;
        ptr->set("vr", ref_v);
        ptr->set("xr", ref_point.x());
        ptr->set("yr", ref_point.y());
        double delta_heading = math::NormalizeAngle(ref_point.theta() - x(3));
        ptr->set("thetar", x(3) + delta_heading);
        ptr->set("kr", ref_point.kappa());

        double front_s = curr_s + unit_v * vehicle_param_->front_edge_to_ego();
        auto ref_point_f = getInitPoint(front_s);
        ptr->set("xrf", ref_point_f.x());
        ptr->set("yrf", ref_point_f.y());
        double delta_heading_f = math::NormalizeAngle(ref_point_f.theta() - x(3));
        ptr->set("thetarf", x(3) + delta_heading_f);

        double rear_s = curr_s - unit_v * vehicle_param_->rear_edge_to_ego();
        auto ref_point_r = getInitPoint(rear_s);
        ptr->set("xrr", ref_point_r.x());
        ptr->set("yrr", ref_point_r.y());
        double delta_heading_r = math::NormalizeAngle(ref_point_r.theta() - x(3));
        ptr->set("thetarr", x(3) + delta_heading_r);

        // barrier
        // 后轴中心
        auto bound_res = barrier_path_boundary_.interpolate(curr_s);
        auto& [barrier_bound, barrier_bound_valid] = bound_res;
        if (curr_s > ego_s + constaint_ignore_buffer && barrier_bound_valid) {
          auto& [s, lower, upper] = barrier_bound;
          ptr->set("ll", lower);
          ptr->set("lu", upper);
        } else {
          ptr->set("ll", -max_bound);
          ptr->set("lu", max_bound);
        }
        // 车头中心
        auto bound_res_f = barrier_path_boundary_.interpolate(front_s);
        auto& [barrier_bound_f, barrier_bound_valid_f] = bound_res_f;
        if (curr_s > ego_s + constaint_ignore_buffer && barrier_bound_valid_f) {
          auto& [s, lower, upper] = barrier_bound_f;
          ptr->set("lfl", lower);
          ptr->set("lfu", upper);
        } else {
          ptr->set("lfl", -max_bound);
          ptr->set("lfu", max_bound);
        }
        // 车尾中心
        auto bound_res_r = barrier_path_boundary_.interpolate(rear_s);
        auto& [barrier_bound_r, barrier_bound_valid_r] = bound_res_r;
        if (curr_s > ego_s + constaint_ignore_buffer && barrier_bound_valid_r) {
          auto& [s, lower, upper] = barrier_bound_r;
          ptr->set("lrl", lower);
          ptr->set("lru", upper);
        } else {
          ptr->set("lrl", -max_bound);
          ptr->set("lru", max_bound);
        }
        // soft
        // 后轴中心
        auto decide_bound_res = soft_path_boundary_.interpolate(curr_s);
        auto& [decide_bound, decide_bound_valid] = decide_bound_res;
        if (curr_s > ego_s + constaint_ignore_buffer) {
          auto& [s, lower, upper] = decide_bound;
          ptr->set("sll", lower);
          ptr->set("slu", upper);
        } else {
          ptr->set("sll", -max_bound);
          ptr->set("slu", max_bound);
        }
        // 车头中心
        auto decide_bound_res_f = soft_path_boundary_.interpolate(front_s);
        auto& [decide_bound_f, decide_bound_valid_f] = decide_bound_res_f;
        if (curr_s > ego_s + constaint_ignore_buffer) {
          auto& [s, lower, upper] = decide_bound_f;
          ptr->set("slfl", lower);
          ptr->set("slfu", upper);
        } else {
          ptr->set("slfl", -max_bound);
          ptr->set("slfu", max_bound);
        }
        // 车尾中心
        auto decide_bound_res_r = soft_path_boundary_.interpolate(rear_s);
        auto& [decide_bound_r, decide_bound_valid_r] = decide_bound_res_r;
        if (curr_s > ego_s + constaint_ignore_buffer) {
          auto& [s, lower, upper] = decide_bound_r;
          ptr->set("slrl", lower);
          ptr->set("slru", upper);
        } else {
          ptr->set("slrl", -max_bound);
          ptr->set("slru", max_bound);
        }
        string valid_tag = "";
        for (auto& [tag, start_s, end_s, is_inrange] : scenarioTags()) {
          if (curr_s >= start_s && curr_s <= end_s && is_inrange) {
            int32_t priority_level = 0;
            if (priority_map_.count(tag) > 0) {
              priority_level = priority_map_[tag];
              if (valid_tag == "") {
                valid_tag = tag;
              } else {
                if (priority_map_[tag] < priority_map_[valid_tag]) {
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
  model->init();
  return true;
}

bool OpenspaceOcpOptimizer::initFallback(std::shared_ptr<OptimalControlProblem> model) {
  model->mutable_config()->set_integrator_type(OcpConfig::ERK4);
  model->mutable_config()->mutable_solver()->mutable_ipm()->set_barrier_strategy(IPMConfig::ADAPTIVE_SIGMA);
  model->default_param().set("lf", vehicle_param_->front_edge_to_ego());
  model->default_param().set("lr", vehicle_param_->rear_edge_to_ego());
  model->setParam("wheelbase", vehicle_param_->wheel_base());
  constexpr double max_bound = 10.0;
  model->setParam(
      [=](const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double t, const int k, OcpVariable* ptr) {
        double ego_s = 0;
        auto curr_s = x(0);
        double constaint_ignore_buffer = 0.0;  // 预留
        auto ref_point = getInitPoint(curr_s);
        double unit_v = ref_point.direction() == PathPt::Direction::FORWARD ? 1.0 : -1.0;
        if (unit_v > 0) {
          ptr->set("v_lower", 0.0);
        }
        if (unit_v < 0) {
          ptr->set("v_upper", 0.0);
        }
        double ref_v = unit_v;
        ptr->set("vr", ref_v);
        ptr->set("xr", ref_point.x());
        ptr->set("yr", ref_point.y());
        double delta_heading = math::NormalizeAngle(ref_point.theta() - x(3));
        ptr->set("thetar", x(3) + delta_heading);
        ptr->set("kr", ref_point.kappa());

        double front_s = curr_s + unit_v * vehicle_param_->front_edge_to_ego();
        auto ref_point_f = getInitPoint(front_s);
        ptr->set("xrf", ref_point_f.x());
        ptr->set("yrf", ref_point_f.y());
        double delta_heading_f = math::NormalizeAngle(ref_point_f.theta() - x(3));
        ptr->set("thetarf", x(3) + delta_heading_f);

        double rear_s = curr_s - unit_v * vehicle_param_->rear_edge_to_ego();
        auto ref_point_r = getInitPoint(rear_s);
        ptr->set("xrr", ref_point_r.x());
        ptr->set("yrr", ref_point_r.y());
        double delta_heading_r = math::NormalizeAngle(ref_point_r.theta() - x(3));
        ptr->set("thetarr", x(3) + delta_heading_r);

        // barrier
        // 后轴中心
        auto bound_res = barrier_path_boundary_.interpolate(curr_s);
        auto& [barrier_bound, barrier_bound_valid] = bound_res;
        if (curr_s > ego_s + constaint_ignore_buffer && barrier_bound_valid) {
          auto& [s, lower, upper] = barrier_bound;
          ptr->set("ll", lower);
          ptr->set("lu", upper);
        } else {
          ptr->set("ll", -max_bound);
          ptr->set("lu", max_bound);
        }
        // 车头中心
        ptr->set("lfl", -max_bound);
        ptr->set("lfu", max_bound);
        // 车尾中心
        ptr->set("lrl", -max_bound);
        ptr->set("lru", max_bound);
        // soft
        // 后轴中心
        auto decide_bound_res = soft_path_boundary_.interpolate(curr_s);
        auto& [decide_bound, decide_bound_valid] = decide_bound_res;
        if (curr_s > ego_s + constaint_ignore_buffer) {
          auto& [s, lower, upper] = decide_bound;
          ptr->set("sll", lower);
          ptr->set("slu", upper);
        } else {
          ptr->set("sll", -max_bound);
          ptr->set("slu", max_bound);
        }
        // 车头中心
        auto decide_bound_res_f = soft_path_boundary_.interpolate(front_s);
        auto& [decide_bound_f, decide_bound_valid_f] = decide_bound_res_f;
        if (curr_s > ego_s + constaint_ignore_buffer) {
          auto& [s, lower, upper] = decide_bound_f;
          ptr->set("slfl", lower);
          ptr->set("slfu", upper);
        } else {
          ptr->set("slfl", -max_bound);
          ptr->set("slfu", max_bound);
        }
        // 车尾中心
        auto decide_bound_res_r = soft_path_boundary_.interpolate(rear_s);
        auto& [decide_bound_r, decide_bound_valid_r] = decide_bound_res_r;
        if (curr_s > ego_s + constaint_ignore_buffer) {
          auto& [s, lower, upper] = decide_bound_r;
          ptr->set("slrl", lower);
          ptr->set("slru", upper);
        } else {
          ptr->set("slrl", -max_bound);
          ptr->set("slru", max_bound);
        }
        string valid_tag = "";
        for (auto& [tag, start_s, end_s, is_inrange] : scenarioTags()) {
          if (curr_s >= start_s && curr_s <= end_s && is_inrange) {
            int32_t priority_level = 0;
            if (priority_map_.count(tag) > 0) {
              priority_level = priority_map_[tag];
              if (valid_tag == "") {
                valid_tag = tag;
              } else {
                if (priority_map_[tag] < priority_map_[valid_tag]) {
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
  model->init();
  return true;
}

bool OpenspaceOcpOptimizer::updateParkingGeneral(std::shared_ptr<OptimalControlProblem> model) {
  // initial state
  Eigen::VectorXd start_state = Eigen::VectorXd::Zero(6);
  const auto& [unit_v, ref_v] = parseVelocity(planning_start_s_);
  constexpr double v_scale_factor = 1e-6;
  start_state << planning_start_s_, xy_planning_start_point_.x(), xy_planning_start_point_.y(),
      xy_planning_start_point_.theta(), xy_planning_start_point_.front_steer(), unit_v * v_scale_factor;
  OPENSPACE_LOG(D, "[OCP] sl_planning_start_point_ = ", planning_start_s_, ", ", xy_planning_start_point_.x(), ", ",
                xy_planning_start_point_.y(), ", ", xy_planning_start_point_.theta(), ", ",
                xy_planning_start_point_.front_steer(), ", ", unit_v * v_scale_factor);
  model->setX0(start_state);
  double prev_heading = xy_planning_start_point_.theta();
  for (size_t i = 1; i <= model->N(); i++) {
    model->updateParams(i - 1);
    auto point = getInitPoint(accumulated_s_[i]);
    // SFIELD_DEBUG(openspace, " i s x y = {}, {}, {}, {}", i, accumulated_s_[i], point.x(), point.y())
    model->setXGuess("s", accumulated_s_[i], i);
    model->setXGuess("x", point.x(), i);
    model->setXGuess("y", point.y(), i);
    model->setXGuess("theta", getUnifySpaceHeading(prev_heading, point.theta()), i);
    model->setXGuess("steer", point.front_steer(), i);
    model->setXGuess("v", unit_v, i);
    prev_heading = model->x(i, "theta");
  }
  model->updateParams(model->N());
  return true;
}

bool OpenspaceOcpOptimizer::tranStatesToDiscretizedPath(const TrajectoryPt& init_point,
                                                        std::shared_ptr<OptimalControlProblem> model,
                                                        DiscretizedPath& final_path) {
  vector<PathPt> path_points;
  OPENSPACE_LOG(D, "################ tranStatesToDiscretized = ( s  ,  x  ,  y  , theta,  k  ,  fs ,  rs ,  d)");
  auto globals = model->getGlobals();
  for (size_t i = 0; i <= model->N(); i++) {
    const auto& state = model->x(i);
    const auto& param = model->p(i);
    const auto& control = model->u(i);
    auto& global = globals[i];
    PathPt::Direction direction = PathPt::Direction::FORWARD;
    if (model->name() == "ParkingGeneral") {
      double kappa = global(0);
      double dkappa = control(0) * (1 + tan(state(4)) * tan(state(4))) / vehicle_param_->wheel_base();
      path_points.emplace_back(state(1), state(2), init_point.path_pt().z(), 0.0, state(3), kappa, model->t(i), dkappa,
                               0.0);  // x, y, z, slope, theta, kappa, s, dkappa, ddkappa
      path_points.back().set_front_steer(state(4));
      double vr = state(5);
      direction = vr < 0 ? PathPt::Direction::BACKWARD : PathPt::Direction::FORWARD;
    }
    path_points.back().set_direction(direction);
    OPENSPACE_LOG(D, "[OCP][asyncProc] tranStatesToDiscretizedPath: ", path_points.back().s(), " ",
                  path_points.back().x(), " ", path_points.back().y(), " ",
                  math::NormalizeAngle(path_points.back().theta()), " ", path_points.back().kappa(), " ",
                  path_points.back().front_steer(), " ", path_points.back().rear_steer(), " ",
                  (int)path_points.back().direction());
  }
  final_path.clear();
  final_path = DiscretizedPath(path_points);
  return true;
}

double OpenspaceOcpOptimizer::getUnifySpaceHeading(const double heading_base, const double heading) {
  double delta_heading = math::NormalizeAngle(heading - heading_base);
  return heading_base + delta_heading;
}

const OcpPathOptimizerProfile& OpenspaceOcpOptimizer::profile() const {
  return config_.profiles().at(curr_profile_type_);
}

const PathPt OpenspaceOcpOptimizer::getInitPoint(const double s) { return target_ref_path_->evaluate(s); }

void OpenspaceOcpOptimizer::considerVehicleWidth(const double width, OpenspaceOcpOptimizer::BoundRes* bound_res) {
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

std::pair<double, double> OpenspaceOcpOptimizer::parseVelocity(const double& curr_s) {
  auto ref_point = getInitPoint(curr_s);
  double unit_v = ref_point.direction() == PathPt::Direction::FORWARD ? 1.0 : -1.0;

  // TODO cal ref_v
  double ref_v = unit_v;
  return {unit_v, ref_v};
}

const std::vector<std::tuple<std::string, float, float, bool>>& OpenspaceOcpOptimizer::scenarioTags() const {
  return scenario_tags_;
}
void OpenspaceOcpOptimizer::setScenarioTags(std::vector<std::tuple<std::string, float, float, bool>> scenario_tags) {
  scenario_tags_.clear();
  for (auto& tag : scenario_tags) {
    if (std::get<3>(tag)) {
      addScenarioTag(tag);
    }
  }
}
void OpenspaceOcpOptimizer::addScenarioTag(std::tuple<std::string, float, float, bool> scenario_tag) {
  scenario_tags_.emplace_back(scenario_tag);
}

}  // namespace gpal::pnc::planning
