/**
 * @file speed_ocp_qp_optimizer.cpp
 * @brief 速度最优控制问题二次规划求解器
 * @details 本类负责构建和求解速度规划的最优控制问题（OCP）
 */
#include "speed_optimizer/speed_ocp_qp_optimizer.h"

namespace gpal::pnc::planning {
/**
 * @brief 初始化优化器配置（核心实现）
 * 
 * @details 实现步骤：
 * 1. 获取配置管理器单例实例
 * 2. 加载车辆基础参数（轴距/轮距等）
 * 3. 加载QP优化器专用配置参数
 * 
 * @note 必须在调用其他成员函数前执行
 * 
 * @par 关键配置项:
 * - 车辆动力学参数：vehicle_param_
 * - 优化器超参数：speed_ocp_qp_optimizer_config_
 */
void SpeedOCPQPOptimizer::init() {
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  vehicle_param_ = config_manager_->vehicle_config().vehicle_param();
  speed_ocp_qp_optimizer_config_ = config_manager_->getConfig<SpeedOcpQpOptimizerConfig>("SpeedOcpQpOptimizerConfig");
}
/**
 * @brief 执行双阶段速度优化主流程（二次规划核心入口）
 * @param[in] speed_model_param 速度模型参数容器
 * @param[in] speed_init_state 初始速度状态（s,v,a）
 * @return OCPQPSolveState 优化结果状态码
 * 
 * @par 输入/输出参数说明:
 * | 参数                | 类型               | 说明                     |
 * |---------------------|--------------------|--------------------------|
 * | speed_model_param   | SpeedModelParam&   | 包含时空约束的模型参数   |
 * | speed_init_state    | SpeedState&        | 初始纵向运动状态         |
 * | 返回值              | OCPQPSolveState    | 优化过程状态指示器       |
 *
 * @par 处理流程:
 * @startuml
start
partition 主优化阶段 {
  :加载严格约束参数;
  :构建OCP优化问题;
  :调用IPM求解器;
}
if (主优化成功?) then (是)
  :返回SUCCESS_AND_VALID;
else (否)
partition 重优化阶段 {
  :放宽s/jerk约束;
  :重建松弛优化问题;
  :二次调用求解器;
}
endif
:综合返回最终状态码;
@enduml
 *
 * @note 核心策略:
 * - 主优化阶段：采用完整时空约束（SHardUpperBound等）
 * - 重优化阶段：SHardUpperBound放宽至300m，Jerk上限提升至10m/s³
 * - 状态保护机制：初始速度不满足约束时重置加速度
 *
 * @warning 实现约束:
 * - 需确保speed_model_param与当前优化器时间网格一致
 * - 配置管理器必须完成初始化（通过init方法）
 * - 初始速度v需满足v >= 0的基础运动学约束
 */
SpeedOCPQPOptimizer::OCPQPSolveState SpeedOCPQPOptimizer::runOptimizer(SpeedModelParam& speed_model_param,
                                                                       SpeedState& speed_init_state) {
  init();      
  x_0_ = speed_init_state;
  auto status = solveSpeedOCPModel(speed_model_param, false);
  if (status == SolveStatus::SOLVED) {
    return OCPQPSolveState::SUCCESS_AND_VALID;
  } else {
    auto re_status = solveSpeedOCPModel(speed_model_param, true);
    if (re_status == SolveStatus::SOLVED) {
      ERT_PLOG_I << "[STQP_SUMMARY] Re-optimize successed." ;
      return OCPQPSolveState::SUCCESS_AND_VALID;
    } else {
      return OCPQPSolveState::FAILED;
    }
  }
}
/**
 * @brief 构建并求解速度优化控制问题（核心实现）
 * @param[in] speed_model_param 速度模型参数容器
 * @param[in] is_relaxed 约束松弛模式标志位
 * @return SolveStatus 求解器状态码
 * 
 * @par 输入/输出参数说明:
 * | 参数                | 类型               | 取值范围        | 说明                     |
 * |---------------------|--------------------|----------------|--------------------------|
 * | speed_model_param   | SpeedModelParam&   | -              | 时空约束参数容器          |
 * | is_relaxed          | bool               | [true, false]  | 是否启用约束松弛模式      |
 * | 返回值              | SolveStatus        | -              | 求解器数值状态码          |
 *
 * @par 处理流程:
 * @startuml
start
partition 初始化阶段 {
  if (首次调用?) then (是)
    :配置时间网格(0.5秒间隔);
    :设置ERK4积分器;
    :初始化IPM求解器参数;
  endif
}
partition 参数加载阶段 {
  :遍历所有时间节点;
  :设置代价函数权重;
  :加载硬/软约束边界;
  if (is_relaxed==true) then (是)
    :放宽s和jerk约束;
  endif
}
partition 初始状态保护 {
  :校验速度约束合法性;
  if (不合法?) then (是)
    :重置加速度为0;
  endif
}
partition 求解阶段 {
  :调用IPM求解器;
  if (启用风险场模型?) then (是)
    :二次优化风险场参数;
  endif
}
@enduml
 *
 * @note 关键配置参数:
 * - 主优化互补容差: 1e-2
 * - 风险场优化容差: 1e-1
 * - 最大迭代次数: 500次
 * - 热启动策略: 主优化PRIMAL，风险场COLDSTART
 *
 * @warning 实现约束:
 * - 必须通过init方法初始化配置管理器
 * - 时间网格需与speed_model_param严格对齐
 * - 初始速度必须满足v >= 0的物理约束
 */
SolveStatus SpeedOCPQPOptimizer::solveSpeedOCPModel(SpeedModelParam& speed_model_param, bool is_relaxed) {
  if (speed_ocp_ == nullptr) {
    std::string model_name = "SpeedOCPModel";
    OcpConfig ocp_config;
    for (int i = 0; i < time_grid_.size(); i++) {
      ocp_config.add_time_grid(time_grid_[i]);
    }
    ocp_config.set_time_type(OcpConfig::TIME_GRID);
    ocp_config.set_integrator_type(OcpConfig::ERK4);
    ocp_config.mutable_solver()->mutable_ipm()->set_complementary_tol(1e-2);
    ocp_config.mutable_solver()->mutable_ipm()->set_first_order_tol(1e-3);
    ocp_config.mutable_solver()->mutable_ipm()->set_inequality_constraint_tol(1e-3);
    ocp_config.mutable_solver()->mutable_ipm()->set_mu_min(1e-4);
    ocp_config.mutable_solver()->mutable_ipm()->set_max_iter_num(500);
    ocp_config.mutable_solver()->mutable_ipm()->set_enable_aggressive_mu_init(true);
    ocp_config.mutable_solver()->mutable_ipm()->set_warmstart_strategy(IPMConfig::PRIMAL);
    ocp_config.mutable_solver()->mutable_ipm()->set_auto_warmstart_fallback(true);
    speed_ocp_ = OptimalControlProblem::create(model_name, ocp_config);
  }

  for (int i = 0; i <= speed_ocp_->N(); i++) {
    speed_ocp_->setParam("SRef", speed_model_param.getParameter("SRef", i), i);
    speed_ocp_->setParam("VRef", speed_model_param.getParameter("VRef", i), i);
    speed_ocp_->setParam("K", speed_model_param.getParameter("K", i), i);
    speed_ocp_->setParam("SWeight", speed_model_param.getParameter("SWeight", i), i);
    speed_ocp_->setParam("VWeight", speed_model_param.getParameter("VWeight", i), i);
    speed_ocp_->setParam("AWeight", speed_model_param.getParameter("AWeight", i), i);
    speed_ocp_->setParam("JWeight", speed_model_param.getParameter("JWeight", i), i);
    speed_ocp_->setParam("SlackSUpperWeight", speed_model_param.getParameter("SlackSUpperWeight", i), i);
    speed_ocp_->setParam("SlackSLowerWeight", speed_model_param.getParameter("SlackSLowerWeight", i), i);
    speed_ocp_->setParam("SlackVUpperWeight", speed_model_param.getParameter("SlackVUpperWeight", i), i);
    speed_ocp_->setParam("SlackAUpperWeight", speed_model_param.getParameter("SlackAUpperWeight", i), i);
    speed_ocp_->setParam("SlackALowerWeight", speed_model_param.getParameter("SlackALowerWeight", i), i);
    speed_ocp_->setParam("SlackDVWeight", speed_model_param.getParameter("SlackDVWeight", i), i);
    speed_ocp_->setParam("SHardUpperBound", speed_model_param.getParameter("SHardUpperBound", i), i);
    speed_ocp_->setParam("SHardLowerBound", speed_model_param.getParameter("SHardLowerBound", i), i);
    speed_ocp_->setParam("SSoftUpperBound", speed_model_param.getParameter("SSoftUpperBound", i), i);
    speed_ocp_->setParam("SSoftLowerBound", speed_model_param.getParameter("SSoftLowerBound", i), i);
    speed_ocp_->setParam("SUpperBoundForDVConstraint", speed_model_param.getParameter("SUpperBoundForDVConstraint", i),
                         i);
    speed_ocp_->setParam("VHardUpperBound", speed_model_param.getParameter("VHardUpperBound", i), i);
    speed_ocp_->setParam("VHardLowerBound", speed_model_param.getParameter("VHardLowerBound", i), i);
    speed_ocp_->setParam("VSoftUpperBound", speed_model_param.getParameter("VSoftUpperBound", i), i);
    speed_ocp_->setParam("VSoftLowerBound", speed_model_param.getParameter("VHardLowerBound", i), i);
    speed_ocp_->setParam("AHardUpperBound", speed_model_param.getParameter("AHardUpperBound", i), i);
    speed_ocp_->setParam("AHardLowerBound", speed_model_param.getParameter("AHardLowerBound", i), i);
    speed_ocp_->setParam("ASoftUpperBound", speed_model_param.getParameter("ASoftUpperBound", i), i);
    speed_ocp_->setParam("ASoftLowerBound", speed_model_param.getParameter("ASoftLowerBound", i), i);
    speed_ocp_->setParam("JHardUpperBound", speed_model_param.getParameter("JHardUpperBound", i), i);
    speed_ocp_->setParam("JHardLowerBound", speed_model_param.getParameter("JHardLowerBound", i), i);
    speed_ocp_->setParam("SafeDistForDVConstraint", speed_model_param.getParameter("SafeDistForDVConstraint", i), i);
    speed_ocp_->setParam("k", speed_model_param.getParameter("k", i), i);

    if (is_relaxed) {
      speed_ocp_->setParam("SHardUpperBound", 300.0, i);
      speed_ocp_->setParam("JHardUpperBound", std::max(10.0, speed_model_param.getParameter("JHardUpperBound", i)), i);
    }
  }

  // protect v hard lower bound
  bool isVinitInvalid =
      x_0_.v + x_0_.a * time_grid_[1] +
          0.5 * time_grid_[1] * time_grid_[1] * speed_model_param.getParameter("JHardUpperBound", 1) >=
      speed_model_param.getParameter("VHardLowerBound", 1);
  if (!isVinitInvalid) {
    x_0_.a = 0.0;
  }
  Eigen::VectorXd x0(3);
  x0 << x_0_.s, x_0_.v, x_0_.a;
  speed_ocp_->setX0(x0);
  ERT_PLOG_I << " solver init state: s = " << x_0_.s << "  v = " << x_0_.v << "  a = " << x_0_.a ;

#ifdef LOG_OCP_DATA
  if (speed_ocp_->logData(&ocp_data_field_) > debug_size) {
    std::ofstream of(SpdlogWrapper::instance().getLogConfig().log_path() +
                     fmt::format("{}_data_{}.bin", ocp_data_field_.model(), debug_count_++));
    if (of) {
      google::protobuf::io::OstreamOutputStream ofs(&of);
      ocp_data_field_.SerializeToOstream(&of);
      ocp_data_field_.clear_data();
      of.close();
    }
  }
#endif
  speed_ocp_->solve();
  auto info = speed_ocp_->getInfo();
  //   SFIELD_DEBUG(qp_optimizer, "Status: {}, Iteration: {}, Computation time: {}ms", info.status,
  //   info.iteration_number,
  //                info.computation_time * 1000.0);
  if (info.status != 1) {
    // PERROR << "[STQP_FAILED] Initial state: x_0: " << x_0_.s << ", v_0: " << x_0_.v << ", a_0: " << x_0_.a;
    // PERROR << "[STQP_FAILED] Status: " << info.status << ", iteration: " << info.iteration_number
    //        << ", computation time: " << info.computation_time * 1000.0 << "ms";
    if (info.status == -1) {
      Eigen::VectorXd idx(info.infeasible_index.size());
      for (int i = 0; i < idx.size(); i++) {
        idx(i) = std::get<0>(info.infeasible_index[i]);
      }
      //   PERROR << "[STQP_FAILED] Infeasible index: " << idx.transpose();
      //   SFIELD_DEBUG(qp_optimizer, "Infeasible index: {}", idx.transpose());
    }
  }

  // ------------------------------------------------------------------------------------------
  if (speed_ocp_qp_optimizer_config_.enable_speed_risk_model()) {
    if (speed_risk_ocp_ == nullptr) {
      std::string model_name = "SpeedRiskModel";
      OcpConfig ocp_config;
      for (int i = 0; i < time_grid_.size(); i++) {
        ocp_config.add_time_grid(time_grid_[i]);
      }
      ocp_config.set_time_type(OcpConfig::TIME_GRID);
      ocp_config.set_integrator_type(OcpConfig::ERK4);
      ocp_config.mutable_solver()->mutable_ipm()->set_max_iter_num(500);
      ocp_config.mutable_solver()->mutable_ipm()->set_complementary_tol(1e-1);
      ocp_config.mutable_solver()->mutable_ipm()->set_first_order_tol(1e-2);
      ocp_config.mutable_solver()->mutable_ipm()->set_inequality_constraint_tol(1e-3);
      ocp_config.mutable_solver()->mutable_ipm()->set_equality_constraint_tol(1e-3);
      ocp_config.mutable_solver()->mutable_ipm()->set_mu_min(1e-3);
      ocp_config.mutable_solver()->mutable_ipm()->set_auto_update_param(true);
      ocp_config.mutable_solver()->mutable_ipm()->set_barrier_strategy(IPMConfig::ADAPTIVE);
      ocp_config.mutable_solver()->mutable_ipm()->set_warmstart_strategy(IPMConfig::COLDSTART);
      ocp_config.mutable_solver()->mutable_ipm()->set_auto_warmstart_fallback(true);
      speed_risk_ocp_ = OptimalControlProblem::create(model_name, ocp_config);
    }

    for (int i = 0; i <= speed_risk_ocp_->N(); i++) {
      speed_risk_ocp_->setParam("SRef", speed_model_param.getParameter("SRef", i), i);
      speed_risk_ocp_->setParam("VRef", speed_model_param.getParameter("VRef", i), i);
      speed_risk_ocp_->setParam("K", speed_model_param.getParameter("K", i), i);
      speed_risk_ocp_->setParam("SWeight", speed_model_param.getParameter("SWeight", i), i);
      speed_risk_ocp_->setParam("VWeight", speed_model_param.getParameter("VWeight", i), i);
      speed_risk_ocp_->setParam("AWeight", speed_model_param.getParameter("AWeight", i), i);
      speed_risk_ocp_->setParam("JWeight", speed_model_param.getParameter("JWeight", i), i);
      speed_risk_ocp_->setParam("SlackSUpperWeight", speed_model_param.getParameter("SlackSUpperWeight", i), i);
      speed_risk_ocp_->setParam("SlackSLowerWeight", speed_model_param.getParameter("SlackSLowerWeight", i), i);
      speed_risk_ocp_->setParam("SlackVUpperWeight", speed_model_param.getParameter("SlackVUpperWeight", i), i);
      speed_risk_ocp_->setParam("SlackAUpperWeight", speed_model_param.getParameter("SlackAUpperWeight", i), i);
      speed_risk_ocp_->setParam("SlackALowerWeight", speed_model_param.getParameter("SlackALowerWeight", i), i);
      speed_risk_ocp_->setParam("SlackDVWeight", speed_model_param.getParameter("SlackDVWeight", i), i);
      speed_risk_ocp_->setParam("SHardUpperBound", speed_model_param.getParameter("SHardUpperBound", i), i);
      speed_risk_ocp_->setParam("SHardLowerBound", speed_model_param.getParameter("SHardLowerBound", i), i);
      speed_risk_ocp_->setParam("SSoftUpperBound", speed_model_param.getParameter("SSoftUpperBound", i), i);
      speed_risk_ocp_->setParam("SSoftLowerBound", speed_model_param.getParameter("SSoftLowerBound", i), i);
      speed_risk_ocp_->setParam("SUpperBoundForDVConstraint",
                                speed_model_param.getParameter("SUpperBoundForDVConstraint", i), i);
      speed_risk_ocp_->setParam("VHardUpperBound", speed_model_param.getParameter("VHardUpperBound", i), i);
      speed_risk_ocp_->setParam("VHardLowerBound", speed_model_param.getParameter("VHardLowerBound", i), i);
      // speed_risk_ocp_->setParam("VSoftUpperBound", speed_ocp_model_.getParameter("VSoftUpperBound", i), i);
      speed_risk_ocp_->setParam("VSoftLowerBound", speed_model_param.getParameter("VHardLowerBound", i), i);
      speed_risk_ocp_->setParam("AHardUpperBound", speed_model_param.getParameter("AHardUpperBound", i), i);
      speed_risk_ocp_->setParam("AHardLowerBound", speed_model_param.getParameter("AHardLowerBound", i), i);
      speed_risk_ocp_->setParam("ASoftUpperBound", speed_model_param.getParameter("ASoftUpperBound", i), i);
      speed_risk_ocp_->setParam("ASoftLowerBound", speed_model_param.getParameter("ASoftLowerBound", i), i);
      speed_risk_ocp_->setParam("JHardUpperBound", speed_model_param.getParameter("JHardUpperBound", i), i);
      speed_risk_ocp_->setParam("JHardLowerBound", speed_model_param.getParameter("JHardLowerBound", i), i);
      speed_risk_ocp_->setParam("SafeDistForDVConstraint", speed_model_param.getParameter("SafeDistForDVConstraint", i),
                                i);
      speed_risk_ocp_->setParam("k", speed_model_param.getParameter("k", i), i);
      if (is_relaxed) {
        speed_risk_ocp_->setParam("SHardUpperBound", 300.0, i);
        speed_risk_ocp_->setParam("JHardUpperBound", std::max(10.0, speed_model_param.getParameter("JHardUpperBound", i)),
                             i);
      }
    }
    speed_risk_ocp_->setParam(
        [=](const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double t, const int k, OcpVariable* ptr) {
          double s = x(0);
          double df = speed_model_param.getStSpeedLimit()[k].jacobian(s).speed_limit;
          ptr->set("VSoftUpperBound", speed_model_param.getStSpeedLimit()[k].evaluate(s).speed_limit);
          ptr->set("residual", df * s);
          ptr->set("df", df);
        });

    // protect v hard lower bound
    speed_risk_ocp_->setX0(x0);
    speed_risk_ocp_->solve();
    auto new_info = speed_risk_ocp_->getInfo();
    // SFIELD_DEBUG(risk_field, "Status: {}, Iteration: {}, Computation time: {}ms", new_info.status,
    //              new_info.iteration_number, new_info.computation_time * 1000.0);
    ERT_PLOG_I << "  Status: " << (int)new_info.status << ", Iteration: " << new_info.iteration_number
         << ", Computation time: " << new_info.computation_time * 1000.0 << "ms" ;
    auto param = speed_risk_ocp_->getParam();
    auto res = speed_risk_ocp_->getX();
    for (int i = 0; i < time_grid_.size(); i++) {
      double s = res[i](0);
      double speed_limit = speed_model_param.getStSpeedLimit()[i].evaluate(s).speed_limit;
      std::string id = speed_model_param.getStSpeedLimit()[i].evaluate(s).id;
      ERT_PLOG_D << "t: " << time_grid_[i] << ", id: " << id << ", v limit: " << speed_limit << ", v: " << res[i](1)
           << ",  s: " << s ;
    }

    if (new_info.status == -1) {
      ERT_PLOG_I << " [STQP_FAILED] Infeasible index: " << new_info.infeasible_index.size() ;
      Eigen::VectorXd idx(new_info.infeasible_index.size());
      for (int i = 0; i < idx.size(); i++) {
        idx(i) = std::get<0>(new_info.infeasible_index[i]);
        ERT_PLOG_I << " [" << i << "] Infeasible index: " << idx(i) ;
      }
      // SFIELD_DEBUG(risk_field, "Infeasible index: {}", idx.transpose());
    }
    // PFIELD(risk_field) << "init v: " << x_0_.v << ", a: " << x_0_.a;
  }

  return info.status;
}
/**
 * @brief 获取主优化器求解结果（核心接口）
 * 
 * @return std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>> 
 *         包含状态变量序列和控制变量序列的pair
 * 
 * @details 数据结构说明：
 * - 状态变量序列：每个时刻的[s,v,a]状态值
 * - 控制变量序列：每个时刻的[jerk]控制值
 * 
 * @note 使用约束：
 * - 必须在runOptimizer返回SUCCESS_AND_VALID后调用
 * - 当speed_ocp_未初始化时返回空向量
 */
std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>> SpeedOCPQPOptimizer::getOcpSolvedResult() {
  std::vector<OcpVariable> x_res;
  std::vector<OcpVariable> u_res;
  if (speed_ocp_ != nullptr) {
    x_res = speed_ocp_->getX();
    u_res = speed_ocp_->getU();
  }
  return make_pair(x_res, u_res);
}
/**
 * @brief 获取风险场优化器求解结果（扩展接口）
 * 
 * @return std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>> 
 *         包含风险场优化后的状态变量和控制变量序列
 * 
 * @details 功能特性：
 * - 数据结构与主优化器保持格式一致
 * - 包含风险场约束下的优化轨迹
 * - 支持动态速度限制的灵敏度分析结果
 * 
 * @note 使用条件：
 * - 需启用speed_ocp_qp_optimizer_config_.enable_speed_risk_model()
 * - 必须在风险场优化成功后调用（solveSpeedOCPModel返回SOLVED）
 * - 当speed_risk_ocp_未初始化时返回空向量
 */
std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>> SpeedOCPQPOptimizer::getRiskOcpSolvedResult() {
  std::vector<OcpVariable> x_res;
  std::vector<OcpVariable> u_res;
  if (speed_risk_ocp_ != nullptr) {
    x_res = speed_risk_ocp_->getX();
    u_res = speed_risk_ocp_->getU();
  }
  return make_pair(x_res, u_res);
}

}  // namespace gpal::pnc::planning
