/**
 * @file ocp_line_smoother.cpp
 * @brief 基于最优控制问题（OCP）的线平滑器实现文件
 * @details 该文件实现了OcpLineSmoother类，用于对输入的线进行平滑处理，生成平滑后的线。
 *          主要功能包括：初始化OCP模型、求解最优控制问题、获取平滑后的线等。
 */
#include "reference_line/ocp_line_smoother.h"

#include <fmt/chrono.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <algorithm>
#include <filesystem>
#include <numeric>
#include <utility>

#include "base/log.h"
#include "math/math_utils.h"

namespace gpal::pnc::planning {
/**
 * @brief OcpLineSmoother构造函数
 * @details 该构造函数用于初始化OCP参考线平滑器对象
 *
 * @param[in] config 参考线平滑器配置，类型为planning::ReferenceLineSmootherConfig
 *
 * @par 关键变量说明:
 * - config: 参考线平滑器配置，包含OCP平滑器的相关参数
 *
 * @par 处理逻辑:
 * - 调用基类构造函数，传入配置参数
 *
 * @note 该构造函数用于创建OCP参考线平滑器对象，适用于需要初始化平滑器的场景
 *
 * @warning 需确保传入的配置参数有效，否则可能无法正确初始化平滑器
 */
OcpLineSmoother::OcpLineSmoother(const planning::ReferenceLineSmootherConfig& config) : config_(config) {}

/**
 * @brief 平滑参考线
 * @details 该函数用于对输入的原始参考线进行平滑处理，生成平滑后的参考线
 *
 * @param[in] raw_line 输入的原始参考线，类型为std::vector<math::Vec3d>
 * @param[out] res_line 平滑后的参考线，类型为std::vector<math::Vec3d>*
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - raw_line: 输入的原始参考线，包含未平滑的路径信息
 * - res_line: 平滑后的参考线，存储平滑后的路径信息
 * - config_: 参考线平滑器配置，包含OCP平滑器的相关参数
 * - h: 采样间隔，用于确定参考线上的采样点
 * - raw_ref_line: 原始参考线对象
 * - length: 参考线的长度
 * - num_of_pieces: 分段数量
 * - delta_s: 每段的弧长
 * - raw_point2d: 原始参考线的二维点集合
 * - raw_point2d_theta: 原始参考线的角度集合
 * - model: OCP模型，用于执行平滑操作
 * - optimizer_status_: 优化器的状态，表示平滑操作是否成功
 * - smoothed_point2d: 平滑后的二维点集合
 *
 * @par 处理逻辑:
 * - 预处理：设置数据记录标志，计算采样间隔和路径长度
 * - 初始化OCP：创建OCP模型，设置初始状态和参数
 * - 求解问题：调用OCP求解器进行平滑处理
 * - 获取结果：将平滑后的路径点转换为参考线
 * - 评估结果：对平滑后的参考线进行评估
 *
 * @note 该函数用于使用OCP平滑器对参考线进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的原始参考线有效，否则可能无法正确生成平滑参考线
 */
bool OcpLineSmoother::smooth(const std::vector<math::Vec3d>& raw_line, std::vector<math::Vec3d>* const res_line) {
  res_line->clear();
  if (raw_line.empty()) {
    ERT_PLOG_I << "[OcpLineSmoother::smooth]: raw_line is empty";
    return false;
  }

  // 1.preProcess
  double h = config_.ocp_smoother().s_resolution();

  ReferenceLine raw_ref_line(raw_line);
  double length = raw_ref_line.length();
  unsigned int num_of_pieces = std::max(1u, static_cast<unsigned int>(length / h));
  const double delta_s = length / num_of_pieces;
  std::vector<Eigen::Vector2d> raw_point2d;
  std::vector<double> raw_point2d_theta;
  for (double s = 0; s <= length; s += delta_s) {
    ReferencePoint rlp = raw_ref_line.getReferencePoint(s);
    if (!raw_point2d.empty()) {
      double theta = std::atan2(rlp.y() - raw_point2d.back().y(), rlp.x() - raw_point2d.back().x());
      raw_point2d_theta.emplace_back(std::move(theta));
    }
    raw_point2d.emplace_back(rlp.x(), rlp.y());
  }
  raw_point2d_theta.emplace_back(raw_point2d_theta.back());
  // ERT_PLOG_I << "[OcpLineSmoother::smooth]: raw_point2d.size() = " << raw_point2d.size()
  //      << " raw_point2d_theta.size() = " << raw_point2d_theta.size()
  //      ;

  // initialize ocp
  double t_resolution = h;
  double x0 = raw_point2d.at(0).x();
  double y0 = raw_point2d.at(0).y();
  double theta0 = std::atan2(raw_point2d[1].y() - raw_point2d[0].y(), raw_point2d[1].x() - raw_point2d[0].x());
  double kappa0 = 0;
  double thetar = 0.0;
  Eigen::VectorXd xguess = Eigen::VectorXd::Zero(NStates);    // 4
  Eigen::VectorXd uguess = Eigen::VectorXd::Zero(NControls);  // 2

  int N = raw_point2d.size() - 1;
  if (N < 0) {
    ERT_PLOG_I << "[OcpLineSmoother::smooth]: raw_point2d_size is smaller than 1";
    return false;
  }

  start_state(0) = x0;
  start_state(1) = y0;
  start_state(2) = theta0;
  start_state(3) = kappa0;

  double heading_base = theta0;
  double heading = 0.0;

  // 2. get the ocp model, set the stage
  auto model = OptimalControlProblem::create("ReferenceLineModel");
  *model->mutable_config() = config_.sync_ocp_config();
  model->mutable_config()->set_horizon_length(N);
  model->mutable_config()->set_dt(t_resolution);
  if (config_.enable_data_recorder()) {
    model->mutable_config()->set_enb_log_data(true);
  }
  model->default_param().set("l_weight_terminal", config_.ocp_smoother().l_weight_terminal());
  model->default_param().set("slack_offset_weight", config_.ocp_smoother().slack_offset_weight());
  model->default_param().set("kappa_weight_terminal", config_.ocp_smoother().kappa_weight_terminal());
  model->default_param().set("sll", config_.ocp_smoother().soft_l_lower());
  model->default_param().set("slu", config_.ocp_smoother().soft_l_upper());
  model->default_param().set("kappa_weight", config_.ocp_smoother().kappa_weight());
  model->default_param().set("dkappa_weight", config_.ocp_smoother().dkappa_weight());
  model->default_param().set("l_weight", config_.ocp_smoother().l_weight_curve());
  model->default_param().set("KappaLowerBound", config_.ocp_smoother().kappa_lower_curve());
  model->default_param().set("KappaUpperBound", config_.ocp_smoother().kappa_upper_curve());
  model->default_param().set("DKappaLowerBound", config_.ocp_smoother().dkappa_lower_curve());
  model->default_param().set("DKappaUpperBound", config_.ocp_smoother().dkappa_upper_curve());
  // ERT_PLOG_I << " h = " << h
  //           << " max_iter_num = " << model->config().solver().ipm().max_iter_num()
  //           << " max_iter_time = " << model->config().solver().ipm().max_iter_time()
  //           ;

  model->init();
  model->setX0(start_state);
  for (int i = 0; i <= model->N(); i++) {
    model->setParam("xr", raw_point2d.at(i).x(), i);
    model->setParam("yr", raw_point2d.at(i).y(), i);
    heading = raw_point2d_theta.at(i);
    thetar = getUnifySpaceHeading(heading_base, heading);
    model->setParam("thetar", thetar, i);
    xguess << raw_point2d.at(i).x(), raw_point2d.at(i).y(), thetar, 0.0;
    model->setXGuess(xguess, i);
    heading_base = thetar;
  }

  // 3. solve the problem
  auto start = std::chrono::steady_clock::now();
  auto optimizer_status_ = model->solve();
  auto end = std::chrono::steady_clock::now();
  double duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
  std::string optimizer_info =
      fmt::format("Prediction horizon: {}, Solved in {} ms, Iteration: {}, Exit flag: {}", N, duration,
                  model->solver()->iteration_number(), static_cast<int>(optimizer_status_));

  // 4. get the result
  std::vector<gpal::proto::PathPoint> smoothed_point2d;
  if (optimizer_status_ == SolveStatus::SOLVED) {
    auto X = model->getX();
    auto U = model->getU();
    for (int i = 0; i < X.size(); ++i) {
      auto& state = X[i];                           // state: x, y, theta, kappa
      auto& ctrl = i < U.size() ? U[i] : U.back();  // ctrl: dkappa, slack_offset
      auto path_point =
          to_path_point(state(0), state(1), 0.0, i * h, state(2), state(3), ctrl(0));  // x y z s theta kappa dkappa
      smoothed_point2d.push_back(std::move(path_point));
    }
    // ERT_PLOG_I << "[OcpLineSmoother::smooth]: the problem is solved by ocp" ;
  } else {
    if (config_.enable_data_recorder()) {
      debugSolver(model);
    }
    ERT_PLOG_I << "[OcpLineSmoother::smooth]: failed! " << optimizer_info;
    return false;
  }

  // assign the optimized result
  std::vector<ReferencePoint> smoothed_ref_points;
  for (const auto& p : smoothed_point2d) {
    const double x = p.x();
    const double y = p.y();
    const double z = p.z();
    const double heading = p.theta();
    gpal::pnc::SLPoint sl_pt;
    ReferencePoint ref_pt;
    if (raw_ref_line.xy2sl(math::Vec3d(x, y, z), &sl_pt)) {
      double offset = -sl_pt.l();
      ref_pt = raw_ref_line.getReferencePoint(sl_pt.s());
      double lw = ref_pt.left_bound() + offset;
      double rw = ref_pt.right_bound() + offset;
      double roadlw = ref_pt.road_left_bound() + offset;
      double roadrw = ref_pt.road_right_bound() + offset;

      ref_pt.set_x(x);
      ref_pt.set_y(y);
      ref_pt.set_z(ref_pt.z());
      ref_pt.setKappa(p.kappa());
      ref_pt.setDkappa(p.dkappa());
      ref_pt.setSlope(ref_pt.slope());
      ref_pt.setHeading(heading);
      ref_pt.setOffset(offset);
      ref_pt.setLocalS(sl_pt.s());  // lxy 20240812 modify
      ref_pt.setLeftBound(lw);
      ref_pt.setRightBound(rw);
      ref_pt.setRoadLeftBound(roadlw);
      ref_pt.setRoadRightBound(roadrw);

      res_line->emplace_back(x, y, ref_pt.z());
      smoothed_ref_points.emplace_back(std::move(ref_pt));
    }
  }

  return true;
}

/**
 * @brief 将路径点信息转换为gpal::proto::PathPoint对象
 * @details 该函数用于将输入的路径点信息（包括坐标、曲率等）转换为gpal::proto::PathPoint对象
 *
 * @param[in] x 路径点的x坐标，类型为double
 * @param[in] y 路径点的y坐标，类型为double
 * @param[in] z 路径点的z坐标，类型为double
 * @param[in] s 路径点的s值（沿路径的弧长），类型为double
 * @param[in] theta 路径点的航向角，类型为double
 * @param[in] kappa 路径点的曲率，类型为double
 * @param[in] dkappa 路径点的曲率变化率，类型为double
 * @return 返回gpal::proto::PathPoint对象，包含转换后的路径点信息
 *
 * @par 关键变量说明:
 * - x: 路径点的x坐标
 * - y: 路径点的y坐标
 * - z: 路径点的z坐标
 * - s: 路径点的s值（沿路径的弧长）
 * - theta: 路径点的航向角
 * - kappa: 路径点的曲率
 * - dkappa: 路径点的曲率变化率
 *
 * @par 处理逻辑:
 * - 创建gpal::proto::PathPoint对象
 * - 将输入的路径点信息设置到PathPoint对象中
 * - 返回PathPoint对象
 *
 * @note 该函数用于将路径点信息转换为gpal::proto::PathPoint对象，适用于需要生成PathPoint对象的场景
 *
 * @warning 需确保输入的路径点信息有效，否则可能无法正确生成PathPoint对象
 */
gpal::proto::PathPoint OcpLineSmoother::to_path_point(const double x, const double y, const double z, const double s,
                                                      const double theta, const double kappa,
                                                      const double dkappa) const {
  gpal::proto::PathPoint point;
  point.set_x(x);
  point.set_y(y);
  point.set_z(z);
  point.set_s(s);
  point.set_theta(theta);
  point.set_kappa(kappa);
  point.set_dkappa(dkappa);
  return point;
}

/**
 * @brief 统一空间航向角
 * @details 该函数用于将输入的航向角统一到基准航向角的空间中，确保航向角在合理范围内
 *
 * @param[in] heading_base 基准航向角，类型为double
 * @param[in] heading 输入的航向角，类型为double
 * @return 返回统一后的航向角，类型为double
 *
 * @par 关键变量说明:
 * - heading_base: 基准航向角，用于统一航向角的参考值
 * - heading: 输入的航向角，需要被统一到基准航向角的空间中
 *
 * @par 处理逻辑:
 * - 计算输入航向角与基准航向角的差值
 * - 将差值归一化到[-π, π]范围内
 * - 返回统一后的航向角
 *
 * @note 该函数用于确保航向角在合理范围内，适用于需要统一航向角的场景
 *
 * @warning 需确保输入的航向角有效，否则可能无法正确统一航向角
 */
double OcpLineSmoother::getUnifySpaceHeading(const double heading_base, const double heading) {
  double delta_heading = math::NormalizeAngle(heading - heading_base);
  return heading_base + delta_heading;
}

/**
 * @brief 调试求解器
 * @details 该函数用于将OCP模型的求解过程数据保存到文件中，以便后续调试和分析
 *
 * @param[in] model OCP模型，类型为std::shared_ptr<OptimalControlProblem>
 *
 * @par 关键变量说明:
 * - model: OCP模型，包含求解器的相关数据和状态
 * - debug_count: 静态计数器，用于生成唯一的调试文件名
 * - buffer: 当前文件所在目录路径
 * - direction: 调试文件的保存路径
 * - of: 文件输出流，用于将数据写入文件
 * - ocp_data_filed: OCP模型的日志数据
 *
 * @par 处理逻辑:
 * - 获取当前文件所在目录路径
 * - 生成唯一的调试文件名
 * - 创建文件输出流
 * - 将OCP模型的日志数据写入文件
 * - 关闭文件输出流
 *
 * @note 该函数用于调试OCP求解器，适用于需要保存求解过程数据的场景
 *
 * @warning 需确保传入的OCP模型有效，否则可能无法正确保存调试数据
 */
void OcpLineSmoother::debugSolver(std::shared_ptr<OptimalControlProblem> model) {
  static int debug_count = 0;
  std::filesystem::path buffer = std::filesystem::path(__FILE__).parent_path();
  std::string direction = buffer;
  std::ofstream of(direction + fmt::format("{}_data_{}.bin", "ReferenceLineModel", debug_count++));
  if (of) {
    google::protobuf::io::OstreamOutputStream ofs(&of);
    auto ocp_data_filed = model->getLogData();
    ocp_data_filed.SerializeToOstream(&of);
    ocp_data_filed.clear_data();
    of.close();
  }
}

}  // namespace gpal::pnc::planning
