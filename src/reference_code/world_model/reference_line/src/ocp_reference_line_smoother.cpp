/**
 * @file ocp_reference_line_smoother.cpp
 * @brief 基于最优控制问题（OCP）的参考线平滑器实现文件
 * @details 该文件实现了OcpReferenceLineSmoother类，用于对输入的参考线进行平滑处理，生成平滑后的参考线。
 *          主要功能包括：初始化OCP模型、求解最优控制问题、获取平滑后的参考线等。
 */

#include "reference_line/ocp_reference_line_smoother.h"

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
 * @brief OcpReferenceLineSmoother构造函数
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
OcpReferenceLineSmoother::OcpReferenceLineSmoother(const planning::ReferenceLineSmootherConfig& config)
    : ReferenceLineSmoother(config) {}

/**
 * @brief 平滑参考线
 * @details 该函数用于对输入的原始参考线进行平滑处理，生成平滑后的参考线
 *
 * @param[in] raw_reference_line 输入的原始参考线，类型为ReferenceLine
 * @param[out] smoothed_reference_line 平滑后的参考线，类型为ReferenceLine*
 * @param[in] is_async 是否异步执行平滑操作，类型为bool
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - raw_reference_line: 输入的原始参考线，包含未平滑的路径信息
 * - smoothed_reference_line: 平滑后的参考线，存储平滑后的路径信息
 * - is_async: 是否异步执行平滑操作，true表示异步，false表示同步
 * - config_: 参考线平滑器配置，包含OCP平滑器的相关参数
 * - h: 采样间隔，用于确定参考线上的采样点
 * - smooth_start_s: 平滑起始点对应的s值
 * - smooth_end_s: 平滑结束点对应的s值
 * - length: 平滑路径的长度
 * - raw_point2d: 原始参考线的二维点集合
 * - raw_point2d_theta: 原始参考线的角度集合
 * - raw_point2d_speedlimit: 原始参考线的速度限制集合
 * - raw_point2d_direction: 原始参考线的行驶方向集合
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
 * @par 流程图:
 * @startuml
 * start
 * :预处理;
 * :初始化OCP;
 * :求解问题;
 * if (平滑操作是否成功?) then (是)
 *   :获取结果;
 *   :评估结果;
 *   :返回true;
 * else (否)
 *   :输出警告信息;
 *   :返回false;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于使用OCP平滑器对参考线进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的原始参考线有效，否则可能无法正确生成平滑参考线
 */
bool OcpReferenceLineSmoother::smooth(const ReferenceLine& raw_reference_line,
                                      ReferenceLine* const smoothed_reference_line, bool is_async) {
  // 1.preProcess
  // double h = raw_reference_line.shape_type() == ReferenceLine::ShapeType::U_TURN ? 1.0 :
  // config_.ocp_smoother().s_resolution();
  double h = config_.ocp_smoother().s_resolution();
  double smooth_start_s = 0.0;
  double smooth_end_s = raw_reference_line.length();
  double length = raw_reference_line.length();
  if (!is_async) {
    h = 2.0;
    smooth_start_s = std::max<double>(raw_reference_line.adcS() - config_.smooth_length_backward(), 0.0);
    smooth_end_s =
        std::min<double>(raw_reference_line.adcS() + config_.smooth_length_forward(), raw_reference_line.length());
    length = std::min<double>(std::max<double>(smooth_end_s - smooth_start_s, 0.0), raw_reference_line.length());
  }
  const double piecewise_length = h;
  unsigned int num_of_pieces = std::max(1u, static_cast<unsigned int>(length / piecewise_length));
  const double delta_s = length / num_of_pieces;
  std::vector<Eigen::Vector2d> raw_point2d;
  std::vector<double> raw_point2d_theta;
  std::vector<double> raw_point2d_speedlimit;
  double v_lower = 1 * KMH_MS, v_upper = 80 * KMH_MS;
  std::vector<DrivingDirection> raw_point2d_direction;
  for (double s = smooth_start_s; s <= smooth_end_s; s += delta_s) {
    ReferencePoint rlp = raw_reference_line.getReferencePoint(s);
    if (!raw_point2d.empty()) {
      double theta = std::atan2(rlp.y() - raw_point2d.back().y(), rlp.x() - raw_point2d.back().x());
      raw_point2d_theta.emplace_back(std::move(theta));
    }
    raw_point2d.emplace_back(rlp.x(), rlp.y());

    double speed_limit = raw_reference_line.GetSpeedLimitFromS(s);
    raw_point2d_speedlimit.emplace_back(clamp(speed_limit, v_lower, v_upper));

    DrivingDirection direction = raw_reference_line.getDirectionFromS(s).direction;
    raw_point2d_direction.emplace_back(std::move(direction));
  }
  raw_point2d_theta.emplace_back(raw_point2d_theta.back());
  // ERT_PLOG_I << "[LocalRoute]: raw_point2d.size() = " << raw_point2d.size()
  //      << " raw_point2d_theta.size() = " << raw_point2d_theta.size()
  //      << " raw_point2d_speedlimit.size() = " << raw_point2d_speedlimit.size()
  //      << " raw_point2d_direction.size() = " << raw_point2d_direction.size()
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
    ERT_PLOG_I << "[OcpReferenceLineSmoother::smooth]: raw_point2d_size is smaller than 1";
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
  if (is_async) {
    *model->mutable_config() = config_.async_ocp_config();
  } else {
    *model->mutable_config() = config_.sync_ocp_config();
  }
  model->mutable_config()->set_horizon_length(N);
  model->mutable_config()->set_dt(t_resolution);
  if (config_.enable_data_recorder()) {
    model->mutable_config()->set_enb_log_data(true);
  }
  double l_weight_terminal =
      raw_reference_line.isTerminalEnforced() ? 400.0 : config_.ocp_smoother().l_weight_terminal();
  model->default_param().set("l_weight_terminal", l_weight_terminal);
  model->default_param().set("slack_offset_weight", config_.ocp_smoother().slack_offset_weight());
  model->default_param().set("kappa_weight_terminal", config_.ocp_smoother().kappa_weight_terminal());
  // ERT_PLOG_I << "is_async = " << is_async
  //          << " h = " << h
  //          << " max_iter_num = " << model->config().solver().ipm().max_iter_num()
  //          << " max_iter_time = " << model->config().solver().ipm().max_iter_time()
  //          ;

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

    model->setParam("sll", config_.ocp_smoother().soft_l_lower(), i);
    model->setParam("slu", config_.ocp_smoother().soft_l_upper(), i);
    model->setParam("kappa_weight", raw_point2d_speedlimit.at(i) * config_.ocp_smoother().kappa_weight(), i);
    model->setParam("dkappa_weight", raw_point2d_speedlimit.at(i) * config_.ocp_smoother().dkappa_weight(), i);

    int curve_level = 0;
    for (int j = std::max<int>(0, i - 30); j <= std::min<int>(model->N(), i + 30); ++j) {
      if (raw_point2d_direction.at(j) == DrivingDirection::kDirectionUTurnOnly) {
        curve_level = std::max<int>(2, curve_level);
        break;
      } else if (raw_point2d_direction.at(j) == DrivingDirection::kDirectionLeftOnly ||
                 raw_point2d_direction.at(j) == DrivingDirection::kDirectionRightOnly) {
        curve_level = std::max<int>(1, curve_level);
      }
    }
    if (curve_level == 2 || raw_reference_line.isMemorizedParkRoute()) {
      model->setParam("l_weight", config_.ocp_smoother().l_weight_curve(), i);
      model->setParam("KappaLowerBound", config_.ocp_smoother().kappa_lower_curve(), i);
      model->setParam("KappaUpperBound", config_.ocp_smoother().kappa_upper_curve(), i);
      model->setParam("DKappaLowerBound", config_.ocp_smoother().dkappa_lower_curve(), i);
      model->setParam("DKappaUpperBound", config_.ocp_smoother().dkappa_upper_curve(), i);
    } else if (curve_level == 1) {
      model->setParam("l_weight", config_.ocp_smoother().l_weight_curve(), i);
      model->setParam("KappaLowerBound", config_.ocp_smoother().kappa_lower_straight(), i);
      model->setParam("KappaUpperBound", config_.ocp_smoother().kappa_upper_straight(), i);
      model->setParam("DKappaLowerBound", config_.ocp_smoother().dkappa_lower_straight(), i);
      model->setParam("DKappaUpperBound", config_.ocp_smoother().dkappa_upper_straight(), i);
    } else {
      model->setParam("l_weight", config_.ocp_smoother().l_weight_straight(), i);
      model->setParam("KappaLowerBound", config_.ocp_smoother().kappa_lower_straight(), i);
      model->setParam("KappaUpperBound", config_.ocp_smoother().kappa_upper_straight(), i);
      model->setParam("DKappaLowerBound", config_.ocp_smoother().dkappa_lower_straight(), i);
      model->setParam("DKappaUpperBound", config_.ocp_smoother().dkappa_upper_straight(), i);
    }
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

    // 暂时注释掉，以防原参考线末点与平滑后的参考线末点偏差较大时的异常.
    // auto path_point = to_path_point(raw_point2d.back().x(), raw_point2d.back().y(), 0.0, X.size() * h, X.back()(2),
    //                                 X.back()(3), U.back()(0));
    // smoothed_point2d.emplace_back(path_point);

    // ERT_PLOG_I << "[OcpReferenceLineSmoother::smooth]: the problem is solved by ocp" ;
  } else {
    if (config_.enable_data_recorder()) {
      debugSolver(model);
    }
    ERT_PLOG_I << "[LocalRoute][OcpReferenceLineSmoother::smooth]: failed! " << optimizer_info;
    return false;
  }

  // assign the optimized result
  std::vector<ReferencePoint> smoothed_ref_points;
  float min_dis = std::numeric_limits<float>::max();
  float adc_s = 0.0;
  float adc_l = 0.0;
  for (const auto& p : smoothed_point2d) {
    const double x = p.x();
    const double y = p.y();
    const double z = p.z();
    const double heading = p.theta();
    gpal::pnc::SLPoint sl_pt;
    ReferencePoint ref_pt;
    if (raw_reference_line.xy2sl(math::Vec3d(x, y, z), &sl_pt)) {
      double offset = -sl_pt.l();
      ref_pt = raw_reference_line.getReferencePoint(sl_pt.s());
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

      if (!is_async) {
        float dis = std::sqrt((ref_pt.x() - raw_reference_line.adcLocalization().x()) *
                                  (ref_pt.x() - raw_reference_line.adcLocalization().x()) +
                              (ref_pt.y() - raw_reference_line.adcLocalization().y()) *
                                  (ref_pt.y() - raw_reference_line.adcLocalization().y()));
        if (dis < min_dis) {
          min_dis = dis;
          adc_s = ref_pt.local_s();
          adc_l = dis;
        }
      }

      smoothed_ref_points.emplace_back(std::move(ref_pt));
    }
  }

  // result
  std::vector<ReferencePoint> ref_points;
  if (is_async) {
    ref_points = std::move(smoothed_ref_points);
  } else {
    if (!smoothed_ref_points.empty()) {
      float split_start_s = smoothed_ref_points.front().local_s();
      float split_end_s = smoothed_ref_points.back().local_s();
      // ERT_PLOG_I << "aaaaaaa split_start_s = " << split_start_s << "  split_end_s = " << split_end_s ;
      // backward part
      for (double s = 0.0; s < split_start_s; s += delta_s) {
        ReferencePoint ref_pt = raw_reference_line.getReferencePoint(s);
        ref_points.emplace_back(std::move(ref_pt));
        // ERT_PLOG_I << "aaaaaaa backward: s = " << s ;
      }
      // smoothed part
      ref_points.insert(ref_points.end(), std::make_move_iterator(smoothed_ref_points.begin()),
                        std::make_move_iterator(smoothed_ref_points.end()));
      // forward part
      for (double s = split_end_s + delta_s; s <= raw_reference_line.length(); s += delta_s) {
        ReferencePoint ref_pt = raw_reference_line.getReferencePoint(s);
        ref_points.emplace_back(std::move(ref_pt));
        // ERT_PLOG_I << "aaaaaaa forward: s = " << s ;
      }
    }
  }

  *smoothed_reference_line = ReferenceLine(ref_points);
  smoothed_reference_line->set_line_type(raw_reference_line.line_type());
  smoothed_reference_line->set_smooth_type(ReferenceLine::SmoothType::OCP);
  smoothed_reference_line->setLaneDirection(raw_reference_line.getLaneDirection());
  smoothed_reference_line->set_shape_type(raw_reference_line.shape_type());
  smoothed_reference_line->set_global_start_s(raw_reference_line.global_start_s());
  smoothed_reference_line->set_id(raw_reference_line.id());
  smoothed_reference_line->setIsCurrentReferenceLine(raw_reference_line.isCurrentReferenceLine());
  smoothed_reference_line->setIsMemorizedParkRoute(raw_reference_line.isMemorizedParkRoute());
  smoothed_reference_line->setIsMemorizedEndRoute(raw_reference_line.isMemorizedEndRoute());
  smoothed_reference_line->setIsTerminalEnforced(raw_reference_line.isTerminalEnforced());
  smoothed_reference_line->setIsParallelVirtual(raw_reference_line.isParallelVirtual());
  *(smoothed_reference_line->mutableAdcLocalization()) = raw_reference_line.adcLocalization();
  smoothed_reference_line->setAdcS(adc_s);
  smoothed_reference_line->setAdcL(adc_l);

  // stop_lines_
  std::vector<StopLine> stop_lines;
  for (const auto& pt : raw_reference_line.getStopLines()) {
    SLPoint sl;
    smoothed_reference_line->xy2sl(math::Vec3d(pt.x, pt.y, pt.z), &sl);
    stop_lines.emplace_back(pt.id, pt.is_virtual, pt.x, pt.y, pt.z, sl.s(), pt.traffic_light_id, pt.direction);
    stop_lines.back().is_exist = true;
    stop_lines.back().type = pt.type;
  }
  smoothed_reference_line->setStopLines(std::move(stop_lines));

  // gates_
  std::vector<Gate> gates;
  for (const auto& pt : raw_reference_line.getGates()) {
    SLPoint sl;
    smoothed_reference_line->xy2sl(math::Vec3d(pt.x, pt.y, pt.z), &sl);
    gates.emplace_back(pt.id, pt.gate_status, pt.x, pt.y, pt.z, sl.s(), pt.head_stop_distance);
    gates.back().is_exist = true;
    gates.back().type = pt.type;
  }
  smoothed_reference_line->setGates(std::move(gates));

  // speed_limits_
  std::vector<SpeedLimit> speed_limits;
  for (const auto& pt : raw_reference_line.getSpeedLimits()) {
    SLPoint start_sl, end_sl;
    smoothed_reference_line->xy2sl(math::Vec3d(pt.start_point.x(), pt.start_point.y(), pt.start_point.z()), &start_sl);
    smoothed_reference_line->xy2sl(math::Vec3d(pt.end_point.x(), pt.end_point.y(), pt.end_point.z()), &end_sl);
    speed_limits.emplace_back(start_sl.s(), end_sl.s(), pt.max_speed_limit, pt.min_speed_limit, pt.start_point,
                              pt.end_point);
    speed_limits.back().recommended_speed = pt.recommended_speed;
  }
  if (!speed_limits.empty()) {
    speed_limits.front().start_s = 0;
    speed_limits.back().end_s = smoothed_reference_line->length();
  }
  smoothed_reference_line->setSpeedLimits(std::move(speed_limits));

  // directions_
  std::vector<SegmentDirection> directions;
  for (const auto& pt : raw_reference_line.getDirections()) {
    SLPoint start_sl, end_sl;
    smoothed_reference_line->xy2sl(math::Vec3d(pt.start_point.x(), pt.start_point.y(), pt.start_point.z()), &start_sl);
    smoothed_reference_line->xy2sl(math::Vec3d(pt.end_point.x(), pt.end_point.y(), pt.end_point.z()), &end_sl);
    directions.emplace_back(start_sl.s(), end_sl.s(), pt.direction, pt.start_point, pt.end_point);
  }
  if (!directions.empty()) {
    directions.front().start_s = 0;
    directions.back().end_s = smoothed_reference_line->length();
  }
  smoothed_reference_line->setDirections(std::move(directions));

  // source_infos_
  std::vector<std::tuple<LineSourceType, float, float>> source_infos;
  for (const auto& source : raw_reference_line.getSourceInfos()) {
    float start_s = std::fmin(smoothed_reference_line->length(), std::fmax(0.0, std::get<1>(source)));
    float end_s = std::fmin(smoothed_reference_line->length(), std::fmax(0.0, std::get<2>(source)));
    source_infos.emplace_back(std::get<0>(source), std::move(start_s), std::move(end_s));
  }
  if (!source_infos.empty()) {
    std::get<1>(source_infos.front()) = 0.0;
    std::get<2>(source_infos.back()) = smoothed_reference_line->length();
  }
  smoothed_reference_line->setSourceInfos(std::move(source_infos));

  // navigation_lane_change_info_
  smoothed_reference_line->setNavigationLaneChangeInfo(raw_reference_line.getNavigationLaneChangeInfo());

  // boundary_types_
  std::vector<SegmentBoundaryType> boundary_types;
  for (const auto& boundary_type : raw_reference_line.getBoundaryTypes()) {
    SLPoint start_sl, end_sl;
    smoothed_reference_line->xy2sl(
        math::Vec3d(boundary_type.start_point.x(), boundary_type.start_point.y(), boundary_type.start_point.z()),
        &start_sl);
    smoothed_reference_line->xy2sl(
        math::Vec3d(boundary_type.end_point.x(), boundary_type.end_point.y(), boundary_type.end_point.z()), &end_sl);
    boundary_types.emplace_back(start_sl.s(), end_sl.s(), boundary_type.left_type, boundary_type.right_type,
                                boundary_type.start_point, boundary_type.end_point);
  }
  if (!boundary_types.empty()) {
    boundary_types.front().start_s = 0;
    boundary_types.back().end_s = smoothed_reference_line->length();
  }
  smoothed_reference_line->setBoundaryTypes(std::move(boundary_types));

  // left_reference_lines_
  std::unordered_map<std::string, NeighborLocalRoute> left_reference_lines;
  for (const auto& neighbor : raw_reference_line.getLeftReferenceLines()) {
    left_reference_lines[neighbor.first].id = neighbor.second.id;
    left_reference_lines[neighbor.first].elements.clear();
    for (const auto& element : neighbor.second.elements) {
      if (std::get<0>(element.range)) {
        NeighborLocalRoute::Element el = element;
        SLPoint start_sl, end_sl;
        smoothed_reference_line->xy2sl(
            math::Vec3d(element.points.first.x(), element.points.first.y(), element.points.first.z()), &start_sl);
        smoothed_reference_line->xy2sl(
            math::Vec3d(element.points.second.x(), element.points.second.y(), element.points.second.z()), &end_sl);
        std::get<0>(el.range) = true;
        std::get<1>(el.range) = std::fmax(0.0, start_sl.s());
        std::get<2>(el.range) = std::fmin(end_sl.s(), smoothed_reference_line->length());
        left_reference_lines[neighbor.first].elements.emplace_back(std::move(el));
      }
    }
  }
  smoothed_reference_line->setLeftReferenceLines(std::move(left_reference_lines));

  // right_reference_lines_
  std::unordered_map<std::string, NeighborLocalRoute> right_reference_lines;
  for (const auto& neighbor : raw_reference_line.getRightReferenceLines()) {
    right_reference_lines[neighbor.first].id = neighbor.second.id;
    right_reference_lines[neighbor.first].elements.clear();
    for (const auto& element : neighbor.second.elements) {
      if (std::get<0>(element.range)) {
        NeighborLocalRoute::Element el = element;
        SLPoint start_sl, end_sl;
        smoothed_reference_line->xy2sl(
            math::Vec3d(element.points.first.x(), element.points.first.y(), element.points.first.z()), &start_sl);
        smoothed_reference_line->xy2sl(
            math::Vec3d(element.points.second.x(), element.points.second.y(), element.points.second.z()), &end_sl);
        std::get<0>(el.range) = true;
        std::get<1>(el.range) = std::fmax(0.0, start_sl.s());
        std::get<2>(el.range) = std::fmin(end_sl.s(), smoothed_reference_line->length());
        right_reference_lines[neighbor.first].elements.emplace_back(std::move(el));
      }
    }
  }
  smoothed_reference_line->setRightReferenceLines(std::move(right_reference_lines));
  // // print info
  // ERT_PLOG_I << "[SmoothReferenceLine-LocalRoute]: reference_line id = " << smoothed_reference_line->id()
  //           << "  getLeftReferenceLines.size() = " << smoothed_reference_line->getLeftReferenceLines().size() ;
  // for(const auto& neighbor : smoothed_reference_line->getLeftReferenceLines()) {
  //   ERT_PLOG_I << "  key = " << neighbor.first
  //             << "  neighbor local route id = " << neighbor.second.id
  //              << "  element size = " << neighbor.second.elements.size() ;
  //   for(const auto& element : neighbor.second.elements) {
  //     ERT_PLOG_I << "    bound_type = " << static_cast<int>(element.bound_type)
  //               << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range)<< ", " <<
  //               std::get<2>(element.range)
  //               << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size()
  //               ;
  //   }
  // }
  // ERT_PLOG_I << "[SmoothReferenceLine-LocalRoute]: reference_line id = " << smoothed_reference_line->id()
  //           << "  getRightReferenceLines.size() = " << smoothed_reference_line->getRightReferenceLines().size() ;
  // for(const auto& neighbor : smoothed_reference_line->getRightReferenceLines()) {
  //   ERT_PLOG_I << "  key = " << neighbor.first
  //             << "  neighbor local route id = " << neighbor.second.id
  //              << "  element size = " << neighbor.second.elements.size() ;
  //   for(const auto& element : neighbor.second.elements) {
  //     ERT_PLOG_I << "    bound_type = " << static_cast<int>(element.bound_type)
  //               << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range)<< ", " <<
  //               std::get<2>(element.range)
  //               << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size()
  //               ;
  //   }
  // }

  // areas
  smoothed_reference_line->setAreas(raw_reference_line.getAreas());

  // road_info
  smoothed_reference_line->setRoadInfo(raw_reference_line.getRoadInfo());

  // merge_fork_ranges_
  std::vector<MergeForkRange> ranges;
  for (const auto& range : raw_reference_line.getMergeForkRanges()) {
    MergeForkRange res = range;
    SLPoint sl;
    smoothed_reference_line->xy2sl(math::Vec3d(range.point().x(), range.point().y(), range.point().z()), &sl);
    res.setS(sl.s());
    ranges.emplace_back(std::move(res));
  }
  smoothed_reference_line->setMergeForkRanges(std::move(ranges));

  // navigation_scores_
  std::vector<std::tuple<float, float, float>> navigation_scores;
  for (const auto& score : raw_reference_line.getNavigationScores()) {
    float start_s = std::fmin(smoothed_reference_line->length(), std::fmax(0.0, std::get<1>(score)));
    float end_s = std::fmin(smoothed_reference_line->length(), std::fmax(0.0, std::get<2>(score)));
    navigation_scores.emplace_back(std::get<0>(score), std::move(start_s), std::move(end_s));
  }
  if (!navigation_scores.empty()) {
    std::get<1>(navigation_scores.front()) = 0.0;
    std::get<2>(navigation_scores.back()) = smoothed_reference_line->length();
  }
  smoothed_reference_line->setNavigationScores(std::move(navigation_scores));

  // navigation lane change ranges from memorized route for POC
  std::vector<std::tuple<float, float, bool, bool>> navigation_lane_change_ranges;
  for (const auto& range : raw_reference_line.getNavigationLaneChangeRanges()) {
    float start_s = std::fmin(smoothed_reference_line->length(), std::fmax(0.0, std::get<0>(range)));
    float end_s = std::fmin(smoothed_reference_line->length(), std::fmax(0.0, std::get<1>(range)));
    navigation_lane_change_ranges.emplace_back(std::move(start_s), std::move(end_s), std::get<2>(range),
                                               std::get<3>(range));
  }
  smoothed_reference_line->setNavigationLaneChangeRanges(std::move(navigation_lane_change_ranges));

  // perception lane follow ranges from memorized route for POC
  std::vector<std::pair<float, float>> lane_follow_ranges;
  for (const auto& range : raw_reference_line.getLaneFollowRanges()) {
    float start_s = std::fmin(smoothed_reference_line->length(), std::fmax(0.0, range.first));
    float end_s = std::fmin(smoothed_reference_line->length(), std::fmax(0.0, range.second));
    lane_follow_ranges.emplace_back(std::move(start_s), std::move(end_s));
  }
  smoothed_reference_line->setlaneFollowRanges(std::move(lane_follow_ranges));

  // ERT_PLOG_I << "raw_reference_line_size = " << raw_reference_line.reference_points().size() << " length = " <<
  // raw_reference_line.length(); ERT_PLOG_I << "raw_point2d_size = " << raw_point2d.size() ; ERT_PLOG_I <<
  // "raw_point2d_theta_size = " << raw_point2d_theta.size() ; ERT_PLOG_I << "model->N() = " << model->N() << "
  // model->getX().size() = " << model->getX().size() ; ERT_PLOG_I << "smoothed_reference_line_size = " <<
  // smoothed_reference_line->reference_points().size() ;

  if (config_.enable_evaluation()) {
    std::string evaluation_msg =
        ReferenceLineEvaluate(*smoothed_reference_line, raw_point2d, raw_point2d_direction, duration) + optimizer_info +
        "\n";

    // plot
    if (raw_reference_line.reference_points().size() > 0 && smoothed_reference_line->reference_points().size() > 0) {
      std::filesystem::path buffer = std::filesystem::path(__FILE__).parent_path();
      std::string direction = buffer;
      direction += smoothed_reference_line->id() + ".csv";
      ERT_PLOG_I << "direction = " << direction;
      std::ofstream test_file;
      test_file.open(direction, std::ios::out);
      int num =
          std::max(raw_reference_line.reference_points().size(), smoothed_reference_line->reference_points().size());
      auto route_pt = raw_reference_line.reference_points().at(0);
      auto smooth_pt = smoothed_reference_line->reference_points().at(0);
      test_file << evaluation_msg;
      for (int i = 0; i < num; i++) {
        if (i >= raw_reference_line.reference_points().size()) {
          route_pt = raw_reference_line.reference_points().back();
        } else {
          route_pt = raw_reference_line.reference_points().at(i);
        }
        if (i >= smoothed_reference_line->reference_points().size()) {
          smooth_pt = smoothed_reference_line->reference_points().back();
        } else {
          smooth_pt = smoothed_reference_line->reference_points().at(i);
        }
        test_file << i << ", " << route_pt.x() << ", " << route_pt.y() << ", " << smooth_pt.x() << ", " << smooth_pt.y()
                  << ", " << smooth_pt.kappa() << ", " << smooth_pt.dkappa() << ", " << smooth_pt.offset() << "\n";
      }
      test_file.close();
    }
    // all evaluation data
    std::filesystem::path eva = std::filesystem::path(__FILE__).parent_path();
    std::string eva_dir = eva;
    eva_dir += "evaluation.csv";
    std::ofstream eva_file;
    eva_file.open(eva_dir, std::ios::app);
    eva_file << evaluation_msg << "\n";
    eva_file << "\n";
    eva_file.close();
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
gpal::proto::PathPoint OcpReferenceLineSmoother::to_path_point(const double x, const double y, const double z,
                                                               const double s, const double theta, const double kappa,
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
double OcpReferenceLineSmoother::getUnifySpaceHeading(const double heading_base, const double heading) {
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
void OcpReferenceLineSmoother::debugSolver(std::shared_ptr<OptimalControlProblem> model) {
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

/**
 * @brief 评估平滑后的参考线
 * @details 该函数用于评估平滑后的参考线，计算最大曲率、最大曲率变化率、最大偏移量等指标，并生成评估信息
 *
 * @param[in] smoothed_reference_line 平滑后的参考线，类型为ReferenceLine
 * @param[in] raw_point2d 原始参考线的二维点集合，类型为std::vector<Eigen::Vector2d>
 * @param[in] raw_direction 原始参考线的行驶方向集合，类型为std::vector<DrivingDirection>
 * @param[in] duration 平滑操作的耗时，类型为double
 * @return 返回评估信息字符串，类型为std::string
 *
 * @par 关键变量说明:
 * - smoothed_reference_line: 平滑后的参考线
 * - raw_point2d: 原始参考线的二维点集合
 * - raw_direction: 原始参考线的行驶方向集合
 * - duration: 平滑操作的耗时
 * - max_kappa: 最大曲率
 * - max_dkappa: 最大曲率变化率
 * - max_offset: 最大偏移量
 * - min_left_bound: 最小左边界
 * - max_right_bound: 最大右边界
 * - raw_smooth_points2d_delta_sum: 原始参考线与平滑后参考线的点间距离总和
 * - smooth_points2d_delta_sum: 平滑后参考线的点间距离总和
 * - offset_mean: 偏移量的平均值
 * - offset_std: 偏移量的标准差
 * - kappa_square_sum: 曲率的平方和
 * - dkappa_square_sum: 曲率变化率的平方和
 *
 * @par 处理逻辑:
 * - 初始化评估指标
 * - 遍历平滑后的参考线，计算最大曲率、最大曲率变化率、最大偏移量等指标
 * - 计算偏移量的平均值和标准差
 * - 计算曲率和曲率变化率的平方和
 * - 生成评估信息字符串
 *
 * @note 该函数用于评估平滑后的参考线，适用于需要分析平滑效果的场景
 *
 * @warning 需确保输入的参考线有效，否则可能无法正确生成评估信息
 */
std::string OcpReferenceLineSmoother::ReferenceLineEvaluate(const ReferenceLine& smoothed_reference_line,
                                                            const std::vector<Eigen::Vector2d>& raw_point2d,
                                                            const std::vector<DrivingDirection>& raw_direction,
                                                            double duration) {
  std::string evaluation_msg = "";
  double max_kappa = 0.0;
  double max_dkappa = 0.0;
  double max_offset = 0.0;
  int max_kappa_index = 0;
  int max_dkappa_index = 0;
  int max_offset_index = 0;
  int max_kappa_direction = 0;
  int max_dkappa_direction = 0;
  int max_offset_direction = 0;
  double min_left_bound = 100;
  double max_right_bound = -100;
  int min_left_bound_index = 0;
  int max_right_bound_index = 0;
  double raw_smooth_points2d_delta_sum = 0.0;
  double smooth_points2d_delta_sum = 0.0;

  std::vector<ReferencePoint> smoothed_points;
  for (int i = 0; i < smoothed_reference_line.reference_points().size(); ++i) {
    smoothed_points.emplace_back(smoothed_reference_line.reference_points().at(i));
  }

  for (int i = 0; i < std::min(std::min(smoothed_points.size(), raw_point2d.size()), raw_direction.size()); ++i) {
    double kappa = std::abs(smoothed_points.at(i).kappa());
    double dkappa = std::abs(smoothed_points.at(i).dkappa());
    double offset = std::abs(smoothed_points.at(i).offset());
    double left_bound = smoothed_points.at(i).left_bound();
    double right_bound = smoothed_points.at(i).right_bound();

    if (kappa > max_kappa) {
      max_kappa = kappa;
      max_kappa_index = i;
      max_kappa_direction = static_cast<int>(raw_direction.at(i));
    }
    if (dkappa > max_dkappa) {
      max_dkappa = dkappa;
      max_dkappa_index = i;
      max_dkappa_direction = static_cast<int>(raw_direction.at(i));
    }
    if (offset > max_offset) {
      max_offset = offset;
      max_offset_index = i;
      max_offset_direction = static_cast<int>(raw_direction.at(i));
    }
    if (left_bound < min_left_bound) {
      min_left_bound = left_bound;
      min_left_bound_index = i;
    }
    if (right_bound > max_right_bound) {
      max_right_bound = right_bound;
      max_right_bound_index = i;
    }
    double xr = raw_point2d.at(i).x();
    double yr = raw_point2d.at(i).y();
    double xs = smoothed_points.at(i).x();
    double ys = smoothed_points.at(i).y();
    raw_smooth_points2d_delta_sum += sqrt(pow((xr - xs), 2) + pow((yr - ys), 2));
  }

  for (int i = 0; i < smoothed_points.size() - 1; ++i) {
    smooth_points2d_delta_sum += sqrt(pow((sqrt(pow((smoothed_points.at(i).x() - smoothed_points.at(i + 1).x()), 2) +
                                                pow((smoothed_points.at(i).y() - smoothed_points.at(i + 1).y()), 2)) -
                                           1),
                                          2));
  }

  double offset_mean = accumulate(smoothed_points.begin(), smoothed_points.end(), 0.0,
                                  [&](double sum, ReferencePoint pt) { return sum + pt.offset(); }) /
                       smoothed_points.size();
  double offset_var = 0.0;
  for_each(smoothed_points.begin(), smoothed_points.end(),
           [&](const ReferencePoint pt) { offset_var += (pt.offset() - offset_mean) * (pt.offset() - offset_mean); });
  offset_var = offset_var / smoothed_points.size();
  double offset_std = sqrt(offset_var);

  double kappa_square_sum = accumulate(smoothed_points.begin(), smoothed_points.end(), 0.0,
                                       [&](double sum, ReferencePoint pt) { return sum + pt.kappa() * pt.kappa(); });
  double dkappa_square_sum = accumulate(smoothed_points.begin(), smoothed_points.end(), 0.0,
                                        [&](double sum, ReferencePoint pt) { return sum + pt.dkappa() * pt.dkappa(); });

  evaluation_msg += fmt::format(
      "ID: {}\nType: {}   Shape: {}\nPointsSize: {}\nMaxKappa: {}   MaxKappaIndex: {}   MaxKappaDirection: "
      "{}\nMaxDKappa: {}   MaxDKappaIndex: {}   MaxDKappaDirection: {}\nMaxOffset: {}   MaxOffsetIndex: {}   "
      "MaxOffsetDirection: {}\nMeanOffset: {}\nStdOffset: {}\nMinLeftBound: {}   MinLeftBoundIndex: {}\nMaxRightBound: "
      "{}   MaxRightBoundIndex: {}\n",
      smoothed_reference_line.id(), static_cast<int>(smoothed_reference_line.line_type()),
      static_cast<int>(smoothed_reference_line.shape_type()), smoothed_points.size(), max_kappa, max_kappa_index,
      max_kappa_direction, max_dkappa, max_dkappa_index, max_dkappa_direction, max_offset, max_offset_index,
      max_offset_direction, offset_mean, offset_std, min_left_bound, min_left_bound_index, max_right_bound,
      max_right_bound_index);

  return evaluation_msg;
}

}  // namespace gpal::pnc::planning
