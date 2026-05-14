/**
 * @file reference_line_provider.cpp
 * @brief 参考线提供器实现文件
 * @details 该文件实现了ReferenceLineProvider类，负责为规划模块提供平滑后的参考线。
 *          主要功能包括：初始化参考线平滑器、平滑不同类型的路径（如记忆路线、感知车道线、历史路径等）、
 *          以及将原始路径转换为参考线。
 */

#include "reference_line/reference_line_provider.h"

#include <filesystem>

#include "config_manager/config_manager.h"
#include "math/math_utils.h"
#include "reference_line/ocp_reference_line_smoother.h"
#include "time.h"
#include "util/util.h"

namespace gpal::pnc::planning {

using math::Vec2d;

const double kBoundEpsilon = 1e-6;

/**
 * @brief 构造函数，用于初始化ReferenceLineProvider对象
 * @details 该构造函数负责初始化参考线提供器，包括从配置管理器中获取参考线平滑器的配置，并设置初始化标志为true
 *
 * @par 关键变量说明:
 * - config_manager: 配置管理器的单例对象，用于获取参考线平滑器的配置
 * - smoother_config_: 参考线平滑器的配置对象，存储平滑器的相关参数
 * - is_initialized_: 初始化标志，表示对象是否已成功初始化
 *
 * @par 处理逻辑:
 * - 获取配置管理器的单例对象
 * - 从配置管理器中获取参考线平滑器的配置
 * - 设置初始化标志为true
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取配置管理器的单例对象;
 * :从配置管理器中获取参考线平滑器的配置;
 * :设置初始化标志为true;
 * stop
 * @enduml
 *
 * @note 该构造函数用于初始化参考线提供器，适用于需要根据配置管理器中的配置初始化参考线平滑器的场景
 *
 * @warning 需确保配置管理器已正确初始化，否则可能无法获取平滑器配置
 */
ReferenceLineProvider::ReferenceLineProvider() {
  ConfigManager* config_manager = Singleton<ConfigManager>::get_instance();
  smoother_config_ = config_manager->getConfig<ReferenceLineSmootherConfig>("OcpSmootherConfig");
  is_initialized_ = true;
}

/**
 * @brief 平滑记忆路线切片路径
 * @details 该函数用于对输入的记忆路线切片路径进行平滑处理，生成平滑后的参考线
 *
 * @param[in] route 输入的记忆路线切片路径，类型为std::shared_ptr<SlicedRoute>
 * @param[out] reference_lines 平滑后的参考线，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - route: 输入的记忆路线切片路径，包含路径的原始信息
 * - reference_lines: 平滑后的参考线，存储平滑后的路径信息
 * - raw_ref_line: 原始参考线对象，用于存储未平滑的路径信息
 * - short_refline_parameter: 参考线长度阈值，用于判断参考线是否过短
 * - reference_line: 平滑后的参考线对象
 *
 * @par 处理逻辑:
 * - 检查输入参数是否为空
 * - 清空输出参考线容器
 * - 创建原始参考线对象
 * - 判断原始参考线长度是否过短，若过短则输出警告并返回失败
 * - 记录平滑开始时间
 * - 调用平滑函数对原始参考线进行平滑处理
 * - 记录平滑结束时间并计算平滑耗时
 * - 将平滑后的参考线存入输出容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输入参数是否为空;
 * :清空输出参考线容器;
 * :创建原始参考线对象;
 * if (原始参考线长度是否过短?) then (是)
 *   :输出警告信息;
 *   :返回失败;
 * else (否)
 *   :记录平滑开始时间;
 *   :调用平滑函数进行平滑处理;
 *   :记录平滑结束时间并计算平滑耗时;
 *   :将平滑后的参考线存入输出容器;
 *   :返回成功;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于对记忆路线切片路径进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的记忆路线切片路径有效，否则可能无法正确生成平滑参考线
 */
bool ReferenceLineProvider::smoothSlicedRoute(const std::shared_ptr<SlicedRoute> route,
                                              std::vector<ReferenceLine>* reference_lines) {
  CHECK_NOTNULL(route);
  CHECK_NOTNULL(reference_lines);
  reference_lines->clear();

  ReferenceLine raw_ref_line(route.get());
  raw_ref_line.set_smooth_type(ReferenceLine::SmoothType::RAW);
  const double short_refline_parameter = 5.0;
  if (raw_ref_line.length() < short_refline_parameter) {
    ERT_PLOG_I << "[ReferenceLineProvider::smoothSlicedRoute]: raw_ref_line is too short! id = " << raw_ref_line.id()
               << "  num_points = " << raw_ref_line.num_points()
               << "  num_segments = " << raw_ref_line.num_line_segments() << "  length = " << raw_ref_line.length();
    return false;
  }

  // smooth process
  const auto ss_time = std::chrono::system_clock::now();
  ReferenceLine reference_line;
  smoothReferenceLine(raw_ref_line, &reference_line, true);
  const auto se_time = std::chrono::system_clock::now();
  double s_time = static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(se_time - ss_time).count());
  reference_lines->emplace_back(std::move(reference_line));

  return true;
}

/**
 * @brief 平滑感知车道线
 * @details 该函数用于对输入的感知车道线进行平滑处理，生成平滑后的参考线
 *
 * @param[in] lanes 输入的感知车道线，类型为std::vector<PerceptionLane>
 * @param[out] reference_lines 平滑后的参考线，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - lanes: 输入的感知车道线，包含车道的原始信息
 * - reference_lines: 平滑后的参考线，存储平滑后的路径信息
 * - raw_ref_line: 原始参考线对象，用于存储未平滑的路径信息
 * - short_refline_parameter: 参考线长度阈值，用于判断参考线是否过短
 * - reference_line: 平滑后的参考线对象
 *
 * @par 处理逻辑:
 * - 检查输出参数是否为空
 * - 清空输出参考线容器
 * - 遍历输入的感知车道线
 *   - 创建原始参考线对象
 *   - 判断原始参考线长度是否过短，若过短则输出警告并跳过当前车道
 *   - 记录平滑开始时间
 *   - 调用平滑函数对原始参考线进行平滑处理
 *   - 记录平滑结束时间并计算平滑耗时
 *   - 将平滑后的参考线存入输出容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输出参数是否为空;
 * :清空输出参考线容器;
 * :遍历输入的感知车道线;
 *   :创建原始参考线对象;
 *   if (原始参考线长度是否过短?) then (是)
 *     :输出警告信息;
 *     :跳过当前车道;
 *   else (否)
 *     :记录平滑开始时间;
 *     :调用平滑函数进行平滑处理;
 *     :记录平滑结束时间并计算平滑耗时;
 *     :将平滑后的参考线存入输出容器;
 *   endif
 * :返回成功;
 * stop
 * @enduml
 *
 * @note 该函数用于对感知车道线进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的感知车道线有效，否则可能无法正确生成平滑参考线
 */
bool ReferenceLineProvider::smoothPerceptionLanes(const std::vector<PerceptionLane>& lanes,
                                                  std::vector<ReferenceLine>* reference_lines) {
  CHECK_NOTNULL(reference_lines);
  reference_lines->clear();

  for (const auto& lane : lanes) {
    ReferenceLine raw_ref_line(&lane);
    raw_ref_line.set_smooth_type(ReferenceLine::SmoothType::RAW);
    const double short_refline_parameter = 5.0;
    if (raw_ref_line.length() < short_refline_parameter) {
      ERT_PLOG_I << "[PerceptionLane][ReferenceLineProvider::smoothPerceptionLanes]: raw_ref_line is too short! id = "
                 << raw_ref_line.id() << "  num_points = " << raw_ref_line.num_points()
                 << "  num_segments = " << raw_ref_line.num_line_segments() << "  length = " << raw_ref_line.length();
      continue;
    }

    // smooth process
    const auto ss_time = std::chrono::system_clock::now();
    ReferenceLine reference_line;
    smoothReferenceLine(raw_ref_line, &reference_line);
    const auto se_time = std::chrono::system_clock::now();
    double s_time = static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(se_time - ss_time).count());
    reference_lines->emplace_back(std::move(reference_line));
  }

  return true;
}

/**
 * @brief 平滑历史轨迹
 * @details 该函数用于对输入的历史轨迹进行平滑处理，生成平滑后的参考线
 *
 * @param[in] path_history 输入的历史轨迹，类型为MainPathHistory
 * @param[out] reference_lines 平滑后的参考线，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - path_history: 输入的历史轨迹，包含路径的原始信息
 * - reference_lines: 平滑后的参考线，存储平滑后的路径信息
 * - raw_ref_line: 原始参考线对象，用于存储未平滑的路径信息
 * - short_refline_parameter: 参考线长度阈值，用于判断参考线是否过短
 * - reference_line: 平滑后的参考线对象
 *
 * @par 处理逻辑:
 * - 检查输出参数是否为空
 * - 清空输出参考线容器
 * - 创建原始参考线对象
 * - 判断原始参考线长度是否过短，若过短则输出警告
 * - 记录平滑开始时间
 * - 调用平滑函数对原始参考线进行平滑处理
 * - 记录平滑结束时间并计算平滑耗时
 * - 将平滑后的参考线存入输出容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输出参数是否为空;
 * :清空输出参考线容器;
 * :创建原始参考线对象;
 * if (原始参考线长度是否过短?) then (是)
 *   :输出警告信息;
 * else (否)
 *   :记录平滑开始时间;
 *   :调用平滑函数进行平滑处理;
 *   :记录平滑结束时间并计算平滑耗时;
 *   :将平滑后的参考线存入输出容器;
 * endif
 * :返回成功;
 * stop
 * @enduml
 *
 * @note 该函数用于对历史轨迹进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的历史轨迹有效，否则可能无法正确生成平滑参考线
 */
bool ReferenceLineProvider::smoothPathHistory(const MainPathHistory& path_history,
                                              std::vector<ReferenceLine>* reference_lines) {
  CHECK_NOTNULL(reference_lines);
  reference_lines->clear();
  ReferenceLine raw_ref_line(&path_history);
  raw_ref_line.set_smooth_type(ReferenceLine::SmoothType::RAW);
  const double short_refline_parameter = 5.0;
  if (raw_ref_line.length() < short_refline_parameter) {
    ERT_PLOG_I << "[PathHistory][ReferenceLineProvider::smoothPathHistory]: raw_ref_line is too short! id = "
               << raw_ref_line.id() << "  num_points = " << raw_ref_line.num_points()
               << "  num_segments = " << raw_ref_line.num_line_segments() << "  length = " << raw_ref_line.length();
  }
  // smooth process
  const auto ss_time = std::chrono::system_clock::now();
  ReferenceLine reference_line;
  smoothReferenceLine(raw_ref_line, &reference_line);
  const auto se_time = std::chrono::system_clock::now();
  double s_time = static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(se_time - ss_time).count());
  reference_lines->emplace_back(std::move(reference_line));
  return true;
}

/**
 * @brief 平滑LocalRoute
 * @details 该函数用于对输入的LocalRoute进行平滑处理，生成平滑后的参考线
 *
 * @param[in] local_routes 输入的LocalRoute，类型为std::map<int, LocalRoute>
 * @param[out] reference_lines 平滑后的参考线，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - local_routes: 输入的LocalRoute，包含路径的原始信息
 * - reference_lines: 平滑后的参考线，存储平滑后的路径信息
 * - raw_ref_line: 原始参考线对象，用于存储未平滑的路径信息
 * - short_refline_parameter: 参考线长度阈值，用于判断参考线是否过短
 * - reference_line: 平滑后的参考线对象
 *
 * @par 处理逻辑:
 * - 检查输出参数是否为空
 * - 清空输出参考线容器
 * - 遍历输入的LocalRoute
 *   - 检查LocalRoute的有效范围
 *   - 创建原始参考线对象
 *   - 判断原始参考线长度是否过短，若过短则输出警告并跳过当前路径
 *   - 记录平滑开始时间
 *   - 调用平滑函数对原始参考线进行平滑处理
 *   - 记录平滑结束时间并计算平滑耗时
 *   - 将平滑后的参考线存入输出容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输出参数是否为空;
 * :清空输出参考线容器;
 * :遍历输入的LocalRoute;
 *   :检查LocalRoute的有效范围;
 *   :创建原始参考线对象;
 *   if (原始参考线长度是否过短?) then (是)
 *     :输出警告信息;
 *     :跳过当前路径;
 *   else (否)
 *     :记录平滑开始时间;
 *     :调用平滑函数进行平滑处理;
 *     :记录平滑结束时间并计算平滑耗时;
 *     :将平滑后的参考线存入输出容器;
 *   endif
 * :返回成功;
 * stop
 * @enduml
 *
 * @note 该函数用于对LocalRoute进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的LocalRoute有效，否则可能无法正确生成平滑参考线
 */
bool ReferenceLineProvider::smoothLocalRoutes(const std::map<int, LocalRoute>& local_routes,
                                              std::vector<ReferenceLine>* reference_lines) {
  CHECK_NOTNULL(reference_lines);
  reference_lines->clear();

  for (const auto& local_route : local_routes) {
    if (std::get<0>(local_route.second.validRange())) {
      ReferenceLine raw_ref_line(&local_route.second);
      raw_ref_line.set_smooth_type(ReferenceLine::SmoothType::RAW);
      const double short_refline_parameter = 5.0;
      if (raw_ref_line.length() < short_refline_parameter) {
        ERT_PLOG_I << "[LocalRoute][ReferenceLineProvider::smoothLocalRoutes]: raw_ref_line is too short! id = "
                   << raw_ref_line.id() << "  num_points = " << raw_ref_line.num_points()
                   << "  num_segments = " << raw_ref_line.num_line_segments() << "  length = " << raw_ref_line.length();
        continue;
      }

      // smooth process
      const auto ss_time = std::chrono::steady_clock::now();
      ReferenceLine reference_line;
      smoothReferenceLine(raw_ref_line, &reference_line);
      const auto se_time = std::chrono::steady_clock::now();
      double s_time = std::chrono::duration_cast<std::chrono::milliseconds>(se_time - ss_time).count();
      reference_lines->emplace_back(std::move(reference_line));
    }
  }

  return true;
}

/**
 * @brief 平滑参考线
 * @details 该函数用于对输入的记忆路线切片路径进行平滑处理，生成平滑后的参考线
 *
 * @param[in] route 输入的记忆路线切片路径，类型为std::shared_ptr<SlicedRoute>
 * @param[out] smoothed_res 平滑后的参考线，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - route: 输入的记忆路线切片路径，包含路径的原始信息
 * - smoothed_res: 平滑后的参考线，存储平滑后的路径信息
 * - start_time: 平滑开始时间
 * - end_time: 平滑结束时间
 * - last_calculation_time_: 平滑计算耗时
 *
 * @par 处理逻辑:
 * - 检查输入参数是否为空
 * - 记录平滑开始时间
 * - 调用smoothSlicedRoute函数对记忆路线切片路径进行平滑处理
 * - 检查平滑后的参考线是否为空，若为空则输出警告并返回失败
 * - 记录平滑结束时间并计算平滑耗时
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输入参数是否为空;
 * :记录平滑开始时间;
 * :调用smoothSlicedRoute函数进行平滑处理;
 * if (平滑后的参考线是否为空?) then (是)
 *   :输出警告信息;
 *   :返回失败;
 * else (否)
 *   :记录平滑结束时间并计算平滑耗时;
 *   :返回成功;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于对记忆路线切片路径进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的记忆路线切片路径有效，否则可能无法正确生成平滑参考线
 */
bool ReferenceLineProvider::smoothReferenceLines(std::shared_ptr<SlicedRoute> route,
                                                 std::vector<ReferenceLine>* smoothed_res) {
  CHECK_NOTNULL(route);
  CHECK_NOTNULL(smoothed_res);

  const auto start_time = std::chrono::system_clock::now();
  if (!smoothSlicedRoute(route, smoothed_res)) {
    ERT_PLOG_I << "[ReferenceLineProvider::smoothReferenceLines]: smoothSlicedRoute failed! id = " << route->id();
    return false;
  }
  if (smoothed_res->empty()) {
    ERT_PLOG_I << "[ReferenceLineProvider::smoothReferenceLines]: smoothed_res is empty";
    return false;
  }
  const auto end_time = std::chrono::system_clock::now();
  last_calculation_time_ =
      static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count());

  // ERT_PLOG_I <<"[ReferenceLineProvider::smoothReferenceLines]: reference_line: smooth_calculation_time: " <<
  // last_calculation_time_ ; ERT_PLOG_I <<"[ReferenceLineProvider::smoothReferenceLines]: smoothed_res->size(): " <<
  // smoothed_res->size() ;
  //  for (int i = 0; i < smoothed_res->size(); ++i) {
  //    ERT_PLOG_I << " id = " << smoothed_res->at(i).id()
  //              << " line_type = " << static_cast<int>(smoothed_res->at(i).line_type()) ;
  //    for (int j = 0; j < smoothed_res->at(i).reference_points().size(); ++j) {
  //      ERT_PLOG_I << "   x = " << smoothed_res->at(i).reference_points().at(j).x()
  //                << "   y = " << smoothed_res->at(i).reference_points().at(j).y()
  //                << "   offset = " << smoothed_res->at(i).reference_points().at(j).offset()
  //                << "   kappa = " << smoothed_res->at(i).reference_points().at(j).kappa()
  //                << "   dkappa = " << smoothed_res->at(i).reference_points().at(j).dkappa()
  //                << "   radius = " << 1 / smoothed_res->at(i).reference_points().at(j).kappa()
  //                ;
  //    }
  //  }

  return true;
}

/**
 * @brief 平滑感知车道线
 * @details 该函数用于对输入的感知车道线进行平滑处理，生成平滑后的参考线
 *
 * @param[in] lanes 输入的感知车道线，类型为std::vector<PerceptionLane>
 * @param[out] smoothed_res 平滑后的参考线，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - lanes: 输入的感知车道线，包含车道的原始信息
 * - smoothed_res: 平滑后的参考线，存储平滑后的路径信息
 * - start_time: 平滑开始时间
 * - end_time: 平滑结束时间
 * - last_calculation_time_: 平滑计算耗时
 *
 * @par 处理逻辑:
 * - 检查输出参数是否为空
 * - 记录平滑开始时间
 * - 调用smoothPerceptionLanes函数对感知车道线进行平滑处理
 * - 检查平滑后的参考线是否为空，若为空则输出警告并返回失败
 * - 记录平滑结束时间并计算平滑耗时
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输出参数是否为空;
 * :记录平滑开始时间;
 * :调用smoothPerceptionLanes函数进行平滑处理;
 * if (平滑后的参考线是否为空?) then (是)
 *   :输出警告信息;
 *   :返回失败;
 * else (否)
 *   :记录平滑结束时间并计算平滑耗时;
 *   :返回成功;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于对感知车道线进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的感知车道线有效，否则可能无法正确生成平滑参考线
 */
bool ReferenceLineProvider::smoothReferenceLines(const std::vector<PerceptionLane>& lanes,
                                                 std::vector<ReferenceLine>* smoothed_res) {
  CHECK_NOTNULL(smoothed_res);

  const auto start_time = std::chrono::system_clock::now();
  smoothPerceptionLanes(lanes, smoothed_res);
  if (smoothed_res->empty()) {
    ERT_PLOG_I << "[PerceptionLane][ReferenceLineProvider::smoothReferenceLines]: smoothed_res is empty";
    return false;
  }
  const auto end_time = std::chrono::system_clock::now();
  last_calculation_time_ =
      static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count());

  // ERT_PLOG_I <<"[ReferenceLineProvider::smoothReferenceLines]: reference_line: smooth_calculation_time: " <<
  // last_calculation_time_ ; ERT_PLOG_I <<"[ReferenceLineProvider::smoothReferenceLines]: smoothed_res->size(): " <<
  // smoothed_res->size() ;
  //  for (int i = 0; i < smoothed_res->size(); ++i) {
  //    ERT_PLOG_I << " id = " << smoothed_res->at(i).id()
  //              << " line_type = " << static_cast<int>(smoothed_res->at(i).line_type()) ;
  //    for (int j = 0; j < smoothed_res->at(i).reference_points().size(); ++j) {
  //      ERT_PLOG_I << "   x = " << smoothed_res->at(i).reference_points().at(j).x()
  //                << "   y = " << smoothed_res->at(i).reference_points().at(j).y()
  //                << "   offset = " << smoothed_res->at(i).reference_points().at(j).offset()
  //                << "   kappa = " << smoothed_res->at(i).reference_points().at(j).kappa()
  //                << "   dkappa = " << smoothed_res->at(i).reference_points().at(j).dkappa()
  //                << "   radius = " << 1 / smoothed_res->at(i).reference_points().at(j).kappa()
  //                ;
  //    }
  //  }

  return true;
}

/**
 * @brief 平滑历史路径
 * @details 该函数用于对输入的历史路径进行平滑处理，生成平滑后的参考线
 *
 * @param[in] path_history 输入的历史路径，类型为MainPathHistory
 * @param[out] smoothed_res 平滑后的参考线，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - path_history: 输入的历史路径，包含路径的原始信息
 * - smoothed_res: 平滑后的参考线，存储平滑后的路径信息
 * - start_time: 平滑开始时间
 * - end_time: 平滑结束时间
 * - last_calculation_time_: 平滑计算耗时
 *
 * @par 处理逻辑:
 * - 检查输出参数是否为空
 * - 记录平滑开始时间
 * - 调用smoothPathHistory函数对历史路径进行平滑处理
 * - 检查平滑后的参考线是否为空，若为空则输出警告并返回失败
 * - 记录平滑结束时间并计算平滑耗时
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输出参数是否为空;
 * :记录平滑开始时间;
 * :调用smoothPathHistory函数进行平滑处理;
 * if (平滑后的参考线是否为空?) then (是)
 *   :输出警告信息;
 *   :返回失败;
 * else (否)
 *   :记录平滑结束时间并计算平滑耗时;
 *   :返回成功;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于对历史路径进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的历史路径有效，否则可能无法正确生成平滑参考线
 */
bool ReferenceLineProvider::smoothReferenceLines(const MainPathHistory& path_history,
                                                 std::vector<ReferenceLine>* smoothed_res) {
  CHECK_NOTNULL(smoothed_res);
  const auto start_time = std::chrono::system_clock::now();
  smoothPathHistory(path_history, smoothed_res);
  if (smoothed_res->empty()) {
    ERT_PLOG_I << "[PathHistory][ReferenceLineProvider::smoothReferenceLines]: smoothed_res is empty";
    return false;
  }
  const auto end_time = std::chrono::system_clock::now();
  last_calculation_time_ =
      static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count());
  return true;
}

/**
 * @brief 平滑LocalRoute
 * @details 该函数用于对输入的LocalRoute进行平滑处理，生成平滑后的参考线
 *
 * @param[in] local_routes 输入的LocalRoute，类型为std::map<int, LocalRoute>
 * @param[out] smoothed_res 平滑后的参考线，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - local_routes: 输入的LocalRoute，包含路径的原始信息
 * - smoothed_res: 平滑后的参考线，存储平滑后的路径信息
 * - start_time: 平滑开始时间
 * - end_time: 平滑结束时间
 * - last_calculation_time_: 平滑计算耗时
 *
 * @par 处理逻辑:
 * - 检查输出参数是否为空
 * - 记录平滑开始时间
 * - 调用smoothLocalRoutes函数对LocalRoute进行平滑处理
 * - 检查平滑后的参考线是否为空，若为空则输出警告并返回失败
 * - 记录平滑结束时间并计算平滑耗时
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输出参数是否为空;
 * :记录平滑开始时间;
 * :调用smoothLocalRoutes函数进行平滑处理;
 * if (平滑后的参考线是否为空?) then (是)
 *   :输出警告信息;
 *   :返回失败;
 * else (否)
 *   :记录平滑结束时间并计算平滑耗时;
 *   :返回成功;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于对LocalRoute进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的LocalRoute有效，否则可能无法正确生成平滑参考线
 */
bool ReferenceLineProvider::smoothReferenceLines(const std::map<int, LocalRoute>& local_routes,
                                                 std::vector<ReferenceLine>* smoothed_res) {
  CHECK_NOTNULL(smoothed_res);

  const auto start_time = std::chrono::steady_clock::now();
  smoothLocalRoutes(local_routes, smoothed_res);
  if (smoothed_res->empty()) {
    ERT_PLOG_I << "[LocalRoute][ReferenceLineProvider::smoothReferenceLines]: smoothed_res is empty";
    return false;
  }
  const auto end_time = std::chrono::steady_clock::now();
  last_calculation_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  // ERT_PLOG_I <<"[LocalRoute]:[ReferenceLineProvider::smoothReferenceLines]: reference_line: smooth_calculation_time:
  // " << last_calculation_time_ ; ERT_PLOG_I <<"[ReferenceLineProvider::smoothReferenceLines]: smoothed_res->size(): "
  // << smoothed_res->size() ;
  //  for (int i = 0; i < smoothed_res->size(); ++i) {
  //    ERT_PLOG_I << " id = " << smoothed_res->at(i).id()
  //              << " line_type = " << static_cast<int>(smoothed_res->at(i).line_type()) ;
  //    for (int j = 0; j < smoothed_res->at(i).reference_points().size(); ++j) {
  //      ERT_PLOG_I << "   x = " << smoothed_res->at(i).reference_points().at(j).x()
  //                << "   y = " << smoothed_res->at(i).reference_points().at(j).y()
  //                << "   offset = " << smoothed_res->at(i).reference_points().at(j).offset()
  //                << "   kappa = " << smoothed_res->at(i).reference_points().at(j).kappa()
  //                << "   dkappa = " << smoothed_res->at(i).reference_points().at(j).dkappa()
  //                << "   radius = " << 1 / smoothed_res->at(i).reference_points().at(j).kappa()
  //                ;
  //    }
  //  }

  return true;
}

/**
 * @brief 添加原始参考线
 * @details 该函数用于将输入的记忆路线切片路径转换为原始参考线，并将其添加到输出容器中
 *
 * @param[in] route 输入的记忆路线切片路径，类型为std::shared_ptr<SlicedRoute>
 * @param[out] raw_res 输出的原始参考线容器，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示操作是否成功
 *
 * @par 关键变量说明:
 * - route: 输入的记忆路线切片路径，包含路径的原始信息
 * - raw_res: 输出的原始参考线容器，存储转换后的参考线
 * - raw_ref_line: 原始参考线对象，用于存储未平滑的路径信息
 * - short_refline_parameter: 参考线长度阈值，用于判断参考线是否过短
 *
 * @par 处理逻辑:
 * - 检查输入参数是否为空，若为空则输出警告并返回失败
 * - 创建原始参考线对象
 * - 判断原始参考线长度是否过短，若过短则输出警告并返回失败
 * - 将原始参考线添加到输出容器中
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输入参数是否为空;
 * if (输入参数为空?) then (是)
 *   :输出警告信息;
 *   :返回失败;
 * else (否)
 *   :创建原始参考线对象;
 *   if (原始参考线长度是否过短?) then (是)
 *     :输出警告信息;
 *     :返回失败;
 *   else (否)
 *     :将原始参考线添加到输出容器中;
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于将记忆路线切片路径转换为原始参考线，适用于需要生成未平滑参考线的场景
 *
 * @warning 需确保输入的记忆路线切片路径有效，否则可能无法正确生成原始参考线
 */
bool ReferenceLineProvider::addRawReferenceLines(const std::shared_ptr<SlicedRoute>& route,
                                                 std::vector<ReferenceLine>* raw_res) {
  if (route == nullptr) {
    ERT_PLOG_I << "[ReferenceLineProvider::addRawReferenceLines]: ReferenceLineProvider: route == nullptr";
    return false;
  }
  ReferenceLine raw_ref_line(route.get());
  raw_ref_line.set_smooth_type(ReferenceLine::SmoothType::RAW);
  const double short_refline_parameter = 5.0;
  if (raw_ref_line.length() < short_refline_parameter) {
    ERT_PLOG_I << "[ReferenceLineProvider::addRawReferenceLines]: raw_ref_line is too short! id = " << raw_ref_line.id()
               << "  num_points = " << raw_ref_line.num_points()
               << "  num_segments = " << raw_ref_line.num_line_segments() << "  length = " << raw_ref_line.length();
    return false;
  }

  raw_res->emplace_back(std::move(raw_ref_line));

  return true;
}

/**
 * @brief 添加原始参考线
 * @details 该函数用于将输入的感知车道线转换为原始参考线，并将其添加到输出容器中
 *
 * @param[in] lanes 输入的感知车道线，类型为std::vector<PerceptionLane>
 * @param[out] raw_res 输出的原始参考线容器，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示操作是否成功
 *
 * @par 关键变量说明:
 * - lanes: 输入的感知车道线，包含车道的原始信息
 * - raw_res: 输出的原始参考线容器，存储转换后的参考线
 * - raw_ref_line: 原始参考线对象，用于存储未平滑的路径信息
 * - short_refline_parameter: 参考线长度阈值，用于判断参考线是否过短
 *
 * @par 处理逻辑:
 * - 遍历输入的感知车道线
 *   - 创建原始参考线对象
 *   - 判断原始参考线长度是否过短，若过短则输出警告并跳过当前车道
 *   - 将原始参考线添加到输出容器中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历输入的感知车道线;
 *   :创建原始参考线对象;
 *   if (原始参考线长度是否过短?) then (是)
 *     :输出警告信息;
 *     :跳过当前车道;
 *   else (否)
 *     :将原始参考线添加到输出容器中;
 *   endif
 * :返回成功;
 * stop
 * @enduml
 *
 * @note 该函数用于将感知车道线转换为原始参考线，适用于需要生成未平滑参考线的场景
 *
 * @warning 需确保输入的感知车道线有效，否则可能无法正确生成原始参考线
 */
bool ReferenceLineProvider::addRawReferenceLines(const std::vector<PerceptionLane>& lanes,
                                                 std::vector<ReferenceLine>* raw_res) {
  for (const auto& lane : lanes) {
    ReferenceLine raw_ref_line(&lane);
    raw_ref_line.set_smooth_type(ReferenceLine::SmoothType::RAW);
    const double short_refline_parameter = 5.0;
    if (raw_ref_line.length() < short_refline_parameter) {
      ERT_PLOG_I << "[ReferenceLineProvider::addRawReferenceLines]: raw_ref_line is too short! id = "
                 << raw_ref_line.id() << "  num_points = " << raw_ref_line.num_points()
                 << "  num_segments = " << raw_ref_line.num_line_segments();
      continue;
    }

    raw_res->emplace_back(std::move(raw_ref_line));
  }

  return true;
}

/**
 * @brief 添加原始参考线
 * @details 该函数用于将输入的LocalRoute转换为原始参考线，并将其添加到输出容器中
 *
 * @param[in] local_routes 输入的LocalRoute，类型为std::map<int, LocalRoute>
 * @param[out] raw_res 输出的原始参考线容器，类型为std::vector<ReferenceLine>*
 * @return 返回布尔值，表示操作是否成功
 *
 * @par 关键变量说明:
 * - local_routes: 输入的LocalRoute，包含路径的原始信息
 * - raw_res: 输出的原始参考线容器，存储转换后的参考线
 * - raw_ref_line: 原始参考线对象，用于存储未平滑的路径信息
 * - short_refline_parameter: 参考线长度阈值，用于判断参考线是否过短
 *
 * @par 处理逻辑:
 * - 遍历输入的LocalRoute
 *   - 创建原始参考线对象
 *   - 判断原始参考线长度是否过短，若过短则输出警告并跳过当前路径
 *   - 将原始参考线添加到输出容器中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历输入的LocalRoute;
 *   :创建原始参考线对象;
 *   if (原始参考线长度是否过短?) then (是)
 *     :输出警告信息;
 *     :跳过当前路径;
 *   else (否)
 *     :将原始参考线添加到输出容器中;
 *   endif
 * :返回成功;
 * stop
 * @enduml
 *
 * @note 该函数用于将LocalRoute转换为原始参考线，适用于需要生成未平滑参考线的场景
 *
 * @warning 需确保输入的LocalRoute有效，否则可能无法正确生成原始参考线
 */
bool ReferenceLineProvider::addRawReferenceLines(const std::map<int, LocalRoute>& local_routes,
                                                 std::vector<ReferenceLine>* raw_res) {
  for (const auto local_route : local_routes) {
    ReferenceLine raw_ref_line(&local_route.second);
    raw_ref_line.set_smooth_type(ReferenceLine::SmoothType::RAW);
    const double short_refline_parameter = 5.0;
    if (raw_ref_line.length() < short_refline_parameter) {
      ERT_PLOG_I << "[ReferenceLineProvider::addRawReferenceLines]: raw_ref_line is too short! id = "
                 << raw_ref_line.id() << "  num_points = " << raw_ref_line.num_points()
                 << "  num_segments = " << raw_ref_line.num_line_segments() << "  length = " << raw_ref_line.length();
      continue;
    }

    raw_res->emplace_back(std::move(raw_ref_line));
  }

  return true;
}

/**
 * @brief 使用OCP平滑器平滑参考线
 * @details 该函数用于对输入的原始参考线进行平滑处理，生成平滑后的参考线
 *
 * @param[in] raw_reference_line 输入的原始参考线，类型为ReferenceLine
 * @param[out] reference_line 平滑后的参考线，类型为ReferenceLine*
 * @param[in] is_async 是否异步执行平滑操作，类型为bool
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - raw_reference_line: 输入的原始参考线，包含未平滑的路径信息
 * - reference_line: 平滑后的参考线，存储平滑后的路径信息
 * - is_async: 是否异步执行平滑操作，true表示异步，false表示同步
 * - smoother: OCP参考线平滑器对象，用于执行平滑操作
 *
 * @par 处理逻辑:
 * - 创建OCP参考线平滑器对象
 * - 调用平滑器的平滑函数对原始参考线进行平滑处理
 * - 返回平滑操作的结果
 *
 * @par 流程图:
 * @startuml
 * start
 * :创建OCP参考线平滑器对象;
 * :调用平滑器的平滑函数进行平滑处理;
 * if (平滑操作是否成功?) then (是)
 *   :返回true;
 * else (否)
 *   :返回false;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于使用OCP平滑器对参考线进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的原始参考线有效，否则可能无法正确生成平滑参考线
 */
bool ReferenceLineProvider::smoothOcpReferenceLine(const ReferenceLine& raw_reference_line,
                                                   ReferenceLine* reference_line, bool is_async) {
  std::unique_ptr<OcpReferenceLineSmoother> smoother = std::make_unique<OcpReferenceLineSmoother>(smoother_config_);
  if (!smoother->smooth(raw_reference_line, reference_line, is_async)) return false;
  return true;
}

/**
 * @brief 平滑参考线
 * @details 该函数用于对输入的原始参考线进行平滑处理，生成平滑后的参考线
 *
 * @param[in] raw_reference_line 输入的原始参考线，类型为ReferenceLine
 * @param[out] reference_line 平滑后的参考线，类型为ReferenceLine*
 * @param[in] is_async 是否异步执行平滑操作，类型为bool
 * @return 返回布尔值，表示平滑操作是否成功
 *
 * @par 关键变量说明:
 * - raw_reference_line: 输入的原始参考线，包含未平滑的路径信息
 * - reference_line: 平滑后的参考线，存储平滑后的路径信息
 * - is_async: 是否异步执行平滑操作，true表示异步，false表示同步
 * - enable_smooth_ref_line_: 是否启用平滑操作的标志
 *
 * @par 处理逻辑:
 * - 检查是否启用平滑操作，若未启用则直接返回原始参考线
 * - 调用OCP平滑器对原始参考线进行平滑处理
 *   - 若平滑成功，则返回true
 *   - 若平滑失败，则输出警告信息并返回原始参考线
 *
 * @par 流程图:
 * @startuml
 * start
 * if (是否启用平滑操作?) then (否)
 *   :直接返回原始参考线;
 *   :返回true;
 * else (是)
 *   :调用OCP平滑器进行平滑处理;
 *   if (平滑操作是否成功?) then (是)
 *     :返回true;
 *   else (否)
 *     :输出警告信息;
 *     :返回原始参考线;
 *     :返回true;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于对参考线进行平滑处理，适用于需要生成平滑参考线的场景
 *
 * @warning 需确保输入的原始参考线有效，否则可能无法正确生成平滑参考线
 */
bool ReferenceLineProvider::smoothReferenceLine(const ReferenceLine& raw_reference_line, ReferenceLine* reference_line,
                                                bool is_async) {
  if (!enable_smooth_ref_line_) {
    *reference_line = raw_reference_line;
    reference_line->computeKappaAndDkappa();
    ERT_PLOG_I << "[ReferenceLineProvider::smoothReferenceLine]: disable smooth, id = " << reference_line->id();
    return true;
  }

  if (smoothOcpReferenceLine(raw_reference_line, reference_line, is_async)) {
    // ERT_PLOG_I << "[ReferenceLineProvider::smoothReferenceLine]: Success to smooth reference line with ocp solver, id
    // = " << reference_line->id() ;
    return true;
  }

  // 优化失败则输出原始参考线(TODO: 兜底平滑算法)，后续会进行简单的曲率计算等操作
  *reference_line = raw_reference_line;
  reference_line->computeKappaAndDkappa();
  ERT_PLOG_I << "[LocalRoute][ReferenceLineProvider::smoothReferenceLine]: Failed to smooth reference line with ocp "
                "solver, id = "
             << reference_line->id() << "  num_points = " << reference_line->num_points()
             << "  num_segments = " << reference_line->num_line_segments() << "  length = " << reference_line->length()
             << "  global_start_s = " << reference_line->global_start_s();
  return true;
}

}  // namespace gpal::pnc::planning
