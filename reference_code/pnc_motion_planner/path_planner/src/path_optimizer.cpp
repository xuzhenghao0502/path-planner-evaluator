/**
 * @file path_optimizer.cpp
 * @brief 路径优化器实现文件
 * @details 该文件实现了路径优化器的核心功能，包括路径优化、范围判断等。路径优化器用于在路径规划过程中对路径进行平滑和优化，确保路径的连续性和可行性。
 */

#include "path_planner/path_optimizer.h"
#include "basic_algorithm_lib/geometry_calculation.h"
#include "base/singleton.h"
#include "config_manager/config_manager.h"
#include "math/math_utils.h"

namespace gpal::pnc::planning {

/**
 * @brief 初始化路径优化器
 * @details 该函数用于初始化路径优化器，主要完成车辆配置和车辆参数的加载。通过单例模式获取配置管理器，并从中加载车辆配置和车辆参数信息。
 * 
 * @par 输入参数说明:
 * - 无显式输入参数
 * 
 * @par 关键变量说明:
 * - config_manager (ConfigManager*): 配置管理器的单例实例
 * - vehicle_config_ (std::shared_ptr<VehicleConfig>): 车辆配置信息的智能指针
 * - vehicle_param_ (std::shared_ptr<VehicleParam>): 车辆参数信息的智能指针
 * 
 * @par 初始化流程:
 * 1. 获取配置管理器的单例实例
 * 2. 从配置管理器中加载车辆配置信息
 * 3. 从配置管理器中加载车辆参数信息
 * 4. 返回初始化成功状态
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取配置管理器单例实例;
 * :加载车辆配置信息;
 * :加载车辆参数信息;
 * :返回初始化成功状态;
 * stop
 * @enduml
 * 
 * @return bool 初始化是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在路径优化器启动时调用，确保车辆配置和参数信息正确加载
 * 
 * @warning 需确保配置管理器已正确初始化，且包含有效的车辆配置和参数信息
 */
bool PathOptimizer::init() {
  auto config_manager = Singleton<ConfigManager>::get_instance();
  vehicle_config_ = std::make_shared<VehicleConfig>(config_manager->vehicle_config());
  vehicle_param_ = std::make_shared<VehicleParam>(config_manager->vehicle_config().vehicle_param());
  return true;
}

/**
 * @brief 优化路径
 * @details 该函数用于根据当前起点优化路径数据。通过找到路径中与起点最近的点，并检查航向角误差，确保路径的连续性和平滑性。如果航向角误差过大，则清空路径数据；否则，从最近点开始生成新的路径点。
 * 
 * @param[in] start_pt 当前起点，包含起点的位置和航向信息
 * @param[in,out] path_data 路径数据，用于存储优化后的路径点
 * 
 * @par 输入参数说明:
 * - start_pt: 必须包含有效的位置和航向信息
 * - path_data: 必须包含有效的路径数据
 * 
 * @par 关键变量说明:
 * - min_dist (double): 起点与路径的最近距离，初始值为最大值
 * - fpt (PathPt): 路径中与起点最近的点
 * - max_heading_error (double): 最大允许的航向角误差，默认值为π/4
 * - theta_diff_abs (double): 起点与最近点的航向角差值
 * - refine_path_points (std::vector<PathPt>): 优化后的路径点集合
 * - curr_length (double): 当前路径长度，用于生成新的路径点
 * - curr_s (double): 当前s值，用于定位路径点
 * 
 * @par 优化流程:
 * 1. 设置路径标签为"prev"
 * 2. 找到路径中与起点最近的点
 * 3. 计算起点与最近点的航向角误差
 * 4. 如果航向角误差超过最大允许值，则清空路径数据并返回
 * 5. 从最近点开始，沿路径生成新的路径点
 * 6. 将优化后的路径点存储到路径数据中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :设置路径标签为"prev";
 * :找到路径中与起点最近的点;
 * :计算航向角误差;
 * if (航向角误差是否超过最大允许值?) then (是)
 *   :清空路径数据;
 *   :返回;
 * endif
 * :从最近点开始生成新的路径点;
 * :将优化后的路径点存储到路径数据中;
 * stop
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径的连续性和平滑性
 * 
 * @warning 需确保输入参数有效，特别是start_pt和path_data
 */
void PathOptimizer::refinePath(const TrajectoryPt& start_pt, PathData* const path_data) {
  path_data->setPathLabel("prev");
  double min_dist = std::numeric_limits<double>::max();
  auto fpt = path_data->discretized_path().getNearestPoint(start_pt.path_pt(), min_dist);

  constexpr double max_heading_error = M_PI / 4.0;
  double theta_diff_abs = std::abs(math::NormalizeAngle(fpt.theta() - start_pt.path_pt().theta()));
  if (theta_diff_abs >= max_heading_error) {
    path_data->mutableDiscretizedPath()->clear();
    ERT_PLOG_W << "[PathOptimizer::refinePath]: heading change too much, theta_diff_abs = " << theta_diff_abs;
    return;
  }

  std::vector<PathPt> refine_path_points(1, fpt);
  refine_path_points.front().set_s(0.0);
  for (double curr_length = 1.0; curr_length < 200.0; curr_length += 1.0) {
    double curr_s = fpt.s() + curr_length;
    if (curr_s > path_data->discretized_path().back().s()) {
      refine_path_points.emplace_back(path_data->discretized_path().back());
      refine_path_points.back().set_s(path_data->discretized_path().back().s() - fpt.s());
      break;
    }
    refine_path_points.emplace_back(path_data->discretized_path().evaluate(curr_s));
    refine_path_points.back().set_s(curr_length);
  }
  *path_data->mutableDiscretizedPath() = DiscretizedPath(refine_path_points);
}

/**
 * @brief 判断给定s值是否在指定范围内
 * @details 该函数用于判断给定的s值是否在指定的范围内。通过遍历范围列表，检查s值是否落在某个范围内（考虑容差）。
 * 
 * @param[in] ranges 范围列表，包含多个范围对，每个对表示范围的起始和结束s值
 * @param[in] s 需要判断的s值
 * @param[in] s_tolerance s值的容差，用于扩展范围的边界
 * 
 * @par 输入参数说明:
 * - ranges: 范围列表，每个元素为一个std::pair<float, float>，表示范围的起始和结束s值
 * - s: 需要判断的s值，类型为float
 * - s_tolerance: s值的容差，类型为float，默认值为0
 * 
 * @par 关键变量说明:
 * - iter (std::vector<std::pair<float, float>>::const_iterator): 范围列表的迭代器，用于遍历范围
 * 
 * @par 判断条件:
 * - s值是否在某个范围内，考虑容差：range.first - s_tolerance <= s && s < range.second
 * 
 * @par 流程图:
 * @startuml
 * start
 * if (范围列表是否为空?) then (是)
 *   :返回false;
 * endif
 * :遍历范围列表;
 * if (s值是否在当前范围内?) then (是)
 *   :返回true;
 * endif
 * :返回false;
 * stop
 * @enduml
 * 
 * @return bool s值是否在范围内，true表示在范围内，false表示不在范围内
 * 
 * @note 该函数主要用于判断s值是否在指定的范围内，考虑容差
 * 
 * @warning 需确保输入参数有效，特别是ranges参数必须包含有效的范围对
 */
bool PathOptimizer::isInRange(const std::vector<std::pair<float, float>>& ranges, const float& s, float s_tolerance) {
  if (ranges.empty()) {
    return false;
  }
  auto iter = std::find_if(ranges.begin(), ranges.end(), [s, s_tolerance](const std::pair<float, float>& range) {
    return range.first - s_tolerance <= s && s < range.second;
  });
  return iter != ranges.end();
}

/**
 * @brief 判断给定s值是否在指定范围内
 * @details 该函数用于判断给定的s值是否在指定的范围内。通过遍历范围列表，检查s值是否落在某个范围内（考虑容差）。与普通范围判断不同，该函数支持根据标签（tag）进行范围匹配。
 * 
 * @param[in] special_tags_range 范围列表，包含多个范围元组，每个元组表示标签、起始s值和结束s值
 * @param[in] tag 需要匹配的标签
 * @param[in] s 需要判断的s值
 * @param[in] s_tolerance s值的容差，用于扩展范围的边界
 * 
 * @par 输入参数说明:
 * - special_tags_range: 范围列表，每个元素为一个std::tuple<std::string, float, float>，分别表示标签、起始s值和结束s值
 * - tag: 需要匹配的标签，类型为std::string
 * - s: 需要判断的s值，类型为float
 * - s_tolerance: s值的容差，类型为float，默认值为0
 * 
 * @par 关键变量说明:
 * - tag_ (std::string): 范围元组中的标签
 * - start_s (float): 范围元组中的起始s值
 * - end_s (float): 范围元组中的结束s值
 * 
 * @par 判断条件:
 * - 标签是否匹配：tag_ == tag
 * - s值是否在某个范围内，考虑容差：start_s - s_tolerance <= s && end_s > s
 * 
 * @par 流程图:
 * @startuml
 * start
 * if (范围列表是否为空?) then (是)
 *   :返回false;
 * endif
 * :遍历范围列表;
 * if (标签是否匹配且s值是否在当前范围内?) then (是)
 *   :返回true;
 * endif
 * :返回false;
 * stop
 * @enduml
 * 
 * @return bool s值是否在指定标签的范围内，true表示在范围内，false表示不在范围内
 * 
 * @note 该函数主要用于根据标签判断s值是否在指定的范围内，考虑容差
 * 
 * @warning 需确保输入参数有效，特别是special_tags_range参数必须包含有效的范围元组
 */
bool PathOptimizer::isInRange(const std::vector<std::tuple<std::string, float, float>>& special_tags_range, const std::string tag,
               const float& s, float s_tolerance) {
  if (special_tags_range.empty()) {
    return false;
  }
  for (const auto& [tag_, start_s, end_s] : special_tags_range) {
    if (tag_ == tag && start_s - s_tolerance <= s && end_s > s) {
      return true;
    }
  }
  return false;
}

/**
 * @brief 判断给定s值是否在忽略范围内
 * @details 该函数用于判断给定的s值是否在指定的忽略范围内。通过遍历忽略范围列表，检查s值是否落在某个忽略范围内。
 * 
 * @param[in] ranges 忽略范围列表，包含多个忽略范围信息，每个元素包含起始和结束s值
 * @param[in] s 需要判断的s值
 * 
 * @par 输入参数说明:
 * - ranges: 忽略范围列表，每个元素为一个IgnoreRangeInfo结构体，包含起始和结束s值
 * - s: 需要判断的s值，类型为double
 * 
 * @par 关键变量说明:
 * - iter (std::vector<IgnoreRangeInfo>::const_iterator): 忽略范围列表的迭代器，用于遍历范围
 * 
 * @par 判断条件:
 * - s值是否在某个忽略范围内：range.start_s <= s && s < range.end_s
 * 
 * @par 流程图:
 * @startuml
 * start
 * if (忽略范围列表是否为空?) then (是)
 *   :返回false;
 * endif
 * :遍历忽略范围列表;
 * if (s值是否在当前忽略范围内?) then (是)
 *   :返回true;
 * endif
 * :返回false;
 * stop
 * @enduml
 * 
 * @return bool s值是否在忽略范围内，true表示在忽略范围内，false表示不在忽略范围内
 * 
 * @note 该函数主要用于判断s值是否在指定的忽略范围内，通常用于跳过某些特定区域的路径规划
 * 
 * @warning 需确保输入参数有效，特别是ranges参数必须包含有效的忽略范围信息
 */
bool PathOptimizer::isInIgnoreRange(const std::vector<IgnoreRangeInfo>& ranges, const double& s) {
  if (ranges.empty()) {
    return false;
  }
  auto iter = std::find_if(ranges.begin(), ranges.end(),
                           [s](const IgnoreRangeInfo& range) { return range.start_s <= s && s < range.end_s; });
  return iter != ranges.end();
}

}
