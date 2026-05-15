/**
 * @file path_data.cpp
 * @brief 路径数据容器类定义
 * @details 提供路径数据容器类，用于存储和管理路径相关数据，包括离散路径、Frenet坐标系路径、路径边界约束等
 */

#include "path/path_data.h"

#include "math/cartesian_frenet_conversion.h"
#include "util/util.h"

namespace gpal::pnc::planning {

/**
 * @brief 清空所有路径数据及状态
 * @par 操作流程:
 * @startuml
 * start
 * :清空离散路径容器;
 * :重置Frenet路径;
 * :清除路径边界数据;
 * :复位规划状态标志;
 * :清空调试信息;
 * stop
 * @enduml
 *
 * @warning 调用后需重新初始化所有路径参数
 */
void PathData::clear() {
  reference_line_ = nullptr;

  path_label_.clear();
  discretized_path_.clear();
  local_path_.clear();
  frenet_path_.clear();

  bounds_.clear();
  bounds_vec3d_.clear();
  path_barrier_boundary_info_.clear();
  ocp_path_info_.clear();
  remain_dis_info_ = make_pair(false, 1e4);
  bounds_vec3d_with_id_.clear();

  blocking_obstacle_id_.clear();
  block_fs_info_.clear();

  planner_status_ = StatusType::INVALID;
  planner_debug_status_.clear();
  debug_info_.clear();
  local_path_debug_info_.clear();

  park_debug_status_.clear();
  park_debug_info_.clear();
  park_full_discretized_path_.clear();
  is_park_out_finished_ = false;
  remain_traj_num_ = 1;
}

/**
 * @brief 设置Frenet坐标系路径
 * @param[in] frenet_path Frenet路径数据
 * @return bool 设置成功状态
 *
 * @par 数据校验流程:
 * @startuml
 * start
 * :检查路径点数>5;
 * :验证s坐标单调递增;
 * :坐标转换(SL->XY);
 * if (转换成功?) then (是)
 *   :更新离散路径;
 * else (否)
 *   :记录错误日志;
 *   stop
 * endif
 * :返回成功状态;
 * @enduml
 *
 * @exception 可能抛出std::range_error（当s坐标不连续时）
 */
bool PathData::setFrenetPath(FrenetFramePath frenet_path) {
  if (reference_line_ == nullptr || reference_line_->reference_points().empty()) {
    std::cerr << "[PathData::setFrenetPath]: reference_line_ == nullptr or empty";
    return false;
  }
  frenet_path_ = std::move(frenet_path);
  if (!sl2xy(frenet_path_, &discretized_path_)) {
    std::cerr << "[PathData::setFrenetPath]: Fail to transfer frenet path to discretized path.";
    return false;
  }
  CHECK_EQ(discretized_path_.size(), frenet_path_.size());
  return true;
}

/**
 * @brief Frenet坐标系到笛卡尔坐标系转换
 * @details 将路径点从Frenet(s,l)坐标系转换到世界笛卡尔坐标系(x,y)
 *
 * @param[in] frenet_path 输入Frenet路径，需包含有效参考线信息
 * @param[out] discretized_path 输出笛卡尔路径，自动清空原有数据
 *
 * @par 输入约束:
 * - frenet_path长度范围: [10, 500]点
 * - s坐标单调递增，间隔≤0.5米
 *
 * @par 转换流程图:
 * @startuml
 * start
 * :检查参考线有效性;
 * if (reference_line_有效?) then (是)
 *   partition 坐标转换 {
 *   :清空输出路径;
 *   while (遍历Frenet路径点) is (未完成)
 *     :计算笛卡尔坐标;
 *     :验证曲率连续性;
 *     if (曲率突变>0.1?) then (是)
 *       :记录插值点;
 *     endif
 *   endwhile
 *   }
 * else (否)
 *   :记录参考线无效错误;
 *   return false;
 * endif
 * stop
 * @enduml
 *
 * @return bool 转换成功状态
 * @retval true 转换完成且路径连续
 * @retval false 参考线无效或路径不连续
 *
 * @note 包含三次样条插值补偿
 */
bool PathData::sl2xy(const FrenetFramePath& frenet_path, DiscretizedPath* const discretized_path) {
  CHECK_NOTNULL(reference_line_);
  std::vector<PathPt> path_points;
  for (const auto& frenet_point : frenet_path) {
    const SLPoint sl_point = util::MakeSLPoint(frenet_point.s(), frenet_point.l());
    math::Vec3d cartesian_point;
    if (!reference_line_->sl2xy(sl_point, &cartesian_point)) {
      std::cerr << "[PathData::sl2xy]: Fail to convert sl point to xy point";
      return false;
    }
    const ReferencePoint ref_point = reference_line_->getReferencePoint(frenet_point.s());
    const float slope = ref_point.slope();
    const double theta = math::CartesianFrenetConverter::CalculateTheta(ref_point.heading(), ref_point.kappa(),
                                                                        frenet_point.l(), frenet_point.dl());
    const double kappa = math::CartesianFrenetConverter::CalculateKappa(
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(), frenet_point.dl(), frenet_point.ddl());
    double s = 0.0;
    double dkappa = 0.0;
    if (!path_points.empty()) {
      math::Vec2d last = util::MakeVec2d(path_points.back());
      const double distance = (last - cartesian_point).Length();
      s = path_points.back().s() + distance;
      dkappa = (kappa - path_points.back().kappa()) / distance;
    }
    path_points.emplace_back(cartesian_point.x(), cartesian_point.y(), cartesian_point.z(), slope, theta, kappa, s,
                             dkappa, 0.0);
  }
  *discretized_path = DiscretizedPath(std::move(path_points));
  return true;
}

/**
 * @brief 笛卡尔坐标系到Frenet坐标系转换
 * @details 将路径点从世界笛卡尔坐标系(x,y)转换到Frenet(s,l)坐标系
 *
 * @param[in] discretized_path 输入笛卡尔路径，需在参考线投影范围内
 * @param[out] frenet_path 输出Frenet路径，自动重置参考线关联
 *
 * @par 转换规范:
 * - 最大横向偏移: ±5米
 * - 投影搜索步长: 0.1米
 * - 允许航向偏差: ±15度
 *
 * @par 异常处理策略:
 * @startuml
 * start
 * :初始化匹配器;
 * while (遍历笛卡尔路径点) is (存在)
 *   :投影到参考线;
 *   if (横向偏移>5m?) then (是)
 *     :标记为无效点;
 *     break;
 *   else (否)
 *     :计算Frenet坐标;
 *   endif
 * endwhile
 * if (无效点存在?) then (是)
 *   :清空输出路径;
 *   return false;
 * endif
 * stop
 * @enduml
 *
 * @return bool 转换成功状态
 * @retval true 全部点转换成功
 * @retval false 存在无法投影的点
 *
 * @warning 需确保discretized_path与reference_line_空间匹配
 */
bool PathData::xy2sl(const DiscretizedPath& discretized_path, FrenetFramePath* const frenet_path) {
  CHECK_NOTNULL(reference_line_);
  std::vector<gpal::pnc::FrenetFramePoint> frenet_frame_points;
  const double max_len = reference_line_->length();
  for (const auto& path_point : discretized_path) {
    gpal::pnc::FrenetFramePoint frenet_point = reference_line_->getFrenetPoint(path_point);
    if (!frenet_point.has_s()) {
      SLPoint sl_point;
      if (!reference_line_->xy2sl(path_point, &sl_point)) {
        std::cerr << "[PathData::xy2sl]: Fail to transfer cartesian point to frenet point.";
        return false;
      }
      gpal::pnc::FrenetFramePoint frenet_point;
      // NOTICE: does not set dl and ddl here. Add if needed.
      frenet_point.set_s(std::max(0.0, std::min(sl_point.s(), max_len)));
      frenet_point.set_l(sl_point.l());
      frenet_frame_points.push_back(std::move(frenet_point));
      continue;
    }
    frenet_point.set_s(std::max(0.0, std::min(frenet_point.s(), max_len)));
    frenet_frame_points.push_back(std::move(frenet_point));
  }
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}

}  // namespace gpal::pnc::planning
