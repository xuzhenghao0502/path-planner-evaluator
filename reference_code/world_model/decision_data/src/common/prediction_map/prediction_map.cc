/**
 * @file prediction_map.cc
 * @brief 此文件实现了 `PredictionMap` 类，用于管理预测模块的地图数据。
 * @details 包含地图数据的初始化、导航数据插入、空间查询等功能，为预测模块提供地图支持。
 */

#include "decision_data/common/prediction_map/prediction_map.h"

#include "base/log.h"
#include "base/singleton.h"
#include "config_manager/config_manager.h"

namespace gpal {
namespace pnc {
namespace prediction {

PredictionMap::PredictionMap() {
  auto* config_manager = planning::Singleton<planning::ConfigManager>::get_instance();
  config_ = config_manager->getConfig<PredictionConfig>("PredictionConfig").container_config().prediction_map_config();

  lane_point_tree_ =
      std::unique_ptr<KdTreeNanoFLANN>(new KdTreeNanoFLANN(2, lane_point_cloud_, {config_.max_leaf_size()}));

  lane_boudary_search_radius_ = 4.0;
  // road_boudary_search_radius_ = 20.0;
  // ref_search_radius_ = 15.0;

  kdtree_params_.max_depth = -1;
  kdtree_params_.max_leaf_size = 10;
  kdtree_params_.max_leaf_dimension = 5.0;
  kdtree_params_.enable_calc_by_heading = true;
}

/**
 * @brief 插入导航数据
 * @details 解析导航消息并更新内部数据结构，依次调用
 * `BuildLaneTable`、`BuildJunctionTable` 和 `BuildSearchTree` 方法。
 *
 * @param[in] navigation 导航数据消息
 *
 * @par 输入参数说明:
 * - road_instance: 包含导航信息的 `planning::road_instance::EnvRoadInstance`
 * 对象，必须包含有效的车道和路口信息。
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用 BuildLaneTable;
 * :调用 BuildJunctionTable;
 * :调用 BuildSearchTree;
 * stop
 * @enduml
 */
void PredictionMap::Insert(const EnvRoadInstance& road_instance) {
  if (!config_.enable()) {
    ERT_LOG_D("Prediction map is disabled.");
    return;
  }

  Clear();

  BuildLaneTable(road_instance);
  BuildJunctionTable(road_instance);
}

/**
 * @brief 清空所有地图数据
 * @details 重置所有内部数据结构和搜索树，清空
 * `lane_table_`、`junction_table_`，重置 `lane_point_tree_` 和
 * `num_lane_points_`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :清空 lane_table_;
 * :清空 junction_table_;
 * :重置 lane_point_tree_;
 * :重置计数器 num_lane_points_;
 * stop
 * @enduml
 */
void PredictionMap::Clear() {
  lane_table_.clear();

  num_lane_points_ = 0;
  lane_point_ids_.clear();
  lane_point_cloud_.pts.clear();
  lane_point_start_indices_.clear();

  junction_table_.clear();
  junction_feature_table_.clear();
}

/**
 * @brief 检查地图是否就绪
 * @details 验证地图数据是否已加载并可查询，判断条件为 `lane_point_tree_`
 * 不为空且 `num_lane_points_ > 0`。
 *
 * @return bool 就绪状态，就绪返回 `true`，未就绪返回 `false`
 *
 * @par 判断条件:
 * - lane_point_tree_ 不为空
 * - num_lane_points_ > 0
 */
bool PredictionMap::Ready() {
  bool ready = (lane_point_cloud_.kdtree_get_point_count() == lane_point_ids_.size()) && (!lane_point_ids_.empty());
  ERT_LOG_D("PredictionMap ready(", ready, ") num_lane_points: [", num_lane_points_, "], lane_point_cloud_size: [",
            lane_point_cloud_.kdtree_get_point_count(), "], lane_point_ids_size: [", lane_point_ids_.size(), "]");
  return ready;
}

/**
 * @brief 获取指定半径内的所有车道
 * @details 查询给定位置周围指定半径内的所有车道信息，使用 KD 树进行半径搜索。
 *
 * @param[in] x 查询点 X 坐标
 * @param[in] y 查询点 Y 坐标
 * @param[in] radius 搜索半径(米)
 * @param[out] lane_infos 输出车道信息向量
 * @return bool 查询是否成功，成功返回 `true`，失败返回 `false`
 *
 * @par 输入参数说明:
 * - x: 浮点数，查询点的 X 坐标，取值范围为 `(-∞, +∞)`。
 * - y: 浮点数，查询点的 Y 坐标，取值范围为 `(-∞, +∞)`。
 * - radius: 浮点数，搜索半径，单位为米，取值范围为 `[0, +∞)`。
 * - lane_infos: 指向 `std::vector<LaneInfo>`
 * 类型对象的指针，用于存储查询到的车道信息。
 *
 * @par 关键变量说明:
 * - search_radius_sqr (float): 搜索半径平方，用于距离比较
 * - ret_index (vector): 最近邻索引结果
 * - out_dist_sqr (vector): 平方距离结果
 *
 * @par 流程图:
 * @startuml
 * start
 * :验证 lane_point_tree_ 有效性;
 * :执行半径搜索;
 * if (找到有效车道?) then (是)
 * :填充 lane_infos;
 * :返回 true;
 * else (否)
 * :返回 false;
 * endif
 * @enduml
 */
bool PredictionMap::GetLanes(const float x, const float y, const float radius, std::vector<LaneInfo>* lane_infos) {
  GCHECK_EQ(lane_point_cloud_.kdtree_get_point_count(), lane_point_ids_.size());
  GCHECK_GT(lane_point_ids_.size(), 0);

  float query_point[2] = {static_cast<float>(x), static_cast<float>(y)};
  std::vector<nanoflann::ResultItem<uint32_t, float>> ret_matches;

  auto num_match_points = lane_point_tree_->radiusSearch(&query_point[0], radius * radius, ret_matches);
  if (num_match_points < 1) {
    ERT_LOG_D("No lanes around (x:", x, ", y:", y, ") within radius(", radius, ")");
    return false;
  }

  lane_infos->clear();
  lane_infos->reserve(lane_table_.size());
  std::unordered_set<std::string> unique_lane_ids;
  for (const auto& ret_match : ret_matches) {
    const auto& idx = ret_match.first;
    const auto& searched_point = lane_point_cloud_.pts[idx];
    const auto& lane_id = lane_point_ids_[idx];
    if (unique_lane_ids.find(lane_id) != unique_lane_ids.end()) {
      continue;
    }

    auto relative_index = idx - lane_point_start_indices_[idx];
    if (relative_index < 0) {
      ERT_LOG_E("Invalid relative point index [", relative_index, "] for lane [", lane_id, "]. target_idx [", idx,
                "], lane_point_start_indices [", lane_point_start_indices_[idx], "]");
      continue;
    }
    unique_lane_ids.insert(lane_id);
    lane_infos->emplace_back(relative_index, lane_table_[lane_id]);
  }

  return true;
}

/**
 * @brief 获取最近车道 ID
 * @details 查找距离给定位置最近的车道及其索引，使用 KD 树进行最近邻搜索。
 *
 * @param[in] x 查询点 X 坐标
 * @param[in] y 查询点 Y 坐标
 * @param[out] lane_point_index 最近车道点索引
 * @param[out] lane_id 最近车道 ID
 * @return bool 查询是否成功，成功返回 `true`，失败返回 `false`
 *
 * @par 输入参数说明:
 * - x: 浮点数，查询点的 X 坐标，取值范围为 `(-∞, +∞)`。
 * - y: 浮点数，查询点的 Y 坐标，取值范围为 `(-∞, +∞)`。
 * - lane_point_index: 指向整数的指针，用于存储最近车道点的索引。
 * - lane_id: 指向 `std::string` 类型对象的指针，用于存储最近车道的 ID。
 *
 * @note 使用 KD 树进行最近邻搜索
 */
bool PredictionMap::GetNearestLaneId(const float x, const float y, int* lane_point_index, std::string* lane_id) {
  GCHECK_EQ(lane_point_cloud_.kdtree_get_point_count(), lane_point_ids_.size());
  GCHECK_GT(lane_point_ids_.size(), 0);

  static constexpr size_t kSearchNum = 1;

  float query_point[2] = {static_cast<float>(x), static_cast<float>(y)};
  std::vector<uint32_t> ret_index(kSearchNum);
  std::vector<float> out_dist_sqr(kSearchNum);

  auto num_match_points = lane_point_tree_->knnSearch(&query_point[0], kSearchNum, &ret_index[0], &out_dist_sqr[0]);
  if (ret_index.size() < 1) {
    ERT_LOG_E("Not searched any lane points within knn.");
    return false;
  }

  uint32_t target_idx = ret_index[0];
  *lane_id = lane_point_ids_[target_idx];
  *lane_point_index = target_idx - lane_point_start_indices_[target_idx];
  if (*lane_point_index < 0) {
    ERT_LOG_E("Invalid lane point index. target_idx: [", target_idx, "], lane_point_start_indices: [",
              lane_point_start_indices_[target_idx], "], lane_point_index: [", *lane_point_index, "]");
    return false;
  }

  ERT_LOG_D("LaneId: [", *lane_id, "], target_idx: [", target_idx, "], lane_point_start_index [",
            lane_point_start_indices_[target_idx], "], lane_point_index [", *lane_point_index, "]");

  return true;
}

/**
 * @brief 通过 ID 获取车道段
 * @details 根据车道 ID 查找对应的车道段对象，若未找到则返回 `nullptr`。
 *
 * @param[in] id 车道 ID 字符串
 * @return const LocalRouteSegment* 车道段指针，未找到返回 `nullptr`
 *
 * @par 输入参数说明:
 * - id: 有效的车道 ID 字符串。
 *
 * @warning 需确保已调用 `Insert()` 加载数据
 */
const CenterLinePtr PredictionMap::LaneById(const std::string& id) {
  if (!IsValidLaneId(id)) {
    return nullptr;
  }

  return lane_table_[id];
}

/**
 * @brief 获取直行后继车道 ID
 * @details 查找给定车道的直行方向后继车道，检查 `successor_lanes`
 * 是否存在，选择类型为 `STRAIGHT` 的车道。
 *
 * @param[in] id 起始车道 ID
 * @return string 后继车道 ID，无则返回空字符串
 *
 * @par 输入参数说明:
 * - id: 有效的起始车道 ID 字符串。
 *
 * @par 判断逻辑:
 * 1. 检查 `successor_lanes` 是否存在
 * 2. 选择类型为 `STRAIGHT` 的车道
 */
std::string PredictionMap::GetSuccessorStraightLaneId(const std::string& id) {
  if (!IsValidLaneId(id)) {
    ERT_LOG_W("Lane [", id, "] not found in prediction map.");
    return "";
  }

  auto current_lane = lane_table_.at(id);
  if (current_lane->pts.empty()) {
    ERT_LOG_W("Lane [", id, "] has no guide points.");
    return "";
  }

  const auto& start_point = current_lane->pts[0];
  std::pair<std::string, float> straight_lane_info("", std::numeric_limits<float>::max());
  // for (const auto& successor_id : current_lane->nextSegmentIds()) {
  //   if (!IsValidLaneId(successor_id)) {
  //     continue;
  //   }

  //   auto* successor_lane = lane_table_[successor_id];
  //   if (successor_lane->guidePoints().empty()) {
  //     continue;
  //   }

  //   const auto& end_point = successor_lane->guidePoints()[0];
  //   float dx = end_point.x() - start_point.x();
  //   float dy = end_point.y() - start_point.y();
  //   float atan_angle = std::atan2(dy, dx);
  //   if (std::fabs(atan_angle) < straight_lane_info.second) {
  //     straight_lane_info = std::make_pair(successor_id, atan_angle);
  //   }
  // }

  return straight_lane_info.first;
}

/**
 * @brief 获取路口特征
 * @details 根据路口 ID 查询对应的特征信息，若找到则将特征信息存储到 `feature`
 * 中并返回 `true`，否则返回 `false`。
 *
 * @param[in] id 路口 ID
 * @param[out] feature 输出路口特征
 * @return bool 查询是否成功，成功返回 `true`，失败返回 `false`
 *
 * @par 输入参数说明:
 * - id: 有效的路口 ID 字符串。
 * - feature: 指向 `JunctionFeature`
 * 类型对象的指针，用于存储查询到的路口特征信息。
 */
bool PredictionMap::JunctionFeatureById(const std::string& id, JunctionFeature* feature) {
  if (!IsValidJunctionId(id)) {
    ERT_LOG_E("Junction [", id, "] not found in prediction map.");
    return false;
  }

  *feature = junction_feature_table_[id];
  return true;
}

/**
 * @brief 计算车道投影
 * @details 计算点在车道上的投影坐标(SL
 * 坐标系)，获取车道点序列，查找最近点索引，计算 Frenet 坐标。
 *
 * @param[in] lane_info 车道信息(索引和车道段)
 * @param[in] x 点 X 坐标
 * @param[in] y 点 Y 坐标
 * @param[out] accumulate_s 输出纵向距离(沿车道)
 * @param[out] lateral_distance 输出横向距离
 * @return bool 计算是否成功，成功返回 `true`，失败返回 `false`
 *
 * @par 输入参数说明:
 * - lane_info: `LaneInfo` 类型对象，包含最近点索引和车道段指针。
 * - x: 浮点数，点的 X 坐标，取值范围为 `(-∞, +∞)`。
 * - y: 浮点数，点的 Y 坐标，取值范围为 `(-∞, +∞)`。
 * - accumulate_s: 指向浮点数的指针，用于存储沿车道的纵向距离。
 * - lateral_distance: 指向浮点数的指针，用于存储横向距离。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取车道点序列;
 * :查找最近点索引;
 * :计算 Frenet 坐标;
 * if (计算结果有效?) then (是)
 * :填充输出参数;
 * :返回 true;
 * else (否)
 * :返回 false;
 * endif
 * @enduml
 */
bool PredictionMap::GetProjection(const LaneInfo& lane_info, const float x, const float y, float* accumulate_s,
                                  float* lateral_distance) {
  GCHECK_NOTNULL(accumulate_s);
  GCHECK_NOTNULL(lateral_distance);

  const auto& [nearest_point_index, lane] = lane_info;
  if ((nearest_point_index < 0) || (nearest_point_index >= lane->pts.size())) {
    ERT_LOG_E("Invalid nearest point index [", nearest_point_index, "] in Lane [", lane->id, "]");
    return false;
  }

  const auto& lane_centers = lane->pts;
  const auto& nearest_lane_center = lane_centers[nearest_point_index];

  auto line_start_index = std::max((nearest_point_index - 1), 0);
  auto line_end_index = line_start_index + 1;
  planning::math::LineSegment2d nearest_segment(
      {lane_centers[line_start_index].x(), lane_centers[line_start_index].y()},
      {lane_centers[line_end_index].x(), lane_centers[line_end_index].y()});

  Vec2d nearest_point{x, y};
  const auto prod = nearest_segment.ProductOntoUnit(nearest_point);
  const auto proj = nearest_segment.ProjectOntoUnit(nearest_point);
  ERT_LOG_D("Segment__start (", nearest_segment.start().x(), ", ", nearest_segment.start().y(), "), end (",
            nearest_segment.end().x(), ", ", nearest_segment.end().y(), "), point (", nearest_point.x(), ", ",
            nearest_point.y(), "), proj (", proj, "), prod (", prod, ").");

  if (nearest_point_index == 0) {
    *accumulate_s = std::min(proj, nearest_segment.length());
  } else if (nearest_point_index == lane_centers.size() - 1) {
    // *accumulate_s = nearest_lane_center.s() + std::max(0.0, proj);
    *accumulate_s = nearest_lane_center.x() + std::max(0.0, proj);
  } else {
    // *accumulate_s = nearest_lane_center.s() + std::max(0.0, std::min(proj, nearest_segment.length()));
    *accumulate_s = nearest_lane_center.x() + std::max(0.0, std::min(proj, nearest_segment.length()));
  }
  *lateral_distance = prod;
  ERT_LOG_D("GetProjection s: ", *accumulate_s, ", l: ", *lateral_distance, " in Lane ", lane->id,
            " nearest_point_index ", nearest_point_index, ", nearest_segment_length ", nearest_segment.length());

  return true;
}

/**
 * @brief 检查点是否在路口内
 * @details
 * 判断给定坐标点是否位于任何路口区域内，遍历所有路口多边形，使用射线法判断点是否在多边形内。
 *
 * @param[in] x 点 X 坐标
 * @param[in] y 点 Y 坐标
 * @param[out] in_junction_id 所在路口 ID(可选)
 * @return bool 是否在路口内，在路口内返回 `true`，否则返回 `false`
 *
 * @par 输入参数说明:
 * - x: 浮点数，点的 X 坐标，取值范围为 `(-∞, +∞)`。
 * - y: 浮点数，点的 Y 坐标，取值范围为 `(-∞, +∞)`。
 * - in_junction_id: 指向 `std::string`
 * 类型对象的指针，可选参数，用于存储所在路口的 ID。
 *
 * @par 判断逻辑:
 * - 遍历所有路口多边形
 * - 使用射线法判断点是否在多边形内
 */
bool PredictionMap::IsInJunction(float x, float y, std::string* in_junction_id) {
  auto t0 = std::chrono::steady_clock::now();
  for (const auto& [junction_id, junction] : junction_table_) {
    if (IsInJunction(*junction, x, y)) {
      *in_junction_id = junction_id;
      return true;
    }
  }
  auto t1 = std::chrono::steady_clock::now();
  ERT_LOG_D("IsInJunction(", junction_table_.size(), "): ", std::chrono::duration<float, std::milli>(t1 - t0).count(),
            " ms");
  return false;
}

/**
 * @brief 检查点是否在指定路口内
 * @details 判断点是否位于给定路口区域内，使用射线法判断点是否在路口多边形内。
 *
 * @param[in] junction 路口区域数据
 * @param[in] x 点 X 坐标
 * @param[in] y 点 Y 坐标
 * @return bool 是否在路口内，在路口内返回 `true`，否则返回 `false`
 *
 * @par 输入参数说明:
 * - junction: `planning::PerceptionArea` 类型对象，包含路口区域信息。
 * - x: 浮点数，点的 X 坐标，取值范围为 `(-∞, +∞)`。
 * - y: 浮点数，点的 Y 坐标，取值范围为 `(-∞, +∞)`。
 */
bool PredictionMap::IsInJunction(const planning::road_instance::Area& junction, float x, float y) const {
  static constexpr float kMinPolygonSize = 3;
  if (!IsJunction(junction) || junction.pts.size() < kMinPolygonSize) {
    return false;
  }

  auto t0 = std::chrono::steady_clock::now();
  bool is_in_polygon = planning::PointInPolygon(Vec2d(x, y), junction.pts);
  auto t1 = std::chrono::steady_clock::now();
  ERT_LOG_D("IsInJunction: is_in(", is_in_polygon, ", ", std::chrono::duration<float, std::milli>(t1 - t0).count(),
            "ms).");
  return is_in_polygon;
}

/**
 * @brief 构建车道查找表
 * @details 从导航数据中提取车道信息构建哈希表，将车道 ID
 * 与对应的车道段关联起来。
 *
 * @param[in] navigation 导航数据
 *
 * @par 输入参数说明:
 * - navigation: 包含导航信息的 `planning::Navigation`
 * 对象，必须包含有效的车道信息。
 */
void PredictionMap::BuildLaneTable(const EnvRoadInstance& road_instance) {
  auto t0 = std::chrono::steady_clock::now();
  UpdateLaneMap(road_instance);
  auto t1 = std::chrono::steady_clock::now();
  ERT_LOG_D("[UpdateLaneMap] ", lane_table_.size(), " segments, ", num_lane_points_, " points, ",
            std::chrono::duration<float, std::milli>(t1 - t0).count(), " ms");

  BuildSearchTree();
  auto t2 = std::chrono::steady_clock::now();
  ERT_LOG_D("[BuildSearchTree] ", std::chrono::duration<float, std::milli>(t2 - t1).count(), " ms");
  ERT_LOG_D("[BuildLaneTable] ", std::chrono::duration<float, std::milli>(t2 - t0).count(), " ms");
}

/**
 * @brief 更新车道映射
 * @details 增量更新车道数据，根据导航数据更新车道 ID 到车道段的映射。
 *
 * @param[in] navigation 导航数据
 *
 * @par 输入参数说明:
 * - navigation: 包含导航信息的 `planning::Navigation` 对象，用于更新车道数据。
 */
void PredictionMap::UpdateLaneMap(const EnvRoadInstance& road_instance) {
  ERT_LOG_D("LocalRouter center line size: ", road_instance.getCenterLineMap().size(),
            " guide line size : ", road_instance.getGuideLineMap().size());

  for (const auto& center_line : road_instance.getCenterLineMap()) {
    SetDelayTime(center_line.second->time_delay_);
    num_lane_points_ += center_line.second->pts.size();
    lane_table_.insert(std::make_pair(center_line.first, center_line.second));
  }
  for (const auto& guide_line : road_instance.getGuideLineMap()) {
    num_lane_points_ += guide_line.second->pts.size();
    lane_table_.insert(std::make_pair(guide_line.first, guide_line.second));
  }
  bindBoundToCenterLine(road_instance, lane_table_);
  lane_point_ids_.reserve(num_lane_points_);
  lane_point_cloud_.pts.reserve(num_lane_points_);
  lane_point_start_indices_.reserve(num_lane_points_);
}

void PredictionMap::SetDelayTime(const double t) {
  time_delay_ = t;
}

/**
 * @brief 构建搜索树
 * @details 基于车道点构建 KD
 * 树用于空间查询，收集所有车道点，构建点云数据结构，初始化 KD 树索引。
 *
 * @par 流程图:
 * @startuml
 * start
 * :收集所有车道点;
 * :构建点云数据结构;
 * :初始化 KD 树索引;
 * stop
 * @enduml
 */
void PredictionMap::BuildSearchTree() {
  std::string previous_lane_id = "";
  int previous_lane_point_index = -1;
  int lane_point_cnt = 0;
  for (const auto& [lane_id, lane_line] : lane_table_) {
    for (const auto& lane_center : lane_line->pts) {
      if (previous_lane_id != lane_id) {
        // ERT_LOG_D("LaneId prev: [", previous_lane_id, "], curr: [", lane_id, "], lane_point_cnt, prev: [",
        //           previous_lane_point_index, "], curr: [", lane_point_cnt, "]");
        previous_lane_id = lane_id;
        previous_lane_point_index = lane_point_cnt;
      }

      lane_point_ids_.push_back(lane_id);
      lane_point_start_indices_.push_back(previous_lane_point_index);

      nanoflann::PointCloud2d<float>::Point pt_nanoflann{static_cast<float>(lane_center.x()),
                                                         static_cast<float>(lane_center.y())};
      lane_point_cloud_.pts.push_back(std::move(pt_nanoflann));

      ++lane_point_cnt;
    }
  }

  lane_point_tree_->buildIndex();
}

/**
 * @brief 将车道边界绑定到中心线
 * @details 构建车道边界KD-Tree并建立中心线点与最近车道边界的映射关系
 *
 * @param[in] road_instance 道路实例数据，包含车道边界信息
 * @param[in] center_line_map 中心线映射表，需绑定的中心线集合
 *
 * @par 输入参数说明:
 * - road_instance: 必须包含有效的车道边界数据
 * - center_line_map: key为中心线ID，value为CenterLine智能指针
 *
 * @par 输出说明:
 * - 结果存储在center_line->bound_pts成员中，包含左右边界信息对
 *
 * @par 算法流程:
 * @startuml
start
:构建车道边界KD-Tree;
:遍历中心线点;
:计算点航向角;
:搜索半径5m内车道边界;
:绑定左右最近边界;
end
@enduml
 *
 * @note 边界搜索半径通过lane_boudary_search_radius_参数控制(默认5.0米)
 */
void PredictionMap::bindBoundToCenterLine(const EnvRoadInstance& road_instance,
                                          const std::unordered_map<std::string, const CenterLinePtr>& center_line_map) {
  std::unordered_map<std::string, LaneMarkingPtr> lane_boundary_map;

  // 构建车道边界KdTree
  for (const auto& item : road_instance.getLaneMarkingMap()) {
    const auto& lane_marking = item.second;
    if (lane_marking && (!isLaneMarkingIgnore(lane_marking)) && lane_marking->init()) {
      // ERT_PLOG_D << "\t [Perception] lane_marking id = " << item.first;
      // item.second->getMutableRelatedReferenceLineInfos().clear();  // 清空车道边界中的缓存信息
      lane_boundary_map.emplace(item.first, lane_marking);
    }
  }
  lane_boundary_kdtree_.reset(new AABoxKDTree2d(lane_boundary_map, kdtree_params_));

  // 遍历参考线上的点搜索车道和道路边界
  for (const auto& item : center_line_map) {
    const auto& center_line_id = item.first;
    const auto& center_line = item.second;
    const auto& center_line_pts = center_line->pts;
    const auto center_line_pts_size = center_line_pts.size();
    std::vector<std::pair<BoundInfo, BoundInfo>> lane_bound_arr;
    lane_bound_arr.reserve(center_line_pts_size);
    // ERT_PLOG_D << "[Bind Boundary] center_line id: " << center_line_id << ", size = " << center_line->pts.size();
    for (int i = 0; i < center_line_pts_size; ++i) {
      double point_heading = 0.0;
      if (i == 0) {
        point_heading = std::atan2(center_line_pts[i + 1].y() - center_line_pts[i].y(),
                                   center_line_pts[i + 1].x() - center_line_pts[i].x());
      } else {
        point_heading = std::atan2(center_line_pts[i].y() - center_line_pts[i - 1].y(),
                                   center_line_pts[i].x() - center_line_pts[i - 1].x());
      }
      const auto& point = center_line_pts[i];
      // 搜索附近车道边界
      std::map<double, std::pair<LaneMarking*, double>> dist_to_lane_boundaries;
      searchAroundLaneMarkings(lane_boundary_kdtree_, point, point_heading, true, dist_to_lane_boundaries);
      // ERT_LOG_D("i = ", i, ", dist_to_lane_boundaries size = ", dist_to_lane_boundaries.size());
      // for (auto& item : dist_to_lane_boundaries) {
      //   ERT_LOG_D("\t dist = ", item.first, ", marking = ", item.second.first->getId());
      // }
      // 绑定左右两侧最近车道边界
      bindLeftRightBoundaries(point, point_heading, dist_to_lane_boundaries, lane_bound_arr);
    }
    center_line->bound_pts = std::move(lane_bound_arr);
  }
}

/**
 * @brief 搜索目标点周围的车道标线
 * @details 使用AABoxKDTree2d进行空间范围搜索，根据航向角筛选车道标线，返回带符号距离的标线信息
 *
 * @param[in] kdtree 二维AABB KD-Tree共享指针（必须已初始化）
 * @param[in] target_point 目标点世界坐标（Vec3d类型，单位：米）
 * @param[in] point_heading 目标点航向角（弧度制，取值范围[0, 2π)）
 * @param[in] enable_heading_search 是否启用航向角过滤（true-启用，false-禁用）
 * @param[out] dist_to_markings 输出结果映射表（key:有符号距离平方，value:标线指针及s值）
 *
 * @startuml
 start
 :检查KD-Tree有效性;
 if (kdtree == nullptr?) then (是)
   :记录错误并返回false;
 else (否)
   :执行KD-Tree范围搜索（半径5.0米）;
   :收集候选标线及距离信息;
   if (找到有效标线?) then (是)
     :按距离排序结果;
     :填充dist_to_markings;
     :返回true;
   else (否)
     :返回false;
   endif
 endif
 stop
 @enduml
 */
bool PredictionMap::searchAroundLaneMarkings(std::shared_ptr<AABoxKDTree2d>& kdtree, const Vec3d& target_point,
                                             const double point_heading, const bool enable_heading_search,
                                             std::map<double, std::pair<LaneMarking*, double>>& dist_to_markings) {
  if (!kdtree) {
    ERT_LOG_E("kdtree is null");
    return false;
  }
  dist_to_markings.clear();
  std::vector<double> signed_dist_sqr_arr;
  std::vector<double> s_arr;
  std::vector<LaneMarking*> marking_arr;
  marking_arr = kdtree->GetObjectsWithSignedDistance(target_point, point_heading, lane_boudary_search_radius_,
                                                     enable_heading_search, true, signed_dist_sqr_arr, s_arr);
  if ((marking_arr.size() > 0) && (signed_dist_sqr_arr.size() == marking_arr.size())
      && (s_arr.size() == marking_arr.size())) {
    for (int i = 0; i < marking_arr.size(); ++i) {
      dist_to_markings.emplace(signed_dist_sqr_arr[i], std::make_pair(marking_arr[i], s_arr[i]));
    }
    return true;
  }
  // ERT_LOG_W("can not find arround lane maring.");
  return false;
}

/**
 * @brief 绑定目标点的左右车道边界信息
 * @details 通过横向搜索计算左右车道边界交点，筛选最近的有效边界信息
 *
 * @param[in] target_point 目标点世界坐标（Vec3d类型，单位：米）
 * @param[in] point_heading 目标点航向角（弧度制，取值范围[0, 2π)）
 * @param[in] arround_boundaries 周围边界信息映射表（key:有符号距离平方，value:标线指针及s值）
 * @param[out] bound_info_vec 输出边界信息向量（存储左右边界配对信息）
 *
 * @startuml
 start
 :创建左右搜索线段;
 :遍历所有候选边界;
 if (与左搜索线相交?) then (是)
   :记录左侧边界信息;
 endif
 if (与右搜索线相交?) then (是)
   :记录右侧边界信息;
 endif
 :按横向距离排序结果;
 :查找最近左右边界;
 :填充bound_info_vec;
 stop
 @enduml
 */
void PredictionMap::bindLeftRightBoundaries(const Vec3d& target_point, const double point_heading,
                                            const std::map<double, std::pair<LaneMarking*, double>>& arround_boundaries,
                                            std::vector<std::pair<BoundInfo, BoundInfo>>& bound_info_vec) {
  // 基于最近横向距离构建新的arround_boundaries
  std::map<double, std::pair<LaneMarking*, BoundInfo>> new_arround_boundaries;
  double lateral_dist = 0.0;
  double s_in_marking = 0.0;
  // 计算左侧横向边界距离
  Vec2d left_local_pt(0.0, lane_boudary_search_radius_);
  Vec2d left_end_pt = pnc::planning::Local2Global(target_point, point_heading, left_local_pt);
  planning::math::LineSegment2d left_line_seg(target_point, left_end_pt);
  BoundInfo left_bound_info;
  for (const auto& item : arround_boundaries) {
    auto& lane_marking = item.second.first;
    // 若存在交点，则向new_arround_boundaries中添加信息
    if (lane_marking->calcInterSection(left_line_seg, left_bound_info, s_in_marking, lateral_dist)) {
      // ERT_PLOG_D << "[Left] marking: " << lane_marking->getId() << ", lateral_dist = " << lateral_dist
      //            << ", s_in_marking = " << s_in_marking;
      new_arround_boundaries.emplace(lateral_dist, std::make_pair(lane_marking, left_bound_info));
    }
  }
  // 计算右侧横向边界距离
  Vec2d right_local_pt(0, -1.0 * lane_boudary_search_radius_);
  Vec2d right_end_pt = pnc::planning::Local2Global(target_point, point_heading, right_local_pt);
  planning::math::LineSegment2d right_line_seg(target_point, right_end_pt);
  BoundInfo right_bound_info;
  for (const auto& item : arround_boundaries) {
    auto& lane_marking = item.second.first;
    // 若存在交点，则向new_arround_boundaries中添加信息
    if (lane_marking->calcInterSection(right_line_seg, right_bound_info, s_in_marking, lateral_dist)) {
      // ERT_PLOG_D << "[Right] marking: " << lane_marking->getId() << ", lateral_dist = " << -1.0 * lateral_dist
      //            << ", s_in_marking = " << s_in_marking;
      new_arround_boundaries.emplace(-1.0 * lateral_dist, std::make_pair(lane_marking, right_bound_info));
    }
  }
  // 查找左右两侧最近的车道线信息
  BoundInfo left_nearest_bound_info;
  BoundInfo right_nearest_bound_info;
  auto iter = new_arround_boundaries.lower_bound(0.0);
  if (iter != new_arround_boundaries.end()) {  // 查找左侧最近的车道线（正值中最接近0的，包含0）
    left_nearest_bound_info = iter->second.second;
    // 查找右侧最近的车道线（负值中最接近0的, 不包含0）
    if (iter != new_arround_boundaries.begin()) {
      --iter;
      if (iter->first < 0.0) {
        right_nearest_bound_info = iter->second.second;
      }
    }
  } else {  // 直接在负值中查找右侧最近的车道线（最接近0的）
    if (new_arround_boundaries.size() > 0) {
      right_nearest_bound_info = new_arround_boundaries.rbegin()->second.second;
    }
  }
  bound_info_vec.emplace_back(std::move(std::make_pair(left_nearest_bound_info, right_nearest_bound_info)));
}

/**
 * @brief 构建路口查找表
 * @details 从导航数据中提取路口信息，将路口 ID 与对应的路口区域关联起来。
 *
 * @param[in] navigation 导航数据
 *
 * @par 输入参数说明:
 * - navigation: 包含导航信息的 `planning::Navigation`
 * 对象，必须包含有效的路口信息。
 */
void PredictionMap::BuildJunctionTable(const EnvRoadInstance& road_instance) {
  auto t0 = std::chrono::steady_clock::now();
  for (const auto& area : road_instance.getAreaMap()) {
    ERT_LOG_D("Area [", area.first, "] type: ", size_t(area.second->area_type));
    if (!IsJunction(*area.second)) {
      continue;
    }

    junction_table_.insert(std::make_pair(area.first, area.second));
    InsertJunctionFeature(*area.second);
    ERT_LOG_D("Junction [", area.first, "] has ", area.second->related_reference_lines_ids.size(),
              " related in lanes, ", area.second->pts.size(), " polygon points.");
  }
  auto t1 = std::chrono::steady_clock::now();
  ERT_LOG_D("Junction table size: ", junction_table_.size());
  ERT_LOG_D("[BuildJunctionTable] ", std::chrono::duration<float, std::milli>(t1 - t0).count(), " ms");
}

/**
 * @brief 插入路口特征
 * @details 解析单个路口区域的特征信息，将路口 ID 与对应的路口特征关联起来。
 *
 * @param[in] junction 路口区域数据
 *
 * @par 输入参数说明:
 * - junction: `planning::PerceptionArea` 类型对象，包含路口区域信息。
 */
void PredictionMap::InsertJunctionFeature(const planning::road_instance::Area& junction) {
  JunctionFeature feature;
  feature.set_id(junction.id);

  for (const auto& lane_id : junction.related_reference_lines_ids) {
    auto lane = LaneById(lane_id);
    if (!lane) {
      ERT_LOG_E("Junction [", junction.id, "] Lane [", lane_id, "] not found in map!");
      continue;
    }

    auto* exit = feature.add_exits();
    exit->set_lane_id(lane_id);

    const auto& exit_point = lane->pts.front();
    auto* position = exit->mutable_position();
    position->set_x(exit_point.x());
    position->set_y(exit_point.y());
    exit->set_theta(exit_point.Angle());  // TODO: angle not right
    // exit->set_width(exit_point.leftLaneWidth() + exit_point.rightLaneWidth());
    exit->set_width(3.2);
    ERT_LOG_D("Junction [", junction.id, "] exit [", lane_id, "] ", exit->ShortDebugString(), " at Lane [", lane->id,
              "] with position (", exit_point.x(), ", ", exit_point.y(), ")");
  }
  junction_feature_table_.insert(std::make_pair(junction.id, std::move(feature)));
}

/**
 * @brief 判断区域是否为路口
 * @details 根据区域的特征判断其是否为路口。
 *
 * @param[in] area 区域数据
 * @return bool 是路口返回 `true`，否则返回 `false`
 *
 * @par 输入参数说明:
 * - area: `planning::PerceptionArea` 类型对象，包含区域信息。
 */
bool PredictionMap::IsJunction(const planning::road_instance::Area& area) const {
  return (area.area_type == proto::MapCommon_AreaType_kAreaTypeJunction);
}

/**
 * @brief 判断路口 ID 是否有效
 * @details 根据一定规则判断给定的路口 ID 是否有效。
 *
 * @param[in] id 路口 ID
 * @return bool 有效返回 `true`，无效返回 `false`
 *
 * @par 输入参数说明:
 * - id: 路口 ID 字符串。
 */
bool PredictionMap::IsValidJunctionId(const std::string& id) {
  if (id.empty()) {
    ERT_LOG_W("Try to find lane with empty id!");
    return false;
  }

  if (junction_table_.find(id) == junction_table_.end()) {
    ERT_LOG_W("Lane [", id, "] not found in prediction map!");
    return false;
  }

  if (junction_table_[id] == nullptr) {
    ERT_LOG_W("Lane [", id, "] is nullptr!");
    return false;
  }

  return true;
}

/**
 * @brief 判断车道 ID 是否有效
 * @details 根据一定规则判断给定的车道 ID 是否有效。
 *
 * @param[in] id 车道 ID
 * @return bool 有效返回 `true`，无效返回 `false`
 *
 * @par 输入参数说明:
 * - id: 车道 ID 字符串。
 */
bool PredictionMap::IsValidLaneId(const std::string& id) {
  if (id.empty()) {
    ERT_LOG_W("Try to find lane with empty id!");
    return false;
  }

  if (lane_table_.find(id) == lane_table_.end()) {
    ERT_LOG_W("Lane [", id, "] not found in prediction map!");
    return false;
  }

  if (lane_table_[id] == nullptr) {
    ERT_LOG_W("Lane [", id, "] is nullptr!");
    return false;
  }

  return true;
}

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal
