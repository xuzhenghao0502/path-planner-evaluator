/**
 * @file prediction_map.h
 * @brief 预测地图数据管理类头文件，负责管理车道和路口信息，提供空间查询功能
 * @details 此文件定义了 `PredictionMap` 类，该类主要用于管理车道段和路口数据，
 * 并提供最近邻查询、投影计算等空间查询功能。
 */

#pragma once

#include <any>
#include <memory>
#include <pnc_common_util/third_party/nanoflann/nanoflann_utils.hpp>

#include "decision_data/container/container.h"
#include "math/aaboxkdtree2d.h"
#include "proto/prediction/junction_feature.pb.h"
#include "proto/prediction/prediction_config.pb.h"
#include "road_instance/env_road_instance.h"

namespace gpal {
namespace pnc {
namespace prediction {

using Vec2d = pnc::planning::math::Vec2d;
using Vec3d = pnc::planning::math::Vec3d;
using EnvRoadInstance = gpal::pnc::planning::road_instance::EnvRoadInstance;
using CenterLine = gpal::pnc::planning::road_instance::PerceptionCenterLine;
using CenterLinePtr = std::shared_ptr<CenterLine>;
using BoundInfo = CenterLine::BoundInfo;
using LaneMarking = gpal::pnc::planning::road_instance::LaneMarking;
using LaneMarkingPtr = std::shared_ptr<LaneMarking>;
using AABoxKDTree2d = gpal::pnc::planning::math::AABoxKDTree2d<LaneMarking>;
using AABoxKDTree2dParams = gpal::pnc::planning::math::AABoxKDTree2dParams;

/**
 * @class PredictionMap
 * @brief 预测模块使用的地图数据结构
 *
 * @details 主要功能包括:
 * - 管理车道段和路口数据
 * - 提供最近邻查询
 * - 处理投影计算
 *
 * @par 关键数据结构:
 * - lane_table_: 车道 ID 到车道段的映射
 * - junction_table_: 路口 ID 到路口区域的映射
 * - lane_point_tree_: 用于最近邻搜索的 KD 树
 */
class PredictionMap {
 public:
  // [nearest_point_index, lane_segment]
  using LaneInfo = std::pair<int, const planning::road_instance::PerceptionCenterLinePtr>;

  /**
   * @brief 构造函数
   * @details 初始化 `PredictionMap` 对象，进行必要的成员变量初始化操作。
   */
  PredictionMap();

  /**
   * @brief 析构函数
   * @details 虚析构函数，确保正确释放派生类对象的资源，默认实现。
   */
  virtual ~PredictionMap() = default;

  void Insert(const EnvRoadInstance& navigation);

  void Clear();

  bool Ready();

  bool GetLanes(const float x, const float y, const float radius, std::vector<LaneInfo>* lane_infos);

  bool GetNearestLaneId(const float x, const float y, int* lane_point_index, std::string* lane_id);

  const CenterLinePtr LaneById(const std::string& id);

  std::string GetSuccessorStraightLaneId(const std::string& id);

  bool JunctionFeatureById(const std::string& id, JunctionFeature* feature);

  bool GetProjection(const LaneInfo& lane_info, const float x, const float y, float* accumulate_s,
                     float* lateral_distance);

  bool IsInJunction(float x, float y, std::string* in_junction_id);

  bool IsInJunction(const planning::road_instance::Area& junction, float x, float y) const;

  double GetDelayTime() { return time_delay_; };

  /**
   * @brief 获取车道查找表的常量引用
   * @details 返回车道 ID 到车道段映射的常量引用，用于外部只读访问。
   *
   * @return const std::unordered_map<std::string, const planning::LocalRouteSegment*>& 车道查找表的常量引用
   */
  const std::unordered_map<std::string, const CenterLinePtr>& lane_table() const { return lane_table_; }

 private:
  void BuildLaneTable(const EnvRoadInstance& navigation);

  void UpdateLaneMap(const EnvRoadInstance& navigation);

  void BuildSearchTree();

  void BuildJunctionTable(const EnvRoadInstance& navigation);

  void InsertJunctionFeature(const planning::road_instance::Area& junction);

  bool IsJunction(const planning::road_instance::Area& area) const;

  bool IsValidJunctionId(const std::string& id);

  bool IsValidLaneId(const std::string& id);

  void bindBoundToCenterLine(const EnvRoadInstance& road_instance,
                             const std::unordered_map<std::string, const CenterLinePtr>& center_line_map);
  void SetDelayTime(const double t);

  bool isLaneMarkingIgnore(const LaneMarkingPtr& marking) const {
    return (marking->getLineType() == proto::perception::LaneMarking::kTypeIgnore);
  }

  bool searchAroundLaneMarkings(std::shared_ptr<AABoxKDTree2d>& kdtree, const Vec3d& target_point,
                                const double point_heading, const bool enable_heading_search,
                                std::map<double, std::pair<LaneMarking*, double>>& dist_to_markings);
  void bindLeftRightBoundaries(const Vec3d& target_point, const double point_heading,
                               const std::map<double, std::pair<LaneMarking*, double>>& arround_boundaries,
                               std::vector<std::pair<BoundInfo, BoundInfo>>& bound_info_vec);

 private:
  PredictionMapConfig config_;

  // ! LaneSegment
  std::unordered_map<std::string, const planning::road_instance::PerceptionCenterLinePtr>
      lane_table_;  ///< 车道 ID 到车道段的映射

  uint32_t num_lane_points_ = 0;                     ///< 车道点的数量
  std::vector<std::string> lane_point_ids_;          ///< 车道点的 ID 列表
  std::vector<int> lane_point_start_indices_;        ///< 车道点的起始索引列表
  nanoflann::PointCloud2d<float> lane_point_cloud_;  ///< 用于存储车道点的点云数据结构

  using KdTreeNanoFLANN =
      nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, nanoflann::PointCloud2d<float>>,
                                          nanoflann::PointCloud2d<float>, 2>;
  std::unique_ptr<KdTreeNanoFLANN> lane_point_tree_ = nullptr;  ///< 用于最近邻搜索的 KD 树

  // ! Junction
  std::unordered_map<std::string, const planning::road_instance::AreaPtr>
      junction_table_;                                                       ///< 路口 ID 到路口区域的映射
  std::unordered_map<std::string, JunctionFeature> junction_feature_table_;  ///< 路口 ID 到路口特征的映射

  const float kDefaultBoundaryWidth_ = 10.0;             ///< 默认边界宽度,米
  float lane_boudary_search_radius_ = 4.0;               ///< 车道线搜索半径,米
  AABoxKDTree2dParams kdtree_params_;                    ///< 车道线KD-Tree参数
  std::shared_ptr<AABoxKDTree2d> lane_boundary_kdtree_;  ///< 车道线KD-Tree

  double time_delay_ = 0.0;  ///< 时延
};

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal