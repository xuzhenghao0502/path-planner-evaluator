/**
 * @file env_road_instance.h
 * @brief 环境道路实例数据实现文件
 * @details
 * 该文件实现了环境道路实例数据的核心功能，包括数据的初始化、清空、获取和设置等操作。环境道路实例数据用于存储和管理车辆感知到的环境道路信息，包括车道、停止线、地面箭头等。
 */

#pragma once

#include <map>
#include <vector>

#include "common_typedefs.h"
#include "lane_marking.h"
#include "util/base_struct.h"

namespace gpal::pnc::planning::road_instance {

using gpal::pnc::planning::StampedBase;

class EnvRoadInstance : public StampedBase {
 public:
  friend class EnvRoadInstanceAdapter;
  EnvRoadInstance();
  virtual ~EnvRoadInstance();

  void clear();

  const std::unordered_map<std::string, PerceptionCenterLinePtr> &getCenterLineMap() const { return center_line_map_; }
  std::unordered_map<std::string, PerceptionCenterLinePtr> &getMutableCenterLineMap() { return center_line_map_; }

  const std::unordered_map<std::string, PerceptionCenterLinePtr> &getGuideLineMap() const { return guide_line_map_; }
  std::unordered_map<std::string, PerceptionCenterLinePtr> &getMutableGuideLineMap() { return guide_line_map_; }

  const std::unordered_map<std::string, LaneMarkingPtr> &getLaneMarkingMap() const { return lane_marking_map_; }
  std::unordered_map<std::string, LaneMarkingPtr> &getMutableLaneMarkingMap() { return lane_marking_map_; }

  const std::unordered_map<std::string, StopLinePtr> &getStopLineMap() const { return stop_line_map_; }
  std::unordered_map<std::string, StopLinePtr> &getMutableStopLineMap() { return stop_line_map_; }

  const std::unordered_map<std::string, ArrowPtr> &getArrowMap() const { return arrow_map_; }
  std::unordered_map<std::string, ArrowPtr> &getMutableArrowMap() { return arrow_map_; }

  const std::unordered_map<std::string, AreaPtr> &getAreaMap() const { return area_map_; }
  std::unordered_map<std::string, AreaPtr> &getMutableAreaMap() { return area_map_; }

  const std::unordered_map<std::string, KeyPointPtr> &getKeyPointMap() const { return key_point_map_; }
  std::unordered_map<std::string, KeyPointPtr> &getMutableKeyPointMap() { return key_point_map_; }

  const std::unordered_map<std::string, TrafficLightPtr> &getTrafficLightMap() const { return traffic_light_map; }
  std::unordered_map<std::string, TrafficLightPtr> &getMutableTrafficLightMap() { return traffic_light_map; }

  const std::unordered_map<std::string, GatePtr> &getGateMap() const { return gates_map; }
  std::unordered_map<std::string, GatePtr> &getMutableGateMap() { return gates_map; }

 protected:
  std::unordered_map<std::string, PerceptionCenterLinePtr> center_line_map_;  ///< <中心线ID，中心线>
  std::unordered_map<std::string, PerceptionCenterLinePtr> guide_line_map_;   ///< <引导线ID，引导线>
  std::unordered_map<std::string, LaneMarkingPtr> lane_marking_map_;  ///< <车道线ID，车道线（包含道路边界）>
  std::unordered_map<std::string, StopLinePtr> stop_line_map_;        ///< <停止线ID，停止线>
  std::unordered_map<std::string, ArrowPtr> arrow_map_;               ///< <地面箭头ID，地面箭头>
  std::unordered_map<std::string, AreaPtr> area_map_;                 ///< <特殊区域ID，特殊区域>
  std::unordered_map<std::string, KeyPointPtr> key_point_map_;        ///< <关键点ID，关键点>
  std::unordered_map<std::string, TrafficLightPtr> traffic_light_map;  ///< <交通灯ID，交通灯>
  std::unordered_map<std::string, GatePtr> gates_map;                  ///< <闸机ID，闸机>
};

}  // namespace gpal::pnc::planning::road_instance
