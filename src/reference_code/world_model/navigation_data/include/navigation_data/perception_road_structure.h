/**
 * @file perception_road_structure.h
 * @brief 感知道路结构实现文件
 * @details
 * 该文件实现了感知道路结构的核心功能，包括道路结构的初始化、清空、车道和区域信息的获取与更新等。感知道路结构用于存储和管理车辆感知到的道路信息，包括车道、车道段、特殊区域等。
 */

#pragma once

#include <map>
#include <vector>

#include "math/vec2d.h"
#include "navigation_data/common_typedefs.h"
#include "util/base_struct.h"

namespace gpal::pnc::planning {

/**
 * @brief PerceptionRoadStructure类用于表示感知到的道路结构信息
 * @details 该类包含了自车所在车道、车道段、车道信息、特殊区域信息等
 */
class PerceptionRoadStructure : public StampedBase {
 public:
  PerceptionRoadStructure();
  virtual ~PerceptionRoadStructure();

  void clear();

  /**
   * @brief 设置输出坐标系
   * @param frame_id 输出坐标系
   */
  void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }
  /**
   * @brief 获取输出坐标系
   * @return 返回输出坐标系
   */
  const std::string& getFrameId() const { return frame_id_; }

  /**
   * @brief 设置自车所在车道ID
   * @param ego_lane_id 自车所在车道ID
   */
  void setEgoLaneId(const std::string& ego_lane_id) { ego_lane_id_ = ego_lane_id; }
  /**
   * @brief 获取自车所在车道ID
   * @return 返回自车所在车道ID
   */
  const std::string& getEgoLaneId() const { return ego_lane_id_; }

  /**
   * @brief 设置自车所在车道段ID
   * @param ego_lane_segment_id 自车所在车道段ID
   */
  void setEgoLaneSegmentId(const std::string& ego_lane_segment_id) { ego_lane_segment_id_ = ego_lane_segment_id; }
  /**
   * @brief 获取自车所在车道段ID
   * @return 返回自车所在车道段ID
   */
  const std::string& getEgoLaneSegmentId() const { return ego_lane_segment_id_; }

  /**
   * @brief 获取可修改的车道信息映射
   * @return 返回可修改的车道信息映射指针
   */
  std::map<std::string, Lane>* getMutableLaneMap() { return &lane_map_; }
  /**
   * @brief 获取车道信息映射
   * @return 返回车道信息映射
   */
  const std::map<std::string, Lane>& getLaneMap() const { return lane_map_; }

  /**
   * @brief 获取可修改的特殊区域信息映射
   * @return 返回可修改的特殊区域信息映射指针
   */
  std::map<std::string, PerceptionArea>* getMutableAreaMap() { return &area_map_; }
  /**
   * @brief 获取特殊区域信息映射
   * @return 返回特殊区域信息映射
   */
  const std::map<std::string, PerceptionArea>& getAreaMap() const { return area_map_; }

  /**
   * @brief 获取可修改的自车所在道路信息
   * @return 返回可修改的自车所在道路信息指针
   */
  PerceptionRoadInfo* getMutableEgoRoadInfo() { return &ego_road_info_; }
  /**
   * @brief 获取自车所在道路信息
   * @return 返回自车所在道路信息
   */
  const PerceptionRoadInfo& getEgoRoadInfo() const { return ego_road_info_; }

 protected:
  /**
   * @brief 输出坐标系
   * @details 有效的输出坐标系包括：
   * - "map": 绝对定位坐标系（NOA功能、泊车寻库功能以及绝对定位有效时的LCC功能下时使用）
   * - "car": 自车坐标系（ACC功能以及绝对定位无效时的LCC功能下时使用）
   */
  std::string frame_id_ = "";                       ///< 坐标系id
  std::string ego_lane_id_ = "";                    ///< 自车所在车道 id
  std::string ego_lane_segment_id_ = "";            ///< 自车所在车道段 id
  std::map<std::string, Lane> lane_map_;            ///< <车道ID，车道>
  std::map<std::string, PerceptionArea> area_map_;  ///< <特殊区域ID，特殊区域>
  PerceptionRoadInfo ego_road_info_;                ///< 自车所在道路信息
};

}  // namespace gpal::pnc::planning
