/**
 * @file perception_road_marking.h
 * @brief 感知道路标线数据实现文件
 * @details
 * 该文件实现了感知道路标线数据的核心功能，包括数据的初始化、清空、获取和设置等操作。感知道路标线数据用于存储和管理车辆感知到的道路标线信息，包括车道标线组、停止线、地面箭头等。
 */

#pragma once

#include <map>
#include <vector>

#include "navigation_data/common_typedefs.h"
#include "util/base_struct.h"

namespace gpal::pnc::planning {

/**
 * @brief PerceptionRoadMarking类
 * @details 用于表示感知到的道路标线信息，继承自StampedBase
 */
class PerceptionRoadMarking : public StampedBase {
 public:
  friend class PerceptionLaneAdapter;
  PerceptionRoadMarking();
  virtual ~PerceptionRoadMarking();

  void clear();

  /**
   * @brief 设置自车所在的车道线组ID
   * @details 将传入的车道线组ID赋值给成员变量ego_group_id_
   *
   * @param ego_group_id 自车所在的车道线组ID
   */
  void setEgoGroupId(const std::string& ego_group_id) { ego_group_id_ = ego_group_id; }
  /**
   * @brief 获取自车所在的车道线组ID
   * @details 返回成员变量ego_group_id_的值
   *
   * @return 自车所在的车道线组ID
   */
  const std::string& getEgoGroupId() const { return ego_group_id_; }
  /**
   * @brief 获取可修改的车道线组映射
   * @details 返回车道线组映射的指针，允许外部修改
   *
   * @return 车道线组映射的指针
   */
  std::map<std::string, PerceptionLaneMarkingGroup>* getMutableLaneMarkingGroupMap() {
    return &lane_marking_group_map_;
  }
  /**
   * @brief 获取车道线组映射
   * @details 返回车道线组映射的常量引用，不允许外部修改
   *
   * @return 车道线组映射的常量引用
   */
  const std::map<std::string, PerceptionLaneMarkingGroup>& getLaneMarkingGroupMap() const {
    return lane_marking_group_map_;
  }

 protected:
  std::string ego_group_id_ = "";  ///< 自车所在的车道线组ID；特例：自车在路口时, ego_group_id = ""
  std::map<std::string, PerceptionLaneMarkingGroup>
      lane_marking_group_map_;  ///< <车道线组ID，车道线组>，按照车道线所属Road分组
};

}  // namespace gpal::pnc::planning
