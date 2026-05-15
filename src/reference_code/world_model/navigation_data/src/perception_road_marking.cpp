/**
 * @file perception_road_marking.cpp
 * @brief 感知道路标线数据实现文件
 * @details
 * 该文件实现了感知道路标线数据的核心功能，包括数据的初始化、清空、获取和设置等操作。感知道路标线数据用于存储和管理车辆感知到的道路标线信息，包括车道标线组、停止线、地面箭头等。
 */

#include "navigation_data/perception_road_marking.h"

#include "config_manager/config_manager.h"

namespace gpal::pnc::planning {

/**
 * @brief PerceptionRoadMarking构造函数
 * @details 初始化PerceptionRoadMarking对象，并调用clear()函数清空所有数据
 *
 * @par 关键变量说明:
 * - ego_group_id_ (std::string): 自车所在车道组ID，初始化为空字符串
 * - lane_marking_group_map_ (std::map<std::string, LaneMarkingGroup>):
 * 车道标线组信息映射，键为车道标线组ID，值为车道标线组信息，初始化为空
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用clear()函数;
 * :初始化ego_group_id_为空字符串;
 * :初始化lane_marking_group_map_为空;
 * stop
 * @enduml
 *
 * @note 构造函数中调用clear()函数确保所有成员变量被正确初始化
 *
 * @warning 需确保StampedBase基类已正确初始化
 */
PerceptionRoadMarking::PerceptionRoadMarking() { clear(); }

/**
 * @brief PerceptionRoadMarking析构函数
 * @details 释放PerceptionRoadMarking对象占用的资源，确保所有动态分配的内存被正确释放
 *
 * @par 关键变量说明:
 * - ego_group_id_ (std::string): 自车所在车道组ID，析构时自动释放
 * - lane_marking_group_map_ (std::map<std::string, LaneMarkingGroup>):
 * 车道标线组信息映射，键为车道标线组ID，值为车道标线组信息，析构时自动释放
 *
 * @par 流程图:
 * @startuml
 * start
 * :释放ego_group_id_内存;
 * :释放lane_marking_group_map_内存;
 * stop
 * @enduml
 *
 * @note 析构函数中所有成员变量均为栈上对象，无需手动释放内存
 *
 * @warning 确保PerceptionRoadMarking对象生命周期结束时调用析构函数
 */
PerceptionRoadMarking::~PerceptionRoadMarking() {}

/**
 * @brief 清空PerceptionRoadMarking对象的所有数据
 * @details 重置所有成员变量为初始状态，确保对象可以重新使用
 *
 * @par 关键变量说明:
 * - ego_group_id_ (std::string): 自车所在车道组ID，重置为空字符串
 * - lane_marking_group_map_ (std::map<std::string, LaneMarkingGroup>):
 * 车道标线组信息映射，键为车道标线组ID，值为车道标线组信息，清空所有条目
 *
 * @par 流程图:
 * @startuml
 * start
 * :重置ego_group_id_为空字符串;
 * :清空lane_marking_group_map_;
 * stop
 * @enduml
 *
 * @note 该函数用于重置对象状态，通常在对象重用或销毁前调用
 *
 * @warning 调用该函数后，所有存储的数据将被清空，需谨慎使用
 */
void PerceptionRoadMarking::clear() {
  ego_group_id_ = "";
  lane_marking_group_map_.clear();
}

}  // namespace gpal::pnc::planning
