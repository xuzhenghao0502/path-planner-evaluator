/**
 * @file perception_road_structure.cpp
 * @brief 感知道路结构实现文件
 * @details
 * 该文件实现了感知道路结构的核心功能，包括道路结构的初始化、清空、车道和区域信息的获取与更新等。感知道路结构用于存储和管理车辆感知到的道路信息，包括车道、车道段、特殊区域等。
 */

#include "navigation_data/perception_road_structure.h"

namespace gpal::pnc::planning {

/**
 * @brief PerceptionRoadStructure构造函数
 * @details 初始化PerceptionRoadStructure对象，并调用clear()函数清空所有数据
 *
 * @par 关键变量说明:
 * - ego_lane_id_ (std::string): 自车所在车道ID，初始化为空字符串
 * - ego_lane_segment_id_ (std::string): 自车所在车道段ID，初始化为空字符串
 * - lane_map_ (std::map<std::string, Lane>): 车道信息映射，键为车道ID，值为车道信息，初始化为空
 * - area_map_ (std::map<std::string, PerceptionArea>): 特殊区域信息映射，键为区域ID，值为区域信息，初始化为空
 * - ego_road_info_ (PerceptionRoadInfo): 自车所在道路信息，初始化为空
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用clear()函数;
 * :初始化ego_lane_id_为空字符串;
 * :初始化ego_lane_segment_id_为空字符串;
 * :初始化lane_map_为空;
 * :初始化area_map_为空;
 * :初始化ego_road_info_为空;
 * stop
 * @enduml
 *
 * @note 构造函数中调用clear()函数确保所有成员变量被正确初始化
 *
 * @warning 需确保StampedBase基类已正确初始化
 */
PerceptionRoadStructure::PerceptionRoadStructure() { clear(); }

/**
 * @brief PerceptionRoadStructure析构函数
 * @details 释放PerceptionRoadStructure对象占用的资源，确保所有动态分配的内存被正确释放
 *
 * @par 关键变量说明:
 * - ego_lane_id_ (std::string): 自车所在车道ID，析构时自动释放
 * - ego_lane_segment_id_ (std::string): 自车所在车道段ID，析构时自动释放
 * - lane_map_ (std::map<std::string, Lane>): 车道信息映射，键为车道ID，值为车道信息，析构时自动释放
 * - area_map_ (std::map<std::string, PerceptionArea>): 特殊区域信息映射，键为区域ID，值为区域信息，析构时自动释放
 * - ego_road_info_ (PerceptionRoadInfo): 自车所在道路信息，析构时自动释放
 *
 * @par 流程图:
 * @startuml
 * start
 * :释放ego_lane_id_内存;
 * :释放ego_lane_segment_id_内存;
 * :释放lane_map_内存;
 * :释放area_map_内存;
 * :释放ego_road_info_内存;
 * stop
 * @enduml
 *
 * @note 析构函数中所有成员变量均为栈上对象，无需手动释放内存
 *
 * @warning 确保PerceptionRoadStructure对象生命周期结束时调用析构函数
 */
PerceptionRoadStructure::~PerceptionRoadStructure() {}

/**
 * @brief 清空PerceptionRoadStructure对象的所有数据
 * @details 重置所有成员变量为初始状态，确保对象可以重新使用
 *
 * @par 关键变量说明:
 * - ego_lane_id_ (std::string): 自车所在车道ID，重置为空字符串
 * - ego_lane_segment_id_ (std::string): 自车所在车道段ID，重置为空字符串
 * - lane_map_ (std::map<std::string, Lane>): 车道信息映射，键为车道ID，值为车道信息，清空所有条目
 * - area_map_ (std::map<std::string, PerceptionArea>): 特殊区域信息映射，键为区域ID，值为区域信息，清空所有条目
 * - ego_road_info_ (PerceptionRoadInfo): 自车所在道路信息，重置为空
 *
 * @par 流程图:
 * @startuml
 * start
 * :重置ego_lane_id_为空字符串;
 * :重置ego_lane_segment_id_为空字符串;
 * :清空lane_map_;
 * :清空area_map_;
 * :重置ego_road_info_为空;
 * stop
 * @enduml
 *
 * @note 该函数用于重置对象状态，通常在对象重用或销毁前调用
 *
 * @warning 调用该函数后，所有存储的数据将被清空，需谨慎使用
 */
void PerceptionRoadStructure::clear() {
  ego_lane_id_ = "";
  ego_lane_segment_id_ = "";
  lane_map_.clear();
  area_map_.clear();
  ego_road_info_.clear();
}

}  // namespace gpal::pnc::planning
