/**
 * @file navigation.cpp
 * @brief 导航数据核心模块，负责管理导航相关的数据结构和操作
 * @details
 * 该模块用于存储和管理导航过程中所需的各种数据，包括车道段信息、局部路径参考线、区域信息、导航变换矩阵以及导航状态信息等。
 */

#include "navigation_data/navigation.h"

namespace gpal::pnc::planning {

/**
 * @brief Navigation构造函数
 * @details 初始化Navigation对象，并调用clear()函数清空所有数据
 *
 * @par 关键变量说明:
 * - b_valid_ (bool): 导航数据是否有效，初始化为false
 * - lanes_segments_ (std::vector<std::pair<std::string, std::vector<LocalRouteSegment>>>): 车道及其路段信息，初始化为空
 * - local_route_reference_lines_ (std::vector<LocalRoutePoint>): 局部路径参考线信息，初始化为空
 * - areas_ (std::vector<PerceptionArea>): 区域信息，初始化为空
 * - local_route_road_info_ (PerceptionRoadInfo): 局部路径道路信息，初始化为空
 * - navigation_tf_ (std::tuple<Eigen::Matrix4d, Eigen::Matrix4d, math::Vec3d>): 导航变换信息，初始化为单位矩阵和零向量
 * - navigation_status_info_ (NavigationStatusInfo): 导航状态信息，初始化为空
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用clear()函数;
 * :初始化b_valid_为false;
 * :初始化lanes_segments_为空;
 * :初始化local_route_reference_lines_为空;
 * :初始化areas_为空;
 * :初始化local_route_road_info_为空;
 * :初始化navigation_tf_为单位矩阵和零向量;
 * :初始化navigation_status_info_为空;
 * stop
 * @enduml
 *
 * @note 构造函数中调用clear()函数确保所有成员变量被正确初始化
 *
 * @warning 需确保所有成员变量在构造函数中被正确初始化
 */
Navigation::Navigation() { clear(); }

/**
 * @brief Navigation析构函数
 * @details 释放Navigation对象占用的资源，确保所有动态分配的内存被正确释放
 *
 * @par 关键变量说明:
 * - b_valid_ (bool): 导航数据是否有效，析构时自动释放
 * - lanes_segments_ (std::vector<std::pair<std::string, std::vector<LocalRouteSegment>>>):
 * 车道及其路段信息，析构时自动释放
 * - local_route_reference_lines_ (std::vector<LocalRoutePoint>): 局部路径参考线信息，析构时自动释放
 * - areas_ (std::vector<PerceptionArea>): 区域信息，析构时自动释放
 * - local_route_road_info_ (PerceptionRoadInfo): 局部路径道路信息，析构时自动释放
 * - navigation_tf_ (std::tuple<Eigen::Matrix4d, Eigen::Matrix4d, math::Vec3d>): 导航变换信息，析构时自动释放
 * - navigation_status_info_ (NavigationStatusInfo): 导航状态信息，析构时自动释放
 *
 * @par 流程图:
 * @startuml
 * start
 * :释放b_valid_内存;
 * :释放lanes_segments_内存;
 * :释放local_route_reference_lines_内存;
 * :释放areas_内存;
 * :释放local_route_road_info_内存;
 * :释放navigation_tf_内存;
 * :释放navigation_status_info_内存;
 * stop
 * @enduml
 *
 * @note 析构函数中所有成员变量均为栈上对象，无需手动释放内存
 *
 * @warning 确保Navigation对象生命周期结束时调用析构函数
 */
Navigation::~Navigation() {}

/**
 * @brief 清空Navigation对象的所有数据
 * @details 重置所有成员变量为初始状态，确保对象可以重新使用
 *
 * @par 关键变量说明:
 * - b_valid_ (bool): 导航数据是否有效，重置为false
 * - lanes_segments_ (std::vector<std::pair<std::string, std::vector<LocalRouteSegment>>>):
 * 车道及其路段信息，清空所有条目
 * - local_route_reference_lines_ (std::vector<LocalRoutePoint>): 局部路径参考线信息，清空所有条目
 * - areas_ (std::vector<PerceptionArea>): 区域信息，清空所有条目
 * - local_route_road_info_ (PerceptionRoadInfo): 局部路径道路信息，重置为空
 * - navigation_tf_ (std::tuple<Eigen::Matrix4d, Eigen::Matrix4d, math::Vec3d>): 导航变换信息，重置为单位矩阵和零向量
 * - navigation_status_info_ (NavigationStatusInfo): 导航状态信息，重置为空
 *
 * @par 流程图:
 * @startuml
 * start
 * :重置b_valid_为false;
 * :清空lanes_segments_;
 * :清空local_route_reference_lines_;
 * :清空areas_;
 * :重置local_route_road_info_为空;
 * :重置navigation_tf_为单位矩阵和零向量;
 * :重置navigation_status_info_为空;
 * stop
 * @enduml
 *
 * @note 该函数用于重置对象状态，通常在对象重用或销毁前调用
 *
 * @warning 调用该函数后，所有存储的数据将被清空，需谨慎使用
 */
void Navigation::clear() {
  b_valid_ = false;
  lanes_segments_.clear();
  local_route_reference_lines_.clear();
  areas_.clear();
  local_route_road_info_.Clear();
  navigation_tf_ = std::make_tuple(Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity(), math::Vec3d(0, 0, 0));
  navigation_status_info_.Clear();
}

}  // namespace gpal::pnc::planning
