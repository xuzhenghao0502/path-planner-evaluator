/**
 * @file navigation.h
 * @brief 导航数据核心模块，负责管理导航相关的数据结构和操作
 * @details
 * 该模块用于存储和管理导航过程中所需的各种数据，包括车道段信息、局部路径参考线、区域信息、导航变换矩阵以及导航状态信息等。
 */

#pragma once

#include "local_router.h"

namespace gpal::pnc::planning {

using namespace std;
using gpal::pnc::planning::Status;

/**
 * @brief Navigation类
 * @details 用于表示导航信息，继承自StampedBase
 */
class Navigation : public StampedBase {
 public:
  friend class NavigationAdapter;
  explicit Navigation();
  ~Navigation();

  void clear();

  /**
   * @brief 设置导航信息是否有效
   * @details 将传入的布尔值赋值给成员变量b_valid_
   *
   * @param b_valid 导航信息是否有效
   */
  void setIsValid(const bool& b_valid) { b_valid_ = b_valid; }
  /**
   * @brief 获取导航信息是否有效
   * @details 返回成员变量b_valid_的值
   *
   * @return 导航信息是否有效
   */
  bool isValid() const { return b_valid_; }

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
   * @brief 获取车道段列表
   * @details 返回车道段列表的常量引用，不允许外部修改
   *
   * @return 车道段列表的常量引用
   */
  const std::vector<LocalRouteSegment>& getLanesSegments() const { return lanes_segments_; }
  /**
   * @brief 获取可修改的车道段列表
   * @details 返回车道段列表的指针，允许外部修改
   *
   * @return 车道段列表的指针
   */
  std::vector<LocalRouteSegment>* getMutableLanesSegments() { return &lanes_segments_; }

  /**
   * @brief 获取LocalRoute参考线列表
   * @details 返回LocalRoute参考线列表的常量引用，不允许外部修改
   *
   * @return LocalRoute参考线列表的常量引用
   */
  const std::vector<proto::LocalRouteReferenceLine>& getLocalRouteReferenceLines() const {
    return local_route_reference_lines_;
  }
  /**
   * @brief 获取可修改的LocalRoute参考线列表
   * @details 返回LocalRoute参考线列表的指针，允许外部修改
   *
   * @return LocalRoute参考线列表的指针
   */
  std::vector<proto::LocalRouteReferenceLine>* getMutableLocalRouteReferenceLines() {
    return &local_route_reference_lines_;
  }

  /**
   * @brief 获取区域列表
   * @details 返回区域列表的常量引用，不允许外部修改
   *
   * @return 区域列表的常量引用
   */
  const std::vector<PerceptionArea>& getAreas() const { return areas_; }
  /**
   * @brief 获取可修改的区域列表
   * @details 返回区域列表的指针，允许外部修改
   *
   * @return 区域列表的指针
   */
  std::vector<PerceptionArea>* mutableAreas() { return &areas_; }

  /**
   * @brief 获取LocalRoute道路信息
   * @details 返回LocalRoute道路信息的常量引用，不允许外部修改
   *
   * @return LocalRoute道路信息的常量引用
   */
  const proto::LocalRouteRoadInfo& getLocalRouteRoadInfo() const { return local_route_road_info_; }
  /**
   * @brief 设置LocalRoute道路信息
   * @details 将传入的LocalRoute道路信息赋值给成员变量local_route_road_info_
   *
   * @param local_route_road_info LocalRoute道路信息
   */
  void setLocalRouteRoadInfo(const proto::LocalRouteRoadInfo& local_route_road_info) {
    local_route_road_info_ = local_route_road_info;
  }

  /**
   * @brief 获取导航状态信息
   * @details 返回导航状态信息的常量引用，不允许外部修改
   *
   * @return 导航状态信息的常量引用
   */
  const proto::NavigationStatusInfo& getNavigationStatusInfo() const { return navigation_status_info_; }
  /**
   * @brief 设置导航状态信息
   * @details 将传入的导航状态信息赋值给成员变量navigation_status_info_
   *
   * @param navigation_status_info 导航状态信息
   */
  void setNavigationStatusInfo(const proto::NavigationStatusInfo& navigation_status_info) {
    navigation_status_info_ = navigation_status_info;
  }

  /**
   * @brief 获取可修改的导航数据坐标转换信息
   * @details 返回导航数据坐标转换信息的指针，允许外部修改
   *
   * @return 导航数据坐标转换信息的指针
   */
  std::tuple<Eigen::Matrix4d, Eigen::Matrix4d, math::Vec3d>* getMutableNavigationTf() { return &navigation_tf_; }
  /**
   * @brief 获取导航数据坐标转换信息
   * @details 返回导航数据坐标转换信息的常量引用，不允许外部修改
   *
   * @return 导航数据坐标转换信息的常量引用
   */
  const std::tuple<Eigen::Matrix4d, Eigen::Matrix4d, math::Vec3d>& getNavigationTf() const { return navigation_tf_; }

 protected:
  bool b_valid_ = false;                                                     ///< 导航信息是否有效
  std::string frame_id_ = "";                                                ///< 导航输出坐标系
  std::vector<LocalRouteSegment> lanes_segments_;                            ///< 车道段列表
  std::vector<proto::LocalRouteReferenceLine> local_route_reference_lines_;  ///< LocalRoute参考线列表
  std::vector<PerceptionArea> areas_;                                        ///< 区域列表
  proto::LocalRouteRoadInfo local_route_road_info_;                          ///< LocalRoute道路信息
  std::tuple<Eigen::Matrix4d, Eigen::Matrix4d, math::Vec3d> navigation_tf_;  ///< 导航数据坐标转换信息
  proto::NavigationStatusInfo navigation_status_info_;                       ///< 导航状态信息
};

}  // namespace gpal::pnc::planning
