/**
 * @file local_map.h
 * @brief 局部地图数据核心模块，负责管理局部地图中的道路段和路口信息
 * @details 该模块用于存储和管理局部地图中的道路段和路口信息，并提供快速查询功能，支持导航和路径规划。
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "math/aaboxkdtree2d.h"
#include "math/arrow2d.h"
#include "math/vec2d.h"
#include "navigation_data/common_typedefs.h"
#include "util/base_struct.h"

namespace gpal::pnc::planning {

struct LocalMapLaneMarking {
  using LineType = PerceptionLaneMarking::LineType;
  using LineColor = PerceptionLaneMarking::LineColor;

  std::string id = "";                              ///< 车道线ID
  LineType line_type = LineType::kTypeInvalid;      ///< 车道线线型
  LineColor line_color = LineColor::kColorInvalid;  ///< 车道线颜色
  std::vector<math::Vec3d> line_points;             ///< map坐标系下车道线散点，米，按车道行驶方向单向排序
  float confidence = 0.0;                           ///< 车道线置信度，范围：[0.0, 1.0]

  /**
   * @brief 重置车道线标记数据至初始状态
   * @details 该方法用于清空车道线标记对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - line_type: 设为kTypeInvalid (无效线型)
   * - line_color: 设为kColorInvalid (无效颜色)
   *
   * @par 典型使用场景:
   * 1. 车道线跟踪失效时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 车道信息重新加载时
   *
   * @note 该方法仅重置当前对象属性，不影响关联的车道数据
   * @warning 调用后需重新获取最新的车道线属性信息
   */
  void clear() {
    id = "";
    line_type = LineType::kTypeInvalid;
    line_color = LineColor::kColorInvalid;
    line_points.clear();
    confidence = 0.0;
  }
};

struct LocalMapLane {
  std::string lane_id;                              ///< 车道ID
  int lane_seq = 0;                                 ///< 车道序号，从左向右依次为1,2...
  ArrowType arrow_type = ArrowType::kArrowInvalid;  ///< 方向箭头
  float lane_width = 0.0;                           ///< 车道宽度（平均值），单位：m
  LocalMapLaneMarking left_lane_marking;            ///< 左侧车道线
  LocalMapLaneMarking right_lane_marking;           ///< 右侧车道线

  /**
   * @brief 重置车道数据至初始状态
   * @details 该方法用于清空车道对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - lane_id: 清空字符串 (长度归零)
   * - lane_seq: 重置为0 (序号归零)
   * - arrow_type: 设为kArrowInvalid (无效箭头类型)
   * - lane_width: 重置为0.0 (宽度归零)
   * - left_lane_marking/right_lane_marking: 调用clear()方法
   *
   * @par 典型使用场景:
   * 1. 车道跟踪失效时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 局部地图重建时
   *
   * @note 该方法会递归清理左右车道线标记数据
   * @warning 调用后将丢失车道拓扑关联，需重新建立与道路段的连接
   */
  void clear() {
    lane_id = "";
    lane_seq = 0;
    arrow_type = ArrowType::kArrowInvalid;
    lane_width = 0.0;
    left_lane_marking.clear();
    right_lane_marking.clear();
  }
};

struct LocalMapJunction {
  std::string junction_id;                        ///< 路口ID
  std::vector<std::string> in_road_segment_ids;   ///< 路口的入度集合
  std::vector<std::string> out_road_segment_ids;  ///< 路口的出度集合
  std::vector<math::Vec3d>
      polygon_points;  ///< 路口面几何信息 （逆时针排序、首尾闭合的polygon,lla类型坐标,不少于4个点）

  /**
   * @brief 重置路口数据至初始状态
   * @details 该方法用于清空路口对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - junction_id: 清空字符串 (长度归零)
   * - in_road_segment_ids/out_road_segment_ids: 清空道路段ID数组
   * - polygon_points: 清空几何顶点数组 (size=0)
   *
   * @par 典型使用场景:
   * 1. 路口跟踪失效时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 局部地图拓扑重建时
   *
   * @note 该方法仅清理当前对象数据，不影响关联的道路段
   * @warning 调用后需重新建立路口与道路段的拓扑关联
   */
  void clear() {
    junction_id = "";
    in_road_segment_ids.clear();
    out_road_segment_ids.clear();
    polygon_points.clear();
  }
};

struct LocalMapStopLine {
  std::string id = "";                                        ///< 停止线id
  std::string related_junction_id = "";                       ///< 关联的路口id
  math::Vec3d point;                                          ///< 停止线抽象为单点的坐标
  RoadDirection road_direction = RoadDirection::kStartToEnd;  ///< road存在双向通行场景，故需表明停止线关联的方向

  /**
   * @brief 重置停止线数据至初始状态
   * @details 该方法用于清空停止线对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - id/related_junction_id: 清空字符串 (长度归零)
   * - point: 重置坐标值为(0.0, 0.0, 0.0)
   * - road_direction: 设为kStartToEnd (默认道路方向)
   *
   * @par 典型使用场景:
   * 1. 停止线跟踪失效时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 道路方向属性变更时
   *
   * @note 该方法仅清理当前对象数据，不影响关联的路口信息
   * @warning 调用后需重新建立停止线与路口的拓扑关联
   */
  void clear() {
    id = "";
    related_junction_id = "";
    point.set_x(0.0);
    point.set_y(0.0);
    point.set_z(0.0);
    road_direction = RoadDirection::kStartToEnd;
  }
};

/**
 * @brief LocalMapRoadSegment类
 * @details 用于表示局部地图中的道路段信息，包含道路段的基本属性、车道信息、拓扑关系等
 */
class LocalMapRoadSegment {
 public:
  enum class DrivingDirection {
    kDrivingUnknown = 0,   ///< 未知
    kDrivingStraight = 1,  ///< 直行
    kDrivingLeft = 2,      ///< 左转
    kDrivingRight = 3,     ///< 右转
    kDrivingUturn = 4,     ///< 掉头
  };

 public:
  LocalMapRoadSegment();
  virtual ~LocalMapRoadSegment();
  bool init();
  /**
   * @brief 获取初始化状态
   * @details 返回成员变量is_inited_的值
   *
   * @return 初始化状态
   */
  bool isInited() { return is_inited_; }
  void clear();

  // 用于RoadSegment检索
  double DistanceSquareTo(const math::Vec2d& point) const;
  double DistanceSquareTo(const math::Vec2d& point, double heading) const;

  // 用于成员变量访问
  std::string& getMutableRoadSegmentId() { return road_segment_id_; }        ///< 获取可修改的道路段ID
  const std::string& getRoadSegmentId() const { return road_segment_id_; }   ///< 获取道路段ID
  std::string& getMutableRoadId() { return road_id_; }                       ///< 获取可修改的道路ID
  const std::string& getRoadId() const { return road_id_; }                  ///< 获取道路ID
  RoadDirection& getMutableRoadDirection() { return road_direction_; }       ///< 获取可修改的道路方向
  const RoadDirection& getRoadDirection() const { return road_direction_; }  ///< 获取道路方向
  DrivingDirection& getMutableDrivingDirectionInRouting() {
    return driving_direction_in_routing_;
  }  ///< 获取可修改的导航行驶方向
  const DrivingDirection& getDrivingDirectionInRouting() const {
    return driving_direction_in_routing_;
  }                                                                                  ///< 获取导航行驶方向
  RoadClass& getMutableRoadClass() { return road_class_; }                           ///< 获取可修改的道路等级
  const RoadClass& getRoadClass() const { return road_class_; }                      ///< 获取道路等级
  std::vector<RoadType>& getMutableRoadTypes() { return road_types_; }               ///< 获取可修改的道路类型数组
  const std::vector<RoadType>& getRoadTypes() const { return road_types_; }          ///< 获取道路类型数组
  int32_t& getMutableSpeedLimit() { return speed_limit_; }                           ///< 获取可修改的速度限制
  const int32_t& getSpeedLimit() const { return speed_limit_; }                      ///< 获取速度限制
  float& getMutableLength() { return length_; }                                      ///< 获取可修改的道路长度
  const float& getLength() const { return length_; }                                 ///< 获取道路长度
  std::vector<math::Vec3d>& getMutablePoints() { return points_; }                   ///< 获取可修改的道路中心线点集
  const std::vector<math::Vec3d>& getPoints() const { return points_; }              ///< 获取道路中心线点集
  bool& getMutableIsLaneInfoValid() { return is_lane_info_valid_; }                  ///< 获取可修改的车道信息有效性
  const bool& getIsLaneInfoValid() const { return is_lane_info_valid_; }             ///< 获取车道信息有效性
  int32_t& getMutableLaneNum() { return lane_num_; }                                 ///< 获取可修改的车道数
  const int32_t& getLaneNum() const { return lane_num_; }                            ///< 获取车道数量
  std::vector<LocalMapLane>& getMutableLanes() { return lanes_; }                    ///< 获取可修改的车道数组
  const std::vector<LocalMapLane>& getLanes() const { return lanes_; }               ///< 获取车道数组
  std::vector<LocalMapStopLine>& getMutableStopLines() { return stop_lines_; }       ///< 获取可修改的停止线数组
  const std::vector<LocalMapStopLine>& getStopLines() const { return stop_lines_; }  ///< 获取停止线数组
  CrossInOutType& getMutableCrossInOutType() { return cross_in_out_type_; }          ///< 获取可修改的交叉口进出类型
  const CrossInOutType& getCrossInOutType() const { return cross_in_out_type_; }     ///< 获取交叉口进出类型

  std::vector<std::string>& getMutableInRoadSegmentIds() {
    return in_road_segment_ids_;
  }  ///< 获取可修改的进入道路段ID列表
  const std::vector<std::string>& getInRoadSegmentIds() const {
    return in_road_segment_ids_;
  }  ///< 获取进入道路段ID列表
  std::vector<std::string>& getMutableOutRoadSegmentIds() {
    return out_road_segment_ids_;
  }  ///< 获取可修改的驶出道路段ID列表
  const std::vector<std::string>& getOutRoadSegmentIds() const {
    return out_road_segment_ids_;
  }  ///< 获取驶出道路段ID列表
  std::vector<std::string>& getMutableInRoadSegmentThroughJunctionIds() {
    return in_road_segment_through_junction_ids;
  }  ///< 获取可修改的跨越路口的进入道路段ID列表
  const std::vector<std::string>& getInRoadSegmentThroughJunctionIds() const {
    return in_road_segment_through_junction_ids;
  }  ///< 获取跨越路口的进入道路段ID列表
  std::vector<std::string>& getMutableOutRoadSegmentThroughJunctionIds() {
    return out_road_segment_through_junction_ids;
  }  ///< 获取可修改的跨越路口的驶出道路段ID列表
  const std::vector<std::string>& getOutRoadSegmentThroughJunctionIds() const {
    return out_road_segment_through_junction_ids;
  }                                                                                  ///< 获取跨越路口的驶出道路段ID列表
  std::string& getMutableLeftRoadSegmentId() { return left_road_segment_id_; }       ///< 获取可修改的左侧道路段ID
  const std::string& getLeftRoadSegmentId() const { return left_road_segment_id_; }  ///< 获取左侧道路段ID
  std::string& getMutableRightRoadSegmentId() { return right_road_segment_id_; }     ///< 获取可修改的右侧道路段ID
  const std::string& getRightRoadSegmentId() const { return right_road_segment_id_; }  ///< 获取右侧道路段ID
  std::vector<std::string>& getMutableRelatedJunctionIds() {
    return related_junction_ids_;
  }  ///< 获取可修改的关联路口ID列表
  const std::vector<std::string>& getRelatedJunctionIds() const {
    return related_junction_ids_;
  }  ///< 获取关联路口ID列表

  math::AABox2d& getMutableAABox() { return aabox_; }       ///< 获取可修改的AABB box
  const math::AABox2d& getAABox() const { return aabox_; }  ///< 获取AABB box
  std::vector<math::LineSegment2d>& getMutableLineSegmentArray() {
    return line_segment_array_;
  }  ///< 获取可修改的线段数组
  const std::vector<math::LineSegment2d>& getLineSegmentArray() const { return line_segment_array_; }  ///< 获取线段数组
  std::vector<float>& getMutableLengthArray() { return length_array_; }       ///< 获取可修改的长度数组
  const std::vector<float>& getLengthArray() const { return length_array_; }  ///< 获取长度数组

 private:
  int findLineSegmentIndex(const math::Vec2d& point) const;

 protected:
  bool is_inited_ = false;  ///< 初始化状态
  std::string road_segment_id_ =
      "";  ///< 道路区间ID，（道路区间是道路上按照车道数量变化和车道线属性变化纵向切割打断形成的区间段）
  std::string road_id_ = "";                                                           ///< 道路区间所属道路ID
  RoadDirection road_direction_ = RoadDirection::kStartToEnd;                          ///< 道路通行方向
  DrivingDirection driving_direction_in_routing_ = DrivingDirection::kDrivingUnknown;  ///< 道路在导航路线中的转向信息
  RoadClass road_class_ = RoadClass::kRoadInvalid;                                     ///< 道路等级
  std::vector<RoadType> road_types_;                                                   ///< 道路类型数组
  int32_t speed_limit_ = 0;                                                            ///< 道路级限速 单位：km/h
  float length_ = 0.0;                                                                 ///< 道路长度，单位：m
  std::vector<math::Vec3d> points_;                                                    ///< 道路中心线形点序列(lla坐标)
  bool is_lane_info_valid_;  ///< 是否存在车道级数据，若为false，则车道相关信息为空
  int32_t lane_num_;         ///< 车道个数（SD数据中能获取车道数量，但可能获取不到车道详细信息）
  std::vector<LocalMapLane>
      lanes_;  ///< 车道数组，按照通行方向从左往右排序，若未知则数组为空（最左侧车道对应的为数组下标为0的车道）
  std::vector<LocalMapStopLine> stop_lines_;                          ///< 停止线数组，可能不同方向有多个
  CrossInOutType cross_in_out_type_ = CrossInOutType::kCrossInvalid;  ///< 进入退出路口类型
  // 拓扑相关信息
  std::vector<std::string> in_road_segment_ids_;   ///< 道路区间的所有进入的road_seg_id集合
  std::vector<std::string> out_road_segment_ids_;  ///< 道路区间的所有脱出的road_seg_id集合
  std::vector<std::string>
      in_road_segment_through_junction_ids;  ///< 道路区间的所有跨越路口的进入的road_seg_id集合(当roadsegment非路口时，则字段为空)
  std::vector<std::string>
      out_road_segment_through_junction_ids;  ///< 道路区间的所有跨越路口的脱出的road_seg_id集合(当roadsegment非路口时，则字段为空）
  /* 复合路口示意图
   *             |              ^
   *             |              |
   *           road4(下行)     road5(上行)
   *             |              |
   *             |              |
   *<---road1----<-----road2----<-----road3-----
   *             |              ^
   *             |              |
   *           road9(下行)     road10(上行)
   *             |              |
   *---road6----->----road7----->---road8----->
   *             |              ^
   *             |              |
   *           road11(下行)    road12(上行)
   *             |              |
   *             |              |
   */
  // 说明：上图中road2、road9、road7和road10是路口内虚拟的无车道线的道路，旨在表达拓扑连接关系；
  //           road12、road8、road3、road5、road4、road1、road6和road11是现实中存在车道线的路口前进入道路或过路口后脱出道路。
  // 以road12 为例，out_road_segment_ids（road8,road10）
  //               out_road_segment_through_junction_ids (road8,road5,road1,road11)

  std::string
      left_road_segment_id_;  ///< 左侧RoadSegment(为空表示未知或没有),有多个时选取离roadsegment起始node最近的一个
  std::string
      right_road_segment_id_;  ///< 右侧RoadSegment(为空表示未知或没有),有多个时选取离roadsegment起始node最近的一个
  std::vector<std::string> related_junction_ids_;  ///< 关联的路口ID集合（为空表示无关联）
  // 用于Road搜索
  math::AABox2d aabox_;                                  ///< AABB box
  std::vector<math::LineSegment2d> line_segment_array_;  ///< 线段数组
  std::vector<float> length_array_;                      ///< 长度数组
};

/**
 * @brief LocalMap类
 * @details 用于表示局部地图信息，包含道路段、路口等地图元素，并支持基于KDTree的快速检索
 */
class LocalMap : public StampedBase {
 public:
  enum class MapErrorCode {
    kOK = 0,                  ///< 无异常
    kNoSdRoute = 1,           ///< 无SD路线输入
    kNoMapData = 2,           ///< 无地图数据
    kNoLocalizationData = 3,  ///< 无定位点输入
  };

  using LocalMapRoadSegmentPtr = std::shared_ptr<LocalMapRoadSegment>;
  using LocalMapJunctionPtr = std::shared_ptr<LocalMapJunction>;

 public:
  friend class LocalMapAdapter;
  LocalMap();
  /**
   * @brief 默认析构函数
   */
  virtual ~LocalMap() = default;
  bool init();
  bool init(const math::AABoxKDTree2dParams& tree_params);
  /**
   * @brief 获取初始化状态
   * @details 返回成员变量is_inited_的值
   *
   * @return 初始化状态
   */
  bool isInited() { return is_inited_; }
  bool isValid();
  void clear();

  // 用于地图元素检索
  bool getNearestRoadSegment(const math::Arrow2d& pose, LocalMapRoadSegment** nearest_road_seg);
  std::string getNearestRoadSegmentId(const math::Arrow2d& pose);

  // 用于成员变量访问
  MapErrorCode& getMutableMapErrorCode() { return map_error_code_; }       ///< 获取可修改的地图错误码
  const MapErrorCode& getMapErrorCode() const { return map_error_code_; }  ///< 获取地图错误码
  int32_t& getMutableMapRange() { return map_range_; }                     ///< 获取可修改的地图数据半径
  const int32_t& getMapRange() const { return map_range_; }                ///< 获取地图数据半径
  std::unordered_map<std::string, LocalMapRoadSegmentPtr>* getMutableRoadSegmentMap() {
    return &road_segment_map_;
  }  ///< 获取可修改的道路段映射表
  const std::unordered_map<std::string, LocalMapRoadSegmentPtr>& getRoadSegmentMap() const {
    return road_segment_map_;
  }  ///< 获取道路段映射表
  std::unordered_map<std::string, LocalMapJunctionPtr>* getMutableJunctionMap() {
    return &junction_map_;
  }  ///< 获取可修改的路口映射表
  const std::unordered_map<std::string, LocalMapJunctionPtr>& getJunctionMap() const {
    return junction_map_;
  }  ///< 获取路口映射表
  math::AABoxKDTree2dParams* getMutableRoadSegmentTreeParams() {
    return &road_segment_tree_params_;
  }  ///< 获取可修改的KDTree参数
  const math::AABoxKDTree2dParams& getRoadSegmentTreeParams() const {
    return road_segment_tree_params_;
  }  ///< 获取KDTree参数
  std::shared_ptr<math::AABoxKDTree2d<LocalMapRoadSegment>> getMutableRoadSegmentTree() {
    return road_segment_tree_;
  }  ///< 获取可修改的KDTree
  const std::shared_ptr<math::AABoxKDTree2d<LocalMapRoadSegment>>& getRoadSegmentTree() const {
    return road_segment_tree_;
  }                                                                            ///< 获取KDTree
  std::string& getMutableEgoRoadSegmentId() { return ego_road_seg_id_; }       ///< 获取可修改的自车所在RoadSegment ID
  const std::string& getEgoRoadSegmentId() const { return ego_road_seg_id_; }  ///< 获取自车所在RoadSegment ID

 protected:
  bool is_inited_ = false;                                  ///< 初始化状态
  MapErrorCode map_error_code_ = MapErrorCode::kNoMapData;  ///< LocalMap错误码
  int32_t map_range_ = 0;                                   ///< 地图数据半径 单位：m
  std::unordered_map<std::string, LocalMapRoadSegmentPtr>
      road_segment_map_;  ///< <LocalMapRoadSegment ID, LocalMapRoadSegment>
  std::unordered_map<std::string, LocalMapJunctionPtr> junction_map_;            ///< <Junction ID, Junction>
  math::AABoxKDTree2dParams road_segment_tree_params_;                           ///< KDTree参数
  std::shared_ptr<math::AABoxKDTree2d<LocalMapRoadSegment>> road_segment_tree_;  ///< 基于RoadSegment构建的KDTree
  std::string ego_road_seg_id_ = "";                                             ///< 自车所在的RoadSegment ID
};

}  // namespace gpal::pnc::planning
