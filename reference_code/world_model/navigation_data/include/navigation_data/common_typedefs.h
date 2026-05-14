/**
 * @file common_typedefs.h
 * @brief 导航数据通用类型定义文件
 * @details 该文件定义了导航数据模块中使用的通用类型和结构体，包括道路、车道、标线、区域等相关的数据结构。
 */

#pragma once

#include <string>
#include <vector>

#include "gpal-interface/navigation/navigation_result.pb.h"
#include "gpal-interface/road_cognition/env_road_cognition.pb.h"
#include "math/vec2d.h"

namespace gpal::pnc::planning {

using LineSourceType = proto::LocalRouteReferenceLine::LineSource::LineSourceType;

enum class RoadClass {    ///< 道路等级
  kRoadInvalid = 0,       ///< 无效
  kRoadExpress = 1,       ///< 高速
  kRoadUrbanExpress = 2,  ///< 都市快速路
  kRoadNational = 3,      ///< 国道
  kRoadProvincial = 4,    ///< 省道
  kRoadCounty = 5,        ///< 县道
  kRoadVillage = 6,       ///< 乡镇村路
  kRoadPedestrian = 7,    ///< 行人道路
  kRoadFerry = 8,         ///< 轮渡
  kRoadClass9 = 9,        ///< 九级路
  kRoadOther = 20,        ///< 其他
};

enum class RoadType {  ///< 道路类型
  kRoadTypeNone = 0,   ///< 无属性
  kRoundabout = 1,     ///< 环岛
  kServiceArea = 2,    ///< 服务区
  kBridge = 3,         ///< 固定桥
  kSecondary = 4,      ///< 辅路
  kMainRoad = 5,       ///< 主路
  kRamp = 6,           ///< 匝道
  kTunnel = 7,         ///< 隧道
  kElevated = 8,       ///< 高架路
  kJunction = 9,       ///< 路口
};

enum class RoadDirection {
  kStartToEnd = 0,     ///< 正向
  kEndToStart = 1,     ///< 逆向
  kBothDirection = 2,  ///< 双向通行
};

enum class CrossInOutType {
  kCrossInvalid = 0,  ///< 无效类型
  kCrossIn = 1,       ///< 进入路口类型
  kCrossOut = 2,      ///< 退出路口类型
  kCrossInOut = 3     ///< 进入+退出路口类型
};

enum class StopLineType {
  kTypeInvalid = 0,    ///< 无效类型
  kTypeNormal = 1,     ///< 常规停止线
  kTypeGiveway = 2,    ///< 停车让行线
  kTypeSlowDown = 3,   ///< 减速让行线
  kTypeVirtual = 4,    ///< 虚拟停止线
  kTypeBikeCross = 5,  ///< 非机动车横穿线
  kTypeUnknown = 99    ///< 未知类型
};

enum class ArrowType {
  kArrowInvalid = 0,                 ///< 无效
  kArrowForwardOnly = 1,             ///< 直行
  kArrowLeftOnly = 2,                ///< 左转
  kArrowRightOnly = 3,               ///< 右转
  kArrowUTurnOnly = 4,               ///< 掉头
  kArrowNoLeft = 5,                  ///< 禁止左转
  kArrowNoRight = 6,                 ///< 禁止右转
  kArrowNoUTurn = 7,                 ///< 禁止掉头
  kArrowForwardLeft = 10,            ///< 直行+左转
  kArrowForwardRight = 11,           ///< 直行+右转
  kArrowLeftUTurn = 12,              ///< 左转+掉头
  kArrowForwardUTurn = 13,           ///< 直行+掉头
  kArrowLeftRight = 14,              ///< 左转+右转
  kArrowRightUTurn = 15,             ///< 右转+掉头
  kArrowForwardLeftRight = 21,       ///< 直行+左转+右转
  kArrowForwardLeftUturn = 22,       ///< 直行+左转+掉头
  kArrowForwardLeftRightUturn = 23,  ///< 直行+左转+右转+掉头
  kArrowChangeToLeft = 30,           ///< 向左合流箭头
  kArrowChangeToRight = 31,          ///< 向右合流箭头
  kArrowPedWarning = 32,             ///< 人行横道预告
  kArrowGiveway = 33,                ///< 让行标志
  kArrowBikeDirection = 34,          ///< 非机动车行驶方向
  kArrowUnknown = 66,                ///< 未知
};

struct PerceptionLaneMarking {
  std::string id = "";  ///< 车道线跟踪ID, 全局唯一
  enum class LineType {
    kTypeInvalid = 0,               ///< 无效
    kTypeSingleSolid = 1,           ///< 单实线
    kTypeSingleDash = 2,            ///< 单虚线
    kTypeDualSolid = 3,             ///< 双实线
    kTypeDualDash = 4,              ///< 双虚线
    kTypeDashSolid = 5,             ///< 左虚右实线
    kTypeSolidDash = 6,             ///< 左实右虚线
    kTypeTidal = 7,                 ///< 潮汐车道线
    kTypeGuideLine = 8,             ///< 导流线
    kTypeCurb = 9,                  ///< 路沿
    kTypeDecelerationMarking = 10,  ///< 减速标线，鱼骨线
    kTypeWaitLine = 11,             ///< 待行区左右边线
    kTypeVariable = 12,             ///< 可变车道线
    kTypeIgnore = 13,               ///< 忽略线，如对向车道线、与当前车道有实物隔离的同向车道线
    kTypeUnknown = 30               ///< 未知
  };
  LineType line_type = LineType::kTypeInvalid;  ///< 车道线线型
  enum class LineColor {
    kColorInvalid = 0,  ///< 无效
    kColorWhite = 1,    ///< 白色，包括一些特殊颜色线
    kColorYellow = 2,   ///< 黄色
    kColorBlue = 3,     ///< 蓝色
    kColorOrange = 4,   ///< 橙色
    kColorRed = 5,      ///< 红色
    kColorUnknown = 10  ///< 未知
  };
  LineColor line_color = LineColor::kColorInvalid;  ///< 车道线颜色
  std::vector<math::Vec3d> line_points;             ///< 全局坐标系下车道线散点，米，按车道行驶方向单向排序
  float confidence = 0.0;                           ///< 车道线置信度，范围：[0.0, 1.0]
  int32_t index = 0;  ///< 车道线标签，0：未定义；1 左侧最近第一车道线，2 左侧最近第二车道线，以此类推；-1
                      ///< 右侧最近第一车道线，-2 右侧最近第二车道线，以此类推
  float start_offset = 0.0;  ///< 车道线起点在自车坐标系下的纵向坐标，米，只在自车所在LaneMarkingGourp中有效
  float end_offset = 0.0;    ///< 车道线终点在自车坐标系下的纵向坐标，米，只在自车所在LaneMarkingGourp中有效

  /**
   * @brief 重置感知道路标线至初始状态
   * @details 该方法用于清空车道线对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - id: 清空字符串 (长度归零)
   * - line_type: 设为kTypeInvalid (无效线型)
   * - line_color: 设为kColorInvalid (无效颜色)
   * - line_points: 清空点集 (size=0)
   * - confidence: 重置为0.0 (置信度归零)
   * - index: 重置为0 (索引归零)
   * - start_offset/end_offset: 重置为0.0 (纵向坐标归零)
   *
   * @par 典型使用场景:
   * 1. 对象重用前清理残留数据
   * 2. 感知数据失效时重置状态
   * 3. 跨周期数据更新前的预处理
   *
   * @note 该方法不会释放内存，仅重置成员变量值
   * @warning 调用后对象将失去原有数据关联性，需谨慎使用
   */
  void clear() {
    id = "";
    line_type = LineType::kTypeInvalid;
    line_color = LineColor::kColorInvalid;
    line_points.clear();
    confidence = 0.0;
    index = 0;
    start_offset = 0.0;
    end_offset = 0.0;
  }
};

struct PerceptionMergeForkPoint {
  std::string id = "";  ///< 汇入汇出点ID，全局唯一
  enum class MergeForkType {
    kTypeInvalid = 0,  ///< 无效
    kTypeMerge = 1,    ///< 汇入
    kTypeFork = 2,     ///< 汇出
    kTypeUnknown = 10  ///< 未知
  };
  MergeForkType type = MergeForkType::kTypeInvalid;  ///< 汇入汇出类型
  math::Vec3d point;                                 ///< 位置点坐标，米

  struct RelatedLaneMarking {
    std::string id = "";        ///< 关联车道线ID
    math::Vec3d related_point;  ///< 关联车道线上的对应点坐标，米

    /**
     * @brief 重置关联车道线数据至初始状态
     * @details 该方法用于清空关联车道线对象的所有属性，使其恢复到可重新使用的初始状态
     *
     * @par 成员变量重置说明:
     * - id: 清空字符串 (长度归零)
     * - related_point: 坐标归零 (x,y,z=0)
     *
     * @par 典型使用场景:
     * 1. 关联车道线跟踪失效时
     * 2. 跨感知周期数据更新前
     * 3. 父级MergeForkPoint对象重置时
     *
     * @note 该方法仅清理当前对象数据，不影响原始车道线数据
     * @warning 调用后需重新建立与车道线对象的关联关系
     */
    void clear() {
      id = "";
      related_point.set_x(0);
      related_point.set_y(0);
      related_point.set_z(0);
    }
  };
  std::vector<RelatedLaneMarking> related_lane_markings;  ///< 关联车道线ID数组

  /**
   * @brief 重置汇入汇出点数据至初始状态
   * @details 该方法用于清空汇入汇出点对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - id: 清空字符串 (长度归零)
   * - type: 设为kTypeInvalid (无效类型)
   * - point: 坐标归零 (x,y,z=0)
   * - related_lane_markings: 清空关联车道线数组 (size=0)
   *
   * @par 典型使用场景:
   * 1. 跨感知周期数据更新前
   * 2. 汇入汇出点跟踪丢失时
   * 3. 导航决策重置时
   *
   * @note 该方法会递归调用关联车道线元素的clear()
   * @warning 调用后关联的车道线信息将丢失，需重新建立关联关系
   */
  void clear() {
    id = "";
    type = MergeForkType::kTypeInvalid;
    point.set_x(0);
    point.set_y(0);
    point.set_z(0);
    related_lane_markings.clear();
  }
};

struct PerceptionStopLine {
  std::string id = "";      ///< 停止线ID
  bool is_virtual = false;  ///< 是否是虚拟停止线，默认真实停止线
  math::Vec3d point;        ///< 停止线中心坐标，米，在车道段中心线附近

  /**
   * @brief 重置停止线数据至初始状态
   * @details 该方法用于清空停止线对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - id: 清空字符串 (长度归零)
   * - is_virtual: 重置为false (默认真实停止线)
   * - point: 坐标归零 (x,y,z=0)
   *
   * @par 典型使用场景:
   * 1. 停止线跟踪失效时
   * 2. 跨感知周期数据更新前
   * 3. 父级车道段对象重置时
   *
   * @note 该方法不影响关联的车道段状态
   * @warning 虚拟停止线状态重置后会丢失原始属性，需谨慎调用
   */
  void clear() {
    id = "";
    is_virtual = false;
    point.set_x(0);
    point.set_y(0);
    point.set_z(0);
  }
};

struct PerceptionGate {
  std::string id = "";  ///< 闸机ID
  enum class GateStatus {
    kStatusUnknown = 0,  //< 未知
    kStatusClosed = 1,   //< 静止并处于关闭状态
    kStatusOpen = 2,     //< 静止并处于开启状态
    kStatusMoving = 3    //< 运动但无法区分开或关
  };
  GateStatus gate_status = GateStatus::kStatusUnknown;  ///< 闸机开关状态
  math::Vec3d point;                                    ///< 闸机杆中心坐标，米
  float head_stop_distance = 1.0;                       ///< 期望车头停车距离

  /**
   * @brief 重置闸机数据至初始状态
   * @details 该方法用于清空闸机对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - id: 清空字符串 (长度归零)
   * - gate_status: 设为kStatusInvalid (无效状态)
   * - point: 坐标归零 (x,y,z=0)
   * - head_stop_distance: 重置为1.0米 (默认停车距离)
   *
   * @par 典型使用场景:
   * 1. 闸机跟踪丢失时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 导航路径重新规划时
   *
   * @note 该方法不影响关联的车道段状态
   * @warning 调用后需重新获取闸机最新状态数据
   */
  void clear() {
    id = "";
    gate_status = GateStatus::kStatusUnknown;
    point.set_x(0);
    point.set_y(0);
    point.set_z(0);
    head_stop_distance = 1.0;
  }
};

struct LaneMarkingSourceInfo {
  float start_ratio = 0.0;  ///< 描述车道线来源信息的起始纵向长度比例
  float end_ratio = 0.0;    ///< 描述车道线来源信息的终止纵向长度比例
  LineSourceType source_type =
      LineSourceType::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeInvalid;  ///< 车道线来源类型
};

struct PerceptionLaneSegment {
  std::string id = "";           ///< 车道段id, 每帧唯一
  float navigation_score = 0.0;  ///< 导航分，范围：[0.0, 1.0]

  enum class LaneSegmentType {  ///< 道路等级
    kTypeInvalid = 0,           ///< 无效
    kTypeVehicle = 1,           ///< 机动车道
    kTypeBicycle = 2,           ///< 非机动车道
    kTypeRamp = 3,              ///< 匝道
    kTypeReversing = 4,         ///< 倒车车道
    kTypePocket = 5,            ///< 待行车道
    kTypeParking = 6,           ///< 停车道
    kTypeBus = 7,               ///< 公交车道
    kTypeEmergency = 8,         ///< 应急车道
    kTypeReversible = 9,        ///< 潮汐车道
    kTypeVirtual = 20,          ///< 虚拟车道
    kTypeOpposite = 30,         ///< 逆向车道
  };
  LaneSegmentType type = LaneSegmentType::kTypeInvalid;  ///< 道路等级
  enum class LaneSegmentDirection {
    kDirectionInvalid = 0,      ///< 无效
    kDirectionForwardOnly = 1,  ///< 直行
    kDirectionLeftOnly = 2,     ///< 左转
    kDirectionRightOnly = 3,    ///< 右转
    kDirectionUTurnOnly = 4,    ///< 掉头
    kDirectionUnknown = 66,     ///< 未知
  };
  LaneSegmentDirection direction = LaneSegmentDirection::kDirectionInvalid;  ///< 车道方向
  PerceptionLaneMarking left_marking;                                        ///< 车道左边线
  PerceptionLaneMarking
      right_marking;  ///< 车道右边线，当存在既有车道线又有路沿/围栏等情形，仅给出一条LaneMarking，同时类别设定为路沿
  std::vector<LaneMarkingSourceInfo> source_infos;  ///< 车道线来源类型数组，分段表示
  float distance_to_change_lane =
      -1.0;  ///< 距离导航换道点的距离，米，提前通知最远距离由navigation节点的navigation_score_notify_ahead_distance参数控制
             ///< 若提前通知距离内无换道点，则该字段值为-1.0
             ///< 典型导航换道场景：出匝道、高速连接处分岔口、路口处存在不允许通行的车道等场景

  std::vector<std::string> previous_segment_ids;  ///< 前驱车道段ID，可能有多个
  std::vector<std::string> next_segment_ids;      ///< 后继车道段ID，可能有多个
  std::string left_segment_id = "";               ///< 左车道段ID，默认无
  std::string right_segment_id = "";              ///< 右车道段ID，默认无
  std::string left_opposite_segment_id = "";      ///< 左对向车道段ID，默认无
  PerceptionStopLine stop_line;                   ///< 停止线, 一个车道段仅有一个停止线, 否则按停止线分段
  PerceptionGate gate;                            ///< 车道绑定的闸机，一个车道段仅有一个闸机，否则按闸机分段
  std::vector<std::string> related_area_ids;      ///< 相交的特殊区域ID列表, 空表示无
  float max_speed_limit = 0.0;                    ///< 道路/车道最高限速，km/h
  float min_speed_limit = 0.0;                    ///< 道路/车道最低限速，km/h
  CrossInOutType cross_in_out_type = CrossInOutType::kCrossInvalid;  ///< 进入退出路口类型

  /**
   * @brief 重置车道段数据至初始状态
   * @details 该方法用于清空车道段对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - 基础属性:
   *   - id: 清空字符串 (长度归零)
   *   - navigation_score: 重置为0.0 (导航评分归零)
   *   - type/direction: 设为无效枚举值
   * - 车道标线:
   *   - left_marking/right_marking: 调用clear()方法
   * - 关联关系:
   *   - source_infos: 清空来源信息数组
   *   - previous_segment_ids/next_segment_ids: 清空前后车道段ID
   *   - left_segment_id/right_segment_id: 清空相邻车道段ID
   * - 特殊元素:
   *   - stop_line/gate: 调用clear()方法
   *   - related_area_ids: 清空关联区域ID
   * - 交通规则:
   *   - max_speed_limit/min_speed_limit: 重置为0.0
   *   - cross_in_out_type: 设为无效枚举值
   *
   * @par 典型使用场景:
   * 1. 车道段跟踪丢失时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 导航路径重新规划时
   *
   * @note 该方法会递归调用子对象(stop_line/gate)的clear()
   * @warning 调用后将丢失所有关联关系，需重新建立车道拓扑连接
   */
  void clear() {
    id = "";
    navigation_score = 0.0;
    type = LaneSegmentType::kTypeInvalid;
    direction = LaneSegmentDirection::kDirectionInvalid;
    left_marking.clear();
    right_marking.clear();
    source_infos.clear();
    previous_segment_ids.clear();
    next_segment_ids.clear();
    left_segment_id = "";
    right_segment_id = "";
    left_opposite_segment_id = "";
    stop_line.clear();
    gate.clear();
    related_area_ids.clear();
    max_speed_limit = 0.0;
    min_speed_limit = 0.0;
    cross_in_out_type = CrossInOutType::kCrossInvalid;
  }
};

struct Lane {
  std::string id = "";                               ///< 车道ID, 时空唯一
  std::vector<PerceptionLaneSegment> lane_segments;  ///< 车道分段结果，按车道行驶方向排列
  std::vector<std::string> previous_lane_ids;        ///< 前驱车道id，可能有多个
  std::vector<std::string> next_lane_ids;            ///< 后继车道id，可能有多个

  /**
   * @brief 重置车道数据至初始状态
   * @details 该方法用于清空车道对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - id: 清空字符串 (长度归零)
   * - lane_segments: 清空车道段数组 (size=0)
   * - previous_lane_ids/next_lane_ids: 清空前后车道ID数组
   *
   * @par 典型使用场景:
   * 1. 车道跟踪丢失时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 导航路径重新规划时
   *
   * @note 该方法不会递归清理车道段元素，仅清空容器
   * @warning 调用后将丢失车道拓扑连接关系，需重新建立前后车道关联
   */
  void clear() {
    id = "";
    lane_segments.clear();
    previous_lane_ids.clear();
    next_lane_ids.clear();
  }
};

struct PerceptionArea {
  std::string id = "";                                                                ///< 区域id
  proto::MapCommon::AreaType area_type = proto::MapCommon_AreaType_kAreaTypeInvalid;  ///< 区域类型
  std::vector<math::Vec3d> polygon;  ///< 区域边界凸包顶点，米，逆时针排序
  std::vector<std::string>
      related_in_lane_segment_ids;  ///< 相关的进入路径段ID数组，只有当area_type为kTypeJunction时有效
  std::vector<std::string>
      related_out_lane_segment_ids;  ///< 相关的离开路径段ID数组，只有当area_type为kTypeJunction时有效
  float start_s = 0.0;               ///< 区域起始s值，米
  float end_s = 0.0;                 ///< 区域结束s值，米
  std::vector<std::string> related_reference_lines_ids;  //< 关联的参考线ID数组

  /**
   * @brief 重置区域数据至初始状态
   * @details 该方法用于清空区域对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - id: 清空字符串 (长度归零)
   * - area_type: 设为kTypeInvalid (无效类型)
   * - polygon: 清空边界顶点数组 (size=0)
   * - related_in_lane_segment_ids/related_out_lane_segment_ids: 清空关联车道段ID数组
   *
   * @par 典型使用场景:
   * 1. 区域跟踪丢失时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 导航路径重新规划时
   *
   * @note 交叉口类型区域重置后会丢失所有关联的车道段连接信息
   * @warning 调用后需重新建立与车道段的拓扑关联
   */
  void clear() {
    id = "";
    area_type = proto::MapCommon_AreaType_kAreaTypeInvalid;
    polygon.clear();
    related_in_lane_segment_ids.clear();
    related_out_lane_segment_ids.clear();
    start_s = 0.0;
    end_s = 0.0;
    related_reference_lines_ids.clear();
  }
};

struct PerceptionRoadInfo {
  std::string road_id = "";                        ///< 时空唯一, 来自地图Road/Link数据，若为空表示无效
  RoadClass road_class = RoadClass::kRoadInvalid;  ///< 道路等级
  std::vector<RoadType> road_kind_types;           ///< 道路类型
  float start_s = 0.0;                             ///< 区域起始s值，米
  float end_s = 0.0;                               ///< 区域结束s值，米

  /**
   * @brief 重置道路信息至初始状态
   * @details 该方法用于清空道路信息对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - road_id: 清空字符串 (长度归零)
   * - road_class: 设为kRoadInvalid (无效道路等级)
   * - road_kind_types: 清空道路类型数组 (size=0)
   *
   * @par 典型使用场景:
   * 1. 道路跟踪丢失时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 导航路径重新规划时
   *
   * @note 该方法不会释放内存，仅重置成员变量值
   * @warning 调用后将丢失道路类型属性信息，需重新获取道路元数据
   */
  void clear() {
    road_id = "";
    road_class = RoadClass::kRoadInvalid;
    road_kind_types.clear();
    start_s = 0.0;
    end_s = 0.0;
  }
};

/**
 * @brief 感知提供的原始停止线标识信息
 */
struct PerceptionStopLineMarking {
  std::string id = "";                             ///< 停止线ID, 时空唯一
  std::vector<math::Vec3d> line_points;            ///< 全局坐标系下的停止线散点，当前仅 x,y 维度有效, 点数>=2，米
  StopLineType type = StopLineType::kTypeInvalid;  ///< 停止线类型

  /**
   * @brief 重置停止线标识数据至初始状态
   * @details 该方法用于清空停止线标识对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - id: 清空字符串 (长度归零)
   * - line_points: 清空散点数组 (size=0)
   * - type: 设为kTypeInvalid (无效类型)
   *
   * @par 典型使用场景:
   * 1. 停止线跟踪失效时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 地面标识分组重建时
   *
   * @note 该方法仅清理当前对象数据，不影响关联的车道段
   * @warning 调用后需重新建立与车道线组的关联关系
   */
  void clear() {
    id = "";
    line_points.clear();
    type = StopLineType::kTypeInvalid;
  }
};

/**
 * @brief 感知提供的原始地面箭头等标识信息
 *
 */
struct PerceptionArrowMarking {
  std::string id = "";                        ///< 地面箭头ID, 时空唯一
  ArrowType type = ArrowType::kArrowInvalid;  ///< 箭头标识类型
  std::vector<math::Vec3d>
      polygon_points;  ///< 全局坐标系下的外接矩形框包络顶点，当前仅x,y维度有效, 不闭合，逆时针排序，米

  /**
   * @brief 重置地面箭头标识数据至初始状态
   * @details 该方法用于清空箭头标识对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - id: 清空字符串 (长度归零)
   * - type: 设为kArrowInvalid (无效类型)
   * - polygon_points: 清空包络顶点数组 (size=0)
   *
   * @par 典型使用场景:
   * 1. 箭头标识跟踪失效时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 地面标识分组重建时
   *
   * @note 该方法仅清理当前对象数据，不影响关联的车道线组
   * @warning 调用后需重新建立与车道线组的拓扑关联
   */
  void clear() {
    id = "";
    type = ArrowType::kArrowInvalid;
    polygon_points.clear();
  }
};

/**
 * @brief 感知提供的原始特殊区域标识信息
 *
 */
struct PerceptionAreaMarking {
  std::string id = "";                                                           ///< 区域ID, 时空唯一
  proto::MapCommon::AreaType type = proto::MapCommon_AreaType_kAreaTypeInvalid;  ///< 区域类型
  std::vector<math::Vec3d>
      polygon_points;  ///< 全局坐标系下的区域边界包络框顶点, 当前仅x,y维度有效, 点数>=3 不闭合，逆时针排序, 米

  /**
   * @brief 重置特殊区域标识数据至初始状态
   * @details 该方法用于清空区域标识对象的所有属性，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - id: 清空字符串 (长度归零)
   * - type: 设为kTypeInvalid (无效类型)
   * - polygon_points: 清空边界顶点数组 (size=0)
   *
   * @par 典型使用场景:
   * 1. 区域标识跟踪失效时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 地面标识分组重建时
   *
   * @note 该方法仅清理当前对象数据，不影响关联的车道线组
   * @warning 调用后需重新建立与车道线组的拓扑关联
   */
  void clear() {
    id = "";
    type = proto::MapCommon_AreaType_kAreaTypeInvalid;
    polygon_points.clear();
  }
};

/**
 * @brief 感知提供的原始车道线等地面标识分组信息
 *
 */
struct PerceptionLaneMarkingGroup {
  std::string group_id = "";  ///< 车道线组ID, 全局唯一，group按照所属道路划分
  std::vector<PerceptionLaneMarking>
      lane_markings;  ///< 同一group中的车道线数组，沿车道行驶方向从左到右顺序有序排布(Group内每条车道线不一定完全对齐，点数不一定严格一致)
  std::vector<PerceptionStopLineMarking> stop_lines;        ///< 同一group中的停止线数组
  std::vector<PerceptionArrowMarking> arrows;               ///< 同一group中的地面箭头等标识数组
  std::vector<PerceptionAreaMarking> areas;                 ///< 同一group中的特殊区域标识数组
  std::vector<PerceptionMergeForkPoint> merge_fork_points;  ///< 同一group中的车道线汇入汇出点数组

  /**
   * @brief 重置地面标识分组数据至初始状态
   * @details 该方法用于清空整个地面标识分组对象及其所有子元素，使其恢复到可重新使用的初始状态
   *
   * @par 成员变量重置说明:
   * - group_id: 清空字符串 (长度归零)
   * - lane_markings: 清空车道线数组，并递归调用各元素的clear()
   * - stop_lines: 清空停止线数组，并递归调用各元素的clear()
   * - arrows: 清空箭头标识数组，并递归调用各元素的clear()
   * - areas: 清空区域标识数组，并递归调用各元素的clear()
   * - merge_fork_points: 清空汇入汇出点数组，并递归调用各元素的clear()
   *
   * @par 典型使用场景:
   * 1. 道路分组跟踪失效时重置状态
   * 2. 跨感知周期数据更新前
   * 3. 全局路径重新规划时
   *
   * @note 该方法会递归清理所有子元素数据
   * @warning 调用后将丢失整个分组的所有拓扑关联，需重新建立与道路的绑定关系
   */
  void clear() {
    group_id = "";
    lane_markings.clear();
    stop_lines.clear();
    arrows.clear();
    areas.clear();
    merge_fork_points.clear();
  }
};

}  // namespace gpal::pnc::planning
