/**
 * @file common_typedefs.h
 * @brief 道路结构认知数据通用类型定义文件
 * @details 该文件定义了道路结构认知数据模块中使用的通用类型和结构体。
 */

#pragma once

#include <algorithm>
#include <filesystem>
#include <fmt/chrono.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <limits>
#include <list>
#include <map>
#include <numeric>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "base/log.h"
#include "boost/math/tools/minima.hpp"
#include "gpal-interface/map_server/map_common.pb.h"
#include "gpal-interface/perception/env_road_instance.pb.h"
#include "gpal-interface/perception/perception_freespace.pb.h"
#include "gpal-interface/road_cognition/env_road_cognition.pb.h"
#include "math/aaboxkdtree2d.h"
#include "math/angle.h"
#include "math/box2d.h"
#include "math/cartesian_frenet_conversion.h"
#include "math/linear_interpolation.h"
#include "math/math_utils.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
#include "point/path_pt.h"
#include "point/point.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/sl_boundary.pb.h"
#include "util/base_struct.h"
#include "util/string_util.h"
#include "util/util.h"

namespace gpal::pnc::planning {
namespace road_instance {

using Vec2d = pnc::planning::math::Vec2d;
using Vec3d = pnc::planning::math::Vec3d;
using LineSourceType = proto::road_cognition::LineAttributeRange::LineSourceType;

struct StopLine;
struct Arrow;
struct TrafficLight;
struct Gate;
struct Area;
struct Road;
struct RiskBoundary;
struct KeyPoint;
struct PerceptionCenterLine;
struct MapLane;
struct MapRoad;
struct MapLandMark;
struct MapJunction;
using StopLinePtr = std::shared_ptr<StopLine>;
using ArrowPtr = std::shared_ptr<Arrow>;
using TrafficLightPtr = std::shared_ptr<TrafficLight>;
using GatePtr = std::shared_ptr<Gate>;
using AreaPtr = std::shared_ptr<Area>;
using RoadPtr = std::shared_ptr<Road>;
using RiskBoundaryPtr = std::shared_ptr<RiskBoundary>;
using KeyPointPtr = std::shared_ptr<KeyPoint>;
using PerceptionCenterLinePtr = std::shared_ptr<PerceptionCenterLine>;
using MapLanePtr = std::shared_ptr<MapLane>;
using MapRoadPtr = std::shared_ptr<MapRoad>;
using MapLandMarkPtr = std::shared_ptr<MapLandMark>;
using MapJunctionPtr = std::shared_ptr<MapJunction>;
using LaneTypeRange = proto::perception::CenterLine::TypeRange;

/**
 * @brief 换道场景类型（来自SD地图）
 */
enum class LaneChangeScenarioType {
  kTypeInvalid = 0,              ///< 无效
  kLaneChangeForJunction = 1,    ///< 路口场景换道
  kLaneChangeForRampFork = 10,   ///< 匝道汇出、分岔场景变道（一般有车道箭头）
  kLaneChangeForRampMerge = 11,  ///< 匝道汇入场景变道（一般无车道箭头, 自车仍处于匝道中）
};

struct LineAttributeRange {
  LineAttributeRange() = default;

  LineAttributeRange(const proto::road_cognition::LineAttributeRange::LineSourceType &_source_type,
                     const proto::perception::CenterLine::TypeRange::Type &_line_type, const float &_start_s,
                     const float &_end_s, const Vec3d &_start_point, const Vec3d &_end_point)
      : source_type(_source_type),
        line_type(_line_type),
        start_s(_start_s),
        end_s(_end_s),
        start_point(_start_point),
        end_point(_end_point) {}

  void clear() {
    source_type = proto::road_cognition::LineAttributeRange_LineSourceType_kTypeInvalid;
    line_type = proto::perception::CenterLine_TypeRange_Type_kTypeUnknown;
    start_s = 0.0;
    end_s = 0.0;
    start_point = Vec3d();
    end_point = Vec3d();
  }

  proto::road_cognition::LineAttributeRange::LineSourceType source_type =
      proto::road_cognition::LineAttributeRange_LineSourceType_kTypeInvalid;  // 来源类型
  proto::perception::CenterLine::TypeRange::Type line_type =
      proto::perception::CenterLine_TypeRange_Type_kTypeUnknown;  // 参考线类型
  float start_s = 0.0;  // 当前类型分段起点相对参考线的S坐标，米
  float end_s = 0.0;    // 当前类型分段终点相对参考线的S坐标，米
  Vec3d start_point;    // Range在参考线上的起始点坐标
  Vec3d end_point;      // Range在参考线上的结束点坐标
};

struct SpeedLimit {
  SpeedLimit() = default;

  SpeedLimit(const float &_start_s, const float &_end_s, const float &_max_speed_limit, const float &_min_speed_limit,
             const float &_recommended_speed, const Vec3d &_start_point, const Vec3d &_end_point)
      : start_s(_start_s),
        end_s(_end_s),
        max_speed_limit(_max_speed_limit),
        min_speed_limit(_min_speed_limit),
        recommended_speed(_recommended_speed),
        start_point(_start_point),
        end_point(_end_point) {}

  void clear() {
    start_s = 0.0;
    end_s = 0.0;
    max_speed_limit = 130 * pnc::planning::KMH_MS;  // m/s
    min_speed_limit = 0.0;                          // m/s
    recommended_speed = -1.0;                       // m/s
    start_point = Vec3d();
    end_point = Vec3d();
  }

  float start_s = 0.0;
  float end_s = 0.0;
  float max_speed_limit = 130 * pnc::planning::KMH_MS;  // 最高限速，m/s
  float min_speed_limit = 0.0;                          // 最低限速，m/s
  float recommended_speed = -1.0;                       // 推荐车速，m/s，小于0时无效，默认-1.0
  Vec3d start_point;                                    // Range在参考线上的起始点坐标
  Vec3d end_point;                                      // Range在参考线上的结束点坐标
};

/**
 * @brief StopLine结构体用于表示停止线信息
 * @details 该结构体包含了停止线的位置、关联信号灯、方向等属性
 */
struct StopLine {
  /**
   * @brief 默认构造函数
   */
  StopLine() = default;

  /**
   * @brief 构造函数，初始化停止线的详细信息
   * @param _id 停止线的ID
   * @param _is_virtual 停止线是否为虚拟停止线
   * @param _pt 停止线中心点的坐标
   * @param _s 停止线中心点在LocalRoute的guide_line_上的投影s位置
   * @param _traffic_light_id 关联的信号灯ID
   * @param _direction 停止线的方向
   */
  StopLine(const std::string &_id, const proto::perception::StopLine::LineType &_type, const Vec3d &_pt,
           const float &_s, const proto::MapCommon::DrivingDirection &_direction, const std::string &_traffic_light_id)
      : id(_id), type(_type), pt(_pt), s(_s), direction(_direction), traffic_light_id(_traffic_light_id) {}

  void clear() {
    is_exist = false;
    id = "";
    type = proto::perception::StopLine_LineType_kTypeUnknown;
    pt = Vec3d();
    s = 0.0;
    direction = proto::MapCommon_DrivingDirection_kDrivingUnknown;
    traffic_light_id = "";
  }

  bool is_exist = false;  ///< 停止线是否存在
  std::string id = "";    ///< 停止线的ID
  proto::perception::StopLine::LineType type = proto::perception::StopLine_LineType_kTypeUnknown;  ///< 停止线类型
  Vec3d pt;                ///< 停止线中心点的坐标
  std::vector<Vec3d> pts;  ///< 停止线形点，米，当前仅 x,y 维度有效, 点数>=2
  float s = 0.0;           ///< 中心坐标在LocalRoute的guide_line_上的投影s位置
  proto::MapCommon::DrivingDirection direction =
      proto::MapCommon_DrivingDirection_kDrivingUnknown;  ///< 停止线绑定的行驶方向
  std::string traffic_light_id = "";                      ///< 关联的信号灯id, 默认值 0 表示无关联
};

struct Arrow {
  void clear() {
    id = "";
    type = proto::perception::Arrow::kArrowUnknown;
    pts.clear();
  }

  std::string id = "";  ///< 地面箭头跟踪ID, 时空唯一
  proto::perception::Arrow::ArrowType type = proto::perception::Arrow::kArrowUnknown;  ///< 箭头表示类型
  std::vector<Vec3d> pts;  ///< 外接矩形框包络顶点，米，当前仅 x,y 维度有效,
                           ///< 首尾不重复闭合，odom坐标系，逆时针排序
};

struct TrafficLight {
  TrafficLight() = default;

  TrafficLight(const std::string &_id, proto::perception::TrafficLight::LightStatus &_status,
               const proto::perception::TrafficLight::LightColor &_color,
               const proto::perception::TrafficLight::LightShape &_shape, const int32_t &_countdown_second,
               const std::vector<Vec3d> &_pts, const std::vector<std::string> &_related_stop_lines_ids)
      : id(_id),
        status(_status),
        color(_color),
        shape(_shape),
        countdown_second(_countdown_second),
        pts(_pts),
        related_stop_lines_ids(_related_stop_lines_ids) {}

  void clear() {
    id = "";
    status = proto::perception::TrafficLight_LightStatus_kStatusUnknown;
    color = proto::perception::TrafficLight_LightColor_kColorUnknown;
    shape = proto::perception::TrafficLight_LightShape_kShapeUnknown;
    countdown_second = -1;
    pts.clear();
    related_stop_lines_ids.clear();
  }

  std::string id = "";  // 交通灯ID
  proto::perception::TrafficLight::LightStatus status =
      proto::perception::TrafficLight_LightStatus_kStatusUnknown;  // 交通灯状态
  proto::perception::TrafficLight::LightColor color =
      proto::perception::TrafficLight_LightColor_kColorUnknown;  // 交通灯颜色
  proto::perception::TrafficLight::LightShape shape =
      proto::perception::TrafficLight_LightShape_kShapeUnknown;  // 交通灯形状
  int32_t countdown_second = -1;  // 交通灯倒计时秒数，秒，无倒计时时为-1
  std::vector<Vec3d> pts;         // 交通灯位置信息，米（使用数组表示，提高后续兼容性）
  std::vector<std::string> related_stop_lines_ids;  // 关联的停止线ID数组
};

/**
 * @brief Gate结构体用于表示闸机信息
 * @details 该结构体包含了闸机的位置、状态、期望停车距离等属性
 */
struct Gate {
  /**
   * @brief 默认构造函数
   */
  Gate() = default;

  /**
   * @brief 构造函数，初始化闸机的基本信息
   * @param _id 闸机的ID
   * @param _status 闸机的状态
   * @param _pt 闸机中心点的坐标
   * @param _s 闸机中心点在LocalRoute的guide_line_上的投影s位置
   * @param _head_stop_distance 期望车头停车距离
   */
  Gate(const std::string &_id, const proto::road_cognition::GateRange::GateType &_type,
       const proto::perception::Gate::GateStatus &_status, const std::vector<Vec3d> &_pts, const float &_s,
       const float &_head_stop_distance)
      : id(_id), type(_type), status(_status), pts(_pts), s(_s), head_stop_distance(_head_stop_distance) {}

  void clear() {
    is_exist = false;
    id = "";
    type = proto::road_cognition::GateRange_GateType_kTypeUnknown;
    status = proto::perception::Gate_GateStatus_kStatusUnknown;
    pts.clear();
    s = 0.0;
    head_stop_distance = 0.0;
  }

  bool is_exist = false;  ///< 闸机是否存在
  std::string id = "";    ///< 闸机的ID
  proto::road_cognition::GateRange::GateType type = proto::road_cognition::GateRange_GateType_kTypeUnknown;
  proto::perception::Gate::GateStatus status = proto::perception::Gate_GateStatus_kStatusUnknown;  ///< 闸机的状态
  std::vector<Vec3d> pts;          ///< 闸机边界凸包顶点，米，当前仅 x,y 维度有效,
                                   ///< 点数>=3，不闭合，逆时针排序
  float s = 0.0;                   ///< 中心坐标在参考线上的投影s位置
  float head_stop_distance = 0.0;  ///< 期望车头停车距离
};

struct Area {
  Area() = default;

  Area(const std::string &_id, const proto::MapCommon::AreaType &_area_type, const std::vector<Vec3d> &_pts,
       const std::vector<std::string> &_related_reference_lines_ids)
      : id(_id), area_type(_area_type), pts(_pts), related_reference_lines_ids(_related_reference_lines_ids) {}

  void clear() {
    id = "";
    area_type = proto::MapCommon_AreaType_kAreaTypeInvalid;
    pts.clear();
    related_reference_lines_ids.clear();
  }

  std::string id = "";                                                                ///< 区域id
  proto::MapCommon::AreaType area_type = proto::MapCommon_AreaType_kAreaTypeInvalid;  ///< 区域类型
  std::vector<Vec3d> pts;  ///< 区域边界凸包顶点，米，当前仅 x,y 维度有效,
                           ///< 点数>=3 不闭合，逆时针排序
  std::vector<std::string> related_reference_lines_ids;  ///< 关联的参考线ID数组
};

struct AreaRange {
  AreaRange() = default;

  AreaRange(const std::string &_id, const float &_start_s, const float &_end_s)
      : id(_id), start_s(_start_s), end_s(_end_s) {}

  void clear() {
    id = "";
    start_s = 0.0;
    end_s = 0.0;
  }

  std::string id = "";  // 特殊区域ID
  float start_s = 0.0;  // 起点相对参考线的S坐标，米
  float end_s = 0.0;    // 终点相对参考线的S坐标，米
};

struct Road {
  Road() = default;

  Road(const std::string &_id, const proto::road_cognition::Road::RoadSourceType &_road_source,
       const std::string &_road_name, const float &_length, const proto::MapCommon::RoadClass &_road_class,
       const std::vector<proto::MapCommon::RoadType> &_road_types, const float &_max_speed_limit,
       const uint32_t &_lane_num, const std::set<std::string> &_related_reference_lines_ids)
      : id(_id),
        road_source(_road_source),
        road_name(_road_name),
        length(_length),
        road_class(_road_class),
        road_types(_road_types),
        max_speed_limit(_max_speed_limit),
        lane_num(_lane_num),
        related_reference_lines_ids(_related_reference_lines_ids) {}

  void clear() {
    id = "";
    road_source = proto::road_cognition::Road_RoadSourceType_kSourceInvalid;
    road_name = "";
    length = 0.0;
    road_class = proto::MapCommon_RoadClass_kRoadInvalid;
    road_types.clear();
    max_speed_limit = 0.0;
    lane_num = 0;
    related_reference_lines_ids.clear();
  }

  std::string id = "";  // 道路ID
  proto::road_cognition::Road::RoadSourceType road_source =
      proto::road_cognition::Road_RoadSourceType_kSourceInvalid;  // 道路信息来源类型
  std::string road_name = "";                                     // 道路名称，若无有效道路名称则为空
  float length = 0.0;                                             // 道路长度，米
  proto::MapCommon::RoadClass road_class = proto::MapCommon_RoadClass_kRoadInvalid;  // 所属道路等级
  std::vector<proto::MapCommon::RoadType> road_types;                                // 所属道路类型数组
  float max_speed_limit = 0.0;                                                       // 最大限速，m/s
  uint32_t lane_num = 0;                                                             // 车道个数
  std::set<std::string> related_reference_lines_ids;                                 // 关联的参考线ID数组
};

struct RoadRange {
  RoadRange() = default;

  RoadRange(const std::string &_id, const float &_start_s, const float &_end_s)
      : id(_id), start_s(_start_s), end_s(_end_s) {}

  void clear() {
    id = "";
    start_s = 0.0;
    end_s = 0.0;
  }

  std::string id = "";  ///< 时空唯一, 来自地图Road/Link数据，若为空表示无效
  float start_s = 0.0;  ///< 起点相对参考线的S坐标，米
  float end_s = 0.0;    ///< 终点相对参考线的S坐标，米
};

struct LaneLocation {  // 车道级定位相关信息
  LaneLocation() = default;

  LaneLocation(const float &_s, const int &_lane_num, const int &_ego_lane_idx)
      : s(_s), lane_num(_lane_num), ego_lane_idx(_ego_lane_idx) {}

  void clear() {
    s = 0.0;
    lane_num = -1;
    ego_lane_idx = -1;
  }

  float s = 0.0;          ///< 参考线S坐标，米
  int lane_num = -1;      ///< 总车道数
  int ego_lane_idx = -1;  ///< 车道从左到右排序，idx从0开始
};

struct DrivingDirectionRange {
  DrivingDirectionRange() = default;

  DrivingDirectionRange(const float &_start_s, const float &_end_s,
                        const proto::MapCommon::DrivingDirection &_direction, const Vec3d &_start_point,
                        const Vec3d &_end_point)
      : start_s(_start_s), end_s(_end_s), direction(_direction), start_point(_start_point), end_point(_end_point) {}
  void clear() {
    start_s = 0.0;
    end_s = 0.0;
    direction = proto::MapCommon_DrivingDirection_kDrivingUnknown;
    start_point = Vec3d();
    end_point = Vec3d();
  }

  float start_s = 0.0;
  float end_s = 0.0;
  proto::MapCommon::DrivingDirection direction = proto::MapCommon_DrivingDirection_kDrivingUnknown;
  Vec3d start_point;  // Range在参考线上的起始点坐标
  Vec3d end_point;    // Range在参考线上的结束点坐标
};

/**
 * @brief SegmentBoundaryType结构体用于表示路段边界类型信息
 * @details
 * 该结构体包含了路段边界的起始s值、结束s值、左侧边界类型、右侧边界类型、起止点等属性
 */
struct BoundaryRange {
  /**
   * @brief 默认构造函数
   */
  BoundaryRange() = default;

  BoundaryRange(const float &_start_s, const float &_end_s,
                const proto::road_cognition::BoundaryRange::BoundaryType &_type,
                const proto::perception::LaneMarking::LineShape &_shape,
                const proto::perception::LaneMarking::LineColor &_color, const uint32_t &_related_lane_markings_ids,
                const Vec3d &_start_point, const Vec3d &_end_point)
      : start_s(_start_s),
        end_s(_end_s),
        type(_type),
        shape(_shape),
        color(_color),
        related_lane_markings_ids(_related_lane_markings_ids),
        start_point(_start_point),
        end_point(_end_point) {}

  void clear() {
    start_s = 0.0;
    end_s = 0.0;
    type = proto::road_cognition::BoundaryRange_BoundaryType_kTypeCrossGuide;
    shape = proto::perception::LaneMarking_LineShape_kShapeUnknown;
    color = proto::perception::LaneMarking_LineColor_kColorUnknown;
    related_lane_markings_ids.clear();
    start_point = Vec3d();
    end_point = Vec3d();
  }

  float start_s = 0.0;  ///< Range在参考线上的起始s值
  float end_s = 0.0;    ///< Range在参考线上的结束s值
  proto::road_cognition::BoundaryRange::BoundaryType type =
      proto::road_cognition::BoundaryRange_BoundaryType_kTypeCrossGuide;  ///< 路段边界类型
  proto::perception::LaneMarking::LineShape shape = proto::perception::LaneMarking_LineShape_kShapeUnknown;
  proto::perception::LaneMarking::LineColor color = proto::perception::LaneMarking_LineColor_kColorUnknown;
  std::vector<std::string> related_lane_markings_ids;
  Vec3d start_point;  ///< Range在参考线上的起始点坐标
  Vec3d end_point;    ///< Range在参考线上的结束点坐标
};

struct PolylineBoundary {
  struct BoundaryPoint {
    BoundaryPoint(const float &_s, const float &_l) : s(_s), l(_l) {}
    float s = 0.0;
    float l = 0.0;
  };
  std::vector<BoundaryPoint> boundary;
  proto::PerceptionPolyline::PolylineType type = proto::PerceptionPolyline_PolylineType_kTypeInvalid;
};

struct RiskBoundary {
  RiskBoundary() = default;

  RiskBoundary(const std::string &_id, const proto::road_cognition::RiskBoundary::RiskBoundaryType &_type,
               const std::vector<Vec3d> &_pts, const std::unordered_set<std::string> &_related_reference_lines_ids)
      : id(_id), type(_type), pts(_pts), related_reference_lines_ids(_related_reference_lines_ids) {}

  void clear() {
    id = "";
    type = proto::road_cognition::RiskBoundary_RiskBoundaryType_kTypeUnknown;
    pts.clear();
    related_reference_lines_ids.clear();
  }

  std::string id = "";  // 风险边界ID
  proto::road_cognition::RiskBoundary::RiskBoundaryType type =
      proto::road_cognition::RiskBoundary_RiskBoundaryType_kTypeUnknown;  // 风险边界类型
  std::vector<Vec3d> pts;                                       // 边界形状点，米，当前仅 x,y 维度有效
  std::unordered_set<std::string> related_reference_lines_ids;  // 关联的参考线ID数组
};

/**
 * @brief NeighborLocalRoute结构体用于表示相邻LocalRoute的信息
 * @details 该结构体包含了相邻LocalRoute的ID及其元素信息
 */
struct NeighborReferenceLine {
  NeighborReferenceLine() = default;

  NeighborReferenceLine(const std::string &_id, const std::vector<BoundaryRange> &_boundaries)
      : id(_id), boundaries(_boundaries) {}

  void clear() {
    id = "";
    boundaries.clear();
  }

  std::string id = "";                    ///< 邻居参考线ID
  std::vector<BoundaryRange> boundaries;  ///< 邻居参考线在本参考线一侧沿s方向的共用车道边界信息
};

struct KeyPoint {
  struct Linkage {
    std::string link_id = "";  ///< 关联的对象ID
    uint32_t link_index = 0;   ///< 关联对象中关联形点的索引, 当 link_id 为 0 时, 字段无效
    float link_s = 0.0; ///< 关键点相对关联对象的S坐标
    proto::perception::KeyPoint::Linkage::LinkClass link_class =
        proto::perception::KeyPoint_Linkage_LinkClass_kLinkUnknown;  ///< 关联对象类型
    int linkages_num = -1;     ///< 所有关联对象个数
    int seq_in_linkages = -1;  ///< 本关联对象在所有关联对象中的从左到右排序序号，从0开始
  };

  void clear() {
    id = "";
    type = proto::road_cognition::KeyPoint_PointType_kTypeUnknown;
    s = 0.0;
    s_index = -1;
    pt = Vec3d();
    linkages.clear();
    related_reference_lines_ids.clear();
  }

  std::string id = "";  ///< 关键点ID
  proto::road_cognition::KeyPoint::PointType type =
      proto::road_cognition::KeyPoint_PointType_kTypeUnknown;  ///< 关键点类型
  float s = 0.0;                                               ///< 关键点相对参考线的S坐标，米
  int s_index = -1;                                            ///< 关键点相对参考线的形点索引
  Vec3d pt;                                                    ///< 关键点位置坐标，米
  std::vector<Linkage> linkages;                               ///< 与线的关联信息
  // std::unordered_map<std::string, Linkage> linkages_map;       ///< 与线的关联信息的映射: <关联对象ID，关联对象信息>
  std::vector<std::string> related_reference_lines_ids;        ///< 相关其他参考线的ID数组
  // 内部辅助信息
  int idx_in_linkages = -1;       ///< 本关联对象在所有关联对象(linkages)中的索引，从0开始
  std::string road_id = "";       ///< 所在的Road ID
  std::string next_road_id = "";  ///< 后继Road ID
};

struct TopoInstance {
  void clear() {
    id = "";
    in_instance_ids.clear();
    out_instance_ids.clear();
    in_instance_pt = Vec3d();
    out_instance_pt = Vec3d();
  }

  std::string id = "";
  std::vector<std::string> in_instance_ids;
  std::vector<std::string> out_instance_ids;
  Vec3d in_instance_pt;
  Vec3d out_instance_pt;
};

struct NavigationScore {
  NavigationScore() = default;

  NavigationScore(const float &_score, const float &_start_s, const float &_end_s, const size_t _start_idx = -1,
                  const size_t _end_idx = -1)
      : score(_score), start_s(_start_s), end_s(_end_s), start_idx(_start_idx), end_idx(_end_idx) {}

  void clear() {
    score = 0.0;
    start_s = 0.0;
    end_s = 0.0;
    start_idx = -1;
    end_idx = -1;
  }

  float score = 0.0;
  float start_s = 0.0;
  float end_s = 0.0;
  // 内部辅助字段
  size_t start_idx = -1;
  size_t end_idx = -1;
};

struct NavigationInfo {
  NavigationInfo() = default;

  NavigationInfo(const std::vector<NavigationScore> &_navigation_scores, const float &_lane_change_distance,
                 const int32_t &_lane_change_direction, const int32_t &_lane_change_times)
      : navigation_scores(_navigation_scores),
        lane_change_distance(_lane_change_distance),
        lane_change_direction(_lane_change_direction),
        lane_change_times(_lane_change_times) {}

  void clear() {
    navigation_scores.clear();
    lane_change_distance = -1;
    lane_change_direction = 0;
    lane_change_times = 0;
  }

  std::vector<NavigationScore> navigation_scores;  ///< 导航分数组
  float lane_change_distance = -1;  ///< 当前距导航换道点的距离, 米，默认-1.0, 表征无需导航换道
  int32_t lane_change_direction = 0;  ///< 从当前参考线导航换道至目标参考线的换道方向: 左换道:1, 右换道:-1,
                                      ///< 不换道:0, 默认不换道
  int32_t lane_change_times = 0;  ///< 从当前参考线导航换道至目标参考线的连续换道次数, 默认0
};

/**
 * @brief 感知中心线/引导线的统一结构体
 */
struct PerceptionCenterLine {
  enum class Direction {
    kDirectionUnknown = 0,    // 未知
    kDirectionSame = 1,       // 自车不掉头时，与自车行驶方向相同
    kDirectionOpposite = 2,   // 自车不掉头时，与自车行驶方向相反
    kDirectionUndivided = 3,  // 双向
  };
  struct TypeRange {
    uint32_t start_index = 0;  // 当前类型分段起始索引, 0或上一段的end_index
    uint32_t end_index = 0;    // 当前类型分段结束索引, 不含
    proto::perception::CenterLine::TypeRange::Type type =
        proto::perception::CenterLine::TypeRange::kTypeUnknown;  // 车道类型
  };
  struct BoundInfo {
    std::string lane_marking_id = "";
    proto::perception::LaneMarking::LineType bound_type =
        proto::perception::LaneMarking_LineType_kTypeNormal;
    proto::perception::LaneMarking::LineShape bound_shape =
        proto::perception::LaneMarking_LineShape_kShapeSingleSolid;
    Vec3d bound_pt;
  };

  void clear() {
    id = "";
    pts.clear();
    direction = Direction::kDirectionUnknown;
    type_ranges.clear();
  }

  std::string id = "";     ///< 中心线/引导线跟踪ID, 时空唯一
  std::vector<Vec3d> pts;  ///< 形点, 当前仅 x,y 维度有效, unit: m, 按车辆行驶方向单向排序;
  std::vector<std::pair<BoundInfo, BoundInfo>> bound_pts;  ///< 形点边界, 每个形点的边界框, 格式: <left, right>
  Direction direction = Direction::kDirectionUnknown;  ///< 中心线代表的行驶方向
  std::vector<TypeRange> type_ranges;  ///< 类型分段信息, 分段间互斥且总和覆盖所有points

  // 用于KdTree搜索
  bool is_inited_ = false;                    ///< 初始化状态
  gpal::pnc::planning::math::AABox2d aabox_;  ///< AABB box

  double time_delay_ = 0.0;  ///< 时延
};

/**
 * @brief 道路结构认知相关时间消耗信息
 */
struct CognitionTimeConsumption {
  void clear() {
    total_process_time_in_ms = 0;
    perception_ref_line_smooth_time_in_ms = 0;
    perception_instance_bind_time_in_ms = 0;
    perception_process_time_in_ms = 0;
    navi_sd_process_time_in_ms = 0;
    parking_ref_line_smooth_time_in_ms = 0;
    parking_instance_bind_time_in_ms = 0;
    parking_process_time_in_ms = 0;
  }

  uint32_t total_process_time_in_ms = 0;  ///< 总处理耗时, 单位: ms

  uint32_t perception_ref_line_smooth_time_in_ms = 0;  ///< 基于Perception的参考线平滑相关处理耗时, 单位: ms
  uint32_t perception_instance_bind_time_in_ms = 0;  ///< 基于Perception的实例绑定处理耗时, 单位: ms
  uint32_t perception_process_time_in_ms = 0;        ///< Perception相关处理耗时, 单位: ms

  uint32_t navi_sd_process_time_in_ms = 0;  ///< SD导航相关处理耗时, 单位: ms

  uint32_t parking_ref_line_smooth_time_in_ms = 0;  ///< Parking相关参考线平滑处理耗时, 单位: ms
  uint32_t parking_instance_bind_time_in_ms = 0;    ///< Parking相关实例绑定处理耗时, 单位: ms
  uint32_t parking_process_time_in_ms = 0;          ///< Parking相关处理耗时, 单位: ms
};

struct MapLane {
  void clear() {
    id = "";
    related_land_mark_ids.clear();
    lane_width = 0.0;
    lane_type = proto::perception::CenterLine::TypeRange::kTypeUnknown;
    left_boundary_ids.clear();
    right_boundary_ids.clear();
    points.clear();
    length = 0.0;
    in_lane_ids.clear();
    out_lane_ids.clear();
  }

  std::string id = "";                             ///< 车道id
  std::string road_id = "";                        ///< 车道所属道路(MapRoad)ID
  std::vector<std::string> related_land_mark_ids;  ///< 车道关联的地物要素id集合，可结合land_marks获取具体属性值
  float lane_width = 0.0;                          ///< 车道宽度（平均值），单位：m
  proto::perception::CenterLine::TypeRange::Type lane_type =
      proto::perception::CenterLine::TypeRange::kTypeUnknown;  ///< 车道类型，和感知输出对齐
  std::vector<std::string> left_boundary_ids;                  ///< 沿着车道通行方向左侧车道线id集合
  std::vector<std::string> right_boundary_ids;                 ///< 沿着车道通行方向右侧车道线id集合
  std::vector<Vec3d> points;                                   ///< 车道中心线形点序列(odom坐标)
  float length = 0.0;                                          ///< 车道长度，米
  std::vector<std::string> in_lane_ids;                        ///< 当前车道的所有进入车道id集合
  std::vector<std::string> out_lane_ids;                       ///< 当前车道的所有脱出车道id集合
};

/**
 * @brief 泊车自建图/SD导航信息中表示Road的统一结构体
 */
struct MapRoad {
  void clear() {
    id = "";
    road_direction = proto::MapCommon::kStartToEnd;
    driving_direction = proto::MapCommon::kDrivingUnknown;
    road_class = proto::MapCommon::kRoadInvalid;
    road_types.clear();
    speed_limit = 0;
    length = 0.0;
    points.clear();
    lane_num = 0;
    lanes.clear();
    lane_boundary_ids.clear();
    related_land_mark_ids.clear();
    floor = 0;

    in_road_ids.clear();
    out_road_ids.clear();
    adjacent_same_direction_road_ids.clear();
    adjacent_opposite_direction_road_ids.clear();
    related_junction_ids.clear();

    has_traffic_light = false;
    has_multi_out = false;
    has_parallel = false;
    main_action = proto::MapCommon::kMainActionNull;
    assistant_action = proto::MapCommon::kAssistantActionNull;
    turn_kind = proto::MapCommon::kTurnInvalid;
    is_on_route = true;
  }

  std::string id = "";                                                             ///< 道路区间ID
  proto::MapCommon::RoadDirection road_direction = proto::MapCommon::kStartToEnd;  ///< 道路通行方向
  proto::MapCommon::DrivingDirection driving_direction =
      proto::MapCommon::kDrivingUnknown;  ///< 道路转向信息（非路口为直行，路口场景时有其他类型）
  proto::MapCommon::RoadClass road_class = proto::MapCommon::kRoadInvalid;  ///< 道路等级
  std::vector<proto::MapCommon::RoadType> road_types;                       ///< 道路类型数组
  int32_t speed_limit = 0;                                                  ///< 道路级限速 单位：km/h
  float length = 0.0;                                                       ///< 道路长度，单位：m
  std::vector<Vec3d> points;                                                ///< 道路中心线形点序列(odom坐标)

  int32_t lane_num;  ///< 车道个数（SD数据中能获取车道数量，但可能获取不到车道详细信息）
  std::vector<MapLanePtr> lanes;  ///< 车道数组，按照通行方向从左往右排序，若未知则数组为空
  std::vector<std::string> lane_boundary_ids;  ///< 车道线数组（包含道路边界），从左往右排序，若未知则数组为空
  std::vector<std::string> related_land_mark_ids;  ///< 关联的地物要素id集合，可结合land_marks获取具体属性值
  int32_t floor = 0;                               ///< 道路所在楼层【预留】

  // 拓扑相关信息
  std::vector<std::string> in_road_ids;                           ///< 所有进入的road_id集合
  std::vector<std::string> out_road_ids;                          ///< 所有脱出的road_id集合
  std::vector<std::string> adjacent_same_direction_road_ids;      ///< 相邻同向的road_id集合
  std::vector<std::string> adjacent_opposite_direction_road_ids;  ///< 相邻反向的road_id集合
  std::vector<std::string> related_junction_ids;                  ///< 关联的路口ID集合

  // 专用于NavigationSd消息的字段
  bool has_traffic_light = false;                                                ///< 是否包含红绿灯
  bool has_multi_out = false;                                                    ///< 是否有岔路, 多个出口
  bool has_parallel = false;                                                     ///< 是否有平行路
  proto::MapCommon::MainAction main_action = proto::MapCommon::kMainActionNull;  // 主机动点
  proto::MapCommon::AssistantAction assistant_action = proto::MapCommon::kAssistantActionNull;  // 次机动点
  proto::MapCommon::TurnKind turn_kind = proto::MapCommon::kTurnInvalid;                        // 转向类型
  bool is_on_route = true;  ///< 是否在导航路线上
};

/**
 * @brief 来自地图的地物要素
 * 包含地面标记如停止线、地面箭头，地面凸起物如减速带以及地上交通元素等信息
 */
struct MapLandMark {
  void clear() {
    id = "";
    related_road_ids.clear();
    related_lane_ids.clear();
    main_type = proto::MapCommon::kLandMarkInvalid;
    sub_type = 0;
    name = "";
    shape_points.clear();
  }

  std::string id = "";                                                            ///< 地物要素id
  std::vector<std::string> related_road_ids;                                      ///< 关联的道路id
  std::vector<std::string> related_lane_ids;                                      ///< 关联的车道id
  proto::MapCommon::LandMarkType main_type = proto::MapCommon::kLandMarkInvalid;  ///< 地物要素主类型
  /**
   * @brief 地物要素子类型，
   *        - 如果是地面箭头，则子类型为箭头类型，ref@MapCommon.ArrowType
   *        - 如果是停车位，则子类型为库位类型，ref@PerceptionParkingSlot.ParkingSlotType
   */
  int32_t sub_type = 0;
  std::string name = "";            ///< 地物要素名称 【预留】
  std::vector<Vec3d> shape_points;  ///< 地物要素形状点序列(odom坐标)
};

struct MapJunction {
  void clear() {
    junction_id = "";
    in_road_segment_ids.clear();
    out_road_segment_ids.clear();
    shape_points.clear();
  }

  std::string junction_id;                        ///< 路口ID
  std::vector<std::string> in_road_segment_ids;   ///< 路口的入度集合
  std::vector<std::string> out_road_segment_ids;  ///< 路口的出度集合
  std::vector<Vec3d> shape_points;  ///< 路口面几何信息(逆时针排序、首尾闭合的polygon,odom类型坐标,不少于4个点)
};

struct LaneGroupInfo {
  void clear() {
    back_lanes.clear();
    front_lanes.clear();
    optimal_lanes.clear();
    back_exten_lanes.clear();
    front_exten_lanes.clear();
    extension_lanes.clear();
    point = Vec3d();
    road_index = -1;
    front_lane_types.clear();
    back_lane_types.clear();
    time_stamp = 0;
    dis_to_guide_point = -1.0;
  }

  std::vector<proto::MapCommon::LaneAction>
      back_lanes;  ///< 背景车道，巡航、导航都有效， 默认按通行方向从走往右的规则对车道排序，下同
  std::vector<proto::MapCommon::LaneAction> front_lanes;             ///< 前景车道，仅导航有效
  std::vector<proto::MapCommon::LaneAction> optimal_lanes;           ///< 建议车道，仅在线导航有效
  std::vector<proto::MapCommon::ExtenLaneAction> back_exten_lanes;   ///< 背景车道，扩展车道信息
  std::vector<proto::MapCommon::ExtenLaneAction> front_exten_lanes;  ///< 前景车道，扩展车道信息
  std::vector<proto::MapCommon::ExtenLaneAction> extension_lanes;    ///< 扩展车道信息
  Vec3d point;  // 车道坐标点，仅巡航有效，x:经度，y:纬度,z:预留（百度SD暂不支持）
  int32_t road_index = -1;                            // roads数组中对应的索引
  std::vector<LaneTypeRange::Type> front_lane_types;  ///< 前景分时车道类型，仅在线导航有效
  std::vector<LaneTypeRange::Type> back_lane_types;   ///< 背景分时车道类型，仅在线导航有效
  uint64_t time_stamp = 0;                            ///< 导航SDK给出该车道信息的时间戳，单位ms
  float dis_to_guide_point = -1.0;                    ///< 自车到引导点的距离，单位：m
};

struct GuideInfo {
  void clear() {
    road_idx_to_lane_group.clear();
    next_intersection.clear();
  }

  ///< 前方多组车道信息，如多个路口前的车道组的转向信息
  ///< map<相对routing_road_id_list_的road_index, LaneGroupInfo>
  std::map<int32_t, LaneGroupInfo> road_idx_to_lane_group;
  LaneGroupInfo next_intersection;  ///< 自车前方车道信息，如前方最近的路口的车道组的转向信息
};

}  // namespace road_instance
}  // namespace gpal::pnc::planning
