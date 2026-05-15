/**
 * @file local_router.h
 * @brief 本地路径路由器实现文件
 * @details
 * 该文件实现了本地路径路由器的核心功能，包括路径边界的重建、速度限制的重建、方向信息的重建等。本地路径路由器用于管理和处理车辆在局部路径上的各种信息，如边界、速度限制、方向等。
 */

#pragma once
#include <list>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// #include "math/box2d.h"
// #include "math/polygon2d.h"
// #include "math/line_segment2d.h"
#include "math/math_utils.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
#include "util/base_struct.h"

// #include "point/path_pt.h"
#include "gpal-interface/geometry.pb.h"
#include "memorized_route/memorized_route.h"
#include "perception_road_structure.h"

namespace gpal::pnc::planning {

struct LineAttributeRange {
  proto::road_cognition::LineAttributeRange::LineSourceType source_type =
      proto::road_cognition::LineAttributeRange_LineSourceType_kTypeInvalid;  // 来源类型
  proto::perception::CenterLine::TypeRange::Type line_type =
      proto::perception::CenterLine_TypeRange_Type_kTypeUnknown;  // 参考线类型
  float start_s = 0.0;                                            // 当前类型分段起点相对参考线的S坐标，米
  float end_s = 0.0;                                              // 当前类型分段终点相对参考线的S坐标，米
  math::Vec3d start_point;
  math::Vec3d end_point;

  void clear() {
    source_type = proto::road_cognition::LineAttributeRange_LineSourceType_kTypeInvalid;
    line_type = proto::perception::CenterLine_TypeRange_Type_kTypeUnknown;
    start_s = 0.0;
    end_s = 0.0;
    start_point = math::Vec3d();
    end_point = math::Vec3d();
  }
};

struct BoundaryRange {
  float start_s = 0.0;  ///< 路段边界的起始s值
  float end_s = 0.0;    ///< 路段边界的结束s值
  proto::road_cognition::BoundaryRange::BoundaryType type =
      proto::road_cognition::BoundaryRange_BoundaryType_kTypeCrossGuide;  ///< 路段边界类型
  proto::perception::LaneMarking::LineShape shape = proto::perception::LaneMarking_LineShape_kShapeUnknown;
  proto::perception::LaneMarking::LineColor color = proto::perception::LaneMarking_LineColor_kColorUnknown;
  std::vector<uint32_t> related_lane_markings_ids;
  math::Vec3d start_point;  ///< 路段边界的起始点
  math::Vec3d end_point;    ///< 路段边界的结束点

  void clear() {
    start_s = 0.0;
    end_s = 0.0;
    type = proto::road_cognition::BoundaryRange_BoundaryType_kTypeCrossGuide;
    shape = proto::perception::LaneMarking_LineShape_kShapeUnknown;
    color = proto::perception::LaneMarking_LineColor_kColorUnknown;
    related_lane_markings_ids.clear();
    start_point = math::Vec3d();
    end_point = math::Vec3d();
  }
};

struct RiskBoundary {
  std::string id = "";  // 风险边界ID
  proto::road_cognition::RiskBoundary::RiskBoundaryType type =
      proto::road_cognition::RiskBoundary_RiskBoundaryType_kTypeUnknown;  // 风险边界类型
  std::vector<math::Vec3d> points;                                        // 边界形状点，米，当前仅 x,y 维度有效
  std::vector<std::string> related_reference_lines_ids;                   // 关联的参考线ID数组

  void clear() {
    id = "";
    type = proto::road_cognition::RiskBoundary_RiskBoundaryType_kTypeUnknown;
    points.clear();
    related_reference_lines_ids.clear();
  }
};


struct GodBoundary {
  struct BoundaryPoint {
    BoundaryPoint(const float &_s, const float &_l) : s(_s), l(_l) {}
    float s = 0.0;
    float l = 0.0;
  };
  std::vector<BoundaryPoint> boundary;
  proto::PerceptionPolyline::PolylineType type = proto::PerceptionPolyline_PolylineType_kTypeInvalid;
};

struct EnvTrafficLight {
  std::string id = "";  // 交通灯ID
  proto::perception::TrafficLight::LightStatus status =
      proto::perception::TrafficLight_LightStatus_kStatusUnknown;  // 交通灯状态
  proto::perception::TrafficLight::LightColor color =
      proto::perception::TrafficLight_LightColor_kColorUnknown;  // 交通灯颜色
  proto::perception::TrafficLight::LightShape shape =
      proto::perception::TrafficLight_LightShape_kShapeUnknown;  // 交通灯形状
  int32_t countdown_second = -1;                                 // 交通灯倒计时秒数，秒，无倒计时时为-1
  std::vector<math::Vec3d> points;                               // 交通灯位置信息，米（使用数组表示，提高后续兼容性）
  std::vector<std::string> related_stop_lines_ids;               // 关联的停止线ID数组

  void clear() {
    id = "";
    status = proto::perception::TrafficLight_LightStatus_kStatusUnknown;
    color = proto::perception::TrafficLight_LightColor_kColorUnknown;
    shape = proto::perception::TrafficLight_LightShape_kShapeUnknown;
    countdown_second = -1;
    points.clear();
    related_stop_lines_ids.clear();
  }
};

struct EnvLaneMarking {
  std::string id = "";                                                                                   // 车道线ID
  proto::perception::LaneMarking::LineType type = proto::perception::LaneMarking_LineType_kTypeUnknown;  // 车道线类型
  proto::perception::LaneMarking::LineShape shape =
      proto::perception::LaneMarking_LineShape_kShapeUnknown;  // 车道线形状
  proto::perception::LaneMarking::LineColor color =
      proto::perception::LaneMarking_LineColor_kColorUnknown;  // 车道线颜色
  std::vector<math::Vec3d> points;  // 形点, 当前仅 x,y 维度有效, unit: m, 按车辆行驶方向单向排序;
  proto::perception::TrackingStatus tracking_status =
      proto::perception::TrackingStatus::kTrackingUnknown;  // 车道线跟踪状态
  float confidence = 1.0;                                   // 模型车道线的平均置信度，取值范围0~1，默认置信度最高

  void clear() {
    id = "";
    type = proto::perception::LaneMarking_LineType_kTypeUnknown;
    shape = proto::perception::LaneMarking_LineShape_kShapeUnknown;
    color = proto::perception::LaneMarking_LineColor_kColorUnknown;
    points.clear();
    tracking_status = proto::perception::TrackingStatus::kTrackingUnknown;
    confidence = 1.0;
  }
};

struct NeighborReferenceLine {
  std::string id = "";                    ///< 邻居参考线ID
  std::vector<BoundaryRange> boundaries;  ///< 邻居参考线在本参考线一侧沿s方向的共用车道边界信息

  void clear() {
    id = "";
    boundaries.clear();
  }
};

struct KeyPoint {
  struct Linkage {
    std::string link_id;  ///< 关联的对象ID
    uint32_t link_index;  ///< 关联对象中关联形点的索引, 当 link_id 为 0 时, 字段无效
    /**
     * @brief 关联对象的类型
     * - 0:未知
     * - 1:关联中心线
     * - 2:关联车道线或道路边界
     */
    uint32_t link_class;
  };

  std::string id = "";  ///< 关键点ID
  proto::road_cognition::KeyPoint::PointType type =
      proto::road_cognition::KeyPoint_PointType_kTypeUnknown;  ///< 关键点类型
  float s = 0.0;                                               ///< 关键点相对参考线的S坐标，米
  math::Vec3d pt;                                              ///< 关键点位置坐标，米
  std::vector<Linkage> linkages;                               ///< 与线的关联信息
  std::vector<std::string> related_reference_lines_ids;        ///< 相关其他参考线的ID数组

  void clear() {
    id = "";
    type = proto::road_cognition::KeyPoint_PointType_kTypeUnknown;
    s = 0.0;
    pt = math::Vec3d();
    linkages.clear();
    related_reference_lines_ids.clear();
  }
};

struct EnvRoad {
  std::string id = "";  // 道路ID
  proto::road_cognition::Road::RoadSourceType road_source =
      proto::road_cognition::Road_RoadSourceType_kSourceInvalid;                     // 道路信息来源类型
  std::string road_name = "";                                                        // 道路名称，若无有效道路名称则为空
  float length = 0.0;                                                                // 道路长度，米
  proto::MapCommon::RoadClass road_class = proto::MapCommon_RoadClass_kRoadInvalid;  // 所属道路等级
  std::vector<proto::MapCommon::RoadType> road_types;                                // 所属道路类型数组
  float max_speed_limit = 0.0;                                                       // 最大限速，m/s
  uint32_t lane_num = 0;                                                             // 车道个数
  std::vector<std::string> related_reference_lines_ids;                              // 关联的参考线ID数组

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
};

struct RoadRange {
  std::string id = "";  ///< 时空唯一, 来自地图Road/Link数据，若为空表示无效
  float start_s = 0.0;  // 起点相对参考线的S坐标，米
  float end_s = 0.0;    // 终点相对参考线的S坐标，米

  void clear() {
    id = "";
    start_s = 0.0;
    end_s = 0.0;
  }
};

struct LaneMarkingCrossAttributeRange {
  bool is_left_cross = false;            // 当前参考线是否向左跨线；向左跨线为true，向右跨线为false
  std::string lane_marking_id = "";      // 跨线时对应的车道边线id
  float before_cross_ref_start_s = 0.0;  // 跨线前，起点相对当前参考线的S坐标，米
  float before_cross_ref_end_s = 0.0;    // 跨线前，终点相对当前参考线的S坐标，米
  float after_cross_ref_start_s = 0.0;   // 跨线后，起点相对当前参考线的S坐标，米
  float after_cross_ref_end_s = 0.0;     // 跨线后，终点相对当前参考线的S坐标，米
  bool is_merge = false;                 // 是否为merge场景; merge场景为true, fork场景为false
  std::string related_ref_id = "";       // merge场景下为：跨线后，与当前参考线并线的相关参考线id；fork场景下为：跨线前，与参考线并线的相关参考线id
  float before_cross_related_ref_start_s = 0.0;  // 跨线前，起点相对相关参考线的S坐标，米
  float before_cross_related_ref_end_s = 0.0;    // 跨线前，终点相对当前参考线的S坐标，米
  float after_cross_related_ref_start_s = 0.0;   // 跨线后，起点相对相关参考线的S坐标，米
  float after_cross_related_ref_end_s = 0.0;     // 跨线后，终点相对当前参考线的S坐标，米

  void clear() {
    is_left_cross = false;
    lane_marking_id = "";
    before_cross_ref_start_s = 0.0;
    before_cross_ref_end_s = 0.0;
    after_cross_ref_start_s = 0.0;
    after_cross_ref_end_s = 0.0;
    is_merge = false;
    related_ref_id = "";
    before_cross_related_ref_start_s = 0.0;
    before_cross_related_ref_end_s = 0.0;
    after_cross_related_ref_start_s = 0.0;
    after_cross_related_ref_end_s = 0.0;
  }
};

/**
 * @brief LocalRoutePoint类用于表示LocalRoute中的一个点
 * @details 该类包含了点的位置信息、车道宽度、道路宽度等属性
 */
class LocalRoutePoint {
 public:
  /**
   * @brief 默认构造函数
   */
  LocalRoutePoint() = default;
  /**
   * @brief 构造LocalRoutePoint对象
   * @details 初始化路径点的完整空间属性及关联路段信息，用于描述局部路径中的关键位置点
   *
   * @param[in] segment_id 所属路段ID，标识该点归属的路径路段
   * @param[in] x 横向坐标（单位：m）
   * @param[in] y 纵向坐标（单位：m）
   * @param[in] z 高程坐标（单位：m）
   * @param[in] slope 路面坡度（单位：rad）
   * @param[in] theta 航向角（单位：rad）
   * @param[in] s 累计距离（单位：m）
   * @param[in] left_lane_width 左侧车道宽度（单位：m）
   * @param[in] right_lane_width 右侧车道宽度（单位：m）
   * @param[in] left_road_width 左侧道路宽度（单位：m）
   * @param[in] right_road_width 右侧道路宽度（单位：m）
   *
   * @par 典型使用场景:
   * 1. 路径点初始化时构造完整空间信息
   * 2. 导航模块生成路径点序列
   * 3. 地图数据加载时构造路径点对象
   *
   * @note 所有长度参数应保证非负，角度参数需归一化到标准范围
   * @warning 调用前需确保segment_id对应的路段已存在，s值应保持单调递增
   */
  LocalRoutePoint(const std::string& segment_id, const double& x, const double& y, const float& z, const float& slope,
                  const float& theta, const float& s, const float& left_lane_width, const float& right_lane_width,
                  const float& left_road_width, const float& right_road_width)
      : segment_id_(segment_id),
        x_(x),
        y_(y),
        z_(z),
        slope_(slope),
        theta_(theta),
        s_(s),
        left_lane_width_(left_lane_width),
        right_lane_width_(right_lane_width),
        left_road_width_(left_road_width),
        right_road_width_(right_road_width) {}

  /**
   * @brief 默认析构函数
   */
  ~LocalRoutePoint() = default;

  /**
   * @brief 获取点的x坐标
   * @return 返回点的x坐标
   */
  const double& x() const { return x_; }
  /**
   * @brief 设置点的x坐标
   * @param x 要设置的x坐标值
   */
  void setX(const double& x) { x_ = x; }

  /**
   * @brief 获取点的y坐标
   * @return 返回点的y坐标
   */
  const double& y() const { return y_; }
  /**
   * @brief 设置点的y坐标
   * @param y 要设置的y坐标值
   */
  void setY(const double& y) { y_ = y; }

  /**
   * @brief 获取点的z坐标
   * @return 返回点的z坐标
   */
  const float& z() const { return z_; }
  /**
   * @brief 设置点的z坐标
   * @param z 要设置的z坐标值
   */
  void setZ(const float& z) { z_ = z; }

  /**
   * @brief 获取点的坡度
   * @return 返回点的坡度值
   */
  const float& slope() const { return slope_; }
  /**
   * @brief 设置点的坡度
   * @param slope 要设置的坡度值
   */
  void setSlope(const float& slope) { slope_ = slope; }

  /**
   * @brief 获取点的航向角
   * @return 返回点的航向角
   */
  const float& theta() const { return theta_; }
  /**
   * @brief 设置点的航向角
   * @param theta 要设置的航向角值
   */
  void setTheta(const float& theta) { theta_ = theta; }

  /**
   * @brief 获取点的曲率
   * @return 返回点的曲率值
   */
  const float& kappa() const { return kappa_; }
  /**
   * @brief 设置点的曲率
   * @param kappa 要设置的曲率值
   */
  void setKappa(const float& kappa) { kappa_ = kappa; }

  /**
   * @brief 获取点的曲率变化率
   * @return 返回点的曲率变化率
   */
  const float& dkappa() const { return dkappa_; }
  /**
   * @brief 设置点的曲率变化率
   * @param dkappa 要设置的曲率变化率值
   */
  void setDkappa(const float& dkappa) { dkappa_ = dkappa; }

  /**
   * @brief 获取点在路径上的累计距离
   * @return 返回点的累计距离值
   */
  const float& s() const { return s_; }
  /**
   * @brief 设置点在路径上的累计距离
   * @param s 要设置的累计距离值
   */
  void setS(const float& s) { s_ = s; }

  /**
   * @brief 获取点的左侧车道宽度
   * @return 返回点的左侧车道宽度
   */
  const float& leftLaneWidth() const { return left_lane_width_; }
  /**
   * @brief 设置点的左侧车道宽度
   * @param left_lane_width 要设置的左侧车道宽度值
   */
  void setLeftLaneWidth(const float& left_lane_width) { left_lane_width_ = left_lane_width; }

  /**
   * @brief 获取点的右侧车道宽度
   * @return 返回点的右侧车道宽度
   */
  const float& rightLaneWidth() const { return right_lane_width_; }
  /**
   * @brief 设置点的右侧车道宽度
   * @param right_lane_width 要设置的右侧车道宽度值
   */
  void setRightLaneWidth(const float& right_lane_width) { right_lane_width_ = right_lane_width; }

  /**
   * @brief 获取点的左侧道路宽度
   * @return 返回点的左侧道路宽度
   */
  const float& leftRoadWidth() const { return left_road_width_; }
  /**
   * @brief 设置点的左侧道路宽度
   * @param left_road_width 要设置的左侧道路宽度值
   */
  void setLeftRoadWidth(const float& left_road_width) { left_road_width_ = left_road_width; }

  /**
   * @brief 获取点的右侧道路宽度
   * @return 返回点的右侧道路宽度
   */
  const float& rightRoadWidth() const { return right_road_width_; }
  /**
   * @brief 设置点的右侧道路宽度
   * @param right_road_width 要设置的右侧道路宽度值
   */
  void setRightRoadWidth(const float& right_road_width) { right_road_width_ = right_road_width; }

  /**
   * @brief 获取点所属的路段ID
   * @return 返回点所属的路段ID
   */
  const std::string& segmentId() const { return segment_id_; }
  /**
   * @brief 设置点所属的路段ID
   * @param segment_id 要设置的路段ID
   */
  void setSegmentId(const std::string& segment_id) { segment_id_ = segment_id; }

 private:
  double x_ = 0.0;                ///< 点的x坐标
  double y_ = 0.0;                ///< 点的y坐标
  float z_ = 0.0;                 ///< 点的z坐标
  float slope_ = 0.0;             ///< 点的坡度
  float theta_ = 0.0;             ///< 点的航向角
  float kappa_ = 0.0;             ///< 点的曲率
  float dkappa_ = 0.0;            ///< 点的曲率变化率
  float s_ = 0.0;                 ///< 点在路径上的累计距离
  float left_lane_width_ = 0.0;   ///< 点的左侧车道宽度
  float right_lane_width_ = 0.0;  ///< 点的右侧车道宽度
  float left_road_width_ = 0.0;   ///< 点的左侧道路宽度
  float right_road_width_ = 0.0;  ///< 点的右侧道路宽度
  std::string segment_id_ = "";   ///< 点所属的路段ID
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
   * @brief 构造函数，初始化停止线的基本信息
   * @param _x 停止线中心点的x坐标
   * @param _y 停止线中心点的y坐标
   * @param _z 停止线中心点的z坐标
   * @param _s 停止线中心点在LocalRoute的guide_line_上的投影s位置
   * @param _direction 停止线的方向
   */
  StopLine(const double& _x, const double& _y, const float& _z, const float& _s, const DrivingDirection& _direction)
      : x(_x), y(_y), z(_z), s(_s), direction(_direction) {}

  /**
   * @brief 构造函数，初始化停止线的详细信息
   * @param _id 停止线的ID
   * @param _is_virtual 停止线是否为虚拟停止线
   * @param _x 停止线中心点的x坐标
   * @param _y 停止线中心点的y坐标
   * @param _z 停止线中心点的z坐标
   * @param _s 停止线中心点在LocalRoute的guide_line_上的投影s位置
   * @param _traffic_light_id 关联的信号灯ID
   * @param _direction 停止线的方向
   */
  StopLine(const std::string& _id, const bool& _is_virtual, const double& _x, const double& _y, const float& _z,
           const float& _s, const std::string& _traffic_light_id, const DrivingDirection& _direction)
      : id(_id),
        is_virtual(_is_virtual),
        x(_x),
        y(_y),
        z(_z),
        s(_s),
        traffic_light_id(_traffic_light_id),
        direction(_direction) {}

  /**
   * @brief 默认析构函数
   */
  ~StopLine() = default;
  bool is_exist = false;    ///< 停止线是否存在
  std::string id = "";      ///< 停止线的ID
  bool is_virtual = false;  ///< 停止线是否为虚拟停止线
  proto::perception::StopLine::LineType type = proto::perception::StopLine_LineType_kTypeUnknown;  ///< 停止线类型
  double x = 0.0;                                                    ///< 停止线中心点的x坐标
  double y = 0.0;                                                    ///< 停止线中心点的y坐标
  float z = 0.0;                                                     ///< 停止线中心点的z坐标
  float s = 0.0;                                                     ///< 中心坐标在LocalRoute的guide_line_上的投影s位置
  std::string traffic_light_id = "";                                ///< 关联的信号灯id, 默认值无关联
  uint32_t traffic_light_priority = 0;
  DrivingDirection direction = DrivingDirection::kDirectionInvalid;  ///< 停止线绑定的行驶方向
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
   * @param _gate_status 闸机的状态
   * @param _x 闸机中心点的x坐标
   * @param _y 闸机中心点的y坐标
   * @param _z 闸机中心点的z坐标
   * @param _s 闸机中心点在LocalRoute的guide_line_上的投影s位置
   * @param _head_stop_distance 期望车头停车距离
   */
  Gate(const std::string& _id, const proto::perception::Gate::GateStatus& _gate_status, const double& _x,
       const double& _y, const double& _z, const float& _s, const float& _head_stop_distance)
      : id(_id), gate_status(_gate_status), x(_x), y(_y), z(_z), s(_s), head_stop_distance(_head_stop_distance) {}

  /**
   * @brief 默认析构函数
   */
  ~Gate() = default;
  bool is_exist = false;  ///< 闸机是否存在
  std::string id = "";    ///< 闸机的ID
  proto::road_cognition::GateRange::GateType type =
      proto::road_cognition::GateRange_GateType_kTypeUnknown;                                           ///< 闸机的类型
  proto::perception::Gate::GateStatus gate_status = proto::perception::Gate_GateStatus_kStatusUnknown;  ///< 闸机的状态
  double x = 0.0;                  ///< 闸机中心点的x坐标
  double y = 0.0;                  ///< 闸机中心点的y坐标
  float z = 0.0;                   ///< 闸机中心点的z坐标
  float s = 0.0;                   ///< 中心坐标在LocalRoute的guide_line_上的投影s位置
  float head_stop_distance = 0.0;  ///< 期望车头停车距离
};

/**
 * @brief LocalRouteSegment类用于表示LocalRoute中的一个路段
 * @details 该类包含了路段的导航信息、边界类型、方向、停止线、闸机、速度限制等属性
 */
class LocalRouteSegment {
 public:
  /**
   * @brief 默认构造函数
   */
  LocalRouteSegment() = default;
  /**
   * @brief 默认析构函数
   */
  ~LocalRouteSegment() = default;

  /**
   * @brief 边界类型枚举，表示路段的边界类型
   */
  enum class BoundaryType {
    UNKNOWN = 0,                ///< 未知类型
    DASHED = 1,                 ///< 虚线
    SOLID = 2,                  ///< 实线
    PHYSICALLY_UNCROSSABLE = 3  ///< 物理不可跨越
  };

  /**
   * @brief 获取路段的ID
   * @return 返回路段的ID
   */
  const std::string& id() const { return id_; }
  /**
   * @brief 设置路段的ID
   * @param id 要设置的路段ID
   */
  void setId(const std::string& id) { id_ = id; }

  /**
   * @brief 获取路段的导航评分
   * @return 返回路段的导航评分
   */
  const float& navigationScore() const { return navigation_score_; }
  /**
   * @brief 设置路段的导航评分
   * @param navigation_score 要设置的导航评分
   */
  void setNavigationScore(const float& navigation_score) { navigation_score_ = navigation_score; }

  /**
   * @brief 获取路段的车道变更距离
   * @return 返回路段的车道变更距离
   */
  const float& navigationLaneChangeDistance() const { return navigation_lane_change_distance_; }
  /**
   * @brief 设置路段的车道变更距离
   * @param navigation_lane_change_distance 要设置的车道变更距离
   */
  void setNavigationLaneChangeDistance(const float& navigation_lane_change_distance) {
    navigation_lane_change_distance_ = navigation_lane_change_distance;
  }

  /**
   * @brief 获取路段的范围信息
   * @return 返回路段的范围信息，包含是否存在、起始s值和结束s值
   */
  const std::tuple<bool, float, float>& range() const { return range_; }
  /**
   * @brief 获取可修改的路段范围信息
   * @return 返回可修改的路段范围信息指针
   */
  std::tuple<bool, float, float>* mutableRange() { return &range_; }
  /**
   * @brief 设置路段的范围信息
   * @param range 要设置的范围信息
   */
  void setRange(const std::tuple<bool, float, float>& range) { range_ = range; }

  /**
   * @brief 获取路段范围的起止点
   * @return 返回路段范围的起止点
   */
  const std::pair<LocalRoutePoint, LocalRoutePoint>& rangePoints() const { return range_points_; }
  /**
   * @brief 获取可修改的路段范围起止点
   * @return 返回可修改的路段范围起止点指针
   */
  std::pair<LocalRoutePoint, LocalRoutePoint>* mutableRangePoints() { return &range_points_; }
  /**
   * @brief 设置路段范围的起止点
   * @param range_points 要设置的起止点
   */
  void setRangePoints(const std::pair<LocalRoutePoint, LocalRoutePoint>& range_points) { range_points_ = range_points; }

  /**
   * @brief 获取路段的引导点
   * @return 返回路段的引导点列表
   */
  const std::vector<LocalRoutePoint>& guidePoints() const { return guide_points_; }
  /**
   * @brief 获取可修改的路段引导点
   * @return 返回可修改的路段引导点指针
   */
  std::vector<LocalRoutePoint>* mutableGuidePoints() { return &guide_points_; }
  /**
   * @brief 设置路段的引导点
   * @param guide_points 要设置的引导点列表
   */
  void setGuidePoints(const std::vector<LocalRoutePoint>& guide_points) { guide_points_ = guide_points; }

  /**
   * @brief 获取路段的边界点
   * @return 返回路段的边界点列表
   */
  const std::vector<std::pair<math::Vec3d, math::Vec3d>>& boundaryPoints() const { return boundary_points_; }
  /**
   * @brief 获取可修改的路段边界点
   * @return 返回可修改的路段边界点指针
   */
  std::vector<std::pair<math::Vec3d, math::Vec3d>>* mutableBoundaryPoints() { return &boundary_points_; }
  /**
   * @brief 设置路段的边界点
   * @param boundary_points 要设置的边界点列表
   */
  void setBoundaryPoints(const std::vector<std::pair<math::Vec3d, math::Vec3d>>& boundary_points) {
    boundary_points_ = boundary_points;
  }

  /**
   * @brief 获取路段的前驱路段ID
   * @return 返回路段的前驱路段ID列表
   */
  const std::vector<std::string>& previousSegmentIds() const { return previous_segment_ids_; }
  /**
   * @brief 获取可修改的路段前驱路段ID
   * @return 返回可修改的路段前驱路段ID指针
   */
  std::vector<std::string>* mutablePreviousSegmentIds() { return &previous_segment_ids_; }
  /**
   * @brief 设置路段的前驱路段ID
   * @param previous_segment_ids 要设置的前驱路段ID列表
   */
  void setPreviousSegmentIds(const std::vector<std::string>& previous_segment_ids) {
    previous_segment_ids_ = previous_segment_ids;
  }

  /**
   * @brief 获取路段的后继路段ID
   * @return 返回路段的后继路段ID列表
   */
  const std::vector<std::string>& nextSegmentIds() const { return next_segment_ids_; }
  /**
   * @brief 获取可修改的路段后继路段ID
   * @return 返回可修改的路段后继路段ID指针
   */
  std::vector<std::string>* mutableNextSegmentIds() { return &next_segment_ids_; }
  /**
   * @brief 设置路段的后继路段ID
   * @param next_segment_ids 要设置的后继路段ID列表
   */
  void setNextSegmentIds(const std::vector<std::string>& next_segment_ids) { next_segment_ids_ = next_segment_ids; }

  /**
   * @brief 获取路段的左邻路段ID
   * @return 返回路段的左邻路段ID
   */
  const std::string& leftNeighborId() const { return left_neighbor_id_; }
  /**
   * @brief 获取可修改的路段左邻路段ID
   * @return 返回可修改的路段左邻路段ID指针
   */
  void setLeftNeighborId(const std::string& left_neighbor_id) { left_neighbor_id_ = left_neighbor_id; }

  /**
   * @brief 获取路段的右邻路段ID
   * @return 返回路段的右邻路段ID
   */
  const std::string& rightNeighborId() const { return right_neighbor_id_; }
  /**
   * @brief 获取可修改的路段右邻路段ID
   * @return 返回可修改的路段右邻路段ID指针
   */
  void setRightNeighborId(const std::string& right_neighbor_id) { right_neighbor_id_ = right_neighbor_id; }

  /**
   * @brief 获取路段的对向左邻路段的ID
   * @return 返回路段的对向左邻路段的ID
   */
  const std::string& leftOppositeNeighborId() const { return left_opposite_neighbor_id_; }
  /**
   * @brief 获取可修改的路段对向左邻路段的ID
   * @return 返回可修改的路段对向左邻路段的ID指针
   */
  void setLeftOppositeNeighborId(const std::string& left_opposite_neighbor_id) {
    left_opposite_neighbor_id_ = left_opposite_neighbor_id;
  }

  /**
   * @brief 获取路段的类型
   * @return 返回路段的类型
   */
  const PerceptionLaneSegment::LaneSegmentType& type() const { return type_; }
  /**
   * @brief 设置路段的类型
   * @param type 要设置的路段类型
   */
  void setType(const PerceptionLaneSegment::LaneSegmentType& type) { type_ = type; }

  /**
   * @brief 获取路段的左侧边界类型
   * @return 返回路段的左侧边界类型
   */
  const BoundaryType& leftBoundType() const { return left_bound_type_; }
  /**
   * @brief 设置路段的左侧边界类型
   * @param left_bound_type 要设置的左侧边界类型
   */
  void setLeftBoundType(const BoundaryType& left_bound_type) { left_bound_type_ = left_bound_type; }

  /**
   * @brief 获取路段的右侧边界类型
   * @return 返回路段的右侧边界类型
   */
  const BoundaryType& rightBoundType() const { return right_bound_type_; }
  /**
   * @brief 设置路段的右侧边界类型
   * @param right_bound_type 要设置的右侧边界类型
   */
  void setRightBoundType(const BoundaryType& right_bound_type) { right_bound_type_ = right_bound_type; }

  /**
   * @brief 获取路段的方向
   * @return 返回路段的方向
   */
  const DrivingDirection& direction() const { return direction_; }
  /**
   * @brief 设置路段的方向
   * @param direction 要设置的方向
   */
  void setDirection(const DrivingDirection& direction) { direction_ = direction; }

  /**
   * @brief 获取路段的区域信息
   * @return 返回路段的区域信息列表
   */
  const std::vector<PerceptionArea>& areas() const { return areas_; }
  /**
   * @brief 获取可修改的路段区域信息
   * @return 返回可修改的路段区域信息指针
   */
  std::vector<PerceptionArea>* mutableAreas() { return &areas_; }
  /**
   * @brief 设置路段的区域信息
   * @param areas 要设置的区域信息列表
   */
  void setAreas(const std::vector<PerceptionArea>& areas) { areas_ = areas; }

  /**
   * @brief 获取路段的停止线信息
   * @return 返回路段的停止线信息
   */
  const StopLine& stopLine() const { return stop_line_; }
  /**
   * @brief 获取可修改的路段停止线信息
   * @return 返回可修改的路段停止线信息指针
   */
  StopLine* mutableStopLine() { return &stop_line_; }
  /**
   * @brief 设置路段的停止线信息
   * @param stop_line 要设置的停止线信息
   */
  void setStopLine(const StopLine& stop_line) { stop_line_ = stop_line; }

  /**
   * @brief 获取路段的闸机信息
   * @return 返回路段的闸机信息
   */
  const Gate& gate() const { return gate_; }
  /**
   * @brief 获取可修改的路段闸机信息
   * @return 返回可修改的路段闸机信息指针
   */
  Gate* mutableGate() { return &gate_; }
  /**
   * @brief 设置路段的闸机信息
   * @param gate 要设置的闸机信息
   */
  void setGate(const Gate& gate) { gate_ = gate; }

  /**
   * @brief 获取路段的速度限制信息
   * @return 返回路段的速度限制信息
   */
  const SpeedLimit& speedLimit() const { return speed_limit_; }
  /**
   * @brief 获取可修改的路段速度限制信息
   * @return 返回可修改的路段速度限制信息指针
   */
  SpeedLimit* mutableSpeedLimit() { return &speed_limit_; }
  /**
   * @brief 设置路段的速度限制信息
   * @param speed_limit 要设置的速度限制信息
   */
  void setSpeedLimit(const SpeedLimit& speed_limit) { speed_limit_ = speed_limit; }

  /**
   * @brief 获取自车在此路段上的s值
   * @return 返回自车在此路段上的s值
   */
  const float& adcS() const { return adc_s_; }
  /**
   * @brief 设置自车在此路段上的s值
   * @param adc_s 要设置的自车在此路段上的s值
   */
  void setAdcS(const float& adc_s) { adc_s_ = adc_s; }

  /**
   * @brief 获取自车在此路段上的l值
   * @return 返回自车在此路段上的l值
   */
  const float& adcL() const { return adc_l_; }
  /**
   * @brief 设置自车在此路段上的l值
   * @param adc_l 要设置的自车在此路段上的l值
   */
  void setAdcL(const float& adc_l) { adc_l_ = adc_l; }

  /**
   * @brief 获取自车航向角与此路段方向的夹角
   * @return 返回自车航向角与此路段方向的夹角
   */
  const float& adcHeadingDiff() const { return adc_heading_diff_; }
  /**
   * @brief 设置自车航向角与此路段方向的夹角
   * @param adc_heading_diff 要设置的自车航向角与此路段方向的夹角
   */
  void setAdcHeadingDiff(const float& adc_heading_diff) { adc_heading_diff_ = adc_heading_diff; }

  /**
   * @brief 获取路段的长度
   * @return 返回路段的长度
   */
  const float length() const { return guide_points_.empty() ? 0.0 : guide_points_.back().s(); }

  /**
   * @brief 获取路段的线源信息
   * @return 返回路段的线源信息列表
   */
  const std::vector<std::tuple<LineSourceType, float, float>>& sourceInfos() const { return source_infos_; }
  /**
   * @brief 获取可修改的路段线源信息
   * @return 返回可修改的路段线源信息指针
   */
  std::vector<std::tuple<LineSourceType, float, float>>* mutableSourceInfos() { return &source_infos_; }
  /**
   * @brief 设置路段的线源信息
   * @param source_infos 要设置的线源信息列表
   */
  void setSourceInfos(const std::vector<std::tuple<LineSourceType, float, float>>& source_infos) {
    source_infos_ = source_infos;
  }

 private:
  std::string id_ = "";                                               ///< 路段的ID
  float navigation_score_ = 0.0;                                      ///< 导航评分
  float navigation_lane_change_distance_ = -1.0;                      ///< 导航换道位置距离
  std::tuple<bool, float, float> range_ = {false, 0.0, 0.0};          ///< range范围: 是否存在，start_s, end_s
  std::pair<LocalRoutePoint, LocalRoutePoint> range_points_;          ///< range起止点: start_point, end_point
  std::vector<LocalRoutePoint> guide_points_;                         ///< 路段的引导点
  std::vector<std::pair<math::Vec3d, math::Vec3d>> boundary_points_;  ///< 路段的边界点
  std::vector<std::string> previous_segment_ids_;                     ///< 路段的前驱路段ID
  std::vector<std::string> next_segment_ids_;                         ///< 路段的后继路段ID
  std::string left_neighbor_id_ = "";                                 ///< 路段的左邻路段ID
  std::string right_neighbor_id_ = "";                                ///< 路段的右邻路段ID
  std::string left_opposite_neighbor_id_ = "";                        ///< 路段的对向左邻路段的ID
  PerceptionLaneSegment::LaneSegmentType type_ = PerceptionLaneSegment::LaneSegmentType::kTypeInvalid;  ///< 路段的类型
  BoundaryType left_bound_type_ = BoundaryType::UNKNOWN;                ///< 路段的左侧边界类型
  BoundaryType right_bound_type_ = BoundaryType::UNKNOWN;               ///< 路段的右侧边界类型
  DrivingDirection direction_ = DrivingDirection::kDirectionInvalid;    ///< 路段的方向
  std::vector<PerceptionArea> areas_;                                   ///< 路段的区域信息
  Gate gate_;                                                           ///< 路段的闸机信息
  StopLine stop_line_;                                                  ///< 路段的停止线信息
  SpeedLimit speed_limit_;                                              ///< 路段的速度限制信息
  float adc_s_ = 0.0;                                                   ///< 自车在此路段上的s值
  float adc_l_ = 1000.0;                                                ///< 自车在此路段上的l值
  float adc_heading_diff_ = 0.0;                                        ///< 自车航向角与此路段方向的夹角
  std::vector<std::tuple<LineSourceType, float, float>> source_infos_;  ///< 路段的线源信息
};

/**
 * @brief BoundaryCrossRange类用于表示边界跨越范围信息
 * @details 该类包含了边界类型、范围信息、起止点、关联路段ID等属性
 */
class BoundaryCrossRange {
 public:
  /**
   * @brief 默认构造函数
   */
  BoundaryCrossRange() = default;
  /**
   * @brief 默认析构函数
   */
  ~BoundaryCrossRange() = default;

  /**
   * @brief 获取边界类型
   * @return 返回边界类型
   */
  const LocalRouteSegment::BoundaryType& boundType() const { return bound_type_; }
  /**
   * @brief 设置边界类型
   * @param bound_type 要设置的边界类型
   */
  void setBoundType(const LocalRouteSegment::BoundaryType& bound_type) { bound_type_ = bound_type; }

  /**
   * @brief 获取边界范围信息
   * @return 返回边界范围信息，包含是否存在、起始s值和结束s值
   */
  const std::tuple<bool, float, float>& boundRange() const { return bound_range_; }
  /**
   * @brief 获取可修改的边界范围信息
   * @return 返回可修改的边界范围信息指针
   */
  std::tuple<bool, float, float>* mutableBoundRange() { return &bound_range_; }
  /**
   * @brief 设置边界范围信息
   * @param bound_range 要设置的边界范围信息
   */
  void setBoundRange(const std::tuple<bool, float, float>& bound_range) { bound_range_ = bound_range; }

  /**
   * @brief 获取边界范围的起止点
   * @return 返回边界范围的起止点
   */
  const std::pair<LocalRoutePoint, LocalRoutePoint>& boundRangePoints() const { return bound_range_points_; }
  /**
   * @brief 获取可修改的边界范围起止点
   * @return 返回可修改的边界范围起止点指针
   */
  std::pair<LocalRoutePoint, LocalRoutePoint>* mutableBoundRangePoints() { return &bound_range_points_; }
  /**
   * @brief 设置边界范围的起止点
   * @param bound_range_points 要设置的边界范围起止点
   */
  void setBoundRangePoints(const std::pair<LocalRoutePoint, LocalRoutePoint>& bound_range_points) {
    bound_range_points_ = bound_range_points;
  }

  /**
   * @brief 获取关联的路段ID
   * @return 返回关联的路段ID列表
   */
  const std::vector<std::string>& segmentsId() const { return segments_id_; }
  /**
   * @brief 获取可修改的关联路段ID
   * @return 返回可修改的关联路段ID指针
   */
  std::vector<std::string>* mutableSegmentsId() { return &segments_id_; }
  /**
   * @brief 设置关联的路段ID
   * @param segments_id 要设置的关联路段ID列表
   */
  void setSegmentsId(const std::vector<std::string>& segments_id) { segments_id_ = segments_id; }

  /**
   * @brief 获取边界范围对应的路段范围信息
   * @return 返回边界范围对应的路段范围信息列表
   */
  const std::vector<std::tuple<bool, float, float>>& segmentsRange() const { return segments_range_; }
  /**
   * @brief 获取可修改的边界范围对应的路段范围信息
   * @return 返回可修改的边界范围对应的路段范围信息指针
   */
  std::vector<std::tuple<bool, float, float>>* mutableSegmentsRange() { return &segments_range_; }
  /**
   * @brief 设置边界范围对应的路段范围信息
   * @param segments_range 要设置的路段范围信息列表
   */
  void setSegmentsRange(const std::vector<std::tuple<bool, float, float>>& segments_range) {
    segments_range_ = segments_range;
  }

  /**
   * @brief 获取边界范围对应的路段范围的起止点
   * @return 返回边界范围对应的路段范围的起止点列表
   */
  const std::vector<std::pair<LocalRoutePoint, LocalRoutePoint>>& segmentsRangePoints() const {
    return segments_range_points_;
  }
  /**
   * @brief 获取可修改的边界范围对应的路段范围的起止点
   * @return 返回可修改的边界范围对应的路段范围的起止点列表指针
   */
  std::vector<std::pair<LocalRoutePoint, LocalRoutePoint>>* mutableSegmentsRangePoints() {
    return &segments_range_points_;
  }
  /**
   * @brief 设置边界范围对应的路段范围的起止点
   * @param segments_range_points 要设置的路段范围的起止点列表
   */
  void setSegmentsRangePoints(const std::vector<std::pair<LocalRoutePoint, LocalRoutePoint>>& segments_range_points) {
    segments_range_points_ = segments_range_points;
  }

  /**
   * @brief 获取边界范围对应的邻接路段ID及其所属的所有LocalRoute IDs, neighbor seg id and all neighbor local routes id
   * @return 返回边界范围对应的邻接路段ID及其所属的所有LocalRoute IDs列表
   */
  const std::vector<std::pair<std::string, std::vector<int>>>& neighborSegmentsId() const {
    return neighbor_segments_id_;
  }
  /**
   * @brief 获取可修改的边界范围对应的邻接路段ID及其所属的所有LocalRoute IDs
   * @return 返回可修改的边界范围对应的邻接路段ID及其所属的所有LocalRoute IDs列表指针
   */
  std::vector<std::pair<std::string, std::vector<int>>>* mutableNeighborSegmentsId() { return &neighbor_segments_id_; }
  /**
   * @brief 设置边界范围对应的邻接路段ID及其所属的所有LocalRoute IDs
   * @param neighbor_segments_id 要设置的邻接路段ID及其所属的所有LocalRoute IDs列表
   */
  void setNeighborSegmentsId(const std::vector<std::pair<std::string, std::vector<int>>>& neighbor_segments_id) {
    neighbor_segments_id_ = neighbor_segments_id;
  }

 private:
  LocalRouteSegment::BoundaryType bound_type_ = LocalRouteSegment::BoundaryType::UNKNOWN;  ///< range可跨越类型
  std::tuple<bool, float, float> bound_range_ = {false, 0.0, 0.0};  ///< range范围: 是否存在，start_s, end_s
  std::pair<LocalRoutePoint, LocalRoutePoint> bound_range_points_;  ///< range起止点: start_point, end_point
  std::vector<std::string> segments_id_;                            ///< range对应的本LocalRoute中的路段id
  std::vector<std::tuple<bool, float, float>>
      segments_range_;  ///< range对应的本LocalRoute中的路段id的range, 默认segments_range.size() == segments_id.size()
  std::vector<std::pair<LocalRoutePoint, LocalRoutePoint>>
      segments_range_points_;  ///< range对应的本LocalRoute中的路段range的起止点, segments_range_points.size() ==
                               ///< segments_id.size()
  std::vector<std::pair<std::string, std::vector<int>>>
      neighbor_segments_id_;  ///< range对应的邻接路段id及其所属的所有LocalRoute ids, 默认neighbor_segments_id.size() ==
                              ///< segments_id.size()
};

/**
 * @brief MergeForkRange类用于表示合并或分叉的范围信息
 * @details 该类包含了合并或分叉的类型、位置、关联的ID信息等属性
 */
class MergeForkRange {
 public:
  /**
   * @brief 默认构造函数
   */
  MergeForkRange() = default;
  /**
   * @brief 默认析构函数
   */
  ~MergeForkRange() = default;

  /**
   * @brief 合并或分叉类型枚举
   */
  enum class MergeForkType {
    UNKNOWN = 0,  ///< 未知类型
    MERGE = 1,    ///< 合并
    FORK = 2,     ///< 分叉
  };

  /**
   * @brief 获取合并或分叉类型
   * @return 返回合并或分叉类型
   */
  const MergeForkType& type() const { return type_; }
  /**
   * @brief 设置合并或分叉类型
   * @param type 要设置的合并或分叉类型
   */
  void setType(const MergeForkType& type) { type_ = type; }

  /**
   * @brief 获取合并或分叉的s值
   * @return 返回合并或分叉的s值
   */
  const float& s() const { return s_; }
  /**
   * @brief 设置合并或分叉的s值
   * @param s 要设置的合并或分叉的s值
   */
  void setS(const float& s) { s_ = s; }

  /**
   * @brief 获取合并或分叉的点
   * @return 返回合并或分叉的点
   */
  const LocalRoutePoint& point() const { return point_; }
  /**
   * @brief 设置合并或分叉的点
   * @param point 要设置的合并或分叉的点
   */
  void setPoint(const LocalRoutePoint& point) { point_ = point; }

  /**
   * @brief 获取ID信息
   * @return 返回ID信息，包含lane id和lane segment id
   */
  const std::pair<std::string, std::string>& idInfo() const { return id_info_; }
  /**
   * @brief 获取可修改的ID信息
   * @return 返回可修改的ID信息指针
   */
  std::pair<std::string, std::string>* mutableIdInfo() { return &id_info_; }
  /**
   * @brief 设置ID信息
   * @param id_info 要设置的ID信息
   */
  void setIdInfo(const std::pair<std::string, std::string>& id_info) { id_info_ = id_info; }

  /**
   * @brief 获取关联拓扑的ID信息
   * @return 返回关联拓扑的ID信息，包含lane id, lane segment id, all local routes id(string)
   */
  const std::vector<std::tuple<std::string, std::string, std::vector<std::string>>>& relationIdsInfo() const {
    return relation_ids_info_;
  }
  /**
   * @brief 获取可修改的关联拓扑ID信息
   * @return 返回可修改的关联拓扑ID信息指针
   */
  std::vector<std::tuple<std::string, std::string, std::vector<std::string>>>* mutableRelationIdsInfo() {
    return &relation_ids_info_;
  }
  /**
   * @brief 设置关联拓扑的ID信息
   * @param relation_ids_info 要设置的关联拓扑ID信息
   */
  void setRelationIdsInfo(
      const std::vector<std::tuple<std::string, std::string, std::vector<std::string>>>& relation_ids_info) {
    relation_ids_info_ = relation_ids_info;
  }

 private:
  MergeForkType type_ = MergeForkType::UNKNOWN;  ///< 合并或分叉类型
  float s_ = 0.0;                                ///< 合并或分叉的s值, merge or fork start s
  LocalRoutePoint point_;                        ///< 合并或分叉点, merge or fork point
  std::pair<std::string, std::string> id_info_;  ///< 合并或分叉的ID信息，包含车道ID和路段ID, lane id, lane segment id
  std::vector<std::tuple<std::string, std::string, std::vector<std::string>>>
      relation_ids_info_;  ///< 合并或分叉的关联拓扑ID信息列表: 合并情况：前驱车道ID、其后继路段ID、所有LocalRoute ID,
                           ///< merge case: vector of previous lane id , its back seg id, all local routes
                           ///< id(string),分叉情况：后继车道ID、其前驱路段ID、所有LocalRoute ID,
                           ///< fork case: vector of next lane id , its front seg id, all local routes id(string)
};

/**
 * @brief NeighborLocalRoute结构体用于表示相邻LocalRoute的信息
 * @details 该结构体包含了相邻LocalRoute的ID及其元素信息
 */
struct NeighborLocalRoute {
  std::string id = "";  ///< 相邻LocalRoute的ID, neighbor local route id

  /**
   * @brief Element结构体用于表示相邻LocalRoute的元素信息
   * @details 该结构体包含了边界类型、范围信息、起止点、关联路段ID等属性
   */
  struct Element {
    LocalRouteSegment::BoundaryType bound_type = LocalRouteSegment::BoundaryType::UNKNOWN;  ///< range可跨越类型
    std::tuple<bool, float, float> range = {false, 0.0, 0.0};  ///< range范围: start_s, end_s
    std::pair<LocalRoutePoint, LocalRoutePoint> points;        ///< range起止点: start_point, end_point
    std::vector<std::pair<std::string, std::string>>
        relation_segment_id_pair;  ///< 当前LocalRoute路段ID与相邻LocalRoute路段ID的关联关系, current local route
                                   ///< segment id, and neighbor local route segment id
  };

  std::vector<Element> elements;  ///< 以边界类型bound_type及范围range连续性划分的元素element列表
};

/**
 * @brief SegmentBoundaryType结构体用于表示路段边界类型信息
 * @details 该结构体包含了路段边界的起始s值、结束s值、左侧边界类型、右侧边界类型、起止点等属性
 */
struct SegmentBoundaryType {
  /**
   * @brief 默认构造函数
   */
  SegmentBoundaryType() = default;
  /**
   * @brief 构造路段边界类型信息对象
   * @details 初始化路段边界类型信息的完整属性，用于描述路段边界在局部路径中的空间属性和类型特征
   *
   * @param[in] _start_s 边界起始s值（单位：m），从路段起点开始的沿路径距离
   * @param[in] _end_s 边界结束s值（单位：m），需大于等于_start_s
   * @param[in] _left_type 左侧边界类型，使用LocalRouteSegment::BoundaryType枚举值
   * @param[in] _right_type 右侧边界类型，使用LocalRouteSegment::BoundaryType枚举值
   * @param[in] _start_point 边界起始点三维坐标（LLA坐标系）
   * @param[in] _end_point 边界结束点三维坐标（LLA坐标系），需与_start_point保持坐标系一致
   *
   * @par 典型使用场景:
   * 1. 路段边界信息初始化
   * 2. 导航模块处理道路边界类型变化
   * 3. 地图数据加载时构造边界类型对象
   *
   * @note 参数_start_s和_end_s应满足 0 <= start_s <= end_s <= 路段总长度
   * @warning 起始点与结束点坐标需来自同一坐标系，避免混合使用不同参考系坐标
   */
  SegmentBoundaryType(float _start_s, float _end_s, LocalRouteSegment::BoundaryType _left_type,
                      LocalRouteSegment::BoundaryType _right_type, math::Vec3d _start_point, math::Vec3d _end_point)
      : start_s(_start_s),
        end_s(_end_s),
        left_type(_left_type),
        right_type(_right_type),
        start_point(_start_point),
        end_point(_end_point) {}
  float start_s = 0.0;                                                                    ///< 路段边界的起始s值
  float end_s = 0.0;                                                                      ///< 路段边界的结束s值
  LocalRouteSegment::BoundaryType left_type = LocalRouteSegment::BoundaryType::UNKNOWN;   ///< 路段左侧边界类型
  LocalRouteSegment::BoundaryType right_type = LocalRouteSegment::BoundaryType::UNKNOWN;  ///< 路段右侧边界类型
  math::Vec3d start_point;                                                                ///< 路段边界的起始点
  math::Vec3d end_point;                                                                  ///< 路段边界的结束点
};

/**
 * @brief LocalRoute类用于表示LocalRoute信息
 * @details
 * 该类包含了LocalRoute的ID、车道段、引导线、边界跨越范围、合并分叉范围、停止线、闸机、速度限制、方向、边界类型、区域信息等属性
 */
class LocalRoute {
 public:
  /**
   * @brief 默认构造函数
   */
  LocalRoute() = default;
  /**
   * @brief 默认析构函数
   */
  ~LocalRoute() = default;

  /**
   * @brief 获取LocalRoute的ID
   * @return 返回LocalRoute的ID
   */
  const int& id() const { return id_; }
  /**
   * @brief 设置LocalRoute的ID
   * @param id 要设置的LocalRouteID
   */
  void setId(const int& id) { id_ = id; }

  /**
   * @brief 获取LocalRoute的车道及路段信息
   * @return 返回LocalRoute的车道及路段信息列表
   */
  const std::vector<std::pair<std::string, std::vector<LocalRouteSegment>>>& lanesSegments() const {
    return lanes_segments_;
  }
  /**
   * @brief 获取可修改的LocalRoute的车道及路段信息
   * @return 返回可修改的LocalRoute的车道及路段信息指针
   */
  std::vector<std::pair<std::string, std::vector<LocalRouteSegment>>>* mutableLanesSegments() {
    return &lanes_segments_;
  }
  /**
   * @brief 设置LocalRoute的车道及路段信息
   * @param lanes_segments 要设置的LocalRoute的车道及路段信息
   */
  void setLanesSegments(const std::vector<std::pair<std::string, std::vector<LocalRouteSegment>>>& lanes_segments) {
    lanes_segments_ = lanes_segments;
  }

  /**
   * @brief 获取LocalRoute的引导线
   * @return 返回LocalRoute的引导线
   */
  const std::vector<LocalRoutePoint>& guideLine() const { return guide_line_; }
  /**
   * @brief 获取可修改的LocalRoute的引导线
   * @return 返回可修改的LocalRoute的引导线指针
   */
  std::vector<LocalRoutePoint>* mutableGuideLine() { return &guide_line_; }
  /**
   * @brief 设置LocalRoute的引导线
   * @param guide_line 要设置的LocalRoute的引导线
   */
  void setGuideLine(const std::vector<LocalRoutePoint>& guide_line) { guide_line_ = guide_line; }

  /**
   * @brief 获取LocalRoute的左边界跨越范围列表
   * @return 返回LocalRoute的左边界跨越范围列表
   */
  const std::vector<BoundaryCrossRange>& leftBoundaryCrossRanges() const { return left_boundary_cross_ranges_; }
  /**
   * @brief 获取可修改的LocalRoute的左边界跨越范围列表
   * @return 返回可修改的LocalRoute的左边界跨越范围列表指针
   */
  std::vector<BoundaryCrossRange>* mutableLeftBoundaryCrossRanges() { return &left_boundary_cross_ranges_; }
  /**
   * @brief 设置LocalRoute的左边界跨越范围列表
   * @param left_boundary_cross_ranges 要设置的LocalRoute的左边界跨越范围列表
   */
  void setLeftBoundaryCrossRanges(const std::vector<BoundaryCrossRange>& left_boundary_cross_ranges) {
    left_boundary_cross_ranges_ = left_boundary_cross_ranges;
  }

  /**
   * @brief 获取LocalRoute的右边界跨越范围列表
   * @return 返回LocalRoute的右边界跨越范围列表
   */
  const std::vector<BoundaryCrossRange>& rightBoundaryCrossRanges() const { return right_boundary_cross_ranges_; }
  /**
   * @brief 获取可修改的LocalRoute的右边界跨越范围列表
   * @return 返回可修改的LocalRoute的右边界跨越范围列表指针
   */
  std::vector<BoundaryCrossRange>* mutableRightBoundaryCrossRanges() { return &right_boundary_cross_ranges_; }
  /**
   * @brief 设置LocalRoute的右边界跨越范围列表
   * @param right_boundary_cross_ranges 要设置的LocalRoute的右边界跨越范围列表
   */
  void setRightBoundaryCrossRanges(const std::vector<BoundaryCrossRange>& right_boundary_cross_ranges) {
    right_boundary_cross_ranges_ = right_boundary_cross_ranges;
  }

  /**
   * @brief 获取LocalRoute的合并分叉范围列表
   * @return 返回LocalRoute的合并分叉范围列表
   */
  const std::vector<MergeForkRange>& mergeForkRanges() const { return merge_fork_ranges_; }
  /**
   * @brief 获取可修改的LocalRoute的合并分叉范围列表
   * @return 返回可修改的LocalRoute的合并分叉范围列表指针
   */
  std::vector<MergeForkRange>* mutableMergeForkRanges() { return &merge_fork_ranges_; }
  /**
   * @brief 设置LocalRoute的合并分叉范围列表
   * @param merge_fork_ranges 要设置的LocalRoute的合并分叉范围列表
   */
  void setMergeForkRanges(const std::vector<MergeForkRange>& merge_fork_ranges) {
    merge_fork_ranges_ = merge_fork_ranges;
  }
  std::vector<MergeForkRange> getMergeForkRangesFromSRange(const float& start_s, const float& end_s);
  std::vector<MergeForkRange> getMergeForkRangesFromSRange(const float& s, const float& forward_dis,
                                                           const float& backward_dis);
  std::vector<MergeForkRange> getMergeRangesFromSRange(const float& start_s, const float& end_s);
  std::vector<MergeForkRange> getMergeRangesFromSRange(const float& s, const float& forward_dis,
                                                       const float& backward_dis);
  std::vector<MergeForkRange> getForkRangesFromSRange(const float& start_s, const float& end_s);
  std::vector<MergeForkRange> getForkRangesFromSRange(const float& s, const float& forward_dis,
                                                      const float& backward_dis);

  /**
   * @brief 获取LocalRoute的停止线列表
   * @return 返回LocalRoute的停止线列表
   */
  const std::vector<StopLine>& stopLines() const { return stop_lines_; }
  /**
   * @brief 获取可修改的LocalRoute的停止线列表
   * @return 返回可修改的LocalRoute的停止线列表指针
   */
  std::vector<StopLine>* mutableStopLines() { return &stop_lines_; }
  /**
   * @brief 设置LocalRoute的停止线列表
   * @param stop_lines 要设置的LocalRoute的停止线列表
   */
  void setStopLines(const std::vector<StopLine>& stop_lines) { stop_lines_ = stop_lines; }
  std::vector<StopLine> getStopLinesFromSRange(const float& start_s, const float& end_s);
  std::vector<StopLine> getStopLinesFromSRange(const float& s, const float& forward_dis, const float& backward_dis);

  /**
   * @brief 获取LocalRoute的闸机列表
   * @return 返回LocalRoute的闸机列表
   */
  const std::vector<Gate>& gates() const { return gates_; }
  /**
   * @brief 获取可修改的LocalRoute的闸机列表
   * @return 返回可修改的LocalRoute的闸机列表指针
   */
  std::vector<Gate>* mutableGates() { return &gates_; }
  /**
   * @brief 设置LocalRoute的闸机列表
   * @param gates 要设置的LocalRoute的闸机列表
   */
  void setGates(const std::vector<Gate>& gates) { gates_ = gates; }
  std::vector<Gate> getGatesFromSRange(const float& start_s, const float& end_s);
  std::vector<Gate> getGatesFromSRange(const float& s, const float& forward_dis, const float& backward_dis);

  /**
   * @brief 获取LocalRoute的速度限制列表
   * @return 返回LocalRoute的速度限制列表
   */
  const std::vector<SpeedLimit>& speedLimits() const { return speed_limits_; }
  /**
   * @brief 获取可修改的LocalRoute的速度限制列表
   * @return 返回可修改的LocalRoute的速度限制列表指针
   */
  std::vector<SpeedLimit>* mutableSpeedLimits() { return &speed_limits_; }
  /**
   * @brief 设置LocalRoute的速度限制列表
   * @param speed_limits 要设置的LocalRoute的速度限制列表
   */
  void setSpeedLimits(const std::vector<SpeedLimit>& speed_limits) { speed_limits_ = speed_limits; }
  SpeedLimit getSpeedLimitFromS(const float& s);

  /**
   * @brief 获取LocalRoute的方向列表
   * @return 返回LocalRoute的方向列表
   */
  const std::vector<SegmentDirection>& directions() const { return directions_; }
  /**
   * @brief 获取可修改的LocalRoute的方向列表
   * @return 返回可修改的LocalRoute的方向列表指针
   */
  std::vector<SegmentDirection>* mutableDirections() { return &directions_; }
  /**
   * @brief 设置LocalRoute的方向列表
   * @param directions 要设置的LocalRoute的方向列表
   */
  void setDirections(const std::vector<SegmentDirection>& directions) { directions_ = directions; }
  std::vector<SegmentDirection> getDirectionsFromSRange(const float& start_s, const float& end_s);
  std::vector<SegmentDirection> getDirectionsFromSRange(const float& s, const float& forward_dis,
                                                        const float& backward_dis);
  SegmentDirection getDirectionFromS(const float& s);

  /**
   * @brief 获取LocalRoute的边界类型列表
   * @return 返回LocalRoute的边界类型列表
   */
  const std::vector<SegmentBoundaryType>& boundaryTypes() const { return boundary_types_; }
  /**
   * @brief 获取可修改的LocalRoute的边界类型列表
   * @return 返回可修改的LocalRoute的边界类型列表指针
   */
  std::vector<SegmentBoundaryType>* mutableBoundaryTypes() { return &boundary_types_; }
  /**
   * @brief 设置LocalRoute的边界类型列表
   * @param boundary_types 要设置的LocalRoute的边界类型列表
   */
  void setBoundaryTypes(const std::vector<SegmentBoundaryType>& boundary_types) { boundary_types_ = boundary_types; }
  std::vector<SegmentBoundaryType> getBoundaryTypesFromSRange(const float& start_s, const float& end_s);
  std::vector<SegmentBoundaryType> getBoundaryTypesFromSRange(const float& s, const float& forward_dis,
                                                              const float& backward_dis);
  SegmentBoundaryType getBoundaryTypeFromS(const float& s);

  /**
   * @brief 获取LocalRoute的区域信息列表
   * @return 返回LocalRoute的区域信息列表
   */
  const std::vector<PerceptionArea>& areas() const { return areas_; }
  /**
   * @brief 获取可修改的LocalRoute的区域信息列表
   * @return 返回可修改的LocalRoute的区域信息列表指针
   */
  std::vector<PerceptionArea>* mutableAreas() { return &areas_; }
  /**
   * @brief 设置LocalRoute的区域信息列表
   * @param areas 要设置的LocalRoute的区域信息列表
   */
  void setAreas(const std::vector<PerceptionArea>& areas) { areas_ = areas; }

  /**
   * @brief 获取LocalRoute的道路信息列表
   * @return 返回LocalRoute的道路信息列表
   */
  const PerceptionRoadInfo& roadInfo() const { return road_info_; }
  /**
   * @brief 获取可修改的LocalRoute的道路信息列表
   * @return 返回可修改的LocalRoute的道路信息列表指针
   */
  PerceptionRoadInfo* mutableRoadInfo() { return &road_info_; }
  /**
   * @brief 设置LocalRoute的道路信息列表
   * @param road_info 要设置的LocalRoute的道路信息列表
   */
  void setRoadInfo(const PerceptionRoadInfo& road_info) { road_info_ = road_info; }

  /**
   * @brief 获取可修改的自车位置信息
   * @return 返回可修改的自车位置信息指针
   */
  MapPoint* mutableAdcLocalization() { return &adc_loc_; }
  /**
   * @brief 获取自车位置信息
   * @return 返回自车位置信息
   */
  const MapPoint& adcLocalization() const { return adc_loc_; }

  /**
   * @brief 获取自车在此LocalRoute上的s值
   * @return 返回自车在此LocalRoute上的s值
   */
  const float& adcS() const { return adc_s_; }
  /**
   * @brief 设置自车在此LocalRoute上的s值
   * @param adc_s 要设置的自车在此LocalRoute上的s值
   */
  void setAdcS(const float& adc_s) { adc_s_ = adc_s; }

  /**
   * @brief 获取自车在此LocalRoute上的l值
   * @return 返回自车在此LocalRoute上的l值
   */
  const float& adcL() const { return adc_l_; }
  /**
   * @brief 设置自车在此LocalRoute上的l值
   * @param adc_l 要设置的自车在此LocalRoute上的l值
   */
  void setAdcL(const float& adc_l) { adc_l_ = adc_l; }

  /**
   * @brief 获取自车航向角与此路段方向的夹角
   * @return 返回自车航向角与此路段方向的夹角
   */
  const float& adcHeadingDiff() const { return adc_heading_diff_; }
  /**
   * @brief 设置自车航向角与此路段方向的夹角
   * @param adc_heading_diff 要设置的自车航向角与此路段方向的夹角
   */
  void setAdcHeadingDiff(const float& adc_heading_diff) { adc_heading_diff_ = adc_heading_diff; }

  /**
   * @brief 获取与自车位置匹配最近路段ID
   * @return 返回与自车位置匹配最近路段ID
   */
  const std::string& adcSegId() const { return adc_seg_id_; }
  /**
   * @brief 设置与自车位置匹配最近路段ID
   * @param adc_seg_id 要设置的与自车位置匹配最近路段ID
   */
  void setAdcSegId(const std::string& adc_seg_id) { adc_seg_id_ = adc_seg_id; }

  /**
   * @brief 获取与自车位置匹配最近车道ID
   * @return 返回与自车位置匹配最近车道ID
   */
  const std::string& adcLaneId() const { return adc_lane_id_; }
  /**
   * @brief 设置与自车位置匹配最近车道ID
   * @param adc_lane_id 要设置的与自车位置匹配最近车道ID
   */
  void setAdcLaneId(const std::string& adc_lane_id) { adc_lane_id_ = adc_lane_id; }

  /**
   * @brief 设置是否为当前最近的LocalRoute
   * @param is_current_local_route 要设置的是否为当前最近的LocalRoute
   */
  void setIsCurrentLocalRoute(const bool& is_current_local_route) { is_current_local_route_ = is_current_local_route; }
  /**
   * @brief 获取是否为当前最近的LocalRoute
   * @return 返回是否为当前最近的LocalRoute
   */
  const bool& isCurrentLocalRoute() const { return is_current_local_route_; }

  /**
   * @brief 获取全局系下当前LocalRoute起点对应的s值
   * @return 返回全局系下当前LocalRoute起点对应的s值
   */
  const float& globalStartS() const { return global_start_s_; }
  /**
   * @brief 设置全局系下当前LocalRoute起点对应的s值
   * @param global_start_s 要设置的全局系下当前LocalRoute起点对应的s值
   */
  void setGlobalStartS(const float& global_start_s) { global_start_s_ = global_start_s; }

  /**
   * @brief 获取当前LocalRoute的长度值
   * @return 返回当前LocalRoute的长度值
   */
  const float length() const { return guide_line_.empty() ? 0.0 : guide_line_.back().s(); }

  /**
   * @brief 获取LocalRoute的线源信息
   * @return 返回LocalRoute的线源信息列表
   */
  const std::vector<std::tuple<LineSourceType, float, float>>& sourceInfos() const { return source_infos_; }
  /**
   * @brief 获取可修改的LocalRoute的线源信息
   * @return 返回可修改的LocalRoute的线源信息列表指针
   */
  std::vector<std::tuple<LineSourceType, float, float>>* mutableSourceInfos() { return &source_infos_; }
  /**
   * @brief 设置LocalRoute的线源信息
   * @param source_infos 要设置的LocalRoute的线源信息列表
   */
  void setSourceInfos(const std::vector<std::tuple<LineSourceType, float, float>>& source_infos) {
    source_infos_ = source_infos;
  }
  std::vector<std::tuple<LineSourceType, float, float>> getSourceInfosFromSRange(const float& start_s,
                                                                                 const float& end_s);
  std::vector<std::tuple<LineSourceType, float, float>> getSourceInfosFromSRange(const float& s,
                                                                                 const float& forward_dis,
                                                                                 const float& backward_dis);
  std::tuple<LineSourceType, float, float> getSourceInfoFromS(const float& s);

  /**
   * @brief 获取LocalRoute的有效范围信息
   * @return 返回LocalRoute的有效范围信息
   */
  const std::tuple<bool, float, float, std::vector<std::string>>& validRange() const { return valid_range_; }
  /**
   * @brief 获取可修改的LocalRoute的有效范围信息
   * @return 返回可修改的LocalRoute的有效范围信息指针
   */
  std::tuple<bool, float, float, std::vector<std::string>>* mutableValidRange() { return &valid_range_; }
  /**
   * @brief 设置LocalRoute的有效范围信息
   * @param valid_range 要设置的LocalRoute的有效范围信息
   */
  void setValidRange(const std::tuple<bool, float, float, std::vector<std::string>>& valid_range) {
    valid_range_ = valid_range;
  }

  /**
   * @brief 获取LocalRoute的导航换道信息
   * @return 返回LocalRoute的导航换道信息
   */
  const std::tuple<float, int, int>& navigationLaneChangeInfo() const { return navigation_lane_change_info_; }
  /**
   * @brief 获取可修改的LocalRoute的导航换道信息
   * @return 返回可修改的LocalRoute的导航换道信息指针
   */
  std::tuple<float, int, int>* mutableNavigationLaneChangeInfo() { return &navigation_lane_change_info_; }
  /**
   * @brief 设置LocalRoute的导航换道信息
   * @param navigation_lane_change_info 要设置的LocalRoute的导航换道信息
   */
  void setNavigationLaneChangeInfo(const std::tuple<float, int, int>& navigation_lane_change_info) {
    navigation_lane_change_info_ = navigation_lane_change_info;
  }

  bool RebuildBoundaryCrossRanges(std::vector<BoundaryCrossRange>* boundary_cross_ranges);
  bool RebuildSpeedLimits();
  bool RebuildDirections();
  bool RebuildSourceInfos();
  std::unordered_map<int, NeighborLocalRoute> getLeftNeighborLocalRouteBySRange(const float& s,
                                                                                const float& forward_dis,
                                                                                const float& backward_dis) const;
  std::unordered_map<int, NeighborLocalRoute> getLeftNeighborLocalRouteBySRange(const float& start_s,
                                                                                const float& end_s) const;
  std::unordered_map<int, NeighborLocalRoute> getRightNeighborLocalRouteBySRange(const float& s,
                                                                                 const float& forward_dis,
                                                                                 const float& backward_dis) const;
  std::unordered_map<int, NeighborLocalRoute> getRightNeighborLocalRouteBySRange(const float& start_s,
                                                                                 const float& end_s) const;

 private:
  std::unordered_map<int, NeighborLocalRoute> getNeighborLocalRouteBySRange(
      const std::vector<BoundaryCrossRange>& left_boundary_cross_ranges, const float& start_s,
      const float& end_s) const;
  std::unordered_map<int, NeighborLocalRoute> getNeighborLocalRouteBySRange(
      const std::vector<BoundaryCrossRange>& boundary_cross_ranges, const float& s, const float& forward_dis,
      const float& backward_dis) const;

 private:
  int id_ = 0;  ///< ID
  std::vector<std::pair<std::string, std::vector<LocalRouteSegment>>>
      lanes_segments_;                       ///< 车道及路段信息, include lane id and its all segments,
                                             ///< 其中，segment中各点的s为其在此segment上的累计s
  std::vector<LocalRoutePoint> guide_line_;  ///< 引导线信息, 其中，各点的s为其在guide_line_上的累计s
  std::vector<BoundaryCrossRange> left_boundary_cross_ranges_;          ///< 左边界跨越范围列表
  std::vector<BoundaryCrossRange> right_boundary_cross_ranges_;         ///< 右边界跨越范围列表
  std::vector<MergeForkRange> merge_fork_ranges_;                       ///< 合并分叉范围列表
  std::vector<StopLine> stop_lines_;                                    ///< 停止线列表
  std::vector<Gate> gates_;                                             ///< 闸机列表
  std::vector<SpeedLimit> speed_limits_;                                ///< 速度限制列表
  std::vector<SegmentDirection> directions_;                            ///< 方向列表
  std::vector<SegmentBoundaryType> boundary_types_;                     ///< 边界类型列表
  std::vector<PerceptionArea> areas_;                                   ///< 区域信息列表
  PerceptionRoadInfo road_info_;                                        ///< 道路信息列表
  MapPoint adc_loc_;                                                    ///< 自车位置信息
  float adc_s_ = 0.0;                                                   ///< 自车在此LocalRoute上的s值
  float adc_l_ = 1000.0;                                                ///< 自车在此LocalRoute上的l值
  float adc_heading_diff_ = 0.0;                                        ///< 自车航向角与此路段方向的夹角
  std::string adc_seg_id_ = "";                                         ///< 与自车位置匹配最近路段ID
  std::string adc_lane_id_ = "";                                        ///< 与自车位置匹配最近车道ID
  bool is_current_local_route_ = false;                                 ///< 是否为当前最近的LocalRoute
  float global_start_s_ = 0.0;                                          ///< 全局系下当前LocalRoute起点对应的s值
  std::vector<std::tuple<LineSourceType, float, float>> source_infos_;  ///< 线源信息
  std::tuple<bool, float, float, std::vector<std::string>> valid_range_ = {false, 0.0, 0.0,
                                                                           std::vector<string>()};  ///< 有效范围信息
  std::tuple<float, int, int> navigation_lane_change_info_ = {
      -1.0, 0, 0};  ///< 导航换道信息, distance_to_navi_lane_change, navi_lane_change_direction, navi_lane_change_num
};

/**
 * @brief LaneTopo结构体用于表示车道组建的拓扑信息
 * @details 该结构体包含了车道ID、车道段信息、前驱车道和后继车道信息
 */
struct LaneTopo {
  std::string lane_id = "";  ///< 车道ID，唯一标识
  /**
   * @brief 车道中的路段信息列表
   * @details 每个元素是一个pair，包含路段ID和导航评分, lane segment id and navigation score
   * - first: 路段ID
   * - second: 导航评分，用于评估该路段段的优先级
   */
  std::vector<std::pair<std::string, float>> lane_segments;

  /**
   * @brief 前驱车道信息列表
   * @details 每个元素是一个pair，包含前驱车道ID和其末端路段ID, previous lanes id and corresponding back lane segments
   * id
   * - first: 前驱车道ID
   * - second: 前驱车道的末端路段ID
   */
  std::vector<std::pair<std::string, std::string>> previous_lanes;

  /**
   * @brief 后继车道信息列表
   * @details 每个元素是一个pair，包含后继车道ID和其始端路段ID, next lanes id and corresponding front lane segments id
   * - first: 后继车道ID
   * - second:  后继车道的始端路段ID
   */
  std::vector<std::pair<std::string, std::string>> next_lanes;
};

/**
 * @brief LocalRouter类用于管理和处理LocalRoute的生成与更新
 * @details 该类包含了LocalRoute的生成、更新、选择等功能，支持路径规划和导航决策
 */
class LocalRouter : public StampedBase {
 public:
  friend class PerceptionRoadStructureAdapter;
  /**
   * @brief 默认构造函数
   */
  LocalRouter();
  /**
   * @brief 默认析构函数
   */
  ~LocalRouter();

  void reset();
  void process();

  /**
   * @brief 获取所有LocalRoute的映射
   * @return 返回所有LocalRoute的映射
   */
  const std::map<int, LocalRoute>& getLocalRoutes() const { return local_routes_; }
  /**
   * @brief 获取可修改的所有LocalRoute的映射
   * @return 返回可修改的所有LocalRoute的映射指针
   */
  std::map<int, LocalRoute>* getMutableLocalRoutes() { return &local_routes_; }

  /**
   * @brief 获取车道及其对应路段的映射
   * @return 返回车道及其对应路段的映射
   */
  const std::unordered_map<std::string, std::unordered_map<std::string, LocalRouteSegment>>& getLanesSegmentsMap()
      const {
    return lanes_segments_map_;
  }
  /**
   * @brief 获取可修改的车道及其对应路段的映射
   * @return 返回可修改的车道及其对应路段的映射指针
   */
  std::unordered_map<std::string, std::unordered_map<std::string, LocalRouteSegment>>* getMutableLanesSegmentsMap() {
    return &lanes_segments_map_;
  }

  /**
   * @brief 获取车道组建的拓扑信息
   * @return 返回车道组建的拓扑信息
   */
  const std::vector<std::vector<LaneTopo>>& getLanesTopos() const { return lanes_topos_; }
  /**
   * @brief 获取可修改的车道组建的拓扑信息
   * @return 返回可修改的车道组建的拓扑信息指针
   */
  std::vector<std::vector<LaneTopo>>* getMutableLanesTopos() { return &lanes_topos_; }

  /**
   * @brief 获取自车所在local_route的id
   * @return 返回自车所在local_route的id
   */
  const int& currentLocalRouteId() const { return current_local_route_id_; }
  /**
   * @brief 获取自车所在车道的id
   * @return 返回自车所在车道的id
   */
  const std::string& currentLocalRouteLaneId() const { return current_local_route_lane_id_; }
  /**
   * @brief 获取自车所在的路段id
   * @return 返回自车所在的路段id
   */
  const std::string& currentLocalRouteSegmentId() const { return current_local_route_segment_id_; }

  /**
   * @brief 获取可修改的自车位置信息
   * @return 返回可修改的自车位置信息指针
   */
  MapPoint* mutableAdcLocalization() { return &adc_loc_; }
  /**
   * @brief 获取自车位置信息
   * @return 返回自车位置信息
   */
  const MapPoint& adcLocalization() const { return adc_loc_; }

  /**
   * @brief 获取道路信息
   * @return 返回道路信息
   */
  const PerceptionRoadInfo& roadInfo() const { return road_info_; }
  /**
   * @brief 获取可修改的道路信息
   * @return 返回可修改的道路信息指针
   */
  PerceptionRoadInfo* mutableRoadInfo() { return &road_info_; }
  /**
   * @brief 设置道路信息
   * @param road_info 要设置的道路信息
   */
  void setRoadInfo(const PerceptionRoadInfo& road_info) { road_info_ = road_info; }

  /**
   * @brief 获取计算耗时
   * @return 返回计算耗时
   */
  const double& timeConsumption() const { return time_consumption_; }
  /**
   * @brief 设置计算耗时
   * @param time_consumption 要设置的计算耗时
   */
  void setTimeConsumption(const double& time_consumption) { time_consumption_ = time_consumption; }

  /**
   * @brief 获取存储历史LocalRoute的ID及其对应的车道信息
   * @return 返回存储历史LocalRoute的ID及其对应的车道信息
   */
  const std::vector<std::tuple<bool, int, std::vector<std::string>>>& historyLocalRoutesIdVec() const {
    return history_local_routes_id_vec_;
  }
  /**
   * @brief 获取可修改的存储历史LocalRoute的ID及其对应的车道信息
   * @return 返回可修改的存储历史LocalRoute的ID及其对应的车道信息指针
   */
  std::vector<std::tuple<bool, int, std::vector<std::string>>>* mutableHistoryLocalRoutesIdVec() {
    return &history_local_routes_id_vec_;
  }
  /**
   * @brief 设置存储历史LocalRoute的ID及其对应的车道信息
   * @param road_info 要设置的存储历史LocalRoute的ID及其对应的车道信息
   */
  void setHistoryLocalRoutesIdVec(
      const std::vector<std::tuple<bool, int, std::vector<std::string>>>& history_local_routes_id_vec) {
    history_local_routes_id_vec_ = history_local_routes_id_vec;
  }

  // function
  bool IsSegmentMatchMotionConstraints(const LocalRouteSegment& seg);
  std::pair<bool, LocalRouteSegment> getSegmentBySegmentId(const std::string seg_id);
  std::pair<bool, std::string> getLaneIdBySegmentId(const std::string seg_id);
  std::pair<bool, std::vector<int>> getLocalRoutesIdBySegmentId(const std::string& seg_id);
  std::pair<bool, std::vector<LocalRoute>> getLocalRoutesBySegmentId(const std::string& seg_id);
  std::pair<bool, std::vector<std::string>> getSegmentsIdByLaneId(const std::string& lane_id);
  std::pair<bool, std::vector<LocalRouteSegment>> getSegmentsByLaneId(const std::string& lane_id);
  std::pair<bool, std::vector<int>> getLocalRoutesIdByLaneId(const std::string& lane_id);
  std::pair<bool, std::vector<std::string>> getLocalRoutesStringIdByLaneId(const std::string& lane_id);
  std::pair<bool, std::vector<LocalRoute>> getLocalRoutesByLaneId(const std::string& lane_id);
  std::pair<bool, LocalRoute> getLocalRouteByLocalRouteId(const int& local_route_id);

 protected:
  void generateLocalRoutes();
  int generateUniqueLocalRouteId(const LocalRoute& local_route);

  void rebuildLocalRoutes();
  void rebuildBoundaryCrossRangesByLocalRouteId();
  void rebuildMergeForkRangesByLocalRouteId();

  void selectLocalRouteForSmooth();
  std::tuple<bool, float, float, std::vector<string>> validPolicyByNavigationScore(const LocalRoute& local_route);
  std::tuple<bool, float, float, std::vector<string>> validPolicyBySpecifiedRange(const LocalRoute& local_route);
  void localRoutesSubsetRelationProcess();

  void generateNavigationLaneChangeProperty();
  // lane_change_direction(default: 0  left:1  right:-1), lane_change_num
  std::tuple<int, int> judgeSegmentsNeighborRelation(const std::string& base_seg_id,
                                                     const std::vector<std::string>& target_seg_ids);

  std::map<int, LocalRoute> local_routes_;  ///< 存储所有LocalRoute的信息，以LocalRouteID为键,
                                            ///< 说明：一条local_route由多条存在前驱后继关系的lane顺序组成，一条lane由多条存在前驱后继关系的lane
                                            ///< segment组成.
  std::unordered_map<std::string, std::unordered_map<std::string, LocalRouteSegment>>
      lanes_segments_map_;  ///< 存储车道及其对应的路段信息，以车道ID和路段ID为键, lane id, lane segment id, lane
                            ///< segment info
  std::vector<std::vector<LaneTopo>>
      lanes_topos_;  ///< 存储车道组建的拓扑信息，表示车道的连接关系, lanes topos, each topo include all lanes
                     ///< id(include all lane segments id) list by successors
  int current_local_route_id_ = 0;                   ///< 当前自车所在local_route的id
  std::string current_local_route_lane_id_ = "";     ///< 当前自车所在local_route_lane的id
  std::string current_local_route_segment_id_ = "";  ///< 当前自车所在local_route_segment的id
  MapPoint adc_loc_;                                 ///< 自车位置信息
  PerceptionRoadInfo road_info_;                     ///< 道路信息
  double time_consumption_ = 0.0;                    ///< 计算耗时

 private:
  std::vector<std::tuple<bool, int, std::vector<std::string>>>
      history_local_routes_id_vec_;  ///< 存储历史LocalRoute的ID及其对应的车道信息, is_reused, history local route id
                                     ///< and its all lanes
};

}  // namespace gpal::pnc::planning
