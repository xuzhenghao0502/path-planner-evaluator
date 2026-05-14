#pragma once

#include <vector>

#include "math/vec2d.h"
#include "util/base_struct.h"

namespace gpal::pnc::planning {

enum class LineType {
  kTypeInvalid = 0,               // 无效
  kTypeSingleSolid = 1,           // 单实线
  kTypeSingleDash = 2,            // 单虚线
  kTypeDualSolid = 3,             // 双实线
  kTypeDualDash = 4,              // 双虚线
  kTypeDashSolid = 5,             // 左虚右实线
  kTypeSolidDash = 6,             // 左实右虚线
  kTypeTidal = 7,                 // 潮汐车道线
  kTypeGuideLine = 8,             // 导流线
  kTypeCurb = 9,                  // 路沿
  kTypeDecelerationMarking = 10,  // 减速标线，鱼骨线
  kTypeUnknown = 11               // 未知
};

enum class LineColor {
  kColorInvalid = 0,  // 无效
  kColorWhite = 1,    // 白色，包括一些特殊颜色线
  kColorYellow = 2,   // 黄色
  kColorUnknown = 3   // 未知
};

enum class LaneSegmentType {
  kTypeInvalid = 0,     // 无效
  kTypeVehicle = 1,     // 机动车道
  kTypeBicycle = 2,     // 非机动车道
  kTypeRamp = 3,        // 匝道
  kTypeReversing = 4,   // 倒车车道
  kTypePocket = 5,      // 待行车道
  kTypeParking = 6,     // 停车道
  kTypeBus = 7,         // 公交车道
  kTypeEmergency = 8,   // 应急车道
  kTypeReversible = 9,  // 潮汐车道
  kTypeVirtual = 10,    // 虚拟车道
  kTypeOpposite = 11    // 逆向车道
};

enum class LaneSegmentDirection {
  kDirectionInvalid = 0,      // 无效
  kDirectionForwardOnly = 1,  // 直行
  kDirectionLeftOnly = 2,     // 左转
  kDirectionRightOnly = 3,    // 右转
  kDirectionUTurnOnly = 4,    // 掉头
  kDirectionUnknown = 13      // 未知
};

struct LaneMarking {
  uint32_t id;
  LineType line_type;
  LineColor line_color;
  double start_dist;                     // 为适配目前实时感知车道线暂时所加
  double end_dist;                       // 为适配目前实时感知车道线暂时所加
  std::vector<math::Vec3d> line_points;  // 全局坐标系下
  LaneMarking() {
    id = 0;
    line_type = LineType::kTypeInvalid;
    line_color = LineColor::kColorInvalid;
    start_dist = 0.0;  // 为适配目前实时感知车道线暂时所加
    end_dist = 0.0;    // 为适配目前实时感知车道线暂时所加
    line_points.clear();
  }
  void clear() {
    id = 0;
    line_type = LineType::kTypeInvalid;
    line_color = LineColor::kColorInvalid;
    start_dist = 0.0;  // 为适配目前实时感知车道线暂时所加
    end_dist = 0.0;    // 为适配目前实时感知车道线暂时所加
    line_points.clear();
  }
};

struct LaneSegment {
  uint32_t segment_id;             // 车道段id, 每帧唯一, 从 1 开始
  float navigation_score;          // 导航分
  LaneSegmentType type;            // 车道类型
  LaneSegmentDirection direction;  // 车道方向
  LaneMarking left_marking;        // 车道左边线
  LaneMarking
      right_marking;  // 车道右边线，当存在既有车道线又有路沿/围栏等情形，仅给出一条LaneMarking，同时类别设定为路沿
  std::vector<math::Vec3d> center_line;             // 车道中心线散点
  std::vector<double> offset;                       // 车道中心线散点距离左右边线的距离
  std::vector<uint32_t> previous_segment_id_array;  // 前驱车道段 id，可能有多个
  std::vector<uint32_t> next_segment_id_array;      // 后继车道段 id，可能有多个
  uint32_t left_segment_id;                         // 左车道段 id，默认值 0 表示无
  uint32_t right_segment_id;                        // 右车道段 id，默认值 0 表示无
  uint32_t left_opposite_segment_id;                // 左对向车道段 id，默认值 0 表示无
  LaneSegment() {
    segment_id = 0;
    navigation_score = 0.0f;
    type = LaneSegmentType::kTypeInvalid;
    direction = LaneSegmentDirection::kDirectionInvalid;
    left_marking.clear();
    right_marking.clear();
    center_line.clear();
    offset.clear();
    previous_segment_id_array.clear();
    next_segment_id_array.clear();
    left_segment_id = 0;           // 左车道段 id，默认值 0 表示无
    right_segment_id = 0;          // 右车道段 id，默认值 0 表示无
    left_opposite_segment_id = 0;  // 左对向车道段 id，默认值 0 表示无
  }
  void clear() {
    segment_id = 0;
    navigation_score = 0.0f;
    type = LaneSegmentType::kTypeInvalid;
    direction = LaneSegmentDirection::kDirectionInvalid;
    left_marking.clear();
    right_marking.clear();
    center_line.clear();
    offset.clear();
    previous_segment_id_array.clear();
    next_segment_id_array.clear();
    left_segment_id = 0;           // 左车道段 id，默认值 0 表示无
    right_segment_id = 0;          // 右车道段 id，默认值 0 表示无
    left_opposite_segment_id = 0;  // 左对向车道段 id，默认值 0 表示无
  }
};

struct PerceptionLane {
  std::string id;                          // 车道id, 目前是左右两边车道线的id的组合，默认值为0
  uint32_t left_lane_id;                   // 左车道 id，默认值 0 表示无
  uint32_t right_lane_id;                  // 右车道 id，默认值 0 表示无
  std::vector<LaneSegment> lane_segments;  // 车道分段结果，按自车行驶方向排列
  MapPoint adc_loc;
  PerceptionLane() {
    id = "";
    left_lane_id = 0;
    right_lane_id = 0;
    lane_segments.clear();
  }
  void clear() {
    id = "";
    left_lane_id = 0;
    right_lane_id = 0;
    lane_segments.clear();
  }
};

class RoadStructure : public StampedBase {
 public:
  friend class PerceptionLaneAdapter;
  std::string left_lane_id = "";
  std::string right_lane_id = "";
  std::string ego_lane_id = "";
  uint32_t ego_lane_segment_id;
  std::vector<PerceptionLane> lanes;
  void Clear();
};

}  // namespace gpal::pnc::planning
