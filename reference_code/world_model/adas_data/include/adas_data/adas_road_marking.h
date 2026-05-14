#pragma once

#include <vector>

#include "math/vec2d.h"
#include "util/base_struct.h"

namespace gpal::pnc::adas {

enum class LineType {
  kTypeUnknown = 0,     // 未知
  kTypeNormal = 1,      // 常规车道线
  kTypeFishBone = 2,    // 鱼骨线、减速线
  kTypeCrossGuide = 3,  // 导向线
  kTypeWaitLine = 4,    // 待行区左右边线
  kTypeIgnore = 5,      // 忽略线，如对向车道线、与当前车道有实物隔离的同向车道线
  kTypeRoadSide = 6,    // 道路边界
  kTypeGuideLine = 7,   // 导流线
};

enum class LineShape {
  kShapeInvalid = 0,      // 无效，如 line_type 是道路边界则线型为无效
  kShapeSingleSolid = 1,  // 单实线
  kShapeSingleDash = 2,   // 单虚线
  kShapeDualSolid = 3,    // 双实线
  kShapeDualDash = 4,     // 双虚线
  kShapeDashSolid = 5,    // 左虚右实线
  kShapeSolidDash = 6,    // 左实右虚,
  kShapeReversible = 7,   // 潮汐车道线
  kShapeVariable = 8,     // 可变车道线
  kShapePointLine = 9,    // 波特点线
  kShapeUnknown = 10,     // 未知
  kShapeThickDash = 11,    // 粗虚线
  kShapeWideSolid = 12,    // 粗实线，常见于港湾式停靠站
  kShapeColorRYB = 13,     // 红黄蓝三色线
};

struct LaneLineMarking {
  uint32_t id;
  LineType line_type;
  LineShape line_shape;
  planning::MapPoint adc_loc;
  std::vector<planning::math::Vec3d> line_points;
  std::vector<float> coeffs;      // 传给下游的五次多项式
  std::vector<float> curvatures;  // 车道线的曲率，单位：1/m
  float confidence;               // 车道线置信度，范围：[0.0, 1.0]
  int32_t index;  // 车道线标签，0：未定义；1 左侧最近第一车道线，2 左侧最近第二车道线，以此类推；-1
                  // 右侧最近第一车道线，-2 右侧最近第二车道线，以此类推
  float start_offset;  // 车道线起点在自车坐标系下的纵向坐标，米，只在自车所在LaneMarkingGourp中有效
  float end_offset;    // 车道线终点在自车坐标系下的纵向坐标，米，只在自车所在LaneMarkingGourp中有效
  void clear() {
    id = 0;
    line_type = LineType::kTypeUnknown;
    line_shape = LineShape::kShapeInvalid;
    line_points.clear();
    coeffs.clear();
    curvatures.clear();
    confidence = 0.0;
    index = 0;
    start_offset = 0.0;
    end_offset = 0.0;
  }
};

class AdasRoadMarking : public planning::StampedBase {
 public:
  friend class PerceptionLaneAdapter;
  uint32_t id;
  size_t ego_vehicle_index;
  std::vector<float> ego_lane_widths;
  std::vector<float> ego_lane_curvatures;
  LaneLineMarking center_line;
  std::vector<LaneLineMarking> lane_markings;
  void Clear();
};

}  // namespace gpal::pnc::adas
