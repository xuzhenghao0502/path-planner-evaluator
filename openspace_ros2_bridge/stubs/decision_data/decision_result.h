#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include "math/box2d.h"
#include "math/line_segment2d.h"
#include "math/vec2d.h"

namespace Decision {

enum class ObjectType : uint8_t {
  UNKNOWN = 0, PEDESTRIAN = 1, VRU = 2, VEHICLE = 3, HEAVY_VEHICLE = 4
};

struct DecisionObject {
  ObjectType type = ObjectType::UNKNOWN;
  std::string id;
  std::vector<math::Vec2d> vertices;
  std::vector<math::LineSegment2d> boundary;
  math::Box2d cur_box;
  double s = 0;
  double lateral_offset = 0;
  double heading = 0;
  double length = 0;
  double width = 0;
};

struct DecisionResult {
  std::vector<DecisionObject> decision_objects;
};

}  // namespace Decision

namespace gpal::pnc::planning {
using DecisionResult = ::Decision::DecisionResult;
using DecisionObject = ::Decision::DecisionObject;
}  // namespace gpal::pnc::planning
