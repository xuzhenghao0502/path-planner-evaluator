#include "openspace_path_planner/utils/slot.h"

namespace gpal::pnc::planning {

std::string Slot::debugString() const {
  std::string json_str = "{\"id\":\"" + slot_id_ + "\",\"corners\":[";
  for (const auto& pt : slot_corners_) {
    json_str += "[" + std::to_string(pt.x()) + "," + std::to_string(pt.y()) + "],";
  }
  json_str.pop_back();  // 移除末尾逗号
  json_str += "]}";
  return json_str;
}

bool Slot::isPointInsideSlot(const PathPt& point) const {
  math::Vec2d pt2d(point.x(), point.y());

  // 将四边形的四个角点转换为Vec2d
  std::vector<math::Vec2d> corners;
  corners.reserve(4);
  for (const auto& corner : slot_corners_) {
    corners.emplace_back(corner.x(), corner.y());
  }
  math::Polygon2d polygon(corners);
  return polygon.IsPointIn(pt2d);
}

}  // namespace gpal::pnc::planning