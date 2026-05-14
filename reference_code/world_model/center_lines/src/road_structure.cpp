#include "center_lines/road_structure.h"

namespace gpal::pnc::planning {

void RoadStructure::Clear() {
  left_lane_id = "";
  right_lane_id = "";
  ego_lane_id = "";
  ego_lane_segment_id = 0;
  lanes.clear();
}

}  // namespace gpal::pnc::planning