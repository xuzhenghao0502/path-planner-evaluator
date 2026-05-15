#include "adas_data/adas_road_marking.h"

namespace gpal::pnc::adas {

void AdasRoadMarking::Clear() {
  id = 0;
  ego_vehicle_index = 0;
  ego_lane_widths.clear();
  ego_lane_curvatures.clear();
  center_line.clear();
  lane_markings.clear();
}

}  // namespace gpal::pnc::adas