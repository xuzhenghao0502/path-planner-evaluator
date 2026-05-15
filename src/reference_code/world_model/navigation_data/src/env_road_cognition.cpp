#include "navigation_data/env_road_cognition.h"

namespace gpal::pnc::planning {

EnvRoadCognition::EnvRoadCognition() { clear(); }

EnvRoadCognition::~EnvRoadCognition() {}

void EnvRoadCognition::clear() {
  map_provider_ = proto::road_cognition::EnvRoadCognition_MapProviderType_kMapProviderAmapSd;
  status_info_.Clear();
  reference_lines_.clear();
  areas_.clear();
  risk_boundaries_.clear();
  traffic_lights_.clear();
  roads_.clear();
  scenario_info_.Clear();
  dest_info_.Clear();
  lane_markings_.clear();
}

}  // namespace gpal::pnc::planning
