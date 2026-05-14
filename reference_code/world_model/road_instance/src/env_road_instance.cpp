#include "road_instance/env_road_instance.h"

#include "config_manager/config_manager.h"

namespace gpal::pnc::planning::road_instance {

EnvRoadInstance::EnvRoadInstance() {
  clear();
}

EnvRoadInstance::~EnvRoadInstance() {}

void EnvRoadInstance::clear() {
  center_line_map_.clear();
  guide_line_map_.clear();
  lane_marking_map_.clear();
  stop_line_map_.clear();
  arrow_map_.clear();
  area_map_.clear();
  key_point_map_.clear();
  traffic_light_map.clear();
  gates_map.clear();
}

}  // namespace gpal::pnc::planning::road_instance
