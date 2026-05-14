#pragma once
#include <unordered_map>
#include <string>
#include <memory>
#include <any>
#include "base/singleton.h"
#include "config/vehicle_config.pb.h"
#include "config/freespace_config.pb.h"
#include "config/openspace/openspace_search.pb.h"

namespace gpal::pnc::planning {

// Forward declare additional config types
class ParkingPathGeneratorProfile;
class PlannerProfile;
class ParkingPathPlannerConfig;

class ConfigManager {
  friend class Singleton<ConfigManager>;

 public:
  template <typename T>
  const T& getConfig(const std::string& name) const {
    auto it = configs_.find(name);
    if (it != configs_.end()) {
      if (auto* ptr = std::any_cast<T>(&it->second)) {
        return *ptr;
      }
    }
    static T default_config;
    return default_config;
  }

  template <typename T>
  void setConfig(const std::string& name, const T& config) {
    configs_[name] = config;
  }

  const VehicleConfig& vehicle_config() const {
    auto it = configs_.find("VehicleConfig");
    if (it != configs_.end()) {
      return *std::any_cast<VehicleConfig>(&it->second);
    }
    static VehicleConfig default_vc;
    return default_vc;
  }

  void set_vehicle_config(const VehicleConfig& vc);

 private:
  ConfigManager() = default;
  std::unordered_map<std::string, std::any> configs_;
};

}  // namespace gpal::pnc::planning
