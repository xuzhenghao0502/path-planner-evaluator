#pragma once

#include "common/spatiotemporal_common.h"
#include "config/spatiotemporal_planner/spatiotemporal_manager_config.pb.h"
#include "config_manager/config_manager.h"

namespace gpal::pnc::planning {

class SpatiotemporalPlannerScenarioManager {
 public:
  SpatiotemporalPlannerScenarioManager() = default;
  ~SpatiotemporalPlannerScenarioManager() = default;

  inline MacroScenario macroScenario(const StageState& stage_state) const {
    switch (stage_state) {
      case StageState::IdleStage:
        return MacroScenario::UNKNOWN_MACRO_SCENARIO;
      case StageState::AccDrivingStage:
        return MacroScenario::UNKNOWN_MACRO_SCENARIO;
      case StageState::LccDrivingStage:
        return MacroScenario::LCC;
      case StageState::HighwayNoaDrivingStage:
        return MacroScenario::HIGHWAY_NOA;
      case StageState::CityNoaDrivingStage:
        return MacroScenario::URBAN_NOA;
      case StageState::HpaDrivingStage:
        return MacroScenario::HPA;
      case StageState::ParkInStage:
        return MacroScenario::UNKNOWN_MACRO_SCENARIO;
      case StageState::ParkOutStage:
        return MacroScenario::UNKNOWN_MACRO_SCENARIO;
      default:
        return MacroScenario::UNKNOWN_MACRO_SCENARIO;
    }
  }

  inline DrivingStyle drivingStyle() const {
    // TODO 接入驾驶模式
    return DrivingStyle::NORMAL;
  }

  // 管理器打包
  struct ManagerKey {
    MacroScenario macro = MacroScenario::UNKNOWN_MACRO_SCENARIO;
    DrivingStyle style = DrivingStyle::UNKNOWN_DRIVING_STYLE;
    ManagerKey() = default;
    ManagerKey(MacroScenario _macro, DrivingStyle _style) : macro(_macro), style(_style) {}

    bool operator==(ManagerKey const& o) const noexcept { return macro == o.macro && style == o.style; }
    bool operator!=(ManagerKey const& o) const noexcept { return !(*this == o); }
  };

  struct ManagerKeyHash {
    size_t operator()(ManagerKey const& k) const noexcept {
      // 简单位拼哈希
      return (uint64_t(k.macro) << 48) ^ (uint64_t(k.style) << 32);
    }
  };

  inline ManagerKey managerKey(const StageState& stage_state) const {
    return ManagerKey(macroScenario(stage_state), drivingStyle());
  }

  // ========== 通用配置获取模板 ==========
  template<typename Key, typename Value, typename Hash = std::hash<Key>, typename KeyEqual = std::equal_to<Key>>
  static std::optional<std::reference_wrapper<const Value>> getConfig(
      const std::unordered_map<Key, std::unique_ptr<Value>, Hash, KeyEqual>& cfg_map, const Key& key) {
    auto it = cfg_map.find(key);
    if (it != cfg_map.end() && it->second) {
      return std::cref(*(it->second));
    }
    return std::nullopt;
  }

  // ========== 配置获取模板（供各模块使用） ==========

  template<typename ConfigType>
  class ConfigGetter {
   public:
    using ConfigMap = std::unordered_map<ManagerKey, std::unique_ptr<ConfigType>, ManagerKeyHash>;

    explicit ConfigGetter(const ConfigMap& config_map) : config_map_(config_map) {}

    std::optional<std::reference_wrapper<const ConfigType>> getProfile(const ManagerKey& key) const {
      return getConfig(config_map_, key);
    }

    // 获取配置指针版本（更简单）
    const ConfigType* getProfilePtr(const ManagerKey& key) const {
      auto it = config_map_.find(key);
      if (it != config_map_.end() && it->second) {
        return it->second.get();
      }
      return nullptr;
    }

    // 检查配置是否存在
    bool hasProfile(const ManagerKey& key) const { return config_map_.find(key) != config_map_.end(); }

    // 获取所有可用的配置键
    std::vector<ManagerKey> getAllKeys() const {
      std::vector<ManagerKey> keys;
      keys.reserve(config_map_.size());
      for (const auto& [key, _] : config_map_) {
        keys.push_back(key);
      }
      return keys;
    }

   private:
    const ConfigMap& config_map_;
  };

  // ========== 创建ConfigGetter的工厂方法 ==========

  template<typename ConfigType>
  static std::unique_ptr<ConfigGetter<ConfigType>> createConfigGetter(
      const typename ConfigGetter<ConfigType>::ConfigMap& config_map) {
    return std::make_unique<ConfigGetter<ConfigType>>(config_map);
  }
};

}  // namespace gpal::pnc::planning