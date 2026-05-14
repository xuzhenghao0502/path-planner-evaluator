#pragma once

#include <any>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "config_manager/config_manager.h"
#include "manager/spatiotemporal_planner_data_manager.h"
#include "manager/spatiotemporal_planner_scenario_manager.h"

namespace gpal::pnc::planning {

/**
 * @brief 通用配置加载器模板
 */
template<typename ConfigListType, typename ProfileType>
class SpatiotemporalConfigLoader {
 public:
  using ConfigMap = std::unordered_map<SpatiotemporalPlannerScenarioManager::ManagerKey, std::unique_ptr<ProfileType>,
                                       SpatiotemporalPlannerScenarioManager::ManagerKeyHash>;

  /**
   * @brief 加载配置
   * @param config_name 配置名称
   * @param[out] config_map 输出的配置映射
   * @return 是否加载成功
   */
  static bool loadConfig(const std::string& config_name, ConfigMap& config_map) {
    auto config_manager = Singleton<ConfigManager>::get_instance();
    if (config_manager == nullptr) {
      STLOG(E, "[SpatiotemporalConfigLoader::loadConfig] ConfigManager is not initialized.");
      return false;
    }

    auto list = config_manager->getConfig<ConfigListType>(config_name);
    if (!list.IsInitialized()) {
      STLOG(E, "[SpatiotemporalConfigLoader::loadConfig] Failed to initialize config list: ", config_name);
      return false;
    }

    config_map.clear();

    // 遍历配置列表
    for (const auto& entry : list.entries()) {
      SpatiotemporalPlannerScenarioManager::ManagerKey key{entry.manager().macro(), entry.manager().style()};

      // 创建配置实例
      config_map[key] = std::make_unique<ProfileType>(entry.profiles());

      STLOG(D, "[SpatiotemporalConfigLoader::loadConfig] Loaded config for scenario: ", "macro=",
            static_cast<int>(key.macro), ", style=", static_cast<int>(key.style));
    }

    STLOG(D, "[SpatiotemporalConfigLoader::loadConfig] Loaded ", config_map.size(), " configs from ", config_name);

    return !config_map.empty();
  }

  /**
   * @brief 获取指定场景的配置
   * @param config_map 配置映射
   * @param key 场景键
   * @return 配置的unique_ptr
   */
  static std::unique_ptr<ProfileType> getConfigForKey(const ConfigMap& config_map,
                                                      const SpatiotemporalPlannerScenarioManager::ManagerKey& key) {
    auto it = config_map.find(key);
    if (it != config_map.end() && it->second) {
      // 返回配置的深拷贝
      return std::make_unique<ProfileType>(*it->second);
    }

    STLOG(W, "[SpatiotemporalConfigLoader::getConfigForKey] No config found for key: ", "macro=",
          static_cast<int>(key.macro), ", style=", static_cast<int>(key.style));

    return nullptr;
  }
};

/**
 * @brief 配置注册表 - 管理所有模块的配置加载
 */
class ConfigRegistry {
 public:
  // 配置加载函数类型
  using LoaderFunc = std::function<bool(const SpatiotemporalPlannerScenarioManager::ManagerKey&,
                                        SpatiotemporalPlannerDataManager::ConfigInfo&)>;

  // 配置信息结构
  struct ConfigEntry {
    std::string config_list_name = "";
    std::string profile_type_name = "";
    LoaderFunc loader;
  };

  /**
   * @brief 获取单例实例
   */
  static ConfigRegistry& getInstance() {
    static ConfigRegistry instance;
    return instance;
  }

  /**
   * @brief 注册配置加载器
   * @tparam ConfigListType 配置列表类型
   * @tparam ProfileType 配置文件类型
   * @param config_name 配置名称
   * @param profile_type_name 配置类型名称（用于日志）
   * @param member_ptr 指向ConfigInfo中成员的指针
   */
  template<typename ConfigListType, typename ProfileType>
  void registerConfig(const std::string& config_name, const std::string& profile_type_name,
                      std::unique_ptr<ProfileType> SpatiotemporalPlannerDataManager::ConfigInfo::* member_ptr) {
    configs_.emplace_back(ConfigEntry{
        config_name, profile_type_name,
        // [修正] Lambda函数内部的缓存逻辑已更新
        [config_name, profile_type_name, member_ptr](
            const SpatiotemporalPlannerScenarioManager::ManagerKey& key,
            SpatiotemporalPlannerDataManager::ConfigInfo& config_info) -> bool {
          using ConfigMap =
              std::unordered_map<SpatiotemporalPlannerScenarioManager::ManagerKey, std::unique_ptr<ProfileType>,
                                 SpatiotemporalPlannerScenarioManager::ManagerKeyHash>;

          using ConfigMapPtr = std::shared_ptr<ConfigMap>;

          auto& cache = getInstance().config_caches_;
          ConfigMapPtr config_map_ptr;

          // 1. 检查缓存
          auto it = cache.find(config_name);
          if (it == cache.end()) {
            STLOG(I, "[ConfigRegistry] Cache miss for [", config_name, "]. Loading from file...");

            // 缓存未命中：创建一个新的ConfigMap
            auto new_config_map = std::make_shared<ConfigMap>();
            if (!SpatiotemporalConfigLoader<ConfigListType, ProfileType>::loadConfig(config_name, *new_config_map)) {
              STLOG(E, "[ConfigRegistry] Failed to load config map for ", profile_type_name);
              return false;
            }

            // 将指向map的shared_ptr存入缓存
            cache[config_name] = new_config_map;
            config_map_ptr = new_config_map;
            STLOG(I, "[ConfigRegistry] Loaded config map for [", config_name, "] with size: ", new_config_map->size());
          } else {
            STLOG(I, "[ConfigRegistry] Cache hit for [", config_name, "]. Using cached map.");
            // 缓存命中
            config_map_ptr = std::any_cast<ConfigMapPtr>(it->second);
          }

          if (!config_map_ptr)
            return false;

          // 2. 从缓存的图谱中查找当前场景需要的具体配置
          config_info.*member_ptr =
              SpatiotemporalConfigLoader<ConfigListType, ProfileType>::getConfigForKey(*config_map_ptr, key);

          if (config_info.*member_ptr == nullptr) {
            STLOG(E, "[ConfigRegistry] No ", profile_type_name,
                  " found for scenario key: macro=", static_cast<int>(key.macro),
                  ", style=", static_cast<int>(key.style));
            return false;
          }

          STLOG(I, "[ConfigRegistry] Successfully loaded ", profile_type_name,
                " for scenario: macro=", static_cast<int>(key.macro), ", style=", static_cast<int>(key.style));
          return true;
        }});

    STLOG(I, "[ConfigRegistry] Registered config loader for ", profile_type_name);
  }

  /**
   * @brief 加载所有注册的配置
   * @param manager_key 场景管理键
   * @param[out] config_info 配置信息
   * @return 是否全部加载成功
   */
  bool loadAllConfigs(const SpatiotemporalPlannerScenarioManager::ManagerKey& manager_key,
                      SpatiotemporalPlannerDataManager::ConfigInfo& config_info) const {
    STLOG(I, "[ConfigRegistry] Loading all configs for scenario: macro=", static_cast<int>(manager_key.macro),
          ", style=", static_cast<int>(manager_key.style));

    // 首先重置所有配置
    config_info.reset();

    // 加载所有注册的配置
    bool all_success = true;
    for (const auto& entry : configs_) {
      if (!entry.loader(manager_key, config_info)) {
        STLOG(E, "[ConfigRegistry] Failed to load config: ", entry.config_list_name, " (", entry.profile_type_name,
              ")");
        all_success = false;
      }
    }

    if (all_success) {
      STLOG(I, "[ConfigRegistry] Successfully loaded all ", configs_.size());
    } else {
      STLOG(E, "[ConfigRegistry] Some configs failed to load.");
    }

    return all_success;
  }

  /**
   * @brief 获取已注册的配置数量
   */
  size_t getRegisteredConfigCount() const { return configs_.size(); }

  /**
   * @brief 清空所有注册的配置
   */
  void clearRegistry() {
    configs_.clear();
    config_caches_.clear();
    STLOG(I, "[ConfigRegistry] Cleared all registered configs.");
  }

 private:
  // 私有构造函数（单例模式）
  ConfigRegistry() = default;
  ~ConfigRegistry() = default;

  // 禁用拷贝
  ConfigRegistry(const ConfigRegistry&) = delete;
  ConfigRegistry& operator=(const ConfigRegistry&) = delete;

  // 存储所有注册的配置
  std::vector<ConfigEntry> configs_;
  // 缓存配置结果
  // 使用 std::any 以支持不同类型的配置缓存
  std::unordered_map<std::string, std::any> config_caches_;
};

/**
 * @brief 初始化所有配置的注册
 * @note 应该在程序启动时调用一次
 */
void registerAllSpatiotemporalConfigs();

}  // namespace gpal::pnc::planning