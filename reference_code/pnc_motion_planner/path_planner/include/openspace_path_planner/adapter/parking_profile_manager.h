#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "parking_profile_traits.h"
#include "parking_profile_adapter.h"
#include "config/path_planner/parking_path_planner_config.pb.h"

namespace gpal::pnc::planning {

/**
 * @brief 配置管理器，用于从嵌套配置中统一获取具体 Profile 对象。
 *
 * 支持泛型模板接口，根据传入的配置类型和目标 Profile 类型自动从 proto 中获取指定 profile。
 * 内部依赖 `ProfileTraits<T>` 实现分层提取逻辑，简化用户代码。
 *
 * @note 使用时通常配合 Adapter 进行封装后的对象初始化。
 */
class ProfileManager {
 public:
  /**
   * @brief 获取 ProfileManager 的全局单例实例。
   *
   * @return ProfileManager&
   */
  static ProfileManager& instance() {
    static ProfileManager instance;
    return instance;
  }

  // 禁止拷贝构造与赋值
  ProfileManager(const ProfileManager&) = delete;
  ProfileManager& operator=(const ProfileManager&) = delete;

  /**
   * @brief 获取指定名称的 Profile（主接口）。
   *
   * @tparam T 目标 Profile 类型，例如 ParkingPathOptimizerProfile
   * @tparam M 配置来源类型（父层结构），如 PlannerProfile 或 ParkingPathPlannerConfig
   * @param config 指向 M 类型配置的智能指针，通常由 Adapter 加载。
   * @param name 要获取的 profile 名称
   * @return std::unique_ptr<T> 获取成功返回新建的 profile 对象，失败则为 nullptr
   */
  template <typename T, typename M>
  std::unique_ptr<T> profile(const std::unique_ptr<M>& config, const std::string& name) {
    if (config == nullptr) {
      return nullptr;
    }
    const auto& profiles_map = ProfileTraits<T>::getProfiles(*config);
    auto it = profiles_map.find(name);
    if (it != profiles_map.end()) {
      return std::make_unique<T>(it->second);
    }
    return nullptr;
  }

  /**
   * @brief 支持 string_view 名称参数的 profile 获取接口（重载版本）。
   *
   * @tparam T 目标 Profile 类型
   * @tparam M 配置来源类型
   * @param config 指向配置的智能指针
   * @param name_view 名称视图，避免重复拷贝
   * @return std::unique_ptr<T> 获取成功返回 profile 指针，失败返回 nullptr
   */
  template <typename T, typename M>
  std::unique_ptr<T> profile(const std::unique_ptr<M>& config, std::string_view name_view) {
    return profile<T>(config, std::string(name_view));
  }

 private:
  /**
   * @brief 构造函数私有化，确保单例。
   */
  ProfileManager() = default;

  /**
   * @brief 析构函数默认。
   */
  ~ProfileManager() = default;
};

}  // namespace gpal::pnc::planning
