#pragma once

#include <any>
#include <functional>
#include <iostream>
#include <string>
#include <string_view>
#include <unordered_map>

#include "openspace_path_planner/core/optimizer/openspace_path_optimizer.h"
#include "openspace_path_planner/core/generator/openspace_path_provider.h"
#include "openspace_path_planner/core/generator/arc_model.h"

namespace gpal::pnc::planning {

/**
 * @brief 开放空间核心调度管理器，用于注册和执行轨迹生成和优化器。
 *
 * 支持静态注册（固定名称 → 优化器回调）和动态注册（手动绑定任意优化器），
 * 并通过 std::any 作为通用数据传递接口，使调度逻辑与数据结构解耦。
 */
class OpenspaceCoreManager {
 public:
  using OpenspaceStatus = BaseOpenspacePathPlanner::OpenspaceStatus;

  /**
   * @brief 回调类型，接受 std::any 数据并返回运行状态。
   */
  using Callback = std::function<OpenspaceStatus(std::any& data)>;

  /**
   * @brief 手动注册优化器回调函数，可支持任意逻辑或实例。
   *
   * @param name 逻辑标识名称（如 "ipm_ocp"）
   * @param cb 回调函数对象，将被存入内部事件表
   */
  void Register(const std::string& name, Callback cb) { events_[name] = std::move(cb); }

  /**
   * @brief 根据名称注册系统内置优化器（从静态映射表中查找并绑定）。
   *
   * @param name 优化器名称，如 "ipm_ocp"
   * @return true 注册成功
   * @return false 注册失败，名称未找到
   */
  bool Register(const std::string& name) {
    auto it = kPersistentMap().find(name);
    if (it == kPersistentMap().end()) {
      std::cerr << "[OpenspaceCoreManager] RegisterByName failed: unknown name = " << name << "\n";
      return false;
    }
    active_name_ = name;
    Register(name, it->second(*this));  // 调用绑定器，捕获成员变量
    return true;
  }

  /**
   * @brief 执行当前已激活的优化器。
   *
   * @param data 输入/输出的上下文数据容器（一般为 OpenspaceOptimizerData*）
   * @return OpenspaceStatus 优化器运行状态
   */
  OpenspaceStatus Execute(std::any& data) {
    if (active_name_ == "" || events_.count(active_name_) == 0) {
      return OpenspaceStatus::DEFAULT;
    }
    return events_[active_name_](data);
  }

  /**
   * @brief 清除当前激活的优化器的内部状态。
   */
  void Clear() {
    optimizer_.clear();
    searcher_.clear();
  }

 private:
  /// 名称 → 回调函数映射表
  std::unordered_map<std::string, Callback> events_;

  /// 当前激活的优化器名称
  std::string active_name_ = "";

  /// 优化器成员对象，可复用
  OpenspacePathOptimizer optimizer_;

  /// 搜索器成员对象，可复用
  PathProviderBaseHAStar<ArcModel, ArcPathNodeKeyHash> searcher_;

  /// 回调函数绑定器类型：将成员绑定为回调函数
  using PersistentCallbackBinder = std::function<Callback(OpenspaceCoreManager&)>;

  /**
   * @brief 内部静态注册表：定义各优化器名称与绑定逻辑的对应关系。
   *
   * 可根据名称从此表中生成绑定到成员的回调函数。
   */
  static const std::unordered_map<std::string_view, PersistentCallbackBinder>& kPersistentMap() {
    static const std::unordered_map<std::string_view, PersistentCallbackBinder> map = {
        {
            "ipm_ocp",
            [](OpenspaceCoreManager& self) -> Callback {
              // 返回绑定成员 optimizer_ 的回调
              return [&self](std::any& data) { return self.optimizer_.run(data); };
            },
        },
        {
            "hybrid_astar",
            [](OpenspaceCoreManager& self) -> Callback {
              // 返回绑定成员 searcher_ 的回调
              return [&self](std::any& data) { return self.searcher_.run(data); };
            },
        },
        // 可继续扩展更多优化器，如 ilqr、graph 等
    };
    return map;
  }
};

}  // namespace gpal::pnc::planning
