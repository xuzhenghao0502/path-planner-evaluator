/**
 * @file container_manager.h
 * @brief 定义 `ContainerManager` 类，该类作为数据容器的管理中枢，负责对各类数据容器进行高效管理。
 * @details 借助 `std::unordered_map` 存储不同类型的容器，提供容器的初始化、重置、创建、注册和获取等功能，
 * 能够方便地管理和操作各种数据容器，提升数据管理的灵活性和效率。
 */

/**
 * @file
 * @brief Use container manager to manage all data
 */

#pragma once

#include <memory>
#include <unordered_map>

#include "base/log.h"
#include "decision_data/container/container.h"

namespace gpal {
namespace pnc {
namespace prediction {

/**
 * @class ContainerManager
 * @brief 管理和操作各类数据容器的核心类。
 * @details 该类利用 `std::unordered_map` 存储不同类型的容器，
 * 提供了一系列方法用于容器的初始化、重置、创建、注册以及获取，
 * 实现了对数据容器的集中管理和高效操作。
 */
class ContainerManager {
 public:
  /**
   * @brief 默认构造函数。
   * @details 构造一个 `ContainerManager` 对象，不进行额外的初始化操作，仅完成对象的基本创建。
   */
  ContainerManager() = default;

  /**
   * @brief 默认析构函数。
   * @details 销毁 `ContainerManager` 对象，自动释放管理的容器资源，确保资源的正确回收。
   */
  ~ContainerManager() = default;

  void Init(const PredictionConfig& config);

  void Reset();

  /**
   * @brief 获取可变的容器指针。
   * @details 根据传入的容器类型，从存储的容器中查找对应类型的容器，
   * 并将其转换为指定模板类型的指针返回。若未找到则返回 `nullptr`。
   *
   * @tparam T 要获取的容器的具体类型。
   * @param[in] type 容器的消息类型，用于在 `unordered_map` 中查找对应容器。
   * @par 输入参数说明:
   * - type: `ContainerMessage::MessageType` 类型，用于标识容器类型。
   *
   * @return T* 指向指定类型容器的指针，若未找到则返回 `nullptr`。
   *
   * @par 流程图:
   * @startuml
   * start
   * :接收容器消息类型;
   * :在无序映射表中查找对应容器;
   * if (找到容器?) then (是)
   * :将容器指针转换为指定类型;
   * :返回转换后的指针;
   * else (否)
   * :返回 nullptr;
   * endif
   * stop
   * @enduml
   */
  template <typename T>
  T* GetContainer(const ContainerMessage::MessageType& type) {
    if (containers_.find(type) != containers_.end()) {
      return static_cast<T*>(containers_[type].get());
    }
    return nullptr;
  }

  std::unique_ptr<Container> CreateContainer(const ContainerMessage::MessageType& type);

  void RegisterContainer(const ContainerMessage::MessageType& type);

 private:
  std::unordered_map<ContainerMessage::MessageType, std::unique_ptr<Container>>
      containers_;  ///< 存储不同类型容器的无序映射表
};

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal
