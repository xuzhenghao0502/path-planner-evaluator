/**
 * @file container_manager.cc
 * @brief 此文件实现了 `ContainerManager` 类，该类作为数据容器的管理中枢，负责对各类数据容器进行高效管理。
 * @details 借助 `std::unordered_map` 存储不同类型的容器，实现了容器的初始化、重置、创建、注册和获取等功能。
 */

#include "decision_data/container/container_manager.h"

#include "decision_data/container/localization/localization_container.h"
#include "decision_data/container/obstacles/obstacles_container.h"

namespace gpal {
namespace pnc {
namespace prediction {

/**
 * @brief 初始化容器管理器。
 * @details
 * 根据传入的预测配置信息对容器管理器进行初始化操作，为后续容器管理工作做好准备。
 *
 * @param[in] config 预测配置信息，包含容器管理相关的配置参数。
 * @par 输入参数说明:
 * - config: 有效的 `PredictionConfig` 对象，包含容器管理所需的配置参数。
 *
 * @par 流程图:
 * @startuml
 * start
 * :接收预测配置信息;
 * :依据配置进行初始化操作;
 * stop
 * @enduml
 */
void ContainerManager::Init(const PredictionConfig& config) {
  for (const auto& message : config.container_config().messages()) {
    if (message.has_type()) {
      RegisterContainer(message.type());
    }
  }
}

/**
 * @brief 重置容器管理器。
 * @details
 * 重置容器管理器的状态，清空所有管理的容器，使容器管理器恢复到初始状态。
 *
 * @par 流程图:
 * @startuml
 * start
 * :清空存储容器的无序映射表;
 * :重置管理器状态;
 * stop
 * @enduml
 */
void ContainerManager::Reset() {
  for (auto it = containers_.begin(); it != containers_.end(); ++it) {
    ERT_PLOG_I << "Clear container - " << ContainerMessage::MessageType_Name(it->first);
    it->second->Clear();
  }
  containers_.clear();
}

/**
 * @brief 创建一个容器。
 * @details 根据传入的容器消息类型创建对应的容器，并返回指向该容器的智能指针。
 *
 * @param[in] type 容器的消息类型，用于确定要创建的容器类型。
 * @par 输入参数说明:
 * - type: `ContainerMessage::MessageType` 类型，用于标识要创建的容器类型。
 *
 * @return std::unique_ptr<Container> 指向新创建容器的智能指针。
 *
 * @par 流程图:
 * @startuml
 * start
 * :接收容器消息类型;
 * :根据类型创建对应容器;
 * :返回新容器的智能指针;
 * stop
 * @enduml
 */
std::unique_ptr<Container> ContainerManager::CreateContainer(const ContainerMessage::MessageType& type) {
  std::unique_ptr<Container> container_ptr(nullptr);
  switch (type) {
    case ContainerMessage::PERCEPTION_OBSTACLES:
      container_ptr.reset(new ObstaclesContainer());
      break;
    case ContainerMessage::LOCALIZATION:
      container_ptr.reset(new LocalizationContainer());
      break;
    default:
      ERT_PLOG_E << "Got an unsupported container type[" << type << "].";
      break;
  }

  return container_ptr;
}

/**
 * @brief 注册一个容器。
 * @details 根据传入的容器消息类型注册对应的容器，将其添加到管理器中进行管理。
 *
 * @param[in] type 容器的消息类型，用于确定要注册的容器类型。
 * @par 输入参数说明:
 * - type: `ContainerMessage::MessageType` 类型，用于标识要注册的容器类型。
 *
 * @par 流程图:
 * @startuml
 * start
 * :接收容器消息类型;
 * :创建对应容器;
 * :将容器添加到无序映射表;
 * stop
 * @enduml
 */
void ContainerManager::RegisterContainer(const ContainerMessage::MessageType& type) {
  containers_[type] = CreateContainer(type);
  ERT_PLOG_I << "Container [" << ContainerMessage::MessageType_Name(type) << "] is registered.";
}

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal
