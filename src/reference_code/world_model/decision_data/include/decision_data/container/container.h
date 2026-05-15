/**
 * @file container.h
 * @brief 此文件定义了数据容器的抽象基类 `Container`，为各类具体的数据容器提供统一的接口规范。
 * @details 抽象基类 `Container` 包含纯虚函数，具体的数据容器类需继承该类并实现这些函数，
 * 从而实现数据插入和清空操作，有助于统一管理和扩展不同的数据容器。
 */

#pragma once

#include <any>

#include "base/log.h"
#include "config_manager/config_manager.h"
#include "obstacle/obstacle.h"
#include "proto/prediction/prediction_config.pb.h"

namespace gpal {
namespace pnc {
namespace prediction {

/**
 * @class Container
 * @brief 数据容器的抽象基类，定义了数据操作的通用接口。
 * @details 该类作为抽象基类，包含纯虚函数 `Insert` 和 `Clear`，
 * 具体的数据容器类需要继承该类并实现这些纯虚函数，以实现数据的插入和清空操作。
 */
class Container {
 public:
  /**
   * @brief 默认构造函数。
   * @details 构造一个 `Container` 对象，使用默认实现，不进行额外的初始化操作。
   */
  Container() = default;

  /**
   * @brief 虚析构函数。
   * @details 确保在删除指向派生类对象的基类指针时，能正确调用派生类的析构函数，使用默认实现。
   * 防止内存泄漏，保证资源的正确释放。
   */
  virtual ~Container() = default;

  /**
   * @brief 向容器中插入数据。
   * @details 纯虚函数，具体的数据容器类需要实现该函数，用于将数据插入到容器中。
   *
   * @param[in] message 以类格式表示的消息数据，使用 `std::any` 类型可接受任意类型的数据。
   * @par 输入参数说明:
   * - message: 可以是任意有效的数据类型，由具体容器实现类处理。
   *
   * @return bool 插入成功返回 `true`，失败返回 `false`。
   *
   * @par 流程图:
   * @startuml
   * start
   * :接收消息数据;
   * if (数据有效且插入成功?) then (是)
   * :返回 true;
   * else (否)
   * :返回 false;
   * endif
   * stop
   * @enduml
   */
  virtual bool Insert(const std::any& message, const bool is_hpa_stage = false) = 0;

  /**
   * @brief 清空容器内的所有信息。
   * @details 纯虚函数，具体的数据容器类需要实现该函数，用于清除容器内的所有数据和状态信息。
   *
   * @par 流程图:
   * @startuml
   * start
   * :执行清空操作;
   * :清除所有数据和状态信息;
   * stop
   * @enduml
   */
  virtual void Clear() = 0;
};

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal