#pragma once

#include <any>
#include <memory>
#include <vector>

namespace gpal::pnc::planning {

/**
 * @brief 开放空间路径规划/优化器的基类，用于统一不同规划/优化器的接口。
 *
 * 该基类定义了所有规划/优化器必须实现的核心方法 `run()`，以便在调度器中通过多态调用。
 * 接口使用 `std::any` 传递上下文数据，支持灵活的数据结构扩展。
 */
class BaseOpenspacePathPlanner {
 public:
  /**
   * @brief 规划/优化器运行状态枚举。
   */
  enum OpenspaceStatus : uint8_t {
    DEFAULT = 0,  ///< 默认状态，尚未初始化。
    INIT,         ///< 已初始化，准备运行。
    WAITING,      ///< 等待外部条件触发或输入完成。
    FINISH,       ///< 计算完成，可读取输出结果。
    FAILED,       ///< 计算失败，可能需要重新初始化或调整输入。
  };

 public:
  /**
   * @brief 虚析构函数，支持多态释放。
   */
  virtual ~BaseOpenspacePathPlanner() = default;

  /**
   * @brief 执行规划/优化器主过程。
   *
   * @param data 输入输出通用数据容器，内部应为派生自 BaseOpenspaceCoreData 的结构体指针。
   * @return OpenspaceStatus 返回规划/优化过程的状态，供调用方判断是否完成或继续等待。
   */
  virtual OpenspaceStatus run(std::any& data) = 0;
};

}  // namespace gpal::pnc::planning
