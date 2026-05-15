#pragma once

#include <string>

#include "manager/spatiotemporal_planner_data_manager.h"
#include "util/abstract_factory.h"

namespace gpal::pnc::planning {
/**
 * @class SpatiotemporalAbstractModule
 * @brief 抽象模块类，用于定义时空规划器的基本接口
 * @details 该类提供了时空规划器模块的基本接口，所有具体模块都应继承自此类并实现其纯虚函数。
 */

class SpatiotemporalAbstractModule {
 public:
  SpatiotemporalAbstractModule() = default;
  virtual ~SpatiotemporalAbstractModule() = default;
  using DataManager = SpatiotemporalPlannerDataManager;

  virtual bool init() = 0;
  virtual void reset() = 0;

  /**
   * @brief 模块的运行入口
   * @param data_manager 数据管理器的指针，用于模块间数据交互
   * @return bool 运行是否成功
   */
  virtual bool run(DataManager& data_manager) = 0;
  virtual std::string id() const = 0;

 private:
};

// 定义一个便利的宏，用于注册规划模块
// 调用时只需提供类名，它会自动使用类名的字符串作为ID
#define REGIST_MODULE(classname) REGIST_ABSTRACT_PRODUCT(classname, SpatiotemporalAbstractModule)

}  // namespace gpal::pnc::planning