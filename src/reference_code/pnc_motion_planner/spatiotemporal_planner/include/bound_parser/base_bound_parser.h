// BaseBoundParser.h (修改后)
#pragma once
#include "spatiotemporal_abstract_module.h"

namespace gpal::pnc::planning {

/**
 * @class BaseBoundParser
 * @brief 所有BoundParser模块的基类
 * @details 该类主要存储与边界解析相关的公共数据和方法。
 */
class BaseBoundParser : public SpatiotemporalAbstractModule {
 public:
  BaseBoundParser() = default;
  ~BaseBoundParser() override = default;
};

}  // namespace gpal::pnc::planning