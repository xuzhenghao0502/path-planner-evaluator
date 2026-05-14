/**
 * @file lateral_path.h
 * @brief 横向路径预处理器
 * @details 本类负责对原始路径进行离散化处理，生成可供速度规划使用的路径组。
 */
#pragma once

#include "speed_common/speed_common.h"

namespace gpal::pnc::planning {

class LateralPathProcessor {
 public:
   /**
   * @brief 默认构造函数
   */
  LateralPathProcessor() = default;
    /**
   * @brief 默认析构函数
   */
  ~LateralPathProcessor() = default;

  PathGroup getDiscretizedPathGroup(const DiscretizedPath& path, const double& valid_length = 200.0,
                                    const double& step = 1.0, const double& expand_length = 100.0);
  PathGroup getLocalPathGroup(const DiscretizedPath& path, const double& valid_length = 100.0, const double& step = 1.0,
                              const double& expand_length = 100.0);

 private:
  void getPathGroup(const DiscretizedPath& path, const double& valid_length, const double& step,
                    const double& expand_length, PathGroup& path_group);
};

}  // namespace gpal::pnc::planning