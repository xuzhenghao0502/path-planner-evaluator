#pragma once

#include <string>
#include <vector>

#include "common/spatiotemporal_common.h"
#include "config/spatiotemporal_planner/spatiotemporal_pipeline_config.pb.h"
#include "config_manager/config_manager.h"

namespace gpal::pnc::planning {

class PipelineConfigManager {
 public:
  // 单例访问点保持不变
  static PipelineConfigManager& Instance() {
    static PipelineConfigManager instance;
    return instance;
  }

  /**
   * @brief 根据流水线名称(string)实时获取模块列表
   * @details 该函数每次调用都会从ConfigManager获取最新的配置，并进行查找。
   * @param name 要查找的流水线名称
   * @param out_modules 如果找到，用于存储模块列表的输出参数
   * @return bool 是否成功找到对应的流水线配置
   */
  bool getPipeline(const std::string& name, std::vector<std::string>& out_modules) const;

 private:
  // 私有化构造和析构，以保证单例模式
  PipelineConfigManager() = default;
  ~PipelineConfigManager() = default;
};

}  // namespace gpal::pnc::planning