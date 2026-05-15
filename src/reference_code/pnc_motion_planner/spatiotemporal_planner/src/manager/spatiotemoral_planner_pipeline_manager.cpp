#include "manager/spatiotemoral_planner_pipeline_manager.h"

namespace gpal::pnc::planning {
bool PipelineConfigManager::getPipeline(const std::string& name, std::vector<std::string>& out_modules) const {
  // 1. 获取中心配置管理器实例
  auto config_manager = Singleton<ConfigManager>::get_instance();
  if (config_manager == nullptr) {
    STLOG(E, "[PipelineConfigManager] ConfigManager is not initialized.");
    return false;
  }

  // 2. 实时获取流水线配置列表的Proto对象
  auto list = config_manager->getConfig<SpatiotemporalPipelineConfigList>("SpatiotemporalPipelineConfigList");

  // 3. 遍历Proto对象，查找与输入名称匹配的条目
  for (const auto& entry : list.entries()) {
    if (entry.has_name() && entry.name() == name) {
      // 找到了匹配的条目
      out_modules.clear();
      out_modules.reserve(entry.module_size());
      for (const auto& module_name : entry.module()) {
        out_modules.push_back(module_name);
      }
      // 查找成功，直接返回
      return true;
    }
  }

  // 4. 如果遍历完成仍未找到，则查找失败
  STLOG(W, "[PipelineConfigManager] No pipeline config found for name: ", name);
  out_modules.clear();
  return false;
}
}  // namespace gpal::pnc::planning