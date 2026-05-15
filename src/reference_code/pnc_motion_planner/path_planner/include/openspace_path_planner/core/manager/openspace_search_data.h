#pragma once

#include "base_openspace_core_data.h"
#include "openspace_path_planner/core/generator/openspace_roi_decider.h"

// #include "path_generator.h"

namespace gpal::pnc::planning {

/**
 * @brief 开放空间轨迹搜索模块的数据结构，用于轨迹搜索过程中的输入输出中间结果管理。
 *
 * 本结构体继承自 BaseOpenspaceCoreData，作为开放空间轨迹搜索过程中的核心数据容器，
 * 用于传递路径规划的上下文信息，包括参考空间、配参设置、结果轨迹等。
 */
struct OpenspaceSearchData : public BaseOpenspaceCoreData {
  // 1. 环境数据
  RoiDecideResult roi_;
  std::shared_ptr<Freespace> freespace_ptr_ = nullptr;

  // 2. 任务数据
  PathPt start_pose_;
  PathPt end_pose_;
  PathPt::Direction start_direction_ = PathPt::Direction::FORWARD;  // 搜索起点方向
  bool is_region_search_ = false;
  math::Box2d goal_region_;
  double goal_region_heading_ = 0.0;
  double goal_region_heading_tolerance_ = 0.0;

  std::vector<PathPt> search_path_;  // 结果轨迹

  // 3.其他配置或数据
  std::string search_config_name_ = "";  // 配置名称，例如"regular" 等，用于配置的标识。
  std::string debug_info_ = "search: ";  // 调试信息前缀，可用于日志打印时快速定位轨迹搜索器。
  std::shared_ptr<ParkingPathGeneratorProfile> path_generator_profile_;  // 轨迹搜索器的配置参数
  std::vector<std::string> scenario_tags_;  // 场景标签，用于场景分类，用于动态调整参数，例如"replan"等

  OpenspaceSearchData() = default;

  void clear() {
    search_config_name_.clear();
    debug_info_ = "search: ";
    freespace_ptr_.reset();
    roi_ = RoiDecideResult();
    path_generator_profile_.reset();
    search_path_.clear();
    start_direction_ = PathPt::Direction::FORWARD;
    scenario_tags_.clear();
    is_region_search_ = false;
    goal_region_ = math::Box2d();
    goal_region_heading_ = 0.0;
    goal_region_heading_tolerance_ = 0.0;
  }
};

}  // namespace gpal::pnc::planning
