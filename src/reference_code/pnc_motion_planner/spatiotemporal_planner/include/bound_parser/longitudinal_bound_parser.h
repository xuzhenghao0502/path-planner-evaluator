#pragma once

#include "bound_parser/base_bound_parser.h"
#include "bound_parser/speed_limit_process.h"
#include "bound_parser/speed_wall_process.h"
#include "bound_parser/local_path_process.h"
#include "config/spatiotemporal_planner/longitudinal_bound_parser_config.pb.h"
#include "manager/spatiotemporal_planner_data_manager.h"
#include "manager/spatiotemporal_planner_scenario_manager.h"

namespace gpal::pnc::planning {

class LongitudinalBoundParser : public BaseBoundParser {
 public:
  LongitudinalBoundParser() = default;
  ~LongitudinalBoundParser() override = default;

  // 实现所有从顶层接口继承而来的纯虚函数
  bool init() override;
  void reset() override;
  std::string id() const override;

  /**
   * @brief 执行纵向边界解析
   * @param data_manager 由调度器传入的数据管理器
   * @details 该方法解析纵向边界信息，并将静态和动态障碍物转换为决策对象
   * @return bool 返回是否成功执行
   */
  bool run(DataManager& data_manager) override;

 private:
  bool caculateVBoundary(DataManager& data_manager);
  bool caculateSBoundary(DataManager& data_manager);

 protected:
  // 配置查找表
  LongitudinalBoundParserProfile profile_;

  SpeedWallProcess speed_wall_process_;
  SpeedLimitProcess speed_limit_process_;
  LocalPathProcess local_path_process_;

};

}  // namespace gpal::pnc::planning
