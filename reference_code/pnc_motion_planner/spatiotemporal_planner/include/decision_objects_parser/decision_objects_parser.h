#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "config/base_object_parser_config.pb.h"
#include "config/spatiotemporal_planner/decision_object_parser_config.pb.h"
#include "manager/spatiotemporal_planner_data_manager.h"
#include "spatiotemporal_abstract_module.h"

namespace gpal::pnc::planning {

class DecisionObjectsParser : public SpatiotemporalAbstractModule {
 public:
  DecisionObjectsParser() = default;
  ~DecisionObjectsParser() = default;

  /**
   * @brief 获取模块 ID
   * @return std::string 模块 ID
   * @details 该方法返回当前模块的唯一标识符
   */
  std::string id() const override;

  bool init() override;
  void reset() override;

  /**
   * @brief 直接实现带参数的run方法
   * @param data_manager 由调度器传入的数据管理器指针
   * @details 该方法解析决策对象信息，并将静态和动态障碍物转换为决策对象
   * @return bool 返回是否成功执行
   */
  bool run(SpatiotemporalPlannerDataManager& data_manager) override;

 private:
  /**
   * @brief 根据障碍物类型获取对应的buffer配置
   * @param object_type 障碍物类型
   * @return 对应的buffer配置信息
   */
  BaseObjectParserConfig::ObstacleBufferInfo getObstacleBufferInfo(const BaseObjectParserConfig::ObjectType& object_type) const;

  /**
   * @brief 计算纵向buffer
   * @param buffer_info buffer配置信息
   * @param v_ego 自车速度
   * @param object_speed 障碍物速度
   * @return 计算得到的纵向buffer
   */
  double calculateLongitudinalBuffer(const  BaseObjectParserConfig::ObstacleBufferInfo& buffer_info, double v_ego, double object_speed,
                                     bool is_follower) const;

  /**
   * @brief 计算横向buffer
   * @param buffer_info buffer配置信息
   * @param ego_v 自车速度
   * @param object_speed 障碍物速度
   * @return 计算得到的横向buffer
   */
  double calculateLateralBuffer(const  BaseObjectParserConfig::ObstacleBufferInfo& buffer_info, double ego_v, double object_speed) const;

  BaseObjectParserConfig::ObjectType convertToLateralObjectType(const Decision::ObjectType& object_type) const;

  /**
   * @brief 创建基础障碍物形状协方差矩阵（仅包含障碍物尺寸）
   * @param length 障碍物长度
   * @param width 障碍物宽度
   * @return 局部坐标系下的基础形状协方差矩阵
   */
  Eigen::Matrix2d createBaseShapeCovariance(double length, double width) const;

  /**
   * @brief 创建缓冲区协方差矩阵（包含纵向、横向和自车缓冲）
   * @param length 障碍物长度
   * @param width 障碍物宽度
   * @param long_buffer 纵向缓冲距离
   * @param lateral_buffer 横向缓冲距离
   * @param ego_buffer 自车半宽缓冲距离
   * @return 局部坐标系下的缓冲协方差矩阵
   */
  Eigen::Matrix2d createBufferCovariance(double length, double width, double long_buffer, double lateral_buffer,
                                         double ego_buffer) const;

  /**
   * @brief 计算静态协方差矩阵（基础形状 + 缓冲 + 感知不确定性）
   * @param obs_x, obs_y 障碍物位置
   * @param obs_heading 障碍物朝向
   * @param obs_length, obs_width 障碍物尺寸
   * @param object 障碍物对象指针
   * @param prediction_index 预测点索引
   * @param ellipse_heading 椭圆朝向（输出）
   * @return 最终的静态协方差矩阵
   */
  Eigen::Matrix2d computeStaticCovariance(double obs_x, double obs_y, double obs_heading, double obs_length,
                                          double obs_width, const Decision::DecisionObject* object,
                                          int prediction_index, double& ellipse_heading) const;

  // 配置查找表
  std::unordered_map<SpatiotemporalPlannerScenarioManager::ManagerKey, std::unique_ptr<DecisionObjectParserProfile>,
                     SpatiotemporalPlannerScenarioManager::ManagerKeyHash>
      config_map_;

  // 使用通用的ConfigGetter
  std::unique_ptr<SpatiotemporalPlannerScenarioManager::ConfigGetter<DecisionObjectParserProfile>> config_getter_;

  std::unique_ptr<ReferenceLine> target_ref_line_ = nullptr;
  DecisionObjectParserProfile decision_object_parser_config_;
};

}  // namespace gpal::pnc::planning
