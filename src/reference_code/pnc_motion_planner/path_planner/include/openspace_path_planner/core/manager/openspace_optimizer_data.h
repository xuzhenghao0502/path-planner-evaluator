#pragma once

#include "base_openspace_core_data.h"

// #include "path_generator.h"

namespace gpal::pnc::planning {

/**
 * @brief 开放空间优化模块的数据结构，用于轨迹优化过程中的输入输出中间结果管理。
 *
 * 本结构体继承自 BaseOpenspaceCoreData，作为开放空间路径优化过程中的核心数据容器，
 * 用于传递路径规划的上下文信息，包括初始路径、优化结果、参考空间、决策结果等。
 */
struct OpenspaceOptimizerData : public BaseOpenspaceCoreData {
  /**
   * @brief 模型名称，例如"ocp" 等，用于配置和调试时标识。
   */
  std::string model_name_ = "";

  /**
   * @brief 调试信息前缀，可用于日志打印时快速定位优化器。
   */
  std::string debug_info_ = "optimizer: ";

  /**
   * @brief 初始轨迹（通常由混合A*或其他搜索算法生成）。
   */
  DiscretizedPath origin_path_;

  /**
   * @brief 优化后的多段轨迹序列，通常每段对应一次决策或阶段。
   */
  std::vector<DiscretizedPath> optimizer_flu_path_;

  /**
   * @brief 拼接后的完整优化轨迹，通常用于输出给控制模块。
   */
  DiscretizedPath optimizer_whole_flu_path_;

  // /**
  //  * @brief 可视化边界信息
  //  */
  std::vector<PathData::BoundsVec3dWithId> boundarys_vis_;

  /**
   * @brief 指向当前规划场景下的自由空间信息，用于优化约束生成。
   */
  std::shared_ptr<Freespace> freespace_ptr_ = nullptr;

  /**
   * @brief 当前帧的决策结果，包括泊车目标、障碍物过滤结果等。
   */
  std::shared_ptr<DecisionResult> decision_result_ = nullptr;

  /**
   * @brief 决策模块生成的障碍物对象映射表，用于参考轨迹的障碍物处理。
   */
  std::shared_ptr<Decision::DecisionObjectMap> decision_object_map_ = nullptr;

  /**
   * @brief 当前坐标系转换矩阵（局部坐标 → 全局坐标），通常用于路径变换。
   */
  Eigen::Matrix4d tf_ego_2_map_ = Eigen::Matrix4d::Identity();

  /**
   * @brief 当前坐标系转换矩阵（全局坐标 → 局部坐标），通常用于路径变换。
   */
  Eigen::Matrix4d tf_map_2_ego_ = Eigen::Matrix4d::Identity();

  /**
   * @brief 地图中的原始参考点，作为路径规划的参考原点。
   */
  MapPoint origin_point_ = MapPoint();

  /**
   * @brief 初始车辆位置点，用于路径规划起点设定。
   */
  MapPoint oringin_vehicle_point_ = MapPoint();

  /**
   * @brief 优化器的配置参数
   */

  std::shared_ptr<ParkingPathOptimizerProfile> optimizer_profiles_;

  /**
   * @brief 障碍物线段集合，用于路径规划中的碰撞检测和避障。
   */
  std::unique_ptr<std::vector<ObstaclesLinesegment>> obstacles_linesegments_;

  /**
   * @brief 分段点，用于路径优化中的分段处理。
   */
  PathPt piecewise_point_;
  /**
   * @brief 终点，用于库位抖动时重优化
   */
  PathPt destination_point_;
  /**
   * @brief 终点拼接直线长度，用于库位抖动时重优化
   */
  double destination_straight_line_length_ = 0.0;
  /**
   * @brief 默认构造函数。
   */
  OpenspaceOptimizerData() = default;

  /**
   * @brief 清空所有成员变量，便于数据复用。
   */
  void clear() {
    model_name_.clear();
    debug_info_ = "optimizer: ";

    origin_path_.clear();
    optimizer_flu_path_.clear();
    optimizer_whole_flu_path_.clear();
    boundarys_vis_.clear();

    freespace_ptr_.reset();
    decision_result_.reset();
    decision_object_map_.reset();

    tf_ego_2_map_.setIdentity();
    tf_map_2_ego_.setIdentity();
    origin_point_ = MapPoint();
    oringin_vehicle_point_ = MapPoint();
    obstacles_linesegments_.reset();
    destination_straight_line_length_ = 0.0;
  }
};

}  // namespace gpal::pnc::planning
