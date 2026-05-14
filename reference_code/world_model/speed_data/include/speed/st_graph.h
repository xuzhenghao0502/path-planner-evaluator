/**
 * @file st_graph.h
 * @brief 时空图数据容器
 * @details 本类管理速度规划中的时空约束数据
 */
#pragma once

#include <tuple>
#include <vector>

#include "obstacle/st_boundary.h"
#include "point/trajectory_pt.h"
#include "proto/common/pnc_point.pb.h"

namespace gpal::pnc::planning {

struct STDrivableBoundary {
  double t;                                                        ///< 时间戳（单位：秒）
  std::string upper_obj_id = "None";                               ///< 上方障碍物ID（"None"表示无障碍物）
  bool has_front_yield_obj = false;                                ///< 是否存在需要让行的前方障碍物
  double s_upper_bound = std::numeric_limits<double>::infinity();  ///< S方向上边界（单位：米）
  bool is_upper_bound_soft = false;                                ///< 上边界是否为软约束（允许轻微突破）
  double v_upper = 100.0;                                          ///< 上方障碍物投影速度（单位：m/s，无物时取大值）
  double upper_signed_lateral_distance =
      std::numeric_limits<double>::infinity();  ///< 到上方障碍物的横向符号距离（正表示左，单位：米）

  std::string lower_obj_id = "None";    ///< 下方障碍物ID（"None"表示无障碍物）
  bool has_back_take_over_obj = false;  ///< 是否存在需要超车的后方障碍物
  double s_lower_bound = 0.0;           ///< S方向下边界（单位：米）
  double v_lower = 0.0;                 ///< 下方障碍物投影速度（单位：m/s）
  double lower_signed_lateral_distance =
      std::numeric_limits<double>::infinity();  ///< 到下方障碍物的横向符号距离（正表示左，单位：米）
};
typedef std::vector<STDrivableBoundary> STDrivableBoundaries;

// t, s_lower, s_upper

constexpr double kObsSpeedIgnoreThreshold = 100.0;

class StGraph {
 public:
  /**
   * @brief 默认构造函数
   */
  StGraph() = default;

  /**
   * @brief 重置所有内部状态到初始值
   * @see 方法实现细节见clear()函数注释
   */
  void clear();

  /**
   * @brief 加载规划所需基础数据
   * @param st_boundaries ST障碍物边界集合（时空维度）
   * @param min_s_on_st_boundaries ST边界上的最小s值（单位：米）
   * @param init_point 规划起始点状态
   * @param path_data_length 实际路径数据长度（单位：米）
   * @param total_time_by_conf 配置文件定义的总规划时间（单位：秒）
   */
  void loadData(const std::vector<const STBoundary*>& st_boundaries, const double min_s_on_st_boundaries,
                const TrajectoryPt& init_point, const double path_data_length, const double total_time_by_conf);

  /**
   * @brief 判断是否已完成数据初始化
   * @return true 表示已成功加载数据并完成初始化
   */
  bool is_initialized() const { return init_; }

  /**
   * @brief 获取ST边界集合（只读）
   * @return 包含时空障碍物边界的常量引用
   */
  const std::vector<const STBoundary*>& st_boundaries() const;

  /**
   * @brief 获取ST边界集合（可编辑）
   * @return 包含时空障碍物边界的可变指针
   */
  std::vector<const STBoundary*>* mutable_st_boundaries();

  /**
   * @brief 获取ST边界最小s值
   * @return 最小s坐标值（单位：米）
   */
  double min_s_on_st_boundaries() const;

  /**
   * @brief 获取规划起始状态点
   * @return 包含位置、速度、加速度等信息的轨迹点
   */
  const TrajectoryPt& init_point() const;

  /**
   * @brief 获取实际路径长度
   * @return 路径数据物理长度（单位：米）
   */
  double path_length() const;

  /**
   * @brief 获取配置定义的总规划时间
   * @return 总时间约束（单位：秒）
   */
  double total_time_by_conf() const;

  /**
   * @brief 调整可行区域边界容器大小
   * @param drivable_boundary_size 新边界集合尺寸
   * @return 调整是否成功
   */
  bool resizeSTDrivableBoundaries(size_t drivable_boundary_size);

  /**
   * @brief 设置指定时间索引的可行边界
   * @param t_index 时间维度索引
   * @param st_drivable_boundary 待设置的边界数据
   * @return 设置操作是否成功
   */
  bool setSTDrivableBoundaries(size_t t_index, const STDrivableBoundary& st_drivable_boundary);

  /**
   * @brief 计算上边界投影速度
   * @param time_window_length 时间窗口长度（单位：秒）
   */
  void calcSUpperBoundsProjectedSpeed(double time_window_length);

  /**
   * @brief 获取可行驾驶边界（只读）
   * @return 包含速度-距离约束的常量边界集合
   */
  const STDrivableBoundaries& st_drivable_boundaries() const;

  /**
   * @brief 获取可行驾驶边界（可编辑）
   * @return 包含速度-距离约束的可变边界集合
   */
  STDrivableBoundaries& mutable_st_drivable_boundaries() { return st_drivable_boundaries_; }

 private:
  ///< 初始化标志位（true表示已完成数据加载）
  bool init_ = false;
  ///< 后方障碍物风险标识（存在需要特殊处理的后方障碍物）
  bool has_back_obs_risk_ = false;
  ///< ST边界上的最小s值（单位：米）
  double min_s_on_st_boundaries_ = 0.0;
  ///< 规划起始点轨迹状态（包含位置、速度、加速度等信息）
  TrajectoryPt init_point_;
  ///< 巡航速度基准值（单位：m/s）
  double cruise_speed_ = 0.0;
  ///< 实际路径数据长度（单位：米）
  double path_data_length_ = 0.0;
  ///< 配置文件中指定的路径长度（单位：米）
  double path_length_by_conf_ = 0.0;
  ///< 配置文件中指定的总规划时间（单位：秒）
  double total_time_by_conf_ = 0.0;

  // new added
  ///< ST障碍物边界集合（包含时间和空间维度信息）
  std::vector<const STBoundary*> st_boundaries_;
  ///< 可行驾驶区域边界（包含上下边界的速度和距离约束）
  STDrivableBoundaries st_drivable_boundaries_;
};

}  // namespace gpal::pnc::planning
