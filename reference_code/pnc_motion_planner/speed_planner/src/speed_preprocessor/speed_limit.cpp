/**
 * @file speed_limit.cpp
 * @brief 多源速度限制综合处理器
 * @details 本类负责整合道路环境中的各类速度限制条件，生成最终速度规划边界。
 */
#include "speed_preprocessor/speed_limit.h"
#include <filesystem>
#include <iostream>

#include "math/math_utils.h"
namespace gpal::pnc::planning {

using math::TableLookUp1D;
/**
 * @brief 速度限制处理器初始化（配置加载阶段）
 * 
 * @par 功能说明:
 * 1. 获取全局配置管理器单例
 * 2. 加载速度规划相关配置参数
 * 3. 获取车辆动力学参数
 * 4. 执行配置有效性校验
 * 
 * @par 处理流程:
 * @startuml
 :获取ConfigManager单例;
 :加载SpeedPlannerConfig配置;
 :加载SpeedPreProcessorConfig配置;
 :获取车辆基础参数;
 :执行非空校验;
 @enduml
 *
 * @note 关键配置项:
 * - SpeedPlannerConfig: 速度规划核心参数
 * - SpeedPreProcessorConfig: 预处理模块参数
 * - vehicle_param_: 车辆轴距/轮距等参数
 *
 * @warning 需确保调用前:
 * - 配置文件已正确加载到配置管理器
 * - 配置项命名与实际配置文件一致
 */
void SpeedLimitProcessor::init() {
  config_manager_ = Singleton<ConfigManager>::get_instance();
  speed_planner_config_ = config_manager_->getConfig<SpeedPlannerConfig>("SpeedPlannerConfig");
  speed_preprocessor_config_ = config_manager_->getConfig<SpeedPreProcessorConfig>("SpeedPreProcessorConfig");
  vehicle_param_ = config_manager_->vehicle_config().vehicle_param();
  CHECK_NOTNULL(config_manager_);
}
/**
 * @brief 速度限制综合计算入口函数（多约束整合）
 * @param[in] local_view 局部视图数据（包含感知/定位信息）
 * @param[in] decision_result 决策结果（包含障碍物决策标签）
 * @param[in] reference_line_info 参考线信息（包含地图速度限制）
 * @param[in] path 离散化路径数据（Frenet坐标系）
 * @param[in] turn_state 转向状态标记（0-无转向 1-普通转向 2-掉头）
 * @param[in] behavior_state 行为状态（跟车/换道等）
 * @param[in] init_speed_state 初始速度状态（规划起点）
 * @param[out] speed_limit_result 输出速度限制结果
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                 | 取值范围        | 说明                     |
 * |---------------------|----------------------|----------------|--------------------------|
 * | local_view          | LocalView            | -              | 包含感知/定位数据        |
 * | decision_result      | DecisionResult        | -              | 障碍物决策标签集合       |
 * | reference_line_info  | ReferenceLineInfo*    | [0,path.size]  | 参考线信息指针           |
 * | path                 | DiscretizedPath       | -              | 离散路径点集合           |
 * | turn_state           | int                   | {0,1,2}        | 转向状态枚举值           |
 * | behavior_state       | BehaviorState         | -              | 行为状态机输出           |
 * | init_speed_state     | SpeedState            | -              | 初始速度状态             |
 * 
 * @par 处理流程:
 * @startuml|
 :初始化默认速度限制;
 partition 多源限制计算 {
   :地图限制计算(getMapRelatedSpeedlimit);
   if (转向限制使能?) then (yes)
     :转向场景限制计算(getTurnSpeedLimit);
   endif
   if (决策限制使能?) then (yes)
     :决策障碍物限制计算(getDecisionSpeedLimit);
   endif
   :曲率限制计算(getCurveSpeedLimit);
   :恒定速度限制计算(getConstSpeedLimit);
 }
 :生成路径速度曲线(calculatePathVT);
 :整合最终结果;
 @enduml
 *
 * @note 多约束整合策略:
 * 1. 各类限制按优先级覆盖（后计算的覆盖先计算的）
 * 2. 最终结果取多约束中的最小值
 * 3. 包含地图/转向/决策/曲率/恒定速度五类限制
 *
 * @warning 需确保调用前已完成:
 * - 参考线信息有效性校验
 * - 路径数据与参考线坐标系对齐
 * - 配置管理器参数正确加载
 */
void SpeedLimitProcessor::getSpeedLimit(const LocalView& local_view, const DecisionResult& decision_result,
                                        const ReferenceLineInfo* reference_line_info, const DiscretizedPath& path,
                                        const int& turn_state, const BehaviorState& behavior_state, const SpeedState& init_speed_state,
                                        SpeedLimitResult* speed_limit_result) {
  init();
  max_speed_limit_ = kMaxSpeedMS;
  init_speed_state_ = init_speed_state;

  speed_limit_.resize(path.size());
  for (int i = 0; i < path.size(); i++) {
    speed_limit_.at(i).s = path.at(i).s();
    speed_limit_.at(i).speed_limit = kMaxSpeedMS;
    speed_limit_.at(i).id = "default";
  }
  //
  getMapRelatedSpeedlimit(reference_line_info, path);
  if (speed_preprocessor_config_.enable_turn_speed_limit()) {
    getTurnSpeedLimit(local_view, reference_line_info, path, turn_state);
  }
  if (speed_preprocessor_config_.enable_decision_speed_limit()) {
    getDecisionSpeedLimit(local_view, reference_line_info, decision_result);
  }

  getCurveSpeedLimit(local_view, decision_result, path);

  //
  getConstSpeedLimit(local_view, turn_state, behavior_state);

  calculatePathVT(local_view, behavior_state, reference_line_info);
  for (int i = 0; i < speed_limit_.size(); i++) {
    curvature_speed_path_.at(i).set_v(speed_limit_.at(i).speed_limit);
  }

  speed_limit_result->clear();
  speed_limit_result->max_speed_limit_ = max_speed_limit_;
  speed_limit_result->path_v_t_ = path_v_t_;
  speed_limit_result->curvature_speed_limit_ = curvature_speed_path_;
  speed_limit_result->speed_limit_ = speed_limit_;

  caculateAccSpeedLimit(local_view);
}
/**
 * @brief 地图相关速度限制计算（道路基础限速）
 * @param[in] reference_line_info 参考线信息指针
 * @param[in] path 离散化路径数据
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | reference_line_info | const ReferenceLineInfo* | -             | 包含道路限速信息的指针   |
 * | path                | DiscretizedPath        | -              | 待处理路径点集合         |
 * 
 * @par 处理流程:
 * @startuml
 if (参考线无效?) then (yes)
   :直接返回;
 else (no)
   partition 路径点处理 {
     :获取参考线起点s坐标;
     repeat
       :处理当前路径点;
       if (地图限速使能且有效?) then (yes)
         :应用地图限速值;
       else (no)
         :使用默认最大限速;
       endif
       :更新速度限制集合;
     repeat while (存在未处理路径点?)
   }
   :更新全局最大限速;
 endif
 @enduml
 *
 * @note 核心逻辑:
 * 1. 路径点s坐标需叠加参考线起点s值
 * 2. 默认限速为kMaxSpeedKMH常量值
 * 3. 最终结果通过speed_limit_成员变量输出
 *
 * @warning 需确保调用前:
 * - 参考线已完成坐标转换
 * - 路径数据与参考线坐标系对齐
 */
void SpeedLimitProcessor::getMapRelatedSpeedlimit(const ReferenceLineInfo* reference_line_info,
                                                  const DiscretizedPath& path) {
  if (reference_line_info == nullptr || !reference_line_info->isValid()) {
    return;
  }
  SpatialSpeedLimit map_speed_limit;
  auto target_reference_line = reference_line_info->ref_line();
  auto start_pt = target_reference_line.getReferencePoint(reference_line_info->vehicle_state().x(),
                                                          reference_line_info->vehicle_state().y());
  double start_s = start_pt.local_s();
  // PINFO << ">>>>>>>>>>>>>   start_x = " << start_pt.x() << "  start_y =  " << start_pt.y() << "  s = " << start_s;
  // for (int j = 0; j < target_reference_line.reference_points().size(); j++) {
  //   PINFO << "  reference_line x = " << target_reference_line.reference_points().at(j).x()
  //         << "  y =" << target_reference_line.reference_points().at(j).y()
  //         << "  s = " << target_reference_line.reference_points().at(j).local_s()
  // }
  for (int i = 0; i < path.size(); i++) {
    speed_limit_.at(i).s = path.at(i).s();
    if (target_reference_line.GetSpeedLimitFromS(path.at(i).s() + start_s) < kMathEpsilon ||
        !speed_preprocessor_config_.enable_map_speed_limit()) {
      speed_limit_.at(i).speed_limit = kMaxSpeedKMH;
      speed_limit_.at(i).id = "default";
    } else {
      speed_limit_.at(i).speed_limit = target_reference_line.GetSpeedLimitFromS(path.at(i).s() + start_s);
      speed_limit_.at(i).id = "map";
    }

    // PINFO << "  path x = " << path[i].x() << " y = " << path[i].y() << "   s = " <<
    // path[i].s()
    //       << "  speed = " << speed_limit_.at(i).speed_limit
    //       << "   id = " << speed_limit_.at(i).id;
  }
  max_speed_limit_ = min(max_speed_limit_, speed_limit_.front().speed_limit);
}
/**
 * @brief 转向场景速度限制计算（弯道/掉头减速）
 * @param[in] local_view 局部视图数据（含定位信息）
 * @param[in] reference_line_info 参考线信息
 * @param[in] path 离散化路径数据
 * @param[in] turn_state 转向状态标记（0-无 1-普通转向 2-掉头）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | local_view          | LocalView              | -              | 车辆定位信息源           |
 * | reference_line_info | ReferenceLineInfo*     | -              | 道路参考线数据指针       |
 * | path                | DiscretizedPath        | -              | 待处理路径点集合         |
 * | turn_state          | int                    | {0,1,2}        | 转向状态枚举值           |
 * 
 * @par 处理流程:
 * @startuml
 if (参考线无效?) then (yes)
   :直接返回;
 else (no)
   :获取车辆参考点坐标;
   :查找前方100m内的停止线;
   switch (停止线类型)
   case (左转/右转停止线)
     :计算到停止线距离;
     :设置转向限速值;
   case (掉头停止线)
     :设置掉头限速值;
   endswitch
   if (未找到停止线但处于掉头状态) then (yes)
     :强制设置掉头限速值;
   endif
   :遍历路径点应用限速规则;
 @enduml
 *
 * @note 限速策略:
 * 1. 停止线前使用变加速模型计算限速
 * 2. 停止线后直接应用固定限速
 * 3. 掉头场景强制设置最低限速
 *
 * @warning 需确保调用前:
 * - 转向状态标记已正确更新
 * - 参考线与路径坐标系对齐
 */
void SpeedLimitProcessor::getTurnSpeedLimit(const LocalView& local_view, const ReferenceLineInfo* reference_line_info,
                                            const DiscretizedPath& path, const int& turn_state) {
  if (reference_line_info == nullptr || !reference_line_info->isValid()) {
    return;
  }
  auto ltm_point = local_view.getLocalizationPtr()->vehicleAlignPosePoint();
  auto ego_reference_point = reference_line_info->ref_line().getReferencePoint(ltm_point.x(), ltm_point.y());
  auto stop_lines = reference_line_info->ref_line().getStopLinesFromSRange(ego_reference_point.local_s(), 100, 40);
  double turn_speed_limit = kMaxSpeedMS;
  double stop_line_dis = kNegativeInfinity;
  if (!stop_lines.empty()) {
    auto stop_line = stop_lines.front();
    if (stop_line.direction == DrivingDirection::kDirectionLeftOnly ||
        stop_line.direction == DrivingDirection::kDirectionRightOnly) {
      stop_line_dis = stop_line.s - ego_reference_point.local_s();
      turn_speed_limit = speed_preprocessor_config_.turn_speed_limit() * KMH_MS;
    } else if (stop_line.direction == DrivingDirection::kDirectionUTurnOnly) {
      stop_line_dis = stop_line.s - ego_reference_point.local_s();
      turn_speed_limit = speed_preprocessor_config_.u_turn_speed_limit() * KMH_MS;
    }
  } else if (turn_state == 2) {  // uturn state 确保uturn不提前提速
    stop_line_dis = 0.0;
    turn_speed_limit = speed_preprocessor_config_.u_turn_speed_limit() * KMH_MS;
  } else if (turn_state == 1 && speed_preprocessor_config_.enable_turn_accelerate_limit()) {  // left turn state 确保left turn不提前提速
    stop_line_dis = 0.0;
    turn_speed_limit = speed_preprocessor_config_.turn_speed_limit() * KMH_MS;
  }else {
    return;
  }
  for (int i = 0; i < speed_limit_.size(); i++) {
    if (speed_limit_.at(i).s < stop_line_dis) {
      double acc = 1.0;
      double speed_limit = sqrt(turn_speed_limit * turn_speed_limit + 2 * acc * (stop_line_dis - speed_limit_.at(i).s));
      if (speed_limit_.at(i).speed_limit > speed_limit) {
        speed_limit_.at(i).speed_limit = speed_limit;
        speed_limit_.at(i).id = "turn";
      }
    } else if (speed_limit_.at(i).s >= stop_line_dis && speed_limit_.at(i).speed_limit > turn_speed_limit) {
      speed_limit_.at(i).speed_limit = turn_speed_limit;
      speed_limit_.at(i).id = "turn";
    }
  }
}
/**
 * @brief 决策障碍物速度限制计算（安全减速墙）
 * @param[in] local_view 局部视图数据（含定位信息）
 * @param[in] reference_line_info 参考线信息
 * @param[in] decision_result 决策结果（包含障碍物决策标签）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | local_view          | LocalView              | -              | 车辆定位与底盘数据源     |
 * | reference_line_info | ReferenceLineInfo*     | -              | 道路参考线数据指针       |
 * | decision_result      | DecisionResult         | -              | 障碍物决策信息集合       |
 * 
 * @par 处理流程:
 * @startuml
 if (参考线无效?) then (yes)
   :直接返回;
 else (no)
   :遍历纵向边界决策;
   if (发现减速墙类型?) then (yes)
     :计算决策目标点s坐标;
     :计算期望减速度;
     if (减速度不足?) then (yes)
       :跳过当前约束;
     endif
     :遍历路径点应用减速规则;
   endif
 endif
 @enduml
 *
 * @note 核心计算逻辑:
 * 1. 使用运动学公式计算允许的最大减速速度
 * 2. 应用公式：v = sqrt(v_limit^2 - 2a(s - s_i))
 * 3. 在减速墙前采用变减速模型，墙后采用固定限速
 * 
 * @warning 关键参数:
 * - 减速度下限：-2.0 m/s²
 * - 有效减速度阈值：-0.5 m/s²
 * - 速度限制覆盖策略：后处理的覆盖优先级最高
 */
void SpeedLimitProcessor::getDecisionSpeedLimit(const LocalView& local_view,
                                                const ReferenceLineInfo* reference_line_info,
                                                const DecisionResult& decision_result) {
  if (reference_line_info == nullptr || !reference_line_info->isValid()) {
    return;
  }
  // TODO
  auto& boundary_decision = decision_result.getLongitudinalBoundaryDecision();
  for (auto& boundary_constraint : boundary_decision) {
    if (boundary_constraint.type == WallType::LONG_DECC_WALL ) {
      auto ltm_point = local_view.getLocalizationPtr()->vehicleAlignPosePoint();
      auto ego_reference_point = reference_line_info->ref_line().getReferencePoint(ltm_point.x(), ltm_point.y());
      double ego_speed = local_view.getChassisPtr()->Speed();
      double decision_speed_limit = max(boundary_constraint.v, 0.0);
      double s = boundary_constraint.s - ego_reference_point.local_s();
      double desired_acc = max((decision_speed_limit * decision_speed_limit - ego_speed * ego_speed) / (2 * s), -2.0);
      if (desired_acc > -0.5) {
        continue;
      }
      for (int i = 0; i < speed_limit_.size(); i++) {
        if (speed_limit_.at(i).s < s) {
          double speed_limit =
              sqrt(decision_speed_limit * decision_speed_limit - 2 * desired_acc * (s - speed_limit_.at(i).s));
          if (speed_limit_.at(i).speed_limit > speed_limit) {
            speed_limit_.at(i).speed_limit = speed_limit;
            speed_limit_.at(i).id = "decision";
          }
        } else if (speed_limit_.at(i).s > s && speed_limit_.at(i).speed_limit > decision_speed_limit) {
          speed_limit_.at(i).speed_limit = decision_speed_limit;
          speed_limit_.at(i).id = "decison";
        }
      }
    }
  }
}
/**
 * @brief 曲率相关速度限制计算（弯道离心力约束）
 * @param[in] local_view 局部视图数据（含定位信息）
 * @param[in] decision_result 决策结果（含障碍物信息）
 * @param[in] path 离散化路径数据
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | local_view          | LocalView              | -              | 车辆定位与底盘数据源     |
 * | decision_result     | DecisionResult         | -              | 障碍物决策信息集合       |
 * | path                | DiscretizedPath        | -              | 待处理路径点集合         |
 * 
 * @par 处理流程:
 * @startuml
 :初始化大曲率计数器;
 partition 曲率分析 {
    :遍历路径点;
    if (曲率绝对值>0.1?) then (yes)
      :标记大曲率区间;
    else (no)
      :重置计数器;
    endif
 }
 partition 速度限制计算 {
   :建立曲率-横向加速度映射表;
     :遍历每个路径点
     :查表获取横向加速度限制;
     :计算曲率速度限制值;
     :扩展大曲率区间影响范围;
 }
 partition 速度曲线优化 {
   :逆向传播生成基础速度曲线;
   :前向传播考虑加速度连续性;
   :横向传播生成最终限制曲线;
 }
 @enduml
 *
 * @note 核心算法:
 * 1. 使用公式 v = sqrt(a_lat / |kappa|) 计算基础限速
 * 2. 通过前后向传播确保加速度连续性
 * 3. 大曲率区域(连续10个点)扩展2.77米影响范围
 *
 * @warning 关键配置项:
 * - 横向加速度限制表(lat_a_limit)
 * - 大曲率阈值(0.1 rad/m)
 * - 曲率扩展范围(2.77米)
 */
void SpeedLimitProcessor::getCurveSpeedLimit(const LocalView& local_view, const DecisionResult& decision_result,
                                             const DiscretizedPath& path) {
  int very_large_kappa_counter = 0;
  size_t path_size = path.size();
  size_t start_large_kappa_index = 0;
  size_t end_large_kappa_index = path_size - 1;
  double init_v_ = init_speed_state_.v;
  if (1) {
    for (size_t i = 0; i < path.size(); i++) {
      if (fabs(path.at(i).kappa()) > 0.1 && very_large_kappa_counter == 0) {
        very_large_kappa_counter = 1;
        start_large_kappa_index = i;
      } else if (fabs(path.at(i).kappa()) > 0.1 && very_large_kappa_counter > 0) {
        very_large_kappa_counter++;
      } else {
        if (very_large_kappa_counter >= 10) {
          end_large_kappa_index = std::max(i - 1, static_cast<size_t>(0));
          break;
        } else {
          very_large_kappa_counter = 0;
        }
      }
    }
  }
  std::vector<double> kappa_table = {0.001, 0.002, 0.004, 0.01, 0.02, 0.05, 0.1};
  std::vector<double> lat_a_limit = {speed_preprocessor_config_.lateral_acc_limit_for_radius_1000(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_500(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_250(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_100(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_50(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_20(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_10()};
  // std::vector<double> lat_a_limit = {1.2, 0.7, 0.55, 0.5, 0.45, 0.4, 0.68, 0.65};

  // if (decision_result.getCurrFsmState() == FsmState::LEFT_CHANGE ||
  //     decision_result.getCurrFsmState() == FsmState::RIGHT_CHANGE) {
  //   lat_a_limit = {1.2, 1.1, 0.9, 0.85, 0.75, 0.68, 0.65};
  // }
  // std::vector<double> kappa_speed_limit_config = {
  //     parameter_.max_speed_limit_for_radius_1000(), parameter_.max_speed_limit_for_radius_500(),
  //     parameter_.max_speed_limit_for_radius_250(),  parameter_.max_speed_limit_for_radius_100(),
  //     parameter_.max_speed_limit_for_radius_50(),   parameter_.max_speed_limit_for_radius_25(),
  //     parameter_.max_speed_limit_for_radius_10()};

  const std::vector<double> delta_speed_table_for_min_virtual_decel = {0.0, 2.0, 3.0, 10.0, 20.0, 30.0};
  std::vector<double> min_virtual_decel_table = {0.2, 0.3, 0.4, 1.0, 2.0, 2.0};

  double min_speedlimit_for_turn = speed_preprocessor_config_.turn_speed_limit() * KMH_MS;
  if (very_large_kappa_counter >= 10) {
    min_speedlimit_for_turn = speed_preprocessor_config_.turn_speed_limit() * KMH_MS;
  }
  const size_t curve_expande_range = static_cast<size_t>(2.77);
  proto::TrajectoryPoint Tp;
  vector<proto::TrajectoryPoint> curvature_speed_path;
  curvature_speed_path.resize(path.size());
  Tp.mutable_path_point()->set_x(path.back().x());
  Tp.mutable_path_point()->set_y(path.back().y());
  Tp.mutable_path_point()->set_z(path.back().z());
  Tp.mutable_path_point()->set_kappa(path.back().kappa());
  Tp.mutable_path_point()->set_s(path.back().s());
  double kappa = Tp.path_point().kappa();
  if (abs(kappa) < 1E-04 || std::isnan(kappa)) {
    kappa = 1E-04;
  }
  double kappa_lat_a_limit = TableLookUp1D(kappa_table, lat_a_limit, std::fabs(kappa));
  double kappa_speed_limit = std::sqrt(std::fabs(kappa_lat_a_limit / kappa));
  // double config_max_speed_limit = TableLookUp1D(kappa_table, kappa_speed_limit_config, std::fabs(kappa)) / 3.6;
  // kappa_speed_limit = std::fmin(kappa_speed_limit, config_max_speed_limit);
  kappa_speed_limit = fmax(kappa_speed_limit, min_speedlimit_for_turn);
  Tp.set_v(kappa_speed_limit);
  Tp.set_relative_time(0);
  curvature_speed_path.at(path.size() - 1).CopyFrom(Tp);
  // We calculate the kappa-speed-limit and the virtual-decel of every path-point, for our later use
  std::vector<double> kappa_speed_limits;
  std::vector<double> virtual_decelerations;
  for (size_t i = 0UL; i < path.size(); i++) {
    double kappa = path.at(i).kappa();
    if (abs(kappa) < 1E-4 || std::isnan(kappa)) {
      kappa = 1E-4;
    }
    kappa_lat_a_limit = TableLookUp1D(kappa_table, lat_a_limit, std::fabs(kappa));
    double kappa_speed_limit = std::sqrt(std::fabs(kappa_lat_a_limit / kappa));
    if (very_large_kappa_counter > 10) {
      if (fabs(kappa) > 0.02 && (i >= (start_large_kappa_index > curve_expande_range ? start_large_kappa_index -curve_expande_range : static_cast<size_t>(0)) &&
                                 i <= std::min(end_large_kappa_index + curve_expande_range, path_size - 1))) {
        kappa_lat_a_limit = TableLookUp1D(kappa_table, lat_a_limit, std::fabs(0.1));
        kappa_speed_limit = std::sqrt(std::fabs(kappa_lat_a_limit / 0.1));
      }
    }
    // ERT_PLOG_I << "  i = " << i << "   kappa = " << kappa << "  speed_limit = " << kappa_speed_limit * MS_KMH ;
    kappa_speed_limit = kappa_speed_limit * 1.0;  // parameter_.kappa_speed_limit_factor();
    kappa_speed_limit = fmax(kappa_speed_limit, min_speedlimit_for_turn);
    kappa_speed_limits.emplace_back(kappa_speed_limit);
    double delta_v = init_v_ - kappa_speed_limit;
    double s = path.at(i).s();
    double virtual_decel =
        (i == 0UL) ? 0.0F : 0.5F * (-kappa_speed_limit * kappa_speed_limit + init_v_ * init_v_) / fmax(s, 0.1F);
    double min_virtual_decel = TableLookUp1D(delta_speed_table_for_min_virtual_decel, min_virtual_decel_table, delta_v);
    virtual_decel = fmax(virtual_decel, min_virtual_decel);
    virtual_decelerations.emplace_back(virtual_decel);
  }
  // Backward speed-limits consideration
  for (int i = path.size() - 2; i >= 0; i--) {
    Tp.mutable_path_point()->set_x(path.at(i).x());
    Tp.mutable_path_point()->set_y(path.at(i).y());
    Tp.mutable_path_point()->set_z(path.at(i).z());
    double kappa = path.at(i).kappa();
    if (abs(kappa) < 1E-4 || std::isnan(kappa)) {
      kappa = 1E-4;
    }
    Tp.mutable_path_point()->set_kappa(kappa);
    Tp.mutable_path_point()->set_s(path.at(i).s());
    kappa_lat_a_limit = TableLookUp1D(kappa_table, lat_a_limit, std::fabs(static_cast<double>(Tp.path_point().kappa())));
    double kappa_speed_limit = std::sqrt(std::fabs(kappa_lat_a_limit / Tp.path_point().kappa()));
    if (very_large_kappa_counter > 10) {
      if (fabs(kappa) > 0.02 && (i >= (start_large_kappa_index > curve_expande_range ? start_large_kappa_index -curve_expande_range : static_cast<size_t>(0)) &&
                                 i <= std::min(end_large_kappa_index + curve_expande_range, path_size - 1))) {
        kappa_lat_a_limit = TableLookUp1D(kappa_table, lat_a_limit, std::fabs(0.1));
        kappa_speed_limit = std::sqrt(std::fabs(kappa_lat_a_limit / 0.1));
      }
    }
    kappa_speed_limit = kappa_speed_limit * 1.0;  // parameter_.kappa_speed_limit_factor();
    kappa_speed_limit = fmax(kappa_speed_limit, min_speedlimit_for_turn);
    float acc_speed_limit = sqrt(abs(2 * /*parameter_.acc_soft()*/ 1.0 *
                                         (curvature_speed_path.at(i + 1).path_point().s() - Tp.path_point().s()) +
                                     curvature_speed_path.at(i + 1).v() * curvature_speed_path.at(i + 1).v()));
    Tp.set_v(fmin(kappa_speed_limit, acc_speed_limit));
    curvature_speed_path.at(i).CopyFrom(Tp);
  }

  // A Forward speed-limits consideration for a s-horizon
  // double s_horizon = fmax(time_horizon_ * 8.0F, 100.0F);
  for (size_t i = 0UL; i < curvature_speed_path.size(); i++) {
    double s_i = curvature_speed_path.at(i).path_point().s();
    // if (s_i <= s_horizon) {
    for (size_t j = i + 1UL; j < curvature_speed_path.size(); j++) {
      double s_j = curvature_speed_path.at(j).path_point().s();
      // if (s_j <= s_horizon) {
      double kappa_speed_limit_j = kappa_speed_limits.at(j);
      double virtual_decel_j = virtual_decelerations.at(j);
      double kappa_speed_limit_from_j_to_i =
          sqrt(fmax(kappa_speed_limit_j * kappa_speed_limit_j + 2.0F * virtual_decel_j * (s_j - s_i), 0.01F));
      curvature_speed_path.at(i).set_v(fmin(curvature_speed_path.at(i).v(), kappa_speed_limit_from_j_to_i));
      // } else {
      //   break;
      // }
    }
    // } else {
    //   break;
    // }
  }
  curvature_speed_path_ = curvature_speed_path;
  for (int i = 0; i < speed_limit_.size(); i++) {
    if (speed_limit_.at(i).speed_limit > curvature_speed_path.at(i).v()) {
      speed_limit_.at(i).speed_limit = curvature_speed_path.at(i).v();
      speed_limit_.at(i).id = "curve";
    }
  }
}

// const speed limit
/**
 * @brief 恒定速度限制整合（多源最小值选取）
 * @param[in] local_view 局部视图数据（含定位信息）
 * @param[in] turn_state 转向状态标记（0-无 1-普通转向 2-掉头）
 * @param[in] behavior_state 行为状态（含停车/ACC等状态）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | local_view          | LocalView              | -              | 车辆定位与底盘数据源     |
 * | turn_state          | int                    | {0,1,2}        | 转向状态枚举值           |
 * | behavior_state      | BehaviorState          | -              | 行为状态机输出           |
 * 
 * @par 处理流程:
 * @startuml
 :获取基础驾驶限速;
 if (ACC模式使能?) then (yes)
   :获取ACC模式限速;
 endif
 if (停车场景?) then (yes)
   :获取停车限速;
 endif
 if (掉头场景?) then (yes)
   :获取掉头限速;
 endif
 :选择多源最小值;
 :更新全局最大限速;
 @enduml
 *
 * @note 限速源优先级:
 * 1. 基础驾驶限速（HMI配置）
 * 2. ACC模式限速（跟车需求）
 * 3. 停车场景限速（搜索/执行车位）
 * 4. 掉头场景限速（特殊工况）
 *
 * @warning 关键参数:
 * - 停车限速：lstp_speed_limit/parking_speed_limit 配置项
 * - 掉头限速：u_turn_speed_limit 配置项
 * - 单位转换：KMH_MS 常量
 */
void SpeedLimitProcessor::getConstSpeedLimit(const LocalView& local_view, const int& turn_state,
                                             const BehaviorState& behavior_state) {
  pair<string, double> min_const_speed_limit = make_pair("default", static_cast<double>(kMaxSpeedMS));
  auto driving_speed_limit = getDrivingSpeedLimit(local_view);
  if (min_const_speed_limit.second > driving_speed_limit.second) {
    min_const_speed_limit.first = driving_speed_limit.first;
    min_const_speed_limit.second = driving_speed_limit.second;
  }

  if (behavior_state.is_acc_state_ || behavior_state.is_lcc_state_) { // ACC/LCC模式共用限速
    auto acc_driving_speed_limit = getAccDrivingSpeedLimit(local_view);
    if (min_const_speed_limit.second > acc_driving_speed_limit.second) {
      min_const_speed_limit.first = acc_driving_speed_limit.first;
      min_const_speed_limit.second = acc_driving_speed_limit.second;
    }
  }

  auto parking_speed_limit = getParkingSpeedLimit(local_view, behavior_state);
  if (min_const_speed_limit.second > parking_speed_limit.second) {
    min_const_speed_limit.first = parking_speed_limit.first;
    min_const_speed_limit.second = parking_speed_limit.second;
  }
  auto turn_speed_limit = getTurnScenarioSpeedLimit(local_view, turn_state);
  if (min_const_speed_limit.second > turn_speed_limit.second) {
    min_const_speed_limit.first = turn_speed_limit.first;
    min_const_speed_limit.second = turn_speed_limit.second;
  }
  max_speed_limit_ = Min(max_speed_limit_, min_const_speed_limit.second);
  updateSpeedLimit(min_const_speed_limit.second, 0.0, min_const_speed_limit.first);
}
/**
 * @brief 基础驾驶限速获取（配置+HMI）
 * @param[in] local_view 局部视图数据（含HMI输入）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | local_view          | LocalView              | -              | 含HMI人机交互限速数据    |
 * 
 * @par 处理流程:
 * @startuml
 :从配置读取最大速度限制;
 :从HMI获取人工设置限速;
 :取两者较小值作为基准限速;
 :与全局最大限速二次比较;
 :更新全局最大限速值;
 @enduml
 *
 * @note 优先级策略:
 * 1. 配置限速：speed_planner_config_.max_speed_limit()
 * 2. HMI限速：local_view.getHmiPtr()->hmiSpeedLimit()
 * 3. 全局限速：max_speed_limit_
 *
 * @warning 关键参数转换:
 * - 配置限速单位：KMH→MS（通过KMH_MS常量转换）
 * - 输出限速单位：m/s
 */
pair<string, double> SpeedLimitProcessor::getDrivingSpeedLimit(const LocalView& local_view) {
  double speed_limit = kMaxSpeedMS;
  speed_limit = Min(speed_planner_config_.max_speed_limit() * KMH_MS, local_view.getConsolePtr()->speedLimit());
  ERT_PLOG_D << "  speed_limit = " << speed_limit ;
  speed_limit = Min(speed_limit, max_speed_limit_);
  max_speed_limit_ = speed_limit;

  return make_pair("driving", speed_limit);
}
/**
 * @brief ACC模式速度限制获取（跟车需求）
 * @param[in] local_view 局部视图数据（含控制台输入）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | local_view          | LocalView              | -              | 含ACC期望速度的底盘数据  |
 * 
 * @par 处理流程:
 * @startuml
 :从配置读取最大限速;
 :获取ACC期望速度;
 :取两者较小值作为基准限速;
 :与全局最大限速二次比较;
 :更新全局最大限速值;
 @enduml
 *
 * @note 优先级策略:
 * 1. 配置限速：speed_planner_config_.max_speed_limit()
 * 2. ACC期望速度：local_view.getConsolePtr()->accDesiredSpeed()
 * 3. 全局限速：max_speed_limit_
 *
 * @warning 关键特性:
 * - 单位双重转换：配置限速KMH→MS，ACC期望速度保持原始单位
 * - 输出限速单位：m/s
 * - 双重保险机制：两次最小值比较确保安全限速
 */
pair<string, double> SpeedLimitProcessor::getAccDrivingSpeedLimit(const LocalView& local_view) {
  double speed_limit = kMaxSpeedMS;
  speed_limit = Min(speed_planner_config_.max_speed_limit() * KMH_MS, local_view.getConsolePtr()->accDesiredSpeed());
  ERT_PLOG_D << " acc speed_limit = " << speed_limit ;
  speed_limit = Min(speed_limit, max_speed_limit_);
  max_speed_limit_ = speed_limit;

  return make_pair("acc_driving", speed_limit);
}
/**
 * @brief 停车场景速度限制获取（搜索/执行）
 * @param[in] local_view 局部视图数据（含定位信息）
 * @param[in] behavior_state 行为状态（含停车状态标记）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | local_view          | LocalView              | -              | 车辆定位数据源           |
 * | behavior_state      | BehaviorState          | -              | 停车行为状态标记         |
 * 
 * @par 处理流程:
 * @startuml
 :初始化最大限速为系统上限;
 if (车位搜索状态?) then (yes)
   :应用车位搜索限速;
 elseif (泊入执行状态?) then (yes)
   :应用泊入执行限速;
 endif
 @enduml
 *
 * @note 限速策略:
 * 1. 搜索车位状态：lstp_speed_limit 配置项
 * 2. 泊入执行状态：parking_speed_limit 配置项
 * 3. 取两者较小值作为最终限速
 *
 * @warning 关键参数:
 * - 搜索限速：lstp_speed_limit 配置项（典型值15km/h→4.17m/s）
 * - 执行限速：parking_speed_limit 配置项（典型值5km/h→1.39m/s）
 * - 单位转换：KMH_MS 常量
 */
pair<string, double> SpeedLimitProcessor::getParkingSpeedLimit(const LocalView& local_view,
                                                               const BehaviorState& behavior_state) {
  double speed_limit = kMaxSpeedMS;
  if (behavior_state.search_parklot_state_) {
    speed_limit = Min(speed_limit, speed_preprocessor_config_.lstp_speed_limit() * KMH_MS);
  } else if (behavior_state.park_in_state_) {
    speed_limit = Min(speed_limit, speed_preprocessor_config_.parking_speed_limit() * KMH_MS);
  }
  return make_pair("parking", speed_limit);
}
/**
 * @brief 掉头场景速度限制获取（特殊工况）
 * @param[in] local_view 局部视图数据（含底盘信息）
 * @param[in] turn_state 转向状态标记（0-无 1-普通转向 2-掉头）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | local_view          | LocalView              | -              | 含车辆实时速度的底盘数据 |
 * | turn_state          | int                    | {0,1,2}        | 转向状态枚举值           |
 * 
 * @par 处理流程:
 * @startuml
 if (掉头状态?) then (yes)
   if (当前速度 < 掉头限速+0.3m/s?) then (yes)
     :应用掉头限速值;
   endif
 endif
 @enduml
 *
 * @note 安全策略:
 * 1. 速度缓冲机制：当前速度低于限速+0.3m/s时激活限制
 * 2. 单位双重转换：配置项KMH→MS，输出单位m/s
 * 3. 状态优先级：仅响应掉头状态(turn_state=2)
 *
 * @warning 关键参数:
 * - 掉头限速：u_turn_speed_limit 配置项（典型值10km/h→2.78m/s）
 * - 速度缓冲阈值：0.3m/s（避免频繁切换限速状态）
 */
pair<string, double> SpeedLimitProcessor::getTurnScenarioSpeedLimit(const LocalView& local_view,
                                                                    const int& turn_state) {
  double speed_limit = kMaxSpeedMS;
  if (turn_state == 2) {  // turn around
    // 逐渐减速,添加停止线目标，保证目标能先停下来
    if (local_view.getChassisPtr()->Speed() < speed_preprocessor_config_.u_turn_speed_limit() * KMH_MS + 0.3) {
      speed_limit = Min(speed_preprocessor_config_.u_turn_speed_limit() * KMH_MS, speed_limit);
    }
  }
  return make_pair("uturn", speed_limit);
}
/**
 * @brief 速度限制更新（多段限速应用）
 * @param[in] speed_limit 新的速度限制值（单位：m/s）
 * @param[in] speed_limit_dis 限速生效起始距离（单位：米）
 * @param[in] speed_limit_type 限速来源标识（字符串类型）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | speed_limit         | double                 | >0             | 生效的新速度限制值       |
 * | speed_limit_dis     | double                 | >=0            | 限速生效的路径起点s坐标  |
 * | speed_limit_type    | string                 | -              | 限速源标识（如"curve"）  |
 * 
 * @par 处理流程:
 * @startuml
 :遍历所有路径点;
 if (当前点s坐标 >= 生效起始距离
 and 当前限速 > 新限速值) then (yes)
   :更新当前点限速值;
   :更新限速来源标识;
 endif
 @enduml
 *
 * @note 核心规则:
 * 1. 仅更新满足条件（s >= speed_limit_dis）的路径点
 * 2. 采用覆盖式更新策略（仅当新限速更低时生效）
 * 3. 更新范围包含起始点及其后续所有路径点
 *
 * @warning 关键特性:
 * - 单位一致性：输入参数均为国际单位（米、米/秒）
 * - 生效范围：从指定距离点开始向后覆盖
 * - 覆盖策略：后处理的限速覆盖优先级最高
 */
void SpeedLimitProcessor::updateSpeedLimit(double speed_limit, double speed_limit_dis, string speed_limit_type) {
  for (int i = 0; i < speed_limit_.size(); i++) {
    if (speed_limit_.at(i).s >= speed_limit_dis && speed_limit_.at(i).speed_limit > speed_limit) {
      speed_limit_.at(i).speed_limit = speed_limit;
      speed_limit_.at(i).id = speed_limit_type;
    }
  }
}
/**
 * @brief 路径速度曲线生成（多约束融合）
 * @param[in] local_view 局部视图数据（含定位/底盘信息）
 * @param[in] behavior_state 行为状态（含停车状态标记）
 * @param[in] reference_line_info 参考线信息指针
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | local_view          | LocalView              | -              | 含车辆实时状态数据源     |
 * | behavior_state      | BehaviorState          | -              | 停车行为状态标记         |
 * | reference_line_info| ReferenceLineInfo*     | -              | 道路参考线数据指针       |
 * 
 * @par 处理流程:
 * @startuml
 :初始化加速度系数和曲率参数;
 :构建基础速度曲线path_v_t_;
 if (非停车状态?) then (yes)
   if (初始速度>首点限速?) then (yes)
     :前向传播虚拟加速度模型;
   else (no)
     :逆向传播加速度连续性模型;
   endif
   if (存在有效参考线?) then (yes)
     :检测弯道曲率特征;
     if (转向角超阈值?) then (yes)
       :应用转向加速度限制;
     endif
   endif
 endif
 :生成最终速度限制曲线;
 @enduml
 *
 * @note 核心算法:
 * 1. 虚拟加速度模型：根据曲率/地图限速动态调整加速度
 * 2. 转向加速度限制：基于转向角度和车速的查表法
 * 3. 速度传播策略：前向传播保证加速度连续性
 *
 * @warning 关键参数:
 * - 曲率加速度下限：a_lower_for_curvature_speed_limit(-4.0 m/s²)
 * - 地图加速度下限：a_lower_for_map_related_speed_limit(-2.0 m/s²)
 * - 转向角阈值：steer_acc_limit_threshold 配置项
 */
void SpeedLimitProcessor::calculatePathVT(const LocalView& local_view, const BehaviorState& behavior_state, const ReferenceLineInfo* reference_line_info) {
  // Generating a VSoftUpperBound-Curve starting at init_v_
  double init_v_ = init_speed_state_.v;
  double a_lower_for_curvature_speed_limit = -4.0F;
  double a_lower_for_map_related_speed_limit = -2.0F;
  double acc_coeff_for_curvature_speed_limit = 0.2F;
  double decel_coeff_for_curvature_speed_limit = 0.6F;
  double acc_coeff_for_map_related_speed_limit = 1.0F;

  vector<double> overspeed_percent_table = {-0.01, 0.0, 0.1, 0.2, 0.3};
  vector<double> decel_coeff_for_map_table = {1.0, 0.3, 0.5, 0.7, 1.0};
  vector<double> max_virtual_acc_for_map_table = {-1.0F, -0.5F, -0.8F, -1.0F, -1.5F};

  // The acceleration-maximium for the virtual deceleration
  std::vector<double> delta_v_table_for_virtual_acc_max = {0.0001F, 0.5F, 1.0F};
  std::vector<double> virtual_acc_max_table = {-1.5F, -1.5F, -2.0F};
  path_v_t_ = curvature_speed_path_;
  for (size_t i = 0UL; i < path_v_t_.size(); i++) {
    if (speed_limit_.at(i).s > 150) {
      break;
    }
    path_v_t_.at(i).set_v(speed_limit_.at(i).speed_limit);
    // ERT_PLOG_I << " s = " << speed_limit_.at(i).s << "   speed_limit = " << speed_limit_.at(i).speed_limit
    // << "   id = " << speed_limit_.at(i).id << "  S =  " << path_v_t_.at(i).path_point().s() ;
  }
  path_v_t_.at(0UL).set_relative_time(0.0F);
  // Consider init_v_
  if ( !behavior_state.park_in_state_ ) {
    if (path_v_t_.at(0UL).v() < init_v_) {
      double v_soft_upper = init_v_;
      path_v_t_.at(0UL).set_v(v_soft_upper);
      for (size_t i = 1UL; i < path_v_t_.size(); i++) {
        bool over_curvature_speed_limit = (v_soft_upper > curvature_speed_path_.at(i).v());
        double delta_v = v_soft_upper - path_v_t_.at(i).v();
        double virtual_acc_max = TableLookUp1D(delta_v_table_for_virtual_acc_max, virtual_acc_max_table, delta_v);
        if(speed_limit_.at(i).id == "map" || speed_limit_.at(i).id == "driving" || speed_limit_.at(i).id == "acc_driving"){
          double overspeed_percent = (v_soft_upper - speed_limit_.at(i).speed_limit) / (speed_limit_.at(i).speed_limit);
          acc_coeff_for_map_related_speed_limit = TableLookUp1D(overspeed_percent_table, decel_coeff_for_map_table, overspeed_percent);
          virtual_acc_max = TableLookUp1D(overspeed_percent_table, max_virtual_acc_for_map_table, overspeed_percent);
        }
        double virtual_acc;
        // Calculate virtual_acc:
        if (over_curvature_speed_limit) {
          double virtual_acc_for_curvature_speed_limit =
              -decel_coeff_for_curvature_speed_limit * (v_soft_upper - curvature_speed_path_.at(i).v());
          virtual_acc_for_curvature_speed_limit =
              fmax(a_lower_for_curvature_speed_limit, virtual_acc_for_curvature_speed_limit);
          double virtual_acc_for_map_related_speed_limit =
              -acc_coeff_for_map_related_speed_limit * (v_soft_upper - speed_limit_.at(i).speed_limit);
          virtual_acc_for_map_related_speed_limit =
              fmax(virtual_acc_for_map_related_speed_limit, a_lower_for_map_related_speed_limit);
          virtual_acc = fmin(virtual_acc_for_curvature_speed_limit, virtual_acc_for_map_related_speed_limit);
        } else {
          double virtual_acc_for_map_related_speed_limit =
              -acc_coeff_for_map_related_speed_limit * (v_soft_upper - speed_limit_.at(i).speed_limit);
          virtual_acc_for_map_related_speed_limit =
              fmax(virtual_acc_for_map_related_speed_limit, a_lower_for_map_related_speed_limit);
          virtual_acc = virtual_acc_for_map_related_speed_limit;
        }
        if (delta_v >= 0.0001F) {
          virtual_acc = fmax(virtual_acc, virtual_acc_max);
          virtual_acc = fmin(virtual_acc, speed_preprocessor_config_.overspeed_min_acc_limit_threshold());
        }
        // Calculate the next v_soft_upper:
        double ds = path_v_t_.at(i).path_point().s() - path_v_t_.at(i - 1).path_point().s();
        v_soft_upper = sqrt(fmax(v_soft_upper * v_soft_upper + 2.0F * virtual_acc * ds, 0.01F));
        if (v_soft_upper > path_v_t_.at(i).v()) {
          path_v_t_.at(i).set_v(v_soft_upper);
        } else {
          break;
        }
      }
    } else {
      double v_soft_upper = init_v_;
      path_v_t_.at(0UL).set_v(v_soft_upper);
      double acc_coeff_fac = 1.0;
      for (size_t i = 1UL; i < path_v_t_.size(); i++) {
        double acc_coeff = (path_v_t_.at(i).v() < curvature_speed_path_.at(i).v() - 0.3F)
                               ? acc_coeff_for_map_related_speed_limit * acc_coeff_fac
                               : acc_coeff_for_curvature_speed_limit * acc_coeff_fac;
        double virtual_acc = -acc_coeff * (v_soft_upper - path_v_t_.at(i).v());
        virtual_acc = fmin(virtual_acc, 1.0);
        double ds = path_v_t_.at(i).path_point().s() - path_v_t_.at(i - 1).path_point().s();
        v_soft_upper = sqrt(fmax(v_soft_upper * v_soft_upper + 2.0F * virtual_acc * ds, 0.01F));
        if (v_soft_upper < path_v_t_.at(i).v()) {
          path_v_t_.at(i).set_v(v_soft_upper);
        } else {
          break;
        }
      }
    }
  }

  if (reference_line_info != nullptr &&
      reference_line_info->isValid() /*parameter_.use_front_and_rear_steer_acc_limit()*/) {
    bool out_curve_acc_limit = false;
    const double loc_x = local_view.getLocalizationPtr()->vehicleAlignPosePoint().x();
    const double loc_y = local_view.getLocalizationPtr()->vehicleAlignPosePoint().y();
    const double cur_s = reference_line_info->ref_line().getReferencePoint(loc_x, loc_y).local_s();
    const double max_kappa = reference_line_info->ref_line().getMaxCurvatureInRange(
        std::fmax(cur_s - speed_preprocessor_config_.out_curve_consider_back_distance(), 0.0), cur_s);
    if (std::fabs(max_kappa) >= speed_preprocessor_config_.out_curve_consider_curve_threshold()) {
      out_curve_acc_limit = true;
    }

    // need acc limit for out curve road
    if (out_curve_acc_limit) {
      const double steer_angle = local_view.getChassisPtr()->SteeringAngle();
      const double steer_threshold_for_acc = speed_preprocessor_config_.steer_acc_limit_threshold();
      if (std::fabs(steer_angle) > steer_threshold_for_acc || last_steer_curve_acc_limit_) {
        std::vector<double> rear_acc_table = {1.5, 1.0, 0.8, 0.5, 0.3, 0.0};
        std::vector<double> ego_speed_table_kmh = {1.0, 5.0, 10.0, 15.0, 18.0, 25.0};
        const double ego_speed = local_view.getChassisPtr()->Speed();
        const double acc_coeff_for_steer = TableLookUp1D(ego_speed_table_kmh, rear_acc_table, ego_speed * MS_KMH);
        double v_soft_upper = path_v_t_.at(0UL).v();
        // ERT_PLOG_I<<"steer_acc_limit_threshold  = "<< steer_angle
        //  << "   ego_speed = " << ego_speed << "   acc_coeff_for_steer = " << acc_coeff_for_steer ;
        for (size_t i = 1UL; i < path_v_t_.size(); i++) {
          double ds = path_v_t_.at(i).path_point().s() - path_v_t_.at(i - 1).path_point().s();
          v_soft_upper = sqrt(fmax(v_soft_upper * v_soft_upper + 2.0F * acc_coeff_for_steer * ds, 0.01F));
          if (v_soft_upper < path_v_t_.at(i).v()) {
            path_v_t_.at(i).set_v(v_soft_upper);
          }
          //  else {
          //   continue;
          // }
        }
      }

      if (std::fabs(steer_angle) > steer_threshold_for_acc) {
        last_steer_curve_acc_limit_ = true;
      }
      const double steer_threshold_for_release_acc_limit =
          speed_preprocessor_config_.release_steer_acc_limit_threshold();
      bool need_steer_release_acc = std::fabs(steer_angle) <= steer_threshold_for_release_acc_limit;
      if (need_steer_release_acc) {
        last_steer_curve_acc_limit_ = false;
      }
    } else {
      last_steer_curve_acc_limit_ = false;
    }
  }
  speed_limit_.at(0).speed_limit = max(path_v_t_.at(0).v(),1.0f);
  for (size_t i = 1; i < path_v_t_.size(); i++) {
    path_v_t_.at(i).set_relative_time(
        path_v_t_.at(i - 1).relative_time() +
        abs(2 * (path_v_t_.at(i).path_point().s() - path_v_t_.at(i - 1).path_point().s()) /
            (path_v_t_.at(i).v() + path_v_t_.at(i - 1).v())));
    speed_limit_.at(i).speed_limit =  max(path_v_t_.at(i).v(),1.0f);
    // ERT_PLOG_I << ">>>>>>>>>>>>>>>>>>>> s = " << speed_limit_.at(i).s << "   speed_limit = " << path_v_t_.at(i).v()
    // << "   id = " << speed_limit_.at(i).id << "  S =  " << path_v_t_.at(i).path_point().s() ;
  }
}
/**
 * @brief 加速度相关速度限制计算（转向动力学模型）
 * @param[in] local_view 局部视图数据（含底盘信息）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型                   | 取值范围        | 说明                     |
 * |---------------------|------------------------|----------------|--------------------------|
 * | local_view          | LocalView              | -              | 含转向角/横摆率的底盘数据|
 * 
 * @par 处理流程:
 * @startuml
 :执行转向角/横摆率滤波处理;
 :查表获取转向角权重系数;
 :预测未来3秒曲率变化;
 partition 曲率预测 {
     :计算时变转向角/横摆率;
     :融合转向与横摆率曲率;
     :查表获取横向加速度限制;
     :计算曲率速度限制;
 }
 :生成时间-速度限制表;
 @enduml
 *
 * @note 核心算法:
 * 1. 双源曲率融合：转向角与横摆率加权融合（权重查表获取）
 * 2. 时变预测模型：考虑转向变化率预测未来曲率
 * 3. 横向加速度限制：基于曲率的查表法获取限值
 *
 * @warning 关键参数:
 * - 预测时间窗：3秒
 * - 转向变化率系数：k_steering=0.5
 * - 横摆率变化率系数：k_yaw_rate=0.5
 * - 横向加速度限制表：lateral_acc_limit_for_radius_xxx配置项
 */
void SpeedLimitProcessor::caculateAccSpeedLimit(const LocalView& local_view) {
  double fliter_steering_angle = local_view.getChassisPtr()->SteeringAngle();
  double steering_angle_change_rate = 0.0;
  double fliter_yaw_rate = local_view.getChassisPtr()->YawRate();
  double yaw_rate_change_rate = 0.0;
  steeringAngleAndYawRateFiliter(local_view.getChassisPtr()->SteeringAngle(), local_view.getChassisPtr()->YawRate(),
                                 fliter_steering_angle, fliter_yaw_rate, steering_angle_change_rate,
                                 yaw_rate_change_rate);
  // 1.查表获取weight
  std::vector<double> ego_speed_table = {0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0};  // km/h
  std::vector<double> steer_angle_weight = {0.9, 0.9, 0.8, 0.8, 0.7, 0.6, 0.5};
  double weight = math::TableLookUp1D(ego_speed_table, steer_angle_weight, local_view.getChassisPtr()->Speed()* MS_KMH);
  //2. 模型获取weigt
  // double threshold_speed = 20.0;  // m/s
  // double steepness = 5.0;
  // double weight = 1.0 / (1.0 + std::exp(-steepness * (local_view.getChassisPtr()->Speed() - threshold_speed)));
  double current_kappa = weight * calculateKappaBasedSteeringAngle(fliter_steering_angle) + (1 - weight) * calcaulateKappaBasedYawRate(fliter_yaw_rate,local_view.getChassisPtr()->Speed());
  // ERT_PLOG_I<<" ACCSPEED kappa_angle = "<<std::tan(fliter_steering_angle / 17.0)<<"  wheel_base = " <<"    kappa = "<<calculateKappaBasedSteeringAngle(fliter_steering_angle)<<"  kappa_yaw_rate = "<<calcaulateKappaBasedYawRate(fliter_yaw_rate,local_view.getChassisPtr()->Speed());
  std::vector<std::pair<double, double>> t_speed_limit;
  double predict_time = 3.0;
  double k_steering = 0.5, k_yaw_rate = 0.5;
  for (double t = 0; t <= predict_time; t += 0.1) {
    double steering_angle = fliter_steering_angle + t * steering_angle_change_rate * k_steering;
    double yaw_rate = fliter_yaw_rate + t * yaw_rate_change_rate * k_yaw_rate;
    double kappa = weight * calculateKappaBasedSteeringAngle(steering_angle) + (1 - weight) * calcaulateKappaBasedYawRate(yaw_rate,local_view.getChassisPtr()->Speed());
    std::vector<double> kappa_table = {0.001, 0.002, 0.004, 0.01, 0.02, 0.05, 0.1};
    std::vector<double> lat_a_limit = {speed_preprocessor_config_.lateral_acc_limit_for_radius_1000(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_500(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_250(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_100(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_50(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_20(),
                                     speed_preprocessor_config_.lateral_acc_limit_for_radius_10()};    
    kappa = max(1E-4, abs(kappa));
    double kappa_lat_a_limit = math::TableLookUp1D(kappa_table, lat_a_limit,kappa);
    double kappa_speed_limit = std::sqrt(std::fabs(kappa_lat_a_limit / kappa));
    t_speed_limit.push_back(std::make_pair(t, kappa_speed_limit));
  }

  ERT_PLOG_D<<" ACCSPEED : "<< " steering_angle = "<<local_view.getChassisPtr()->SteeringAngle()<<"  yaw_rate = "<<local_view.getChassisPtr()->YawRate()
      << "  fliter_steering_angle = "<<fliter_steering_angle<<"  fliter_yaw_rate = "<<fliter_yaw_rate
      << "  steering_angle_change_rate = "<<steering_angle_change_rate<<"  yaw_rate_change_rate = "<<yaw_rate_change_rate
      << "  weight = "<<weight<<"  current_kappa = "<<current_kappa << "   current_speed_limit = "<<t_speed_limit.front().second ;



  if (0) {
      static int count = 0;
    // plot
      std::filesystem::path buffer = std::filesystem::path(__FILE__).parent_path();
      std::string direction = buffer;
      direction += "acc_speed_limit.csv";
      ERT_PLOG_I << "direction = " << direction ;
      std::ofstream test_file;
      test_file.open(direction, std::ios::out | std::ios::app);
      if (!test_file) {
        std::cerr << "Unable to open file: " << direction ;
        return;
      }
      test_file << std::fixed << std::setprecision(4); // 设置精度为4位小数
      test_file << count++ << "," << local_view.getChassisPtr()->SteeringAngle() << "," << local_view.getChassisPtr()->YawRate()
             << "," << fliter_steering_angle << "," << fliter_yaw_rate << "," << steering_angle_change_rate
             << "," << yaw_rate_change_rate << "," << weight << "," << current_kappa << "," << curvature_speed_path_.front().path_point().kappa() << "," << curvature_speed_path_.front().v() << "," << t_speed_limit.front().second << "\n";
      test_file.close();
  }
}
/**
 * @brief 转向角与横摆率滤波处理（数据平滑与变化率计算）
 * @param[in] steering_angle 原始转向角（单位：度）
 * @param[in] yaw_rate 原始横摆率（单位：rad/s）
 * @param[out] fliter_steering_angle 滤波后转向角（单位：度）
 * @param[out] fliter_yaw_rate 滤波后横摆率（单位：rad/s）
 * @param[out] steering_angle_change_rate 转向角变化率（单位：度/s²）
 * @param[out] yaw_rate_change_rate 横摆率变化率（单位：rad/s²）
 * 
 * @par 输入/输出参数说明:
 * | 参数                       | 类型         | 取值范围       | 说明                  |
 * |----------------------------|--------------|---------------|-----------------------|
 * | steering_angle            | double       | [-540,540]    | 原始方向盘转角        |
 * | yaw_rate                  | double       | [-π~π]        | 原始横摆角速度        |
 * | fliter_steering_angle     | double&      | [-540,540]    | 双重滤波后转向角      |
 * | fliter_yaw_rate           | double&      | [-π~π]        | 双重滤波后横摆率      |
 * | steering_angle_change_rate| double&      | [-∞,∞]        | 转向角变化率（回归）  |
 * | yaw_rate_change_rate      | double&      | [-∞,∞]        | 横摆率变化率（回归）  |
 * 
 * @par 处理流程:
 * @startuml
 :更新历史数据队列;
 if (历史数据非空?) then (yes)
   :执行移动平均滤波;
   :执行指数加权平均滤波;
 endif
 :记录当前滤波值到历史队列;
 :构建时间序列向量;
 if (数据点>1) then (yes)
   :线性回归计算变化率;
 endif
 if (历史数据超窗口?) then (yes)
   :移除最旧数据点;
 endif
 @enduml
 *
 * @note 核心算法:
 * 1. 双重滤波：移动平均(MA) + 指数加权移动平均(EWMA)
 * 2. 变化率计算：基于时间序列的线性回归
 * 3. 历史数据维护：固定窗口大小(FIFO)
 *
 * @warning 关键参数:
 * - 滤波系数：alpha=0.5（EWMA权重）
 * - 时间窗口：window_size=5（历史数据容量）
 * - 时间间隔：0.1秒（隐式时间戳）
 */
void SpeedLimitProcessor::steeringAngleAndYawRateFiliter(const double& steering_angle, const double& yaw_rate,
                                                         double& fliter_steering_angle, double& fliter_yaw_rate,
                                                         double& steering_angle_change_rate,
                                                         double& yaw_rate_change_rate) {
  double alpha = 0.5;
  int window_size = 5;
  if(!history_steering_angle_.empty() && !history_yaw_rate_.empty()){
    fliter_steering_angle = ewmaFilter(fliter_steering_angle, movingAverageFilter(history_steering_angle_), alpha);
    fliter_yaw_rate = ewmaFilter(fliter_yaw_rate, movingAverageFilter(history_yaw_rate_), alpha);
  }
  history_steering_angle_.push_back(fliter_steering_angle);
  history_yaw_rate_.push_back(fliter_yaw_rate);

  std::vector<double> t_vector;
  for (int i = 0; i < history_steering_angle_.size(); i++) {
    t_vector.push_back(i * 0.1);
  }

  if (t_vector.size() > 1) {
    steering_angle_change_rate = (math::Linear1DRegression(t_vector, history_steering_angle_)).first;
    yaw_rate_change_rate = (math::Linear1DRegression(t_vector, history_yaw_rate_)).first;
  }
  if (history_steering_angle_.size() > window_size) {
    history_steering_angle_.erase(history_steering_angle_.begin());
    history_yaw_rate_.erase(history_yaw_rate_.begin());
  }
}
/**
 * @brief 移动平均滤波（数据平滑处理）
 * @param[in] data 输入数据序列（原始观测值集合）
 * @return double 滤波后的平均值
 * 
 * @par 输入参数说明:
 * | 参数    | 类型                 | 取值范围        | 说明                     |
 * |---------|----------------------|----------------|--------------------------|
 * | data    | vector<double>        | -              | 需要滤波的原始数据序列    |
 * 
 * @par 处理流程:
 * @startuml
 :初始化累加器sum=0.0;
 :遍历累加;
 :计算平均值sum/data.size();
 @enduml
 *
 * @note 核心算法:
 * - 算术平均公式：mean = (Σvalue)/n
 * - 线性时间复杂度：O(n)
 * 
 * @warning 使用约束:
 * - 输入数据不能为空（否则除零错误）
 * - 适合小窗口滤波（当前窗口大小由上层控制）
 */
double SpeedLimitProcessor::movingAverageFilter(const std::vector<double>& data) {
  double sum = 0.0;
  for (double value : data) {
    sum += value;
  }
  return sum / data.size();
}
/**
 * @brief 指数加权移动平均滤波（数据平滑处理）
 * @param[in] newValue 新观测值（未滤波）
 * @param[in] oldValue 历史滤波值
 * @param[in] alpha 平滑系数（新值权重）
 * @return double 滤波后的加权平均值
 * 
 * @par 输入/输出参数说明:
 * | 参数     | 类型   | 取值范围       | 说明                     |
 * |----------|--------|---------------|--------------------------|
 * | newValue | double | [-∞,∞]        | 当前时刻的原始测量值      |
 * | oldValue | double | [-∞,∞]        | 上一时刻的滤波结果值      |
 * | alpha    | double | [0.0,1.0]     | 新值权重系数（衰减因子）  |
 * 
 * @note 核心算法:
 * - EWMA公式：result = α*new + (1-α)*old
 * - 滤波特性：α越大对突变越敏感，α越小平滑效果越强
 *
 * @warning 使用约束:
 * - alpha参数需满足 0 ≤ alpha ≤ 1
 * - 典型应用场景：传感器数据去抖
 */
double SpeedLimitProcessor::ewmaFilter(double newValue, double oldValue, double alpha) {
  return alpha * newValue + (1 - alpha) * oldValue;
}
/**
 * @brief 基于转向角计算曲率（阿克曼模型）
 * @param[in] steering_angle 方向盘转角（单位：度）
 * @return double 估算的路径曲率（单位：rad/m）
 * 
 * @par 输入/输出参数说明:
 * | 参数             | 类型   | 取值范围        | 说明                     |
 * |------------------|--------|----------------|--------------------------|
 * | steering_angle   | double | [-540,540]     | 原始方向盘转角输入       |
 * | 返回值           | double | [-∞,∞]         | 计算得到的路径曲率       |
 *
 * @note 核心公式:
 * κ = tan(θ * π/180 / steering_ratio) / wheel_base
 * 
 * @warning 关键参数:
 * - 转向比steering_ratio: 临时值17.0（需根据实车配置调整）
 * - 轴距wheel_base: 从vehicle_param_获取实际值
 */
double SpeedLimitProcessor::calculateKappaBasedSteeringAngle(double steering_angle) {
  double steering_ratio = 17.0;  // temp
  return std::tan(steering_angle *ANG2RAD / steering_ratio) / vehicle_param_.wheel_base();
}
/**
 * @brief 基于横摆角速度计算曲率（运动学模型）
 * @param[in] yaw_rate 横摆角速度（单位：rad/s）
 * @param[in] ego_speed 自车速度（单位：m/s）
 * @return double 估算的路径曲率（单位：rad/m）
 * 
 * @par 输入/输出参数说明:
 * | 参数        | 类型   | 取值范围        | 说明                     |
 * |-------------|--------|----------------|--------------------------|
 * | yaw_rate    | double | [-π, π]        | 车辆横摆角速度测量值      |
 * | ego_speed   | double | (0.0,∞)        | 车辆纵向速度（需>0）      |
 * | 返回值      | double | [-∞,∞]         | 计算得到的路径曲率        |
 *
 * @note 核心公式:
 * κ = ω / v
 * 其中：
 * - ω：横摆角速度（rad/s）
 * - v：纵向速度（m/s）
 * 
 * @warning 使用约束:
 * - 车速必须大于零（防止除零错误）
 * - 适用于低速场景（运动学模型假设）
 */
double SpeedLimitProcessor::calcaulateKappaBasedYawRate(double yaw_rate, double ego_speed) {
  return yaw_rate / ego_speed;
}

}  // namespace gpal::pnc::planning

