/**
 * @file decision_define.h
 * @brief 包含决策相关的重要定义和状态信息，用于规范决策逻辑
 * @author 智能驾驶决策规划团队
 * @date 2024-03-01
 * @version 1.0
 * @copyright Copyright (C) 2024 Shanghai Geometrical Perception and Learning Co., Ltd. All rights reserved.
 * @note 模块功能：
 * - 定义决策意图、变道状态等关键信息和相关操作
 */

#ifndef _PNC_POLICY_MAKER_DECISION_DEFINE_
#define _PNC_POLICY_MAKER_DECISION_DEFINE_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "decision_data/container/container_manager.h"
#include "decision_data/decision_common.h"
#include "decision_data/polyline.h"
#include "gpal-interface/planning/decision_result.pb.h"
// #include "gpal-interface/road_cognition/env_road_cognition.pb.h"
#include "proto/common/sl_boundary.pb.h"
#include "reference_line/reference_line.h"
#include "local_view/function_state.h"

namespace gpal::pnc::planning {
namespace Decision {

// ===== 自车与参考线相关参数 ===== //
/// @brief 自车到参考线信息结构体，记录自车相对于参考线的状态信息
struct EgoToRef {
  /**
   * @brief 默认构造函数
   */
  EgoToRef() = default;

  EgoToRef(const ReferenceLine& ref_line_in) : ref_line_copy(ref_line_in) {}

  pnc::SLBoundary sl_bound;     /// @brief 自车在参考线坐标系下的 SL 边界
  double vs = 0.0;              /// @brief 自车在 s 方向的速度
  double vl = 0.0;              /// @brief 自车在 l 方向的速度
  double heading_to_ref = 0.0;  /// @brief 自车相对于参考线的航向角

  ReferenceLine ref_line_copy;  /// @brief 参考线的副本
  double ego_s = 0.0;           /// @brief 自车在参考线坐标系下的 s 坐标，默认值为 0.0
  int ego_idx = -1;             /// @brief 自车在参考线上对应的索引，-1 表示无效索引，默认值为 -1
  pnc::SLPoint ego_sl;          /// @brief 自车定位点在参考线坐标系下的 sl坐标
  bool is_ego_sl_valid = false; /// @brief 自车定位点在参考线坐标系下的 sl 坐标是否有效
  std::vector<pnc::SLPoint> ego_head_traj_sl{}; /// @brief 自车头轨迹点到参考线的sl信息
  int intrude_idx = -1;          /// @brief 自车头轨迹在参考线上首次满足 |l|<2 的索引，未找到时为 100
  double intrude_end_s = -1.0;   /// @brief 自车头轨迹在 intrude_idx 处的 s 坐标，未找到时为 -1.0
  int min_l_idx = -1;            /// @brief 自车头轨迹最小 |l| 对应的索引
  double min_l_end_s = -1.0;     /// @brief 自车头轨迹在 min_l_idx 处的 s 坐标（即 target_s）
};

// ===== 多段线相关参数 ===== //
/// @brief 多段线信息结构体，记录不同车道内的多段线信息
struct PolylineInfo {
  std::vector<Polyline> polyline_within_ego_lane{};             /// @brief 自车车道内的多段线列表
  std::vector<Polyline> polyline_within_left_neighbor_lane{};   /// @brief 左侧相邻车道内的多段线列表
  std::vector<Polyline> polyline_within_right_neighbor_lane{};  /// @brief 右侧相邻车道内的多段线列表
};

// ===== 自车信息相关参数 ===== //
/// @brief 自车信息结构体，记录自车的物理属性和状态信息
struct EgoInfo {
  double length = 5.0;                               /// @brief 自车长度，默认值为 5.0 米
  double width = 2.0;                                /// @brief 自车宽度，默认值为 2.0 米
  double x = 0.0;                                    /// @brief 自车在全局坐标系下的 x 坐标
  double y = 0.0;                                    /// @brief 自车在全局坐标系下的 y 坐标
  double z = 0.0;                                    /// @brief 自车在全局坐标系下的 z 坐标
  double heading = 0.0;                              /// @brief 自车的航向角
  double spd = 0.0;                                  /// @brief 自车的速度
  double acc = 0.0;                                  /// @brief 自车的加速度
  double yaw_rate = 0.0;                             /// @brief 自车的偏航角速度
  ObjectType type = ObjectType::VEHICLE;             /// @brief 自车的类型，默认值为车辆
  math::Box2d cur_box{};                             /// @brief 自车当前的box框
  std::map<std::string, EgoToRef> ego_to_ref_map{};  /// @brief 自车到不同参考线的信息映射
  std::vector<RefTrajPoint> manual_traj{};           /// @brief 人驾参考轨迹
  std::vector<proto::TrajectoryPoint> motion_traj{};   /// @brief 自车上一帧轨迹，自车后轴中心坐标
};

// ===== SL 边界相关参数 ===== //
/// @brief SL 边界向量类型定义，用于存储多个 SL 边界信息
using SlBoundVec = std::vector<pnc::SLBoundary>;

// ===== 预测信息相关参数 ===== //
/// @brief 预测信息结构体，记录目标对象的预测信息
struct PredToRef {
  SlBoundVec sl_bound_vec{};                           /// @brief 预测的 SL 边界向量
  std::vector<double> vs_vec{};                        /// @brief 预测的 s 方向速度向量
  std::vector<double> vl_vec{};                        /// @brief 预测的 l 方向速度向量
  std::vector<double> heading_to_ref_vec{};            /// @brief 预测的相对于参考线的航向角向量
  std::vector<std::pair<double, double>> attent_vec{}; /// <注意力值, 时间步>
  double prob = 0.0;                                   /// @brief 预测的概率
  int min_l_idx = -1;                                  /// @brief 最小 l 索引
  double min_l = 999.0;                                /// @brief 最小 l 值
  int interfere_idx = -1;                              /// @brief 干扰索引，默认值为 -1
};

// ===== 避障信息相关参数 ===== //
/// @brief 避障信息结构体，记录避障相关的标签和边界信息
struct BypassInfo {
  LateralOdTag lat_tag = LateralOdTag::INVALID;  /// @brief 横向避障标签，默认值为无效标签
  pnc::SLBoundary max_bypass_sl_bound{};         /// @brief 最大避障 SL 边界
  double max_bypass_pred_time = -1.0;            /// @brief 最大避障预测时间
  double start_pred_time = -1.0;                 /// @brief 避障开始预测时间
  double end_pred_time = -1.0;                   /// @brief 避障结束预测时间
  double interactive_bypass_start_t = -1.0;      /// @brief 对象交互避障开始时间，用于横向tag序列化，不影响原接口使用
  double interactive_bypass_end_t = -1.0;        /// @brief 对象交互避障结束时间，用于横向tag序列化，不影响原接口使用

  double start_pred_time_bound = 0.0;  /// @brief 避障开始预测时间边界
  double end_pred_time_bound = 0.0;    /// @brief 避障结束预测时间边界
  double start_s_bound = 0.0;          /// @brief 避障开始 s 边界
  double end_s_bound = 0.0;            /// @brief 避障结束 s 边界
  double end_l_bound = 0.0;            /// @brief 避障结束 l 边界
  double start_l_bound = 0.0;          /// @brief 避障开始 l 边界
};

// ===== 对象与参考线相关参数 ===== //
/// @brief 对象到参考线信息结构体，记录目标对象相对于参考线的状态信息
struct ObjToRef {
  pnc::SLBoundary sl_bound;                  /// @brief 对象在参考线坐标系下的 SL 边界
  double attention = -1.0;                                  /// @brief 对象的注意力值，范围 0.0~1.0，默认值为 -1.0
  double attention_time = -1.0;                             /// @brief 对象的注意力时间
  ObjectGameType cur_game_type = ObjectGameType::NON_GAME;  /// @brief 对象的游戏类型，默认值为非游戏对象
  ObjectGameType game_type = ObjectGameType::NON_GAME;      /// @brief 对象的游戏类型，默认值为非游戏对象

  double ds = 9999.0;  /// @brief 对象的 s 坐标减去自车的 s 坐标
  double dl = 9999.0;  /// @brief 对象的 l 坐标减去自车的 l 坐标
  double ttc = -1.0;   /// @brief 自车到对象的碰撞时间
  double hwt = -1.0;   /// @brief 自车到对象的车头时距

  double start_l_to_left_bound = 9999.0;   /// @brief 对象起始 l 坐标到左边界的距离
  double end_l_to_left_bound = 9999.0;     /// @brief 对象结束 l 坐标到左边界的距离
  double start_l_to_right_bound = 9999.0;  /// @brief 对象起始 l 坐标到右边界的距离
  double end_l_to_right_bound = 9999.0;    /// @brief 对象结束 l 坐标到右边界的距离

  double coarse_ref_s = 0.0;  /// @brief 对象粗略的参考 s 坐标

  double vs = 0.0;              /// @brief 对象在 s 方向的速度
  double vl = 0.0;              /// @brief 对象在 l 方向的速度
  double heading_to_ref = 0.0;  /// @brief 对象相对于参考线的航向角

  double last_attention = 0.0;  /// @brief 对象上一时刻的注意力值
  double cur_attent = 0.0;      /// @brief 对象当前时刻的注意力值

  std::vector<PredToRef> pred_to_ref_vec{};  /// @brief 对象的预测信息列表

  BypassInfo bypass_info{};  /// @brief 对象的避障信息

  bool is_mcts_agent = false;     /// @brief 对象是否为 MCTS 智能体
  bool is_mcts_obstacle = false;  /// @brief 对象是否为 MCTS 障碍物

  bool is_parallel = false;              /// @brief 对象是否与自车平行
  bool is_ignored_by_red_light = false;  /// @brief 对象是否被红灯忽略
};

// ===== 决策对象相关参数 ===== //
/// @brief 决策对象结构体，记录需要进行决策的目标对象的信息
struct DecisionObject {
  /**
   * @brief 默认构造函数
   */
  DecisionObject() {};

  /**
   * @brief 构造函数，使用感知 ID 和对象 ID 初始化决策对象
   * @param[in] perception_id_in 感知系统分配的对象 ID
   * @param[in] id_in 对象的唯一标识符
   */
  DecisionObject(const int perception_id_in, const std::string& id_in) : perception_id(perception_id_in), id(id_in) {}

  /**
   * @brief 构造函数，使用对象的详细信息初始化决策对象
   * @param[in] id_in 对象的唯一标识符
   * @param[in] len 对象的长度
   * @param[in] wid 对象的宽度
   * @param[in] x_in 对象在全局坐标系下的 x 坐标
   * @param[in] y_in 对象在全局坐标系下的 y 坐标
   * @param[in] heading_in 对象的航向角
   * @param[in] spd_in 对象的速度
   * @param[in] is_static_in 对象是否为静态对象
   * @param[in] raw_prediction_in 对象的原始预测信息列表
   */
  DecisionObject(const std::string& id_in, const double len, const double wid, const double x_in, const double y_in,
                 const double heading_in, const double spd_in, const bool is_static_in)
      : id(id_in),
        length(len),
        width(wid),
        x(x_in),
        y(y_in),
        heading(heading_in),
        spd(spd_in),
        is_static(is_static_in) {}
  int perception_id = -1; /// @brief 感知系统分配的对象 ID
  std::string id = ""; /// @brief 对象的唯一标识符
  math::Box2d cur_box{}; /// @brief 对象当前的边界框
  double length = 5.0; /// @brief 对象的长度，默认值为 5.0 米
  double width = 2.0; /// @brief 对象的宽度，默认值为 2.0 米
  double x = 0.0; /// @brief 对象在全局坐标系下的 x 坐标
  double y = 0.0; /// @brief 对象在全局坐标系下的 y 坐标
  double z = 0.0; /// @brief 对象在全局坐标系下的 z 坐标
  double heading = 0.0; /// @brief 对象的航向角
  double pitch = 0.0; /// @brief 对象的俯仰角
  double roll = 0.0; /// @brief 对象的横滚角
  double spd = 0.0; /// @brief 对象的速度
  double acc = 0.0; /// @brief 对象在 x 方向的加速度，默认值为 0.0 m/s^2
  double yaw_rate = 0.0; /// @brief 对象的横摆角速度，默认值为 0.0 rad/s
  ObjectType type = ObjectType::VEHICLE; /// @brief 对象的类型，默认值为车辆
  bool is_static = false; /// @brief 对象是否为静态对象
  std::vector<RawSinglePrediction> raw_predictions{}; /// @brief 对象的原始预测信息指针列表
  std::map<std::string, ObjToRef> obj_to_ref_map = {}; /// @brief 对象到不同参考线的信息映射

  pnc::SLBoundary cur_sl_bound; /// @brief 对象当前的 SL 边界
  ObjectGameType game_type = ObjectGameType::NON_GAME; /// @brief 对象的游戏类型，默认值为非游戏对象
  std::vector<pnc::SLBoundary> pred_sl_bounds{}; /// @brief 对象预测的 SL 边界指针列表
  bool is_mcts_agent = false;     /// @brief 对象是否为 MCTS 智能体
  bool is_mcts_obstacle = false;  /// @brief 对象是否为 MCTS 障碍物
  bool is_high_priority = false;  /// @brief 对象是否有高路权
  bool is_longi_expand = false;   /// @brief 对象是否纵向扩展

  LateralOdTag lat_od_tag = LateralOdTag::INVALID;             /// @brief 对象的横向 OD 标签，默认值为无效标签
  double bypass_start_time = -1.0;                             /// @brief 对象避障开始时间
  double bypass_end_time = -1.0;                               /// @brief 对象避障结束时间
  double interactive_bypass_start_time = -1.0;                 /// @brief 对象交互避障开始时间，用于横向tag序列化，不影响原接口使用
  double interactive_bypass_end_time = -1.0;                   /// @brief 对象交互避障结束时间，用于横向tag序列化，不影响原接口使用

  LongitudinalOdTag long_od_tag = LongitudinalOdTag::INVALID;  /// @brief 对象的纵向 OD 标签，默认值为无效标签
  double obj_cross_s = 9999.0;                                 /// @brief 对象的交叉 s 坐标
  double obj_cross_t = -1.0;                                   /// @brief 对象的交叉时间
  double longi_start_time = -1.0;                              /// @brief 对象的纵向 OD 标签开始时间
  double longi_end_time = -1.0;                                /// @brief 对象的纵向 OD 标签结束时间

  std::vector<LateralOdTag> lat_od_tags_seq;                   /// @brief 对象的横向 OD 标签序列
  std::vector<LongitudinalOdTag> long_od_tags_seq;             /// @brief 对象的纵向 OD 标签序列

  // 用于变道过程，与 long_od_tag 配合，follow - 最小安全距离要求: ego_v * safe_hwt_threshold_lane_change, overtake -
  // 最小安全距离要求: obs_v * safe_hwt_threshold_lane_change
  double safe_hwt_threshold_lane_change = -1.0; /// @brief 变道时的安全车头时距阈值

  template<typename TagType>
  static std::vector<TagType> SetTagSeq(const double start_time, const double end_time, const TagType default_tag,
                                        const TagType active_tag) {
    constexpr double dt = 0.1;
    constexpr int steps = 81;
    std::vector<TagType> seq(steps, default_tag);
    if ((start_time < 0.0) || (end_time < 0.0)) {
      return seq;
    }
    int start_idx = static_cast<int>(std::round(start_time / dt));
    int end_idx = static_cast<int>(std::round(end_time / dt));
    start_idx = std::clamp(start_idx, 0, steps - 1);
    end_idx = std::clamp(end_idx, 0, steps - 1);
    for (int i = start_idx; i <= end_idx; ++i) {
      seq[i] = active_tag;
    }
    return seq;
  }
  void ResetDecisionTag() {
    lat_od_tag = LateralOdTag::IGNORE;
    long_od_tag = LongitudinalOdTag::IGNORE;

    longi_start_time = 0.0;
    longi_end_time = 8.0;
    interactive_bypass_start_time = 0.0;
    interactive_bypass_end_time = 8.0;
    lat_od_tags_seq.clear();
    lat_od_tags_seq = SetTagSeq<LateralOdTag>(interactive_bypass_start_time, interactive_bypass_end_time,
                                              LateralOdTag::IGNORE, LateralOdTag::IGNORE);
    long_od_tags_seq.clear();
    long_od_tags_seq = SetTagSeq<LongitudinalOdTag>(longi_start_time, longi_end_time, LongitudinalOdTag::IGNORE,
                                                    LongitudinalOdTag::IGNORE);
    game_type = ObjectGameType::NON_GAME;
    is_mcts_agent = false;
    is_mcts_obstacle = false;
  }
};

// ===== 决策对象映射相关参数 ===== //
/// @brief 决策对象映射类型定义，用于存储多个决策对象的映射
using DecisionObjectMap = std::map<std::string, DecisionObject>;

// ===== GapNet 结果相关参数 ===== //
/// @brief GapNet 结果结构体，记录 GapNet 模型的预测结果
struct GapNetResult {
  void Clear() {
    gap_infos.clear();
    trajectories.clear();
  }

  std::vector<proto::GapInfo> gap_infos;                ///< 车道间隙信息列表
  std::vector<proto::DecisionTrajectory> trajectories;  ///< 决策轨迹列表
};

// ===== 车道变更间隙相关参数 ===== //
/// @brief 车道变更间隙结构体，记录车道变更时的间隙信息
struct LaneChangeGap {
  constexpr static float kInvalidValue = 999.0f;

  float front_s = kInvalidValue;        /// @brief 前方对象的 s 坐标
  float front_s_speed = kInvalidValue;  /// @brief 前方对象的 s 方向速度
  float front_obj_start_l = kInvalidValue;  /// @brief 前方对象的 l 坐标
  float front_obj_end_l = kInvalidValue;    /// @brief 前方对象的 l 坐标
  std::string front_obj_id;             /// @brief 前方对象的唯一标识符
  float back_s = -kInvalidValue;        /// @brief 后方对象的 s 坐标
  float back_s_speed = -kInvalidValue;  /// @brief 后方对象的 s 方向速度
  float back_obj_start_l = kInvalidValue;  /// @brief 后方对象的 l 坐标
  float back_obj_end_l = kInvalidValue;    /// @brief 后方对象的 l 坐标
  std::string back_obj_id;              /// @brief 后方对象的唯一标识符
  proto::Vector3f center;               /// @brief 间隙中心点

  float left_l = kInvalidValue;          /// @brief 左侧对象的 l 坐标
  float left_l_speed = kInvalidValue;    /// @brief 左侧对象的 l 方向速度
  std::string left_obj_id;               /// @brief 左侧对象的唯一标识符
  float right_l = -kInvalidValue;        /// @brief 右侧对象的 l 坐标
  float right_l_speed = -kInvalidValue;  /// @brief 右侧对象的 l 方向速度
  std::string right_obj_id;              /// @brief 右侧对象的唯一标识符
};

// ===== 车道变更间隙向量相关参数 ===== //
/// @brief 车道变更间隙向量类型定义，用于存储多个车道变更间隙信息
using GapVec = std::vector<LaneChangeGap>;

// ===== 间隙信息相关参数 ===== //
/// @brief 间隙信息结构体，记录参考线相关的间隙信息
struct GapInfo {
  std::string ref_line_id = "";             /// @brief 参考线的唯一标识符
  std::vector<std::string> obj_id_in_lane;  /// @brief 车道内对象的唯一标识符列表
  int first_front_obj_idx = -1;             /// @brief 第一个前方对象的索引
  GapVec gaps;                              /// @brief 车道变更间隙信息列表
  int closest_gap_idx = -1;                 /// @brief 最近间隙的索引
  /**
   * @brief 默认构造函数
   */
  GapInfo() = default;

  /**
   * @brief 构造函数，使用参考线 ID 初始化间隙信息
   * @param[in] ref_line_id_in 参考线的唯一标识符
   */
  GapInfo(const std::string& ref_line_id_in) : ref_line_id(ref_line_id_in) {}
};

// ===== 车道变更抑制信息相关参数 ===== //
/// @brief 车道变更抑制信息结构体，记录车道变更的抑制原因
struct LaneChangeInhibitInfo {
  std::vector<LaneChangeInhibitReason> left_lane_inhibit_reason_vec;   /// @brief 左侧车道变更抑制原因列表
  std::vector<LaneChangeInhibitReason> right_lane_inhibit_reason_vec;  /// @brief 右侧车道变更抑制原因列表
  std::vector<OvertakeInhibitReason>
      left_overtake_inhibit_reason_vec;  /// @brief 左侧超车抑制原因列表
  std::vector<OvertakeInhibitReason>
      right_overtake_inhibit_reason_vec;  /// @brief 右侧超车抑制原因列表
};

struct LaneBoundaryInfo {
  double left_solid_line_start_s_in_ref_line =
      1000.0;  /// @brief 左侧实线车道线的起始 s 坐标
  double right_solid_line_start_s_in_ref_line =
      1000.0;  /// @brief 右侧实线车道线的起始 s 坐标
  double left_solid_line_start_s_to_ego =
      1000.0;  /// @brief 左侧实线车道线的起始 s 坐标
  double right_solid_line_start_s_to_ego =
      1000.0;                     /// @brief 右侧实线车道线的起始 s 坐标
  double left_lane_width = 1.75;  /// @brief 左侧车道宽度
  double right_lane_width = 1.75;  /// @brief 右侧车道宽度
};

// ===== 转向拨杆状态信息相关参数 ===== //
/// @brief 转向拨杆状态信息结构体，记录转向拨杆的状态和切换时间
struct TurnLeverStateInfo {
  /**
   * @brief 默认构造函数
   */
  TurnLeverStateInfo() {};

  /**
   * @brief 构造函数，使用转向拨杆状态和切换时间初始化转向拨杆状态信息
   * @param[in] lever_state 转向拨杆的状态
   * @param[in] switch_time 转向拨杆的切换时间
   */
  TurnLeverStateInfo(TurnLeverState& lever_state, double switch_time)
      : lever_state(lever_state), switch_time(switch_time) {}
  TurnLeverState lever_state = TurnLeverState::OFF;  /// @brief 转向拨杆的状态，默认值为关闭状态
  double switch_time = 0.0;                          /// @brief 转向拨杆的切换时间
};

// ===== 车道变更关键对象 ID ===== //
/// @brief 车道变更关键对象 ID 结构体，记录变道场景中各车道前后方最近障碍物的 ID
struct LaneChangeKeyObjectIds {
  std::string ego_lane_front_obj_id = "";               /// @brief 自车道前向最近对象 ID
  std::string ego_lane_rear_obj_id = "";                /// @brief 自车道后向最近对象 ID
  std::string target_lane_front_obj_id = "";            /// @brief 目标车道前向最近对象 ID
  std::string target_lane_rear_obj_id = "";             /// @brief 目标车道后向最近对象 ID
  std::string change_to_ego_lane_front_obj_id = "";     /// @brief 汇入到本车道的前向最近对象 ID
  std::string change_to_target_lane_front_obj_id = "";  /// @brief 汇入到目标车道的前向最近对象 ID
  std::string change_to_target_lane_rear_obj_id = "";   /// @brief 汇入到目标车道的后向最近对象 ID

  void Clear() {
    ego_lane_front_obj_id.clear();
    ego_lane_rear_obj_id.clear();
    target_lane_front_obj_id.clear();
    target_lane_rear_obj_id.clear();
    change_to_ego_lane_front_obj_id.clear();
    change_to_target_lane_front_obj_id.clear();
    change_to_target_lane_rear_obj_id.clear();
  }
};

// ===== 车道变更信息相关参数 ===== //
/// @brief 车道变更信息结构体，记录车道变更的意图、状态等信息
struct LaneChangeInfo {
  // std::string lk_ref_line_id = "";
  IntentionDirection intent_direction = IntentionDirection::NO_INTENTION;  /// @brief 车道变更的意图方向，默认值为无意图
  IntentionType intent_type = IntentionType::NO_INTENTION;                 /// @brief 车道变更的意图类型，默认值为无意图
  std::string intent_ref_line_id = "";                                     /// @brief 车道变更目标参考线的唯一标识符
  ReferencePoint intent_ref_line_point;                                    /// @brief 车道变更目标参考线的参考点
  std::string ego_ref_line_id = "";                                        /// @brief 自车当前参考线的唯一标识符
  ReferencePoint ego_ref_line_point;                                       /// @brief 自车当前参考线的参考点
  std::map<std::string, GapInfo> gap_info_map;                             /// @brief 不同参考线的间隙信息映射
  int best_gap_idx = -10;                                                  /// @brief 最佳间隙的索引
  LaneChangeGap best_gap;                                                  /// @brief 最佳间隙信息
  double accel_for_best_gap = 0.0;                                         /// @brief 为最佳间隙所需的加速度
  LaneChangeState lc_state = LaneChangeState::STANDBY;       /// @brief 车道变更的当前状态，默认值为待命状态
  LaneChangeState last_lc_state = LaneChangeState::STANDBY;  /// @brief 车道变更的上一状态，默认值为待命状态
  LaneChangeState last_frame_lc_state =
      LaneChangeState::STANDBY;  /// @brief 车道变更的上一状态，默认值为待命状态
  bool is_disable_lc = false;                                /// @brief 是否禁用车道变更
  std::pair<double, double> navi_lc_range = {-1.0, -1.0};    /// @brief 导航指定的车道变更范围
  double hmi_speed_limit = 9999.0;                           /// @brief HMI 显示的速度限制
  double lane_speed_limit = 9999.0;                          /// @brief 车道的速度限制
  LaneChangeInhibitInfo lc_inhibit_info{};                   /// @brief 车道变更的抑制信息
  // 自车后轴中心距离目标车道线的距离，左正右负，只在 changing 状态下有效
  double dis_to_traget_lane_mark = 0.0;         /// @brief 自车后轴中心到目标车道线的距离
  TurnLeverStateInfo turn_lever_state_info{};   /// @brief 转向拨杆的状态信息
  int lane_change_nums_required = 0;            /// @brief 需要进行的车道变更次数
  double available_lane_change_dist = 0.0;      /// @brief 可用的车道变更距离
  LaneChangeMctsResult mcts_result;             /// @brief MCTS 结果是否安全
  bool is_need_clear_lane_change_info = false;  /// @brief 是否需要清除车道变更信息
  GapNetResult gapnet_result;                   /// @brief GapNet 模型的预测结果
  bool is_attempt_for_gap_select = false;       /// @brief 是否尝试 Gap 选择
  bool is_attempt_for_lateral_push = false;     /// @brief 是否尝试横向push
  double intention_net_preprocess_time = 0.0;   /// @brief 意图网络预处理时间
  double intention_net_inference_time = 0.0;    /// @brief 意图网络推理时间
  double intention_net_total_time = 0.0;        /// @brief 意图网络后处理时间
  double left_overtake_prob = 0.0;              /// @brief 左侧超车概率
  double right_overtake_prob = 0.0;             /// @brief 右侧超车概率
  double keep_prob = 0.0;                       /// @brief 保持当前车道概率
  LaneChangeInterruptInfo interrupt_info{};     /// @brief 车道变更中断信息
  LaneBoundaryInfo lane_boundary_info{};        /// @brief 车道边界信息
  LaneChangeKeyObjectIds key_obj_ids{};         /// @brief 车道变更关键对象 ID

  /**
   * @brief 重置相关状态和信息，将多个成员变量恢复到默认值
   * @param[in] 无
   * @par 输入参数说明:
   * - 无输入参数
   * @return 无返回值
   * @par 处理流程：
   * @startuml
   * start
   * :将意图方向 intent_direction 设置为无意图状态 IntentionDirection::NO_INTENTION;
   * :将意图参考线 ID intent_ref_line_id 置为空字符串 "";
   * :将自车参考线 ID ego_ref_line_id 置为空字符串 "";
   * :清空间隙信息映射表 gap_info_map;
   * :将最佳间隙信息 best_gap 重置为空对象 {};
   * :将变道状态 lc_state 设置为待机状态 LaneChangeState::STANDBY;
   * :将变道禁用标志 is_disable_lc 设置为 false;
   * :将导航变道范围 navi_lc_range 设置为 {-1.0, -1.0};
   * stop
   * @enduml
   */
  void Reset() {
    // lk_ref_line_id = "";
    intent_direction = IntentionDirection::NO_INTENTION;
    intent_ref_line_id = "";
    ego_ref_line_id = "";
    gap_info_map.clear();
    best_gap = {};
    lc_state = LaneChangeState::STANDBY;
    is_disable_lc = false;
    navi_lc_range = {-1.0, -1.0};
  }
};

// ===== 车道信息相关参数 ===== //
/// @brief 车道信息结构体，记录车道的相关信息
struct LaneInfo {
  std::string ref_line_id = "";                /// @brief 车道参考线的唯一标识符
  int leading_obj_num = -1;                    /// @brief 车道内前方对象的数量
  double leading_traffic_min_speed = 9999.0;   /// @brief 车道内前方交通的最小速度
  double leading_traffic_flow_speed = 9999.0;  /// @brief 车道内前方交通的平均速度
  double front_obj_speed = 9999.0;             /// @brief 车道内前方对象的速度
  std::string front_obj_id = "";               /// @brief 车道内前方对象的唯一标识符
  double navi_score = 0.0;                     /// @brief 车道的导航得分
  bool is_blocked = false;                     /// @brief 车道是否被阻塞
  bool is_stopped_by_red_light = false;        /// @brief 车道是否因红灯停车
  double dist_to_stop_line =
      9999.0;  /// @brief
               /// 自车到停车线的距离，自车位置-停车线位置，负值表示停车线在自车前方
  bool is_near_uturn = false;                  /// @brief 车道是否靠近掉头区域
  bool is_near_intersection = false;           /// @brief 车道是否靠近交叉口
  DrivingDirection driving_direction =
      DrivingDirection::kDirectionForwardOnly;  /// @brief 车道的行驶方向，默认值为仅向前行驶
};

// ===== 车道变更历史相关参数 ===== //
/// @brief 车道变更历史结构体，记录车道变更的历史信息
struct LaneChangeHistory {
  IntentionDirection intent_direction =
      IntentionDirection::NO_INTENTION;                     /// @brief 车道变更的历史意图方向，默认值为无意图
  IntentionType intent_type = IntentionType::NO_INTENTION;  /// @brief 车道变更的历史意图类型，默认值为无意图
  double finish_time = 0.0;                                 /// @brief 车道变更完成的时间
};

/// @brief 场景识别结果结构体，记录场景识别的结果信息
struct SceneRecognitionResult {
  SceneType scene_type = SceneType::NORMAL;  /// @brief 场景类型，默认为未知场景
  double distance_to_scene_point =
      0.0;                     /// @brief 自车距离场景关键点的纵向距离（米）
  double scene_point_s = 0.0;  /// @brief 场景关键点在ego参考线的s坐标
  LaneTopologyPosition topology_position =
      LaneTopologyPosition::UNKNOWN;  /// @brief 自车当前拓扑位置（左/右）
  LaneHierarchyType lane_hierarchy =
      LaneHierarchyType::UNKNOWN;  /// @brief 自车所在车道的主从关系
  math::Vec3d scene_point;         /// @brief 场景关键点的位置
  bool is_in_conflict_area = false;  /// @brief 自车是否在冲突区域
  bool is_first_in_scene_recognition = false;  /// @brief 是否为首次场景识别
  std::vector<std::string> related_reference_lines_ids;  /// @brief 除ego参考线外的其他相关参考线的ID数组
  double merge_s_calc_by_ref = -999.0;                   /// @brief 基于参考线线型计算的相对稳定的汇入点s值
  bool is_virtual_keypoint = false;  /// @brief 是否为决策内部维持的虚拟keypoint
  bool is_merge_scenario = false;  /// @brief 是否为汇入场景，用于决策场景判断
  UTurnInfo uturn_info;            /// @brief 掉头区域信息
  std::vector<std::string> merge_agent_vec;   /// @brief 汇入场景下的 MCTS 智能体集合
  std::vector<std::string> cross_agent_vec;   /// @brief 交叉场景下的 MCTS 智能体集合
  std::vector<std::string> cutin_agent_vec;   /// @brief cut-in 场景下的 MCTS 智能体集合

  /**
   * @brief 重置场景识别结果
   */
  void Reset() {
    scene_type = SceneType::NORMAL;
    distance_to_scene_point = 0.0;
    scene_point_s = 0.0;
    topology_position = LaneTopologyPosition::UNKNOWN;
    lane_hierarchy = LaneHierarchyType::UNKNOWN;
    is_in_conflict_area = false;
    is_first_in_scene_recognition = false;
    related_reference_lines_ids.clear();
    merge_agent_vec.clear();
    cross_agent_vec.clear();
    cutin_agent_vec.clear();
    merge_s_calc_by_ref = -999.0;
    is_virtual_keypoint = false;
  }
};

// ===== 决策数据相关参数 ===== //
/// @brief 决策数据结构体，整合了车道、自车、决策对象等信息，用于决策过程
struct DecisionData {
  EgoInfo ego_info;                          // 自车的信息
  LaneInfo ego_lane;                         // 自车所在车道的信息
  LaneInfo left_lane;                        // 自车左侧车道的信息
  LaneInfo right_lane;                       // 自车右侧车道的信息
  LaneChangeInfo lc_info;                    // 车道变更的信息
  TrafficLightDecision traffic_light_info;   // 交通灯的决策信息
  SceneRecognitionResult scene_recognition;  // 场景识别结果,在距离场景关键点的距离小于一定值时持续保存

  std::string target_ref_id = "";          // 目标参考线的唯一标识符
  std::set<std::string> hist_cipo_id_set;  // 历史 CIPO ID 集合

  std::array<LaneChangeHistory, 2> lc_history;            // 车道变更的历史记录数组
  std::vector<ReferencePoint> intent_ref_sampled_points;  // 意图参考线的采样点列表
  std::vector<ReferencePoint> ego_ref_sampled_points;     // 自车参考线的采样点列表

  proto::Trajectory* last_planning_traj = nullptr;                                              // 上次规划的轨迹
  std::shared_ptr<RefTrajInfo> ref_traj_info = std::make_shared<RefTrajInfo>();                 // 参考轨迹的信息
  std::shared_ptr<DecisionObjectMap> decision_obj_map = std::make_shared<DecisionObjectMap>();  // 决策对象的映射指针

  std::shared_ptr<std::map<std::string, DecisionObjectMap>> ref_line_virtual_obj_map = std::make_shared<
      std::map<std::string, DecisionObjectMap>>();  // 决策虚拟对象的映射指针，键为参考线的ID，值为决策对象的映射
  std::shared_ptr<DecisionObjectMap> hist_decision_object_map =
      std::make_shared<DecisionObjectMap>();                                       // 历史决策对象的映射指针
  std::shared_ptr<PolylineInfo> polyline_info = std::make_shared<PolylineInfo>();  // 多段线的信息指针
  std::shared_ptr<prediction::ContainerManager> container_manager = nullptr;  // 预测器管理器的智能指针，管理预测器操作
  int frame_seq = 0;                                                          // 帧序列，用于标识当前帧的序号
  std::set<std::string> uncertain_ref_lines_set;                              // 不确定参考线的集合
  std::string base_decider_debug_info = "";                                   // 基础决策器的调试信息
  StageState stage_state = StageState::IdleStage;                             // 场景阶段状态
  std::shared_ptr<RefLineDecesionInfo> ref_line_decision_info =
      std::make_shared<RefLineDecesionInfo>();               // 参考线决策信息的智能指针
  std::unordered_set<std::string> forbidden_bypass_obj_ids;  // 禁止绕行的障碍物ID集合
  std::unordered_set<std::string> corridor_bypass_obj_ids;   // 连通域绕行的障碍物ID集合

  std::set<std::string> joint_opt_obstacles_set{};       // 联合优化约束障碍物ID集合
  std::set<std::string> joint_opt_agents_set{};          // 联合优化博弈障碍物ID集合
  std::vector<KeyObjSortInfo> joint_opt_obstacles_score{};  // 联合优化约束障碍物列表
  std::vector<KeyObjSortInfo> joint_opt_agents_score{};     // 联合优化博弈障碍物列表
  
  std::string ego_ref_line_id_hd = "";  /// @brief HD 参考线 ID
  std::string ego_ref_line_id_sd = "";  /// @brief SD 参考线 ID
};

}  // namespace Decision
}  // namespace gpal::pnc::planning

#endif
