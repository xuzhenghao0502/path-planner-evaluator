#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

#include "config/openspace/openspace_search.pb.h"
#include "config_manager/config_manager.h"
#include "math/box2d.h"
#include "math/vec2d.h"
#include "openspace_path_planner/core/generator/openspace_roi_decider.h"
#include "openspace_path_planner/core/generator/rs_path.h"
#include "openspace_path_planner/utils/openspace_common.h"
#include "openspace_path_planner/utils/openspace_tools.h"
#include "util/base_struct.h"

namespace gpal::pnc::planning {
// code generation or define child class
// contains successor type and generation
// edge cost calculation
// heuristic calculation

struct ArcPathNode {
  enum DIRECTION : uint8_t { NONE = 0, FORWARD, BACKWARD };
  float x = 0;
  float y = 0;
  float theta = 0;
  float s = 0;
  float dis_same_direction = 0;
  float kappa = 0;
  int generation_num = 0;
  float collision_dist = 10.0;
  DIRECTION direction = DIRECTION::NONE;
};

union ArcPathNodeKey {
  struct {
    int16_t idx_x;
    int16_t idx_y;
    uint16_t idx_theta;
    uint8_t direction;
    uint8_t reserve;
  };
  uint64_t mask;
  ArcPathNodeKey(const int16_t _idx_x = 0, const int16_t _idx_y = 0, const uint16_t _idx_theta = 0,
                 const uint8_t _direction = 0)
      : idx_x(_idx_x), idx_y(_idx_y), idx_theta(_idx_theta), direction(_direction), reserve(0) {}
  bool operator==(const ArcPathNodeKey& rhs) const noexcept { return mask == rhs.mask; }
  bool operator<(const ArcPathNodeKey& rhs) const noexcept { return mask < rhs.mask; }
  bool operator>(const ArcPathNodeKey& rhs) const noexcept { return mask > rhs.mask; }
  bool operator>=(const ArcPathNodeKey& rhs) const noexcept { return mask >= rhs.mask; }
  bool operator<=(const ArcPathNodeKey& rhs) const noexcept { return mask <= rhs.mask; }
};

struct ArcPathNodeKeyHash {
  uint64_t operator()(const ArcPathNodeKey& key) const noexcept { return std::hash<uint64_t>()(key.mask); }
};

struct ArcControl {
  float kappa;
  float steer_grade;
  ArcPathNode::DIRECTION target_direction = ArcPathNode::DIRECTION::NONE;
};

class ArcModel {
 public:
  // need to determine path node type, key type and node control
  using Node_t = ArcPathNode;
  using NodeKey_t = ArcPathNodeKey;
  using NodeControl_t = ArcControl;

  struct TargetRegion {
    math::Box2d target_region;                                               // 目标区域框
    double target_theta = 0.0;                                               // 目标的期望航向
    double theta_tolerance = 0.0;                                            // 航向允许的误差范围 (弧度)
    ArcPathNode::DIRECTION target_direction = ArcPathNode::DIRECTION::NONE;  // 到达时的挡位要求
  };

 protected:
  using NodeLoader = std::function<void(std::vector<Node_t>* path_nodes)>;

 public:
  ArcModel() = default;

  void init(const RoiDecideResult& roi, const Freespace& freespace, const string& config_name,
            const std::vector<string>& scenario_tags) {
    auto arc_model_configs = Singleton<ConfigManager>::get_instance()
                                 ->getConfig<OpenspaceSearchConfig>("OpenspaceSearch")
                                 .arc_model_configs();
    if (arc_model_configs.find(config_name) != arc_model_configs.end()) {
      arc_model_config_ = arc_model_configs.at(config_name);
    } else {
      arc_model_config_ = arc_model_configs.at("regular");
    }

    vehicle_config_ = Singleton<ConfigManager>::get_instance()->vehicle_config();
    wheel_base_ = vehicle_config_.vehicle_param().wheel_base();
    move_nums_ = arc_model_config_.move_nums();
    move_step_size_ = arc_model_config_.move_step();
    steering_radian_ = arc_model_config_.steering_radian();
    steering_discrete_num_ = arc_model_config_.steering_angle_discrete_num();
    shot_distance_ = arc_model_config_.shot_distance();
    steering_penalty_ = arc_model_config_.steering_penalty();
    steering_change_penalty_ = arc_model_config_.steering_change_penalty();
    changing_gear_penalty_ = arc_model_config_.changing_gear_penalty();
    moving_penalty_ = arc_model_config_.moving_penalty();
    collision_penalty_ = arc_model_config_.collision_penalty();
    collision_penalty_range_ = arc_model_config_.collision_penalty_range();
    search_time_limit_ = arc_model_config_.search_time_limit();
    debug_print_switch_ = arc_model_config_.debug_print_switch();
    angular_upper_ = arc_model_config_.angular_upper();
    map_grid_resolution_ = arc_model_config_.map_grid_resolution();
    min_path_length_limit_ = arc_model_config_.min_path_length_limit();
    min_path_length_limit_before_rs_ = arc_model_config_.min_path_length_limit_before_rs();

    dynamicParamAdjust(arc_model_config_, scenario_tags);

    freespace_ = &freespace;
    roi_ = roi;
    map_x_lower_ = roi_.roi_xy_boundary[0];
    map_y_lower_ = roi_.roi_xy_boundary[2];
    map_x_upper_ = roi_.roi_xy_boundary[1];
    map_y_upper_ = roi_.roi_xy_boundary[3];
    angular_resolution_ = 360.0 / angular_upper_ * M_PI / 180.0;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;

    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));
    path_nodes_.clear();
    is_target_region_ = false;

    OPENSPACE_LOG(D, "[ArcModel][init]wheel_base_ = ", wheel_base_);
    OPENSPACE_LOG(D, "[ArcModel][init]length = ", vehicle_config_.vehicle_param().length());
    OPENSPACE_LOG(D, "[ArcModel][init]width = ", vehicle_config_.vehicle_param().width());
    OPENSPACE_LOG(D, "[ArcModel][init]width_without_rearview_mirror = ",
                  vehicle_config_.vehicle_param().width_without_rearview_mirror());
    OPENSPACE_LOG(D, "[ArcModel][init]rear_edge_to_ego = ", vehicle_config_.vehicle_param().rear_edge_to_ego());
    OPENSPACE_LOG(D, "[ArcModel][init]steering_radian_ = ", steering_radian_);
    OPENSPACE_LOG(D, "[ArcModel][init]shot_distance = ", shot_distance_);
    OPENSPACE_LOG(I, "[ArcModel][init]最短轨迹约束 = ", min_path_length_limit_, " m");
    OPENSPACE_LOG(I, "[ArcModel][init]末段最短轨迹约束 = ", arc_model_config_.last_leg_min_length_limit(), " m");
    OPENSPACE_LOG(I, "[ArcModel][init]搜索最大允许时间 =  ", search_time_limit_, " s");
    OPENSPACE_LOG(I, "[ArcModel][init]最小转弯半径 =  ", wheel_base_ / std::tan(steering_radian_));
    OPENSPACE_LOG(I, "[ArcModel][init]搜索步长 =  ", move_step_size_);
    OPENSPACE_LOG(I, "[ArcModel][init]搜索角度辨率 =  ", angular_upper_);
    OPENSPACE_LOG(I, "[ArcModel][init]搜索位移分辨率 =  ", map_grid_resolution_);
    OPENSPACE_LOG(D, "[ArcModel][init]map_x_lower : ", map_x_lower_, " map_y_lower : ", map_y_lower_,
                  " map_x_upper : ", map_x_upper_, " map_y_upper : ", map_y_upper_);
  }

  // 单点搜索
  void setEnd(const Node_t& goal_node) {
    goal_node_ = goal_node;
    is_target_region_ = false;
  }

  // 区域搜索
  void setTargetRegion(const TargetRegion& region, const Node_t& center_node) {
    OPENSPACE_LOG(D, "[ArcModel][setTargetRegion] Set Target Region");
    target_region_ = region;
    goal_node_ = center_node;
    is_target_region_ = true;
  }

  void setStart(const Node_t& start_node) { start_node_ = start_node; }

  ~ArcModel() = default;

 public:  // API that must contains
  void dynamicParamAdjust(const ArcModelConfig& config, const std::vector<std::string>& scenario_tags) {
    for (const auto& tag : scenario_tags) {
      if (config.dynamic_arc_model_config_map().find(tag) != config.dynamic_arc_model_config_map().end()) {
        for (const auto& dynamic_param : config.dynamic_arc_model_config_map().at(tag).dynamic_arc_model_configs()) {
          if (dynamic_param.param() == "search_time_limit") {
            search_time_limit_ = dynamic_param.value();
          } else if (dynamic_param.param() == "shot_distance") {
            shot_distance_ = dynamic_param.value();
          } else if (dynamic_param.param() == "min_path_length_limit") {
            min_path_length_limit_ = dynamic_param.value();
          } else if (dynamic_param.param() == "steering_radian") {
            steering_radian_ = dynamic_param.value();
          } else {
            // nothing
          }
        }
      }
    }
  }

  bool isValid(const Node_t& path_node) const {
    return pointCollisonCheck(*freespace_, path_node.x, path_node.y, path_node.theta, roi_,
                              arc_model_config_.width_collision_buff(), arc_model_config_.length_collision_buff());
  }

  bool analyticExpansions(const Node_t& path_node, const Node_t& goal_node, double& length) {
    Eigen::Vector3d current_node(path_node.x, path_node.y, path_node.theta);
    Eigen::Vector3d end_node(goal_node.x, goal_node.y, goal_node.theta);
    int rs_path_num = 0;
    std::vector<vector<PathPt>> rs_path_poses = rs_path_ptr_->GetRSPath(
        current_node, end_node, move_step_size_, length, rs_path_num, arc_model_config_.enable_s_shape());

    std::vector<Node_t> rs_node_res;
    for (const auto& path : rs_path_poses) {
      for (const auto& pt : path) {
        Node_t pose_node;
        pose_node.x = pt.x();
        pose_node.y = pt.y();
        pose_node.theta = pt.theta();
        pose_node.kappa = pt.kappa();
        pose_node.s = pt.s();
        pose_node.direction = pt.direction() == PathPt::Direction::FORWARD ? ArcPathNode::DIRECTION::FORWARD
                                                                           : ArcPathNode::DIRECTION::BACKWARD;
        if (!isValid(pose_node)) {
          return false;
        };
        rs_node_res.emplace_back(pose_node);
      }
    }

    if (rs_node_res.size() < 2) {
      return false;
    }
    // 第一个点未保留kappa信息，需要补充
    rs_node_res.front().kappa = rs_node_res[1].kappa;

    // 判断RS曲线内部是否换挡
    if (rs_path_poses.size() > 1) {
      // OPENSPACE_LOG(D, "rs轨迹中间换档了");
      if (!enable_multi_segment_rs_paths_) {
        return false;
      }

      // 换挡处延长直线
      if (arc_model_config_.straight_extend_num() > 0) {
        for (int i = 0; i < rs_path_poses.size() - 1; i++) {
          if (!rs_path_poses[i].empty()) {
            Node_t shift_node;
            shift_node.x = rs_path_poses[i].back().x();
            shift_node.y = rs_path_poses[i].back().y();
            shift_node.theta = rs_path_poses[i].back().theta();
            shift_node.kappa = rs_path_poses[i].back().kappa();
            shift_node.s = rs_path_poses[i].back().s();
            shift_node.direction = rs_path_poses[i].back().direction() == PathPt::Direction::FORWARD
                                       ? ArcPathNode::DIRECTION::FORWARD
                                       : ArcPathNode::DIRECTION::BACKWARD;
            auto straight_traj = generateStraightTraj4Turning(shift_node);
            for (auto pt : straight_traj) {
              if (!isValid(pt)) {
                return false;
              }
            }
          }
        }
      }
    }

    // 判断RS第一个点是否换档
    bool rs_first_pt_is_shift = false;
    if (rs_node_res.front().direction != path_node.direction) {
      if (path_node.direction != ArcPathNode::DIRECTION::NONE) {
        rs_first_pt_is_shift = true;
        if (!enable_multi_segment_rs_paths_) {
          // OPENSPACE_LOG(D, " rs第一个点方向与自车反了 ");
          return false;
        }

        if (path_node.dis_same_direction < min_path_length_limit_before_rs_) {
          // OPENSPACE_LOG(D, " rs拼接了一个短轨迹 ");
          return false;
        }

        if (arc_model_config_.straight_extend_num() > 0) {
          auto straight_traj = generateStraightTraj4Turning(path_node);
          for (auto pt : straight_traj) {
            if (!isValid(pt)) {
              return false;
            }
          }
        }
      }
    }

    // 判断RS单段轨迹长度是否符合要求
    for (const auto& path : rs_path_poses) {
      if (!path.empty() && path.back().s() < min_path_length_limit_) {
        // OPENSPACE_LOG(D, "rs单段轨迹长度太短: ", path.back().s(), " < ", min_path_length_limit_);
        return false;
      }
    }

    if (rs_first_pt_is_shift) {
      path_nodes_.insert(path_nodes_.end(), rs_node_res.begin(), rs_node_res.end());
    } else {
      path_nodes_.insert(path_nodes_.end(), rs_node_res.begin() + 1, rs_node_res.end());
    }
    for (auto pt : rs_node_res) {
      OPENSPACE_LOG(D, "[ArcModel][analyticExpansions]rs_path ", pt.x, " ", pt.y, " ", pt.theta, " ", pt.kappa, " ",
                    pt.direction);
    }
    OPENSPACE_LOG(I, "[ArcModel][analyticExpansions]rs cal success !");
    return true;
  }

  bool isReachTarget(const Node_t& path_node, const NodeLoader& node_loader) {
    if (is_target_region_) {
      math::Vec2d pt(path_node.x, path_node.y);
      // 检查 xy 是否在目标区域 Box2d 内
      if (target_region_.target_region.IsPointIn(pt)) {
        // 检查角度差是否在范围内
        double angle_err = std::abs(math::NormalizeAngle(path_node.theta - target_region_.target_theta));
        if (angle_err <= target_region_.theta_tolerance) {
          // 检查方向是否满足
          if (target_region_.target_direction == ArcPathNode::DIRECTION::NONE
              || path_node.direction == target_region_.target_direction) {
            path_nodes_.clear();
            node_loader(&path_nodes_);
            path_nodes_.back().theta = path_node.theta;
            OPENSPACE_LOG(I, "[ArcModel][isReachTarget] AStar Node reached Target Box2d Region directly!");
            return true;
          }
        }
      }
    }

    path_nodes_.clear();
    node_loader(&path_nodes_);
    if (sqrt((path_node.x - goal_node_.x) * (path_node.x - goal_node_.x)
             + (path_node.y - goal_node_.y) * (path_node.y - goal_node_.y))
        < shot_distance_) {
      double rs_length = 0.0;
      if (analyticExpansions(path_node, goal_node_, rs_length)) {
        OPENSPACE_LOG(I, "[ArcModel][isReachTarget]isReachTarget rs_length = ", rs_length);
        return true;
      }
    }

    math::Vec2d goal_node_v2d(goal_node_.x, goal_node_.y);
    math::Vec2d goal_node_unit_v2d = goal_node_v2d.CreateUnitVec2d(goal_node_.theta);
    math::Vec2d path_node_v2d(path_node.x, path_node.y);
    math::Vec2d relative_vec2d = path_node_v2d - goal_node_v2d;

    double lateral_error = goal_node_unit_v2d.CrossProd(relative_vec2d);
    double longti_error = goal_node_unit_v2d.InnerProd(relative_vec2d);
    if (abs(lateral_error) < arc_model_config_.search_stop_distance_x()
        && abs(longti_error) < arc_model_config_.search_stop_distance_y()) {
      if (abs(math::NormalizeAngle(path_node.theta - goal_node_.theta))
          < arc_model_config_.search_stop_angle() * ANG2RAD) {
        if (path_node.dis_same_direction > arc_model_config_.last_leg_min_length_limit()) {
          path_nodes_.back().theta = goal_node_.theta;
          OPENSPACE_LOG(I, "[ArcModel][isReachTarget] exact hit! lateral_error = ", lateral_error,
                        " longti_error = ", longti_error,
                        " angle_err = ", (path_node.theta - goal_node_.theta) * RAD2ANG,
                        " dis_same = ", path_node.dis_same_direction);
          return true;
        }
      }
    }
    return false;
  }

  double calcHeuristicCost(const Node_t& path_node) const {
    double h;
    h = rs_path_ptr_->Distance(path_node.x, path_node.y, path_node.theta, goal_node_.x, goal_node_.y, goal_node_.theta);
    return h;
  }

  double calcEdgeCost(const Node_t& predecessor, const double& heuristic_cost, const Node_t& successor,
                      const NodeControl_t& ctrl) const {
    double g = 0;

    if (predecessor.direction != successor.direction && predecessor.direction != ArcPathNode::DIRECTION::NONE) {
      g += changing_gear_penalty_;
    }
    g += successor.s * moving_penalty_;
    g += steering_change_penalty_ * fabs(successor.kappa - predecessor.kappa);
    g += steering_penalty_ * fabs(successor.kappa);
    g += successor.collision_dist < 1.0 && heuristic_cost > collision_penalty_range_
             ? collision_penalty_ / fabs(successor.collision_dist)
             : 0.0;
    return g;
  }

  void dynamicModel(const double step_size, const double kappa, double& x, double& y, double& theta) const {
    if (std::abs(kappa) < 1e-6) {
      // 直线运动
      x += step_size * std::cos(theta);
      y += step_size * std::sin(theta);
    } else {
      // 圆弧运动
      double r = 1.0 / kappa;                  // 转弯半径
      double delta_theta = step_size * kappa;  // 角度变化量
      double cx = x - r * std::sin(theta);     // 圆心 x
      double cy = y + r * std::cos(theta);     // 圆心 y
      theta = math::NormalizeAngle(theta + delta_theta);
      x = cx + r * std::sin(theta);  // 新位置 x
      y = cy - r * std::cos(theta);  // 新位置 y
    }
  }

  NodeKey_t generateNodeKey(const Node_t& node) const {
    int16_t idx_x = int16_t(node.x / map_grid_resolution_);
    int16_t idx_y = int16_t(node.y / map_grid_resolution_);
    uint16_t idx_theta = uint16_t((node.theta - (-M_PI)) / angular_resolution_);
    // uint8_t direction = node.direction;
    uint8_t direction = 0;
    NodeKey_t key(idx_x, idx_y, idx_theta, direction);
    return key;
  }

  std::pair<bool, Node_t> generateSuccessor(const Node_t& node, const NodeControl_t& ctrl, const int move_nums) const {
    std::pair<bool, Node_t> res{false, node};
    auto& [is_success, res_node] = res;

    // 子节点返回父父节点位置，则返回失败
    if (node.direction != ArcPathNode::DIRECTION::NONE && node.direction != ctrl.target_direction
        && node.kappa == ctrl.kappa) {
      return res;
    }

    double x = node.x;
    double y = node.y;
    double theta = node.theta;
    double move_step = move_step_size_;
    if (ctrl.target_direction == ArcPathNode::DIRECTION::BACKWARD) {
      move_step *= -1.0;
    }
    double move_dis = move_step * move_nums;
    double kappa = ctrl.kappa;

    dynamicModel(move_dis, kappa, x, y, theta);
    double dis_check = 10.0;
    // 忽略搜索起点附近的碰撞检测
    if ((!arc_model_config_.enable_start_ignore()
         || node.generation_num + 1 >= arc_model_config_.search_start_ignore_num())
        && (collision_penalty_ < kMathEpsilon
                ? !pointCollisonCheck(*freespace_, x, y, theta, roi_, arc_model_config_.width_collision_buff(),
                                      arc_model_config_.length_collision_buff())
                : !pointCollisonCheck(dis_check, *freespace_, x, y, theta, roi_,
                                      arc_model_config_.width_collision_buff(),
                                      arc_model_config_.length_collision_buff()))) {
      return res;
    }
    res_node.x = x;
    res_node.y = y;
    res_node.theta = theta;
    res_node.direction = ctrl.target_direction;
    res_node.s = std::abs(move_dis);
    res_node.kappa = ctrl.kappa;
    res_node.generation_num = node.generation_num + 1;
    res_node.collision_dist = dis_check;
    if (res_node.direction == node.direction) {
      res_node.dis_same_direction = node.dis_same_direction + res_node.s;
    }

    if (res_node.direction != node.direction && node.generation_num != 0) {
      // 单次方向必须行驶一定距离，否则不允许换档
      if (node.dis_same_direction > min_path_length_limit_) {
        res_node.dis_same_direction = res_node.s;
      } else {
        return res;
      }

      // 换挡处延长直线
      if (arc_model_config_.straight_extend_num() > 0) {
        std::vector<Node_t> straight_traj = generateStraightTraj4Turning(node);
        for (auto pt : straight_traj) {
          if (!isValid(pt)) {
            return res;
          }
        }
      }
    }

    is_success = true;
    return res;
  }

  std::vector<Node_t> generateStraightTraj4Turning(const Node_t& node) const {
    std::vector<Node_t> ress;
    Node_t res;
    double x = node.x;
    double y = node.y;
    double theta = node.theta;
    double move_dis = 0.0;
    if (node.direction == ArcPathNode::DIRECTION::FORWARD) {
      move_dis = 0.25;
    } else if (node.direction == ArcPathNode::DIRECTION::BACKWARD) {
      move_dis = -0.25;
    }
    // 以 move_dis 间隔，生成长度 move_dis*step_num 的直线段
    auto step_num = arc_model_config_.straight_extend_num();
    for (int j = 1; j <= step_num; j += 1) {
      dynamicModel(move_dis, 0.0, x, y, theta);
      res.x = x;
      res.y = y;
      res.theta = theta;
      res.direction = node.direction;
      ress.push_back(res);
    }
    return ress;
  }

  std::vector<Node_t> generateDynamicStraightTraj4Turning(const Node_t& node) const {
    std::vector<Node_t> ress;
    Node_t res;
    double x = node.x;
    double y = node.y;
    double theta = node.theta;
    double move_dis = 0.0;
    if (node.direction == ArcPathNode::DIRECTION::FORWARD) {
      move_dis = 0.25;
    } else if (node.direction == ArcPathNode::DIRECTION::BACKWARD) {
      move_dis = -0.25;
    }
    // 以 move_dis 间隔，生成长度 move_dis*step_num 的直线段
    auto step_num = arc_model_config_.max_dynamic_straight_extend_num() - arc_model_config_.straight_extend_num();
    for (int j = 1; j <= step_num; j += 1) {
      dynamicModel(move_dis, 0.0, x, y, theta);
      res.x = x;
      res.y = y;
      res.theta = theta;
      res.direction = node.direction;
      if (!pointCollisonCheck(*freespace_, res.x, res.y, res.theta, roi_,
                              arc_model_config_.dynamic_straight_extend_width_collision_buff(),
                              arc_model_config_.dynamic_straight_extend_length_collision_buff())) {
        break;
      }
      ress.push_back(res);
    }
    return ress;
  }

  std::vector<NodeControl_t> generateFeasibleControlSpace(const Node_t& node) const {
    std::vector<NodeControl_t> res;
    NodeControl_t ctr;
    int discrete_num = steering_discrete_num_;
    double steering_radian_step = steering_radian_step_size_;
    double delta_theta = goal_node_.theta - start_node_.theta;

    // 接近终点的时候增加搜索采样方向
    if (abs(node.x - goal_node_.x) < 1.0 && abs(node.y - goal_node_.y) < 1.0) {
      discrete_num = steering_discrete_num_ + 2;
      steering_radian_step = steering_radian_ / discrete_num;
    }
    if (abs(node.x - goal_node_.x) < 0.6 && abs(node.y - goal_node_.y) < 0.6) {
      discrete_num = steering_discrete_num_ + 5;
      steering_radian_step = steering_radian_ / discrete_num;
    }
    for (int i = -discrete_num; i <= discrete_num; ++i) {
      const double phi = i * steering_radian_step;
      ctr.kappa = -std::tan(phi) / wheel_base_;
      ctr.steer_grade = i;
      ctr.target_direction = ArcPathNode::DIRECTION::FORWARD;
      res.emplace_back(ctr);
      ctr.target_direction = ArcPathNode::DIRECTION::BACKWARD;
      res.emplace_back(ctr);
    }
    return res;
  }

  std::vector<PathPt> getPathResult() {
    PathPt res;
    std::vector<PathPt> ress;
    Node_t last_node;

    if (path_nodes_.size() >= 2) {
      path_nodes_.front().direction = path_nodes_[1].direction;
      last_node = path_nodes_.front();
    }

    for (auto p : path_nodes_) {
      if (p.direction != last_node.direction) {
        std::vector<Node_t> straight_traj;
        straight_traj = generateStraightTraj4Turning(last_node);
        Node_t straight_traj_last_node = !straight_traj.empty() ? straight_traj.back() : last_node;
        std::vector<Node_t> dynamic_straight_traj = generateDynamicStraightTraj4Turning(straight_traj_last_node);
        straight_traj.insert(straight_traj.begin(), dynamic_straight_traj.begin(), dynamic_straight_traj.end());
        int size = straight_traj.size();
        for (int i = size - 1; i >= 0; i--) {
          Node_t next_straight = straight_traj[i];
          next_straight.direction = p.direction;
          straight_traj.push_back(next_straight);
        }
        for (auto pt : straight_traj) {
          res.set_x(pt.x);
          res.set_y(pt.y);
          res.set_theta(pt.theta);
          res.set_kappa(pt.kappa);
          if (pt.direction == ArcPathNode::DIRECTION::BACKWARD) {
            res.set_direction(PathPt::Direction::BACKWARD);
          } else {
            res.set_direction(PathPt::Direction::FORWARD);
          }
          ress.emplace_back(res);
        }
        // 换挡处插入连续点,避免换挡处轨迹断连
        if (straight_traj.empty()
            && std::sqrt(std::pow(p.x - last_node.x, 2) + std::pow(p.y - last_node.y, 2)) > 1e-2) {
          res.set_x(last_node.x);
          res.set_y(last_node.y);
          res.set_theta(last_node.theta);
          res.set_kappa(p.kappa);
          if (p.direction == ArcPathNode::DIRECTION::BACKWARD) {
            res.set_direction(PathPt::Direction::BACKWARD);
          } else {
            res.set_direction(PathPt::Direction::FORWARD);
          }
          ress.emplace_back(res);
        }
      }
      res.set_x(p.x);
      res.set_y(p.y);
      res.set_theta(p.theta);
      res.set_kappa(p.kappa);
      if (p.direction == ArcPathNode::DIRECTION::BACKWARD) {
        res.set_direction(PathPt::Direction::BACKWARD);
      } else {
        res.set_direction(PathPt::Direction::FORWARD);
      }
      ress.emplace_back(res);
      last_node = p;
    }
    return ress;
  }

  int moveNums() { return move_nums_; }
  double searchTimeLimit() { return search_time_limit_; }
  bool debugPrintSwitch() { return debug_print_switch_; }

 protected:
  double wheel_base_ = 0.0;                       // 轴距
  int move_nums_ = 1;                             // 单方向搜索最大距离
  double move_step_size_ = 0.0;                   // 单方向搜索步长
  double steering_radian_ = 0.0;                  // 单侧转向最大角度
  double steering_radian_step_size_ = 0.0;        // 单侧转向步长
  int steering_discrete_num_ = 0;                 // 单侧转向次数
  double shot_distance_ = 0.0;                    // 判断接近目标点的阈值
  double steering_penalty_ = 0.0;                 // 转向惩罚系数
  double changing_gear_penalty_ = 0.0;            // 换档惩罚系数
  double steering_change_penalty_ = 0.0;          // 改变转向惩罚系数
  double moving_penalty_ = 0.0;                   // 移动惩罚系数
  double collision_penalty_ = 0.0;                // 碰撞惩罚系数
  double collision_penalty_range_ = 0.0;          // 碰撞惩罚范围
  double min_path_length_limit_ = 1.0;            // 最小路径长度限制
  double min_path_length_limit_before_rs_ = 1.0;  // 拼接RS曲线前的一段轨迹的最小路径长度限制

  double map_x_lower_ = 0;
  double map_y_lower_ = 0;
  double map_x_upper_ = 0;
  double map_y_upper_ = 0;
  uint16_t angular_upper_ = 0;

  double map_grid_resolution_ = 1e-6;
  double angular_resolution_ = 1e-6;
  double search_time_limit_ = 10.0;

  std::shared_ptr<RSPath> rs_path_ptr_;
  std::vector<Node_t> path_nodes_;
  const Freespace* freespace_;
  ArcModelConfig arc_model_config_;
  VehicleConfig vehicle_config_;
  RoiDecideResult roi_;

  Node_t goal_node_;
  Node_t start_node_;

  TargetRegion target_region_;
  bool is_target_region_ = false;

  bool debug_print_switch_ = false;

  bool enable_multi_segment_rs_paths_ = true;
};

}  // namespace gpal::pnc::planning