#include "openspace_path_planner/core/generator/geometry_path_generator.h"

namespace gpal::pnc::planning {

void GeometryPathGenerator::init() {
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);

  veh_ego_2_rear_ = config_manager_->vehicle_config().vehicle_param().rear_edge_to_ego();
  veh_ego_2_front_ = config_manager_->vehicle_config().vehicle_param().front_edge_to_ego();
  veh_width_ = config_manager_->vehicle_config().vehicle_param().width_without_rearview_mirror();
}

void GeometryPathGenerator::setVehParam(double veh_ego_2_rear, double veh_ego_2_front, double veh_width) {
  veh_ego_2_rear_ = veh_ego_2_rear;
  veh_ego_2_front_ = veh_ego_2_front;
  veh_width_ = veh_width;
}

bool GeometryPathGenerator::calFineTuningPathInsideParallelSlot(const Slot& slot, const PathPt& start_pt,
                                                                const double radius, const int max_fine_tuning_num,
                                                                std::vector<std::vector<PathPt>>& fine_tuning_paths) {
  const double safe_distance = 0.3;
  // 判断自车ego是否在库位内，若不在库位内则返回失败
  // 判断当前库位是否是水平库位，若不是则返回失败
  if (!slot.isPointInsideSlot(start_pt) || slot.type() != Slot::SlotType::PARALLEL) {
    return false;
  }
  // 计算向前回正轨迹，如果返回false则计算相反方向轨迹
  // 循环计算，直至车辆成功回正或者连续两次无轨迹
  bool is_forward_direction = true;
  math::LineSegment2d front_boundary(slot.corner(0), slot.corner(3));
  math::LineSegment2d rear_boundary(slot.corner(1), slot.corner(2));
  math::LineSegment2d block_side_boundary(slot.corner(2), slot.corner(3));
  math::Vec2d ref_direction =
      is_forward_direction ? block_side_boundary.unit_direction() : -1.0 * block_side_boundary.unit_direction();
  math::LineSegment2d block_boundary = is_forward_direction ? front_boundary : rear_boundary;
  PathPt fine_tunning_start_pt = start_pt;
  int null_path_num = 0;

  for (int i = 0; i < max_fine_tuning_num; i++) {
    // 计算回正轨迹
    std::vector<PathPt> fine_tuning_path;
    bool align_success = calSingleFineTuningAlignPath(fine_tunning_start_pt, {block_boundary, block_side_boundary},
                                                      radius, ref_direction, 0.1, fine_tuning_path);
    if (align_success && !fine_tuning_path.empty()) {
      fine_tuning_paths.push_back(fine_tuning_path);
      return true;
    }
    if (!fine_tuning_path.empty()) {
      fine_tuning_paths.push_back(fine_tuning_path);
      fine_tunning_start_pt = fine_tuning_path.back();
    } else {
      null_path_num++;
    }
    is_forward_direction = !is_forward_direction;
    ref_direction *= -1.0;
    block_boundary = is_forward_direction ? front_boundary : rear_boundary;

    // 连续两次计算都失败
    if (null_path_num >= 2) {
      return false;
    }
  }
  return false;
}

// 向前或向后以最小转弯半径转向，目标是回正车辆。若被库位障碍边界阻挡则提前结束转向
bool GeometryPathGenerator::calSingleFineTuningAlignPath(const PathPt& start_pt,
                                                         const std::vector<math::LineSegment2d>& block_boundarys,
                                                         const double radius, const math::Vec2d& ref_direction,
                                                         const double safe_distance, std::vector<PathPt>& single_path) {
  OPENSPACE_LOG(D, "~~~~~~~~~~~~~~~~~~~ 我是分界线 ~~~~~~~~~~~~~~~~~~~ ");
  bool align_success = false;
  /*0. 判断起始位置是否被block*/
  // TODO:后续需要考虑是否是转向方向上被block
  math::Box2d veh_box(start_pt, start_pt.theta(), veh_ego_2_front_, veh_ego_2_rear_, veh_width_);
  std::vector<math::LineSegment2d> safe_block_boundarys = block_boundarys;
  for (auto& block_boundary : safe_block_boundarys) {
    if (veh_box.HasOverlap(block_boundary)) {
      OPENSPACE_LOG(W, "[GEOMETRY_PATH_GENERATOR] veh_box Block!!");
      return align_success;
    }
    math::Vec2d foot_point;
    block_boundary.GetPerpendicularFoot(start_pt, &foot_point);
    block_boundary = std::move(translateLine(block_boundary, safe_distance, (math::Vec2d)start_pt - foot_point));
  }

  /*1. 计算自车能够摆正车辆的回正位置点*/
  // 计算转向圆
  std::vector<math::Circle> circles;
  math::LineSegment2d veh_start_line = calLineThroughPointInDirection(start_pt);  // 自车起始点所在直线
  calCirclesTangentToLinePoint(veh_start_line, start_pt, radius, circles);

  // 相对参考方向ref_direction，判断回正方向
  bool is_forward_direc = ref_direction.InnerProd(veh_start_line.unit_direction()) > 0;  // true代表同向
  // 相对参考方向ref_direction，判断自车车头朝向
  bool is_left_direc = is_forward_direc
                           ? veh_start_line.unit_direction().CrossProd(ref_direction) < 0
                           : veh_start_line.unit_direction().CrossProd(ref_direction) > 0;  // true代表车头向左
  OPENSPACE_LOG(D, "[GEOMETRY_PATH_GENERATOR] is_left_direc: ", is_left_direc,
                " , is_forward_direc: ", is_forward_direc);

  // 计算转向圆（根据参考方向选择回正的转向圆
  math::Circle align_circle;
  if ((is_forward_direc && !is_left_direc)     //  向前 && 车头朝右
      || (!is_forward_direc && is_left_direc)  //  向后 && 车头朝左
  ) {
    align_circle = circles[0];  // 选择左圆
  } else {
    align_circle = circles[1];  // 选择右圆
  }
  OPENSPACE_LOG(D, "[GEOMETRY_PATH_GENERATOR] align_circle**: ", align_circle.center().x(), " ",
                align_circle.center().y(), " R: ", align_circle.radius());

  // 计算回正位置点
  PathPt align_pt;
  math::LineSegment2d auxiliary_line = calPerpendicularLineThroughPoint(
      math::LineSegment2d(math::Vec2d(0.0, 0.0), ref_direction), align_circle.center());
  std::vector<PathPt> cross_pts;
  uint8_t res_num = calCrossPointOfLineAndCircle(auxiliary_line, align_circle, cross_pts);
  align_pt = start_pt.DistanceTo(cross_pts[0]) < start_pt.DistanceTo(cross_pts[1]) ? cross_pts[0]
                                                                                   : cross_pts[1];  // 取最近的点
  OPENSPACE_LOG(D, "[GEOMETRY_PATH_GENERATOR] align_pt**: ", align_pt.x(), " ", align_pt.y());

  /*2. 计算转向的碰撞点*/
  // 确定自车碰撞校验点
  PathPt veh_coll_check_left_pt, veh_coll_check_right_pt;
  if (is_forward_direc) {
    veh_coll_check_left_pt = calOffsetPoint(start_pt, veh_ego_2_front_, veh_width_ / 2);
    veh_coll_check_right_pt = calOffsetPoint(start_pt, veh_ego_2_front_, -veh_width_ / 2);
  } else {
    veh_coll_check_left_pt = calOffsetPoint(start_pt, -veh_ego_2_rear_, veh_width_ / 2);
    veh_coll_check_right_pt = calOffsetPoint(start_pt, -veh_ego_2_rear_, -veh_width_ / 2);
  }
  OPENSPACE_LOG(D, "[GEOMETRY_PATH_GENERATOR] veh_coll_check_left_pt**: ", veh_coll_check_left_pt.x(), " ",
                veh_coll_check_left_pt.y());
  OPENSPACE_LOG(D, "[GEOMETRY_PATH_GENERATOR] veh_coll_check_right_pt**: ", veh_coll_check_right_pt.x(), " ",
                veh_coll_check_right_pt.y());

  //  计算自车碰撞校验点包络圆
  math::Circle veh_coll_check_left_circle(align_circle.center(),
                                          veh_coll_check_left_pt.DistanceTo(align_circle.center()));
  math::Circle veh_coll_check_right_circle(align_circle.center(),
                                           veh_coll_check_right_pt.DistanceTo(align_circle.center()));

  // 计算自车碰撞校验点包络圆与库位边界的安全碰撞交点
  std::vector<PathPt> left_circle_coll_pts, right_circle_coll_pts;
  for (const auto& block_boundary : safe_block_boundarys) {
    std::vector<PathPt> coll_pts;
    calCrossPointOfLineAndCircle(block_boundary, veh_coll_check_left_circle, coll_pts);
    left_circle_coll_pts.insert(left_circle_coll_pts.end(), coll_pts.begin(), coll_pts.end());
    calCrossPointOfLineAndCircle(block_boundary, veh_coll_check_right_circle, coll_pts);
    right_circle_coll_pts.insert(right_circle_coll_pts.end(), coll_pts.begin(), coll_pts.end());
  }
  sortPointsOnCircle(veh_coll_check_left_circle, veh_coll_check_left_pt, left_circle_coll_pts, is_left_direc);
  sortPointsOnCircle(veh_coll_check_right_circle, veh_coll_check_right_pt, right_circle_coll_pts, is_left_direc);

  std::vector<PathPt> align_circle_cross_pts;
  if (!left_circle_coll_pts.empty()) {
    OPENSPACE_LOG(D, "[GEOMETRY_PATH_GENERATOR] left_circle_coll_pts**: ", left_circle_coll_pts.front().x(), " ",
                  left_circle_coll_pts.front().y());
    cross_pts.clear();
    calCrossPointOfTwoCircles(math::Circle(left_circle_coll_pts.front(), start_pt.DistanceTo(veh_coll_check_left_pt)),
                              align_circle, cross_pts);
    align_circle_cross_pts.insert(align_circle_cross_pts.end(), cross_pts.begin(), cross_pts.end());
  }
  if (!right_circle_coll_pts.empty()) {
    OPENSPACE_LOG(D, "[GEOMETRY_PATH_GENERATOR] right_circle_coll_pts**: ", right_circle_coll_pts.front().x(), " ",
                  right_circle_coll_pts.front().y());
    cross_pts.clear();
    calCrossPointOfTwoCircles(math::Circle(right_circle_coll_pts.front(), start_pt.DistanceTo(veh_coll_check_right_pt)),
                              align_circle, cross_pts);
    align_circle_cross_pts.insert(align_circle_cross_pts.end(), cross_pts.begin(), cross_pts.end());
  }

  /*3. 计算回正时未碰撞的安全回正点*/
  PathPt align_safe_pt;
  align_circle_cross_pts.emplace_back(align_pt);
  sortPointsOnCircle(align_circle, start_pt, align_circle_cross_pts, is_left_direc);
  align_safe_pt = align_circle_cross_pts.front();
  align_success = align_safe_pt.DistanceTo(align_pt) < 1e-6;
  OPENSPACE_LOG(D, "[GEOMETRY_PATH_GENERATOR] align_safe_pt**: ", align_safe_pt.x(), " ", align_safe_pt.y());

  /*4. 计算回正路径*/
  // 计算回正路径
  single_path = std::move(generatePathOnCircle(align_circle, start_pt, align_safe_pt, is_left_direc, 0.2));
  for (auto& pt : single_path) {
    OPENSPACE_LOG(D, "[GEOMETRY_PATH_GENERATOR] single_path: ", pt.x(), " ", pt.y());
    pt.set_direction(is_forward_direc ? PathPt::Direction::FORWARD : PathPt::Direction::BACKWARD);
    pt.set_theta(is_forward_direc ? pt.theta() : headingReversal(pt.theta()));
    pt.set_kappa(((is_forward_direc && !is_left_direc) || (!is_forward_direc && is_left_direc))
                     ? 1.0 / (radius + 1e-6)
                     : -1.0 / (radius + 1e-6));
  }
  return align_success;
}

}  // namespace gpal::pnc::planning