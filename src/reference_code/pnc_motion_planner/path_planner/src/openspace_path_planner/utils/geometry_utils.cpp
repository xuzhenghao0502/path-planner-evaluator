#include "openspace_path_planner/utils/geometry_utils.h"

#include <eka-rt/base/logger.h>

#include "config_manager/config_manager.h"

namespace gpal::pnc::planning {

bool isPointOnLine(const math::LineSegment2d& line, const math::Vec2d& point, double tolerance) {
  // 计算点到直线的距离
  math::Vec2d foot_point;
  double distance = line.GetPerpendicularFoot(point, &foot_point);
  // 检查距离是否小于容差
  return distance < tolerance;
}

uint8_t calCrossPointOfLineAndCircle(const math::LineSegment2d& line, const math::Circle& circle,
                                     std::vector<PathPt>& cross_pt, bool check_on_segment) {
  // 清空输出向量
  cross_pt.clear();

  // 获取圆心和半径
  const math::Vec2d& center = circle.center();
  double radius = circle.radius();

  // 计算圆心到直线的垂足和距离
  math::Vec2d foot_point;
  double distance = line.GetPerpendicularFoot(center, &foot_point);

  // 情况1：距离大于半径，没有交点
  if (distance > radius) {
    return 0;
  }

  // 情况2：距离等于半径，只有一个交点（垂足）
  if (std::abs(distance - radius) < 1e-6) {
    if (!check_on_segment || line.IsPointIn(foot_point)) {
      cross_pt.emplace_back(foot_point.x(), foot_point.y(), 0.0);
      return 1;
    }
    return 0;
  }

  // 情况3：距离小于半径，有两个交点
  double offset = std::sqrt(radius * radius - distance * distance);

  // 获取直线方向向量
  const math::Vec2d& direction = line.unit_direction();

  // 计算两个交点
  math::Vec2d point1 = foot_point + direction * offset;
  math::Vec2d point2 = foot_point - direction * offset;

  // 检查交点是否在线段上（如果需要）
  if (check_on_segment) {
    if (line.IsPointIn(point1)) {
      cross_pt.emplace_back(point1.x(), point1.y(), 0.0);
    }
    if (line.IsPointIn(point2)) {
      cross_pt.emplace_back(point2.x(), point2.y(), 0.0);
    }
  } else {
    cross_pt.emplace_back(point1.x(), point1.y(), 0.0);
    cross_pt.emplace_back(point2.x(), point2.y(), 0.0);
  }

  return cross_pt.size();
}

uint8_t calCirclesTangentToLinePoint(const math::LineSegment2d& line, const math::Vec2d& tangent_point,
                                     const double radius, std::vector<math::Circle>& circles) {
  circles.clear();

  // 检查直线段是否有效（两点不重合）
  if (line.start().DistanceTo(line.end()) < 1e-6) {
    return 0;  // 无效直线段
  }

  // 检查点是否在直线上
  if (!isPointOnLine(line, tangent_point)) {
    return 0;
  }

  // 计算直线的方向向量
  math::Vec2d direction = line.unit_direction();

  // 计算法线向量（旋转90度）
  math::Vec2d normal(-direction.y(), direction.x());

  // 第一种解：圆心在法线正方向
  math::Vec2d center1 = tangent_point + normal * radius;
  circles.emplace_back(math::Circle(center1, radius));

  // 第二种解：圆心在法线负方向
  math::Vec2d center2 = tangent_point - normal * radius;
  circles.emplace_back(math::Circle(center2, radius));

  return 2;  // 总是返回两个解
}

uint8_t calCirclesTangentToLineThroughPoint(const math::LineSegment2d& line, const math::Vec2d& point, double radius,
                                            std::vector<math::Circle>& tangent_circles) {
  // 清空输出向量
  tangent_circles.clear();

  // 检查直线段是否有效（两点不重合）
  if (line.start().DistanceTo(line.end()) < 1e-6) {
    return 0;  // 无效直线段
  }

  // 判断点是否在直线上
  if (isPointOnLine(line, point)) {
    return calCirclesTangentToLinePoint(line, point, radius, tangent_circles);
  }

  // 如果点不在直线上，计算直线平移后的新直线
  // 1. 平移直线L，距离为半径，方向朝向点P
  math::Vec2d foot_point;
  line.GetPerpendicularFoot(point, &foot_point);
  math::LineSegment2d translated_line = translateLine(line, radius, point - foot_point);

  // 2. 以点P为中心，半径为R的圆
  math::Circle point_circle(point, radius);

  // 3. 计算平移后的直线与以点P为圆心R为半径的圆的交点（即目标圆的圆心）
  std::vector<PathPt> intersections;
  uint8_t num_intersections = calCrossPointOfLineAndCircle(translated_line, point_circle, intersections, false);

  // 4. 根据交点数量创建圆
  for (uint8_t i = 0; i < num_intersections; ++i) {
    const math::Vec2d& center = math::Vec2d(intersections[i].x(), intersections[i].y());
    tangent_circles.emplace_back(center, radius);
  }

  return tangent_circles.size();
}

math::LineSegment2d translateLine(const math::LineSegment2d& line, double distance, const math::Vec2d& direction) {
  // 计算单位法向量
  math::Vec2d normal_vec = direction;
  normal_vec.Normalize();

  // 计算平移量
  math::Vec2d offset = normal_vec * distance;

  // 返回平移后的直线
  return math::LineSegment2d(line.start() + offset, line.end() + offset);
}

math::LineSegment2d calPerpendicularLineThroughPoint(const math::LineSegment2d& line, const math::Vec2d& point,
                                                     double extension_length) {
  // 1. 获取直线方向向量
  math::Vec2d line_dir = line.unit_direction();

  // 2. 计算法线方向（旋转90度得到垂线方向）
  math::Vec2d normal_dir(-line_dir.y(), line_dir.x());

  // 3. 创建以给定点为中心，沿法线方向延伸的线段
  return math::LineSegment2d(point - normal_dir * extension_length, point + normal_dir * extension_length);
}

math::LineSegment2d calLineThroughPointInDirection(const PathPt& point, double extension_length) {
  auto exten_pt = calOffsetPoint(point, extension_length);
  return math::LineSegment2d(point, exten_pt);
}

PathPt calOffsetPoint(const PathPt& point, double forward_dis, double left_dis) {
  PathPt offset_point = point;
  // 1. 计算方向单位向量
  const double cos_theta = std::cos(point.theta());
  const double sin_theta = std::sin(point.theta());
  // 2. 计算前进方向偏移
  const double forward_dx = forward_dis * cos_theta;
  const double forward_dy = forward_dis * sin_theta;

  // 3. 计算左侧方向偏移（旋转90度得到左方向）
  const double left_dx = -left_dis * sin_theta;
  const double left_dy = left_dis * cos_theta;

  // 4. 合成新坐标
  offset_point.set_x(point.x() + forward_dx + left_dx);
  offset_point.set_y(point.y() + forward_dy + left_dy);

  return offset_point;
}

uint8_t calCrossPointOfTwoCircles(const math::Circle& circle1, const math::Circle& circle2,
                                  std::vector<PathPt>& cross_pts) {
  cross_pts.clear();

  const math::Vec2d& c1 = circle1.center();
  const math::Vec2d& c2 = circle2.center();
  const double r1 = circle1.radius();
  const double r2 = circle2.radius();

  // 1. 计算圆心距离
  const double d = c1.DistanceTo(c2);

  // 情况1：同心圆或无效圆
  if (d < 1e-6) {
    return 0;  // 同心圆无交点或无限交点（视为无解）
  }

  // 情况2：相离或包含
  if (d > r1 + r2 || d < std::abs(r1 - r2)) {
    return 0;
  }

  // 情况3：相切
  if (std::abs(d - (r1 + r2)) < 1e-6 || std::abs(d - std::abs(r1 - r2)) < 1e-6) {
    const math::Vec2d tangent_pt = c1 + (c2 - c1) * (r1 / d);
    cross_pts.emplace_back(tangent_pt.x(), tangent_pt.y(), 0.0);
    return 1;
  }

  // 情况4：相交（两个交点）
  const double a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const double h = std::sqrt(r1 * r1 - a * a);

  // 计算中间点
  const math::Vec2d p0 = c1 + (c2 - c1) * (a / d);

  // 计算垂直向量
  const math::Vec2d delta = math::Vec2d(-(c2.y() - c1.y()), c2.x() - c1.x()) * (h / d);

  // 添加两个交点
  cross_pts.emplace_back(p0.x() + delta.x(), p0.y() + delta.y(), 0.0);
  cross_pts.emplace_back(p0.x() - delta.x(), p0.y() - delta.y(), 0.0);

  return 2;
}

void sortPointsOnCircle(const math::Circle& circle, const PathPt& base_point, std::vector<PathPt>& points,
                        bool clockwise) {
  // 0. 空输入检查
  if (points.empty())
    return;

  // 1. 获取圆心和半径
  const math::Vec2d& center = circle.center();
  const double radius = circle.radius();

  // 2. 计算基准点角度（从圆心指向基准点的向量角度）
  const double base_angle = std::atan2(base_point.y() - center.y(), base_point.x() - center.x());

  // 3. 定义比较函数
  auto angle_compare = [&](const PathPt& a, const PathPt& b) {
    // 计算相对于圆心的向量角度
    double angle_a = std::atan2(a.y() - center.y(), a.x() - center.x());
    double angle_b = std::atan2(b.y() - center.y(), b.x() - center.x());

    // 转换为相对于基准点的偏移角度
    angle_a -= base_angle;
    angle_b -= base_angle;

    // 规范化到[0, 2π)范围
    angle_a = math::WrapAngle(angle_a);
    angle_b = math::WrapAngle(angle_b);

    // 根据方向决定排序顺序
    return clockwise ? (angle_a > angle_b) : (angle_a < angle_b);
  };

  // 4. 原地排序
  std::sort(points.begin(), points.end(), angle_compare);
}

std::vector<PathPt> generatePathOnCircle(const math::Circle& circle, const PathPt& start_pt, const PathPt& end_pt,
                                         bool clockwise, double step_length) {
  std::vector<PathPt> path;
  // 1. 计算起点和终点的角度（相对于圆心）
  const math::Vec2d center = circle.center();
  double start_angle = math::WrapAngle(std::atan2(start_pt.y() - center.y(), start_pt.x() - center.x()));
  double end_angle = math::WrapAngle(std::atan2(end_pt.y() - center.y(), end_pt.x() - center.x()));

  // 2. 计算总弧长和需要插值的点数
  double arc_length;
  if (clockwise) {
    arc_length = (start_angle > end_angle) ? (start_angle - end_angle) * circle.radius()
                                           : (start_angle + (2 * M_PI - end_angle)) * circle.radius();
  } else {
    arc_length = (end_angle > start_angle) ? (end_angle - start_angle) * circle.radius()
                                           : (end_angle + (2 * M_PI - start_angle)) * circle.radius();
  }

  const int num_points = std::max(1, static_cast<int>(arc_length / step_length));

  step_length = arc_length / num_points;
  const double angle_step = (clockwise ? -1 : 1) * step_length / circle.radius();

  // 3. 生成轨迹点
  for (int i = 0; i <= num_points; ++i) {
    double current_angle = math::WrapAngle(start_angle + i * angle_step);

    // 计算当前点坐标
    double x = center.x() + circle.radius() * std::cos(current_angle);
    double y = center.y() + circle.radius() * std::sin(current_angle);

    // 计算朝向（切线方向）
    double theta = math::NormalizeAngle(current_angle + (clockwise ? -M_PI_2 : M_PI_2));

    path.emplace_back(x, y, 0, 0, theta);
  }
  return path;
}

}  // namespace gpal::pnc::planning