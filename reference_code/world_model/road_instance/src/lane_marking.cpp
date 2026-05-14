#include "road_instance/lane_marking.h"

#include <eka-rt/rt.h>

namespace gpal::pnc::planning::road_instance {

void LaneMarking::clear() {
  id_ = "";
  type_ = proto::perception::LaneMarking::kTypeUnknown;
  shape_ = proto::perception::LaneMarking::kShapeUnknown;
  color_ = proto::perception::LaneMarking::kColorUnknown;
  pts_.clear();
  reference_line_infos_.clear();
  is_inited_ = false;
  aabox_ = AABox2d();
  line_segments_.clear();
  lens_.clear();
  key_points_.clear();
}

bool LaneMarking::xy2sl(const Vec3d &xyz_point, double &s, double &l) const {
  if (line_segments_.empty()) {
    ERT_PLOG_E << "[LaneMarking::xy2sl]: line_segments_ is empty";
    return false;
  }
  s = 0.0;
  l = 0.0;
  float new_s = 0.0, new_l = 0.0;
  int min_idx = -1;
  float distance = 0.0;
  if (!getProjection(xyz_point, new_s, new_l, distance, min_idx)) {
    ERT_PLOG_E << "[LaneMarking::xy2sl]: Cannot get nearest point from path.";
    return false;
  }
  s = new_s;
  l = new_l;
  return true;
}

bool LaneMarking::getProjection(const Vec3d &point, float &accumulate_s, float &lateral, float &min_distance,
                                int &min_index) const {
  if (line_segments_.empty()) {
    return false;
  }

  min_distance = std::numeric_limits<float>::infinity();
  for (int i = 0; i < line_segments_.size(); ++i) {
    const float distance = line_segments_[i].DistanceSquareTo(point);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  if (min_index < 0) {
    return false;
  }
  min_distance = std::sqrt(min_distance);
  calcProjection(point, min_distance, min_index, accumulate_s, lateral);
  // ERT_PLOG_I << "start_index = " << start_index
  //           << "  end_index = " << end_index
  //           << "  num_line_segments_ = " << num_line_segments_
  //           << "  min_index = " << min_index
  //           << "  min_distance = " << *min_distance
  //           << "  nearest_seg.length() = " << nearest_seg.length()
  //           << "  accumulate_s = " << *accumulate_s
  //           << "  lateral = " << *lateral
  //           << "  prod = " << prod
  //           << "  proj = " << proj
  //          ;
  return true;
}

void LaneMarking::calcProjection(const Vec3d &point, const float &min_distance, const int &min_index,
                                 float &accumulate_s, float &lateral) const {
  const auto &nearest_seg = line_segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      lateral = prod;
    } else {
      lateral = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else if (min_index == line_segments_.size() - 1) {
    accumulate_s = lens_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      lateral = prod;
    } else {
      lateral = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else {
    accumulate_s = lens_[min_index] + std::max(0.0, std::min(proj, nearest_seg.length()));
    lateral = (prod > 0.0 ? 1 : -1) * min_distance;
  }
}
float LaneMarking::getLength() const {
  if (lens_.empty()) {
    return 0.0;
  }
  return lens_.back();
}

bool LaneMarking::init() {
  if (is_inited_) {
    return true;
  }
  if (pts_.size() < 2) {
    return false;
  }
  aabox_ = AABox2d(pts_.front(), pts_.back());
  lens_.resize(pts_.size());
  for (int i = 0; i < pts_.size(); ++i) {
    if (i == 0) {
      lens_[i] = 0.0;
    } else {
      lens_[i] = lens_[i - 1] + pts_[i - 1].DistanceTo(pts_[i]);
    }
  }
  for (int i = 0; i < pts_.size() - 1; ++i) {
    LineSegment2d seg(pts_[i], pts_[i + 1]);
    line_segments_.emplace_back(std::move(seg));
  }
  is_inited_ = true;
  return is_inited_;
}

double LaneMarking::DistanceSquareTo(const Vec2d &point) const {
  int idx = findNearestLineSegmentIndex(point);
  if (idx < 0) {
    return std::numeric_limits<double>::max();
  }
  return line_segments_[idx].DistanceSquareTo(point);
}

double LaneMarking::DistanceSquareTo(const Vec2d &point, double heading) const {
  int idx = findNearestLineSegmentIndex(point);
  if (idx < 0) {
    return std::numeric_limits<double>::max();
  }
  auto &nearest_seg = line_segments_[idx];
  double result = std::numeric_limits<double>::max();
  result = nearest_seg.DistanceSquareTo(point, heading);
  // ERT_PLOG_I << "[DistanceSquareTo] Road ID: " << road_id << ", point heading = " << heading
  //            << ", dist = " << result;
  return result;
}

double LaneMarking::DistanceSignedSquareTo(const Vec2d &point, double heading, bool enable_heading_search,
                                           bool is_point_as_base, double &proj_s) const {
  int idx = findNearestLineSegmentIndex(point);
  if (idx < 0) {
    return std::numeric_limits<double>::max();
  }
  auto &nearest_seg = line_segments_[idx];
  double tmp_s = 0.0;
  double result = nearest_seg.DistanceSignedSquareTo(point, heading, enable_heading_search, is_point_as_base, tmp_s);
  proj_s = tmp_s + lens_[idx];
  // ERT_PLOG_D << "[DistanceSignedSquareTo] point: " << point.DebugString() << ", point heading = " << heading
  //            << ", signed dist = " << result << ", tmp_s = " << tmp_s << ", proj_s = " << proj_s;
  return result;
}

bool LaneMarking::calcInterSection(const LineSegment2d &line_seg, BoundInfo& bound_info,
                                   double &s_in_marking, double &dist_to_line_seg_start) const {
  double tmp_s = 0.0;
  double intersection_x = 0.0;
  double intersection_y = 0.0;
  for (int i = 0; i < line_segments_.size(); ++i) {
    if (calcTwoLineSegmentInterSection(line_segments_[i], line_seg, intersection_x, intersection_y,
        tmp_s, dist_to_line_seg_start)) {
      auto& bound_pt = bound_info.bound_pt;
      bound_pt.set_x(intersection_x);
      bound_pt.set_y(intersection_y);
      bound_info.bound_type = type_;
      bound_info.bound_shape = shape_;
      bound_info.lane_marking_id = id_;
      s_in_marking = lens_[i] + tmp_s;
      return true;
    }
  }
  return false;
}

int LaneMarking::findNearestLineSegmentIndex(const Vec2d &point) const {
  if (line_segments_.empty()) {
    return -1;
  }
  double min_distance = std::numeric_limits<double>::infinity();
  int min_index = -1;
  for (int i = 0; i < line_segments_.size(); ++i) {
    const double distance = line_segments_[i].DistanceSquareTo(point);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  return min_index;
}

bool LaneMarking::calcTwoLineSegmentInterSection(const LineSegment2d &seg1, const LineSegment2d &seg2,
                                                 double& intersection_x, double& intersection_y,
                                                 double &s_in_seg1, double &dist_to_seg2_start) const {
  auto p1 = seg1.start();
  auto p2 = seg1.end();
  auto p3 = seg2.start();
  auto p4 = seg2.end();
  // 快速排斥: 两个线段为对角线组成的矩形，如果这两个矩形没有重叠的部分，那么两条线段是不可能出现重叠的
  if (!(std::min(p1.x(), p2.x()) <= std::max(p3.x(), p4.x()) && std::min(p1.y(), p2.y()) <= std::max(p3.y(), p4.y())
        && std::min(p3.x(), p4.x()) <= std::max(p1.x(), p2.x())
        && std::min(p3.y(), p4.y()) <= std::max(p1.y(), p2.y()))) {
    return false;
  }
  // 跨立实验: 如果两条线段相交，那么必须跨立，就是以一条线段为标准，另一条线段的两端点一定在这条线段的两段
  auto cross_product = [](const Vec2d &start_1, const Vec2d &end_1, const Vec2d &start_2, const Vec2d &end_2) {
    return (end_1.x() - start_1.x()) * (end_2.y() - start_2.y())
           - (end_1.y() - start_1.y()) * (end_2.x() - start_2.x());
  };

  double cross1 = cross_product(p3, p4, p3, p1);         // p3p4 X p3p1
  double cross2 = cross_product(p3, p4, p3, p2);         // p3p4 X p3p2
  double cross3 = cross_product(p1, p2, p1, p3);         // p1p2 X p1p3
  double cross4 = cross_product(p1, p2, p1, p4);         // p1p2 X p1p4
  if ((cross1 * cross2 < 0.0) && (cross3 * cross4 < 0.0)) {  // 等于0对应线段端点和交点重合的情况，忽略
    // 计算交交点坐标，利用线性方程组求解
    double t = cross1 / (cross1 - cross2);
    double u = cross3 / (cross3 - cross4);
    if ((t >= 0.0) && (t <= 1.0) && (u >= 0.0) && (u <= 1.0)) {  // 确保交点位于线段上
      intersection_x = p1.x() + t * (p2.x() - p1.x());
      intersection_y = p1.y() + t * (p2.y() - p1.y());
      s_in_seg1 = t * seg1.length();
      dist_to_seg2_start = std::hypot(intersection_x - p3.x(), intersection_y - p3.y());
      // ERT_PLOG_D << "marking seg1: " << seg1.DebugString() << ", ref seg2: " << seg2.DebugString();
      // ERT_PLOG_D << "intersection: " << intersection_x << ", " << intersection_y << ", t = " << t
      //            << ", s_in_seg1 = " << s_in_seg1
      //            << ", dist_to_seg2_start = " << dist_to_seg2_start;
      return true;
    }
  }
  return false;
}

}  // namespace gpal::pnc::planning::road_instance
