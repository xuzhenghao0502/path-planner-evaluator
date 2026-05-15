#include "center_lines/lane_validate.h"
namespace gpal::pnc::planning {

void LaneValidate::Validate(const double veh_spd, const std::array<int32_t, 4> &lane_marking_ids,
                            const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                            std::array<bool, 4> &lane_marking_valid, std::array<double, 3> &lane_widths) {
  lane_marking_ids_ = lane_marking_ids;
  // 车道线长度的判断
  std::array<bool, 4> is_length_valid = {};
  CheckLaneLength(veh_spd, lane_marking_map, is_length_valid);
  // 车道宽度判断
  std::array<bool, 4> is_width_valid = {};
  CheckLaneWidth(lane_marking_map, is_width_valid, lane_widths);
  // 车道线一致性判断
  std::array<bool, 4> is_consistency_valid = {};
  CheckLaneConsistency(lane_marking_map, is_consistency_valid);
  lane_marking_valid[0] = is_length_valid[0] && is_width_valid[0] && is_consistency_valid[0];
  lane_marking_valid[1] = is_length_valid[1] && is_width_valid[1] && is_consistency_valid[1];
  lane_marking_valid[2] = is_length_valid[2] && is_width_valid[2] && is_consistency_valid[2];
  lane_marking_valid[3] = is_length_valid[3] && is_width_valid[3] && is_consistency_valid[3];
}

void LaneValidate::CheckLaneLength(const double veh_spd,
                                   const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                                   std::array<bool, 4> &is_length_valid) {
  std::array<double, 2> speed_limits = {8.3, 16.7};
  std::array<double, 2> valid_vange = {20.0, 50.0};
  double min_valid_vange;
  if (veh_spd < speed_limits[0]) {
    min_valid_vange = valid_vange[0];  // 当车速小于30km/h时，最小为15m
  } else if (veh_spd > speed_limits[1]) {
    min_valid_vange = valid_vange[1];
  } else {
    min_valid_vange =
        ((veh_spd - speed_limits[0]) / (speed_limits[1] - speed_limits[0])) * (valid_vange[1] - valid_vange[0]) +
        valid_vange[0];
  }
  for (size_t i = 0; i < lane_marking_ids_.size(); ++i) {
    if (lane_marking_ids_[i] != -1) {
      const LaneMarking &lane_marking = lane_marking_map.at(lane_marking_ids_[i]);
      double min_dist = lane_marking.start_dist;
      double max_dist = lane_marking.end_dist;
      is_length_valid[i] = ((min_dist < 6.0) && ((max_dist - min_dist) > min_valid_vange));
    }
  }
}

void LaneValidate::CheckLaneWidth(const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                                  std::array<bool, 4> &is_width_valid, std::array<double, 3> &lane_widths) {
  std::array<uint32_t, 4> start_index = {};
  std::array<uint32_t, 4> end_index = {};
  std::array<double, 4> lane_offset = {};
  PointOfIndex(lane_marking_map, start_index, end_index, lane_offset);
  double ego_lane_width = fabs(lane_offset[0] - lane_offset[1]);
  double left_lane_width = fabs(lane_offset[0] - lane_offset[2]);
  double right_lane_width = fabs(lane_offset[1] - lane_offset[3]);
  lane_widths = {ego_lane_width, left_lane_width, right_lane_width};
  if ((ego_lane_width < 1.5) || (ego_lane_width > 5.5)) {
    is_width_valid[0] = false;
    is_width_valid[1] = false;
  } else {
    is_width_valid[0] = true;
    is_width_valid[1] = true;
  }
  if (left_lane_width < 1.5 || left_lane_width > 5.5) {
    is_width_valid[2] = false;
  } else {
    is_width_valid[2] = true;
  }
  if (left_lane_width < 1.5 || left_lane_width > 5.5) {
    is_width_valid[3] = false;
  } else {
    is_width_valid[3] = true;
  }
  is_width_valid = {true, true, true, true};
}

void LaneValidate::CheckLaneConsistency(const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                                        std::array<bool, 4> &is_consistency_valid) {
  is_consistency_valid = {true, true, true, true};  // 默认没有内外八
  std::array<uint32_t, 4> start_index = {};
  std::array<uint32_t, 4> end_index = {};
  std::array<double, 4> lane_offset = {};
  PointOfIndex(lane_marking_map, start_index, end_index, lane_offset);
  if ((lane_marking_ids_[0] != -1) && (lane_marking_ids_[1] != -1)) {
    const LaneMarking &left_lane_marking = lane_marking_map.at(lane_marking_ids_[0]);
    const LaneMarking &right_lane_marking = lane_marking_map.at(lane_marking_ids_[1]);
    CheckEgoLaneConsistency(start_index, end_index, left_lane_marking, right_lane_marking, is_consistency_valid);
  }
  if ((lane_marking_ids_[2] != -1) && (lane_marking_ids_[0] != -1)) {
    const LaneMarking &left_left_lane_marking = lane_marking_map.at(lane_marking_ids_[2]);
    const LaneMarking &left_lane_marking = lane_marking_map.at(lane_marking_ids_[0]);
    CheckLeftLaneConsistency(start_index, end_index, left_left_lane_marking, left_lane_marking, is_consistency_valid);
  }
  if ((lane_marking_ids_[1] != -1) && (lane_marking_ids_[3] != -1)) {
    const LaneMarking &right_lane_marking = lane_marking_map.at(lane_marking_ids_[1]);
    const LaneMarking &rright_right_lane_marking = lane_marking_map.at(lane_marking_ids_[3]);
    CheckRightLaneConsistency(start_index, end_index, right_lane_marking, rright_right_lane_marking,
                              is_consistency_valid);
  }
  is_consistency_valid = {true, true, true, true};
}

void LaneValidate::CheckEgoLaneConsistency(const std::array<uint32_t, 4> &start_index,
                                           const std::array<uint32_t, 4> &end_index,
                                           const LaneMarking &left_lane_marking, const LaneMarking &right_lane_marking,
                                           std::array<bool, 4> &is_consistency_valid) {
  std::array<uint32_t, 3> lane_marking_index;
  bool lines_consistent;
  lane_marking_index[0] = std::max(start_index[0], start_index[1]);
  lane_marking_index[2] = std::min(end_index[0], end_index[1]);
  lane_marking_index[1] = (lane_marking_index[0] + lane_marking_index[2]) / 2;  // 获取本车道开始/中间/结束时的索引
  CheckSingleLaneConsistency(left_lane_marking, right_lane_marking, lane_marking_index, lines_consistent);
  if (lines_consistent) {
    is_consistency_valid[0] = true;
    is_consistency_valid[1] = true;
  } else {
    is_consistency_valid[0] = true;
    is_consistency_valid[1] = false;
  }  // 对于本车道来说，发生内外八时相信左车道线
}

void LaneValidate::CheckLeftLaneConsistency(const std::array<uint32_t, 4> &start_index,
                                            const std::array<uint32_t, 4> &end_index,
                                            const LaneMarking &left_lane_marking, const LaneMarking &right_lane_marking,
                                            std::array<bool, 4> &is_consistency_valid) {
  std::array<uint32_t, 3> lane_marking_index;
  bool lines_consistent;
  lane_marking_index[0] = std::max(start_index[2], start_index[0]);
  lane_marking_index[2] = std::min(end_index[2], end_index[0]);
  lane_marking_index[1] = (lane_marking_index[0] + lane_marking_index[2]) / 2;  // 获取左车道开始/中间/结束时的索引
  CheckSingleLaneConsistency(left_lane_marking, right_lane_marking, lane_marking_index, lines_consistent);
  if (lines_consistent) {
    is_consistency_valid[2] = true;
    is_consistency_valid[0] = true;
  } else {
    is_consistency_valid[0] = true;
    is_consistency_valid[2] = false;
  }  // 对于左车道来说，发生内外八时相信右车道线
}

void LaneValidate::CheckRightLaneConsistency(const std::array<uint32_t, 4> &start_index,
                                             const std::array<uint32_t, 4> &end_index,
                                             const LaneMarking &left_lane_marking,
                                             const LaneMarking &right_lane_marking,
                                             std::array<bool, 4> &is_consistency_valid) {
  std::array<uint32_t, 3> lane_marking_index;
  bool lines_consistent;
  lane_marking_index[0] = std::max(start_index[2], start_index[0]);
  lane_marking_index[2] = std::min(end_index[2], end_index[0]);
  lane_marking_index[1] = (lane_marking_index[0] + lane_marking_index[2]) / 2;  // 获取左车道开始/中间/结束时的索引
  CheckSingleLaneConsistency(left_lane_marking, right_lane_marking, lane_marking_index, lines_consistent);
  if (lines_consistent) {
    is_consistency_valid[1] = true;
    is_consistency_valid[3] = true;
  } else {
    is_consistency_valid[1] = true;
    is_consistency_valid[3] = false;
  }  // 对于右车道来说，发生内外八时相信左车道线
}

void LaneValidate::PointOfIndex(const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                                std::array<uint32_t, 4> &start_index, std::array<uint32_t, 4> &end_index,
                                std::array<double, 4> &lane_offset) {
  for (size_t i = 0; i < lane_marking_ids_.size(); ++i) {
    if (lane_marking_ids_[i] != -1) {
      const LaneMarking &lane_marking = lane_marking_map.at(lane_marking_ids_[i]);
      const std::vector<math::Vec3d> &line_points = lane_marking.line_points;  // 全局坐标系下
      uint32_t &line_start_index = start_index[i];
      for (size_t j = 0; j < line_points.size(); ++j) {
        double point_x = line_points[j].x();
        if (point_x > 0.0) {
          break;
        } else {
          ++line_start_index;
        }
      }
      end_index[i] = line_points.size() - 1;
      lane_offset[i] = line_points[line_start_index].y();
    }
  }
}

void LaneValidate::CheckSingleLaneConsistency(const LaneMarking &left_lane_marking,
                                              const LaneMarking &right_lane_marking,
                                              const std::array<uint32_t, 3> &lane_marking_index,
                                              bool &lines_consistent) {
  const std::vector<math::Vec3d> &left_points = left_lane_marking.line_points;
  const std::vector<math::Vec3d> &right_points = right_lane_marking.line_points;
  if ((!left_points.empty()) && (!right_points.empty())) {
    std::vector<double> left_slopes = CubicSplineInterpolation::CalcCubicSplineSlopes(left_points);
    std::vector<double> right_slopes = CubicSplineInterpolation::CalcCubicSplineSlopes(right_points);
    double heading_left, heading_right, left_to_right;
    left_to_right = 0.0;
    for (size_t i = 0; i < lane_marking_index.size(); ++i) {
      heading_left = atan2(left_slopes[lane_marking_index[i]], 1.0);
      heading_right = atan2(right_slopes[lane_marking_index[i]], 1.0);
      left_to_right = left_to_right + std::fabs(heading_left - heading_right);
    }
    left_to_right = left_to_right / 3.0;  // 起点/中点/终点 三个点
    bool same_direction =
        (((heading_left <= 0.0) && (heading_right <= 0.0)) || ((heading_left >= 0.0) && (heading_right >= 0.0)));
    double heading_threshold = 0.01;
    bool heading_ok = std::min(std::fabs(heading_left), std::fabs(heading_right)) >= heading_threshold;
    double divergence_threshold;
    if (same_direction && heading_ok) {  // 表示进入了弯道或车辆与车道线倾角较大，可适当放宽限制
      divergence_threshold = 0.1;
    } else {
      divergence_threshold = 0.05;
    }
    lines_consistent = left_to_right < divergence_threshold;
  }
}

}  // namespace gpal::pnc::planning
