#include "center_lines/center_lines_process.h"

#include "base/log.h"

namespace gpal::pnc::planning {
using namespace std;

CenterLinesProcess::CenterLinesProcess() {
  left_marking_ = std::make_shared<LaneMarkingProcess>();
  right_marking_ = std::make_shared<LaneMarkingProcess>();
  left_left_marking_ = std::make_shared<LaneMarkingProcess>();
  right_right_marking_ = std::make_shared<LaneMarkingProcess>();
  lane_validate_ = std::make_shared<LaneValidate>();
  max_x_value_ = 200.0;
}

void CenterLinesProcess::CenterLinesMain(const double veh_spd, const double yaw_rate,
                                         const array<int32_t, 4> &lane_marking_ids,
                                         const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                                         vector<vector<math::Vec3d>> &lane_markings,
                                         vector<vector<math::Vec3d>> &center_lines) {
  // // 初步处理，按照左，右，左左，右右车道线的顺序填入
  // std::unordered_map<uint32_t, LaneMarking> lane_marking_map;
  // lane_marking_map.clear();
  // RoadStructureToMap(road_structure, lane_marking_map, lane_marking_ids);
  // Transform(veh_spd, yaw_rate);
  // Trim();
  std::array<bool, 4> lane_marking_valid = {false, false, false,
                                            false};  // 目前传入的是实时感知车道线，代表每条车道线是否有效，
  std::array<double, 3> lane_widths = {0.0f, 0.0f, 0.0f};
  lane_validate_->Validate(veh_spd, lane_marking_ids, lane_marking_map, lane_marking_valid, lane_widths);
  // if (veh_spd >= 0.3) {
  //     Update(veh_spd, lane_marking_valid, lane_marking_ids, lane_marking_map);
  // }
  MergeLanes(lane_marking_valid, lane_marking_ids, lane_marking_map, lane_widths, lane_markings, center_lines);
}

// void CenterLinesProcess::RoadStructureToMap(const std::shared_ptr<RoadStructure> &road_structure,
// std::unordered_map<uint32_t, LaneMarking> &lane_marking_map, std::array<uint32_t, 4> &lane_marking_ids)
// {
//     LaneMarking left_marking, right_marking, left_left_marking, right_right_marking;
//     vector<uint32_t> ego_lane_previous_segment_ids; // 本车道前驱车道段id，可能有多个，目前未用到
//     uint32_t ego_lane_id = 0;
//     uint32_t left_lane_id = 0;
//     uint32_t right_lane_id = 0;
//     uint32_t ego_lane_segment_id = 0;
//     uint32_t left_lane_segment_id = 0;
//     uint32_t right_lane_segment_id = 0;

//     ego_lane_id = road_structure->ego_lane_id;
//     ego_lane_segment_id = road_structure->ego_lane_segment_id;
//     const auto &lanes = road_structure->lanes;
//     for (size_t i = 0; i < lanes.size(); ++i) {
//         if (ego_lane_id == lanes[i].id) {
//             const auto &ego_lane = lanes[i];
//             uint32_t left_lane_id = ego_lane.left_lane_id;
//             uint32_t right_lane_id = ego_lane.right_lane_id;
//             const auto &ego_lane_segments = ego_lane.lane_segments;
//             for (size_t j = 0; j < ego_lane_segments.size(); ++j) {
//                 if (ego_lane_segment_id == ego_lane_segments[j].segment_id) {
//                     const auto &ego_lane_segment = ego_lane_segments[j];
//                     left_lane_segment_id = ego_lane_segment.left_segment_id;
//                     right_lane_segment_id = ego_lane_segment.right_segment_id;
//                     left_marking = ego_lane_segment.left_marking;
//                     right_marking = ego_lane_segment.right_marking;
//                     lane_marking_ids[0] = left_marking.id;
//                     lane_marking_ids[1] = right_marking.id;
//                 }
//             }
//         }
//     }
//     for (size_t i = 0; i < lanes.size(); ++i) {
//         if (left_lane_id == lanes[i].id) {
//             const auto &left_lane = lanes[i];
//             const auto &left_lane_segments = left_lane.lane_segments;
//             for (size_t j = 0; j < left_lane_segments.size(); ++j) {
//                 if (left_lane_segment_id == left_lane_segments[j].segment_id) {
//                     const auto &left_lane_segment = left_lane_segments[j];
//                     left_left_marking = left_lane_segment.left_marking;
//                     lane_marking_ids[2] = left_left_marking.id;
//                 }
//             }
//         } else if (right_lane_id == lanes[i].id) {
//             const auto &right_lane = lanes[i];
//             const auto &right_lane_segments = right_lane.lane_segments;
//             for (size_t j = 0; j < right_lane_segments.size(); ++j) {
//                 if (right_lane_segment_id == right_lane_segments[j].segment_id) {
//                     const auto &right_lane_segment = right_lane_segments[j];
//                     right_right_marking = right_lane_segment.right_marking;
//                     lane_marking_ids[3] = right_right_marking.id;
//                 }
//             }
//         }
//     }
//     if (lane_marking_ids[0] != 0) {
//         lane_marking_map[lane_marking_ids[0]] = left_marking;
//     }
//     if (lane_marking_ids[1] != 0) {
//         lane_marking_map[lane_marking_ids[1]] = right_marking;
//     }
//     if (lane_marking_ids[2] != 0) {
//         lane_marking_map[lane_marking_ids[2]] = left_left_marking;
//     }
//     if (lane_marking_ids[3] != 0) {
//         lane_marking_map[lane_marking_ids[3]] = right_right_marking;
//     }
// }

void CenterLinesProcess::MergeLanes(const std::array<bool, 4> &lane_marking_valid,
                                    const std::array<int32_t, 4> &lane_marking_ids,
                                    const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                                    const std::array<double, 3> lane_widths, vector<vector<math::Vec3d>> &lane_markings,
                                    vector<vector<math::Vec3d>> &center_lines) {
  std::vector<math::Vec3d> left_marking;
  left_marking.clear();
  if (lane_marking_valid[0]) {
    const auto &left_lane_marking = lane_marking_map.at(lane_marking_ids[0]);
    left_marking = left_lane_marking.line_points;
  }
  lane_markings.emplace_back(left_marking);

  std::vector<math::Vec3d> right_marking;
  right_marking.clear();
  if (lane_marking_valid[1]) {
    const auto &right_lane_marking = lane_marking_map.at(lane_marking_ids[1]);
    right_marking = right_lane_marking.line_points;
  }
  lane_markings.emplace_back(right_marking);

  std::vector<math::Vec3d> left_left_marking;
  left_left_marking.clear();
  if (lane_marking_valid[2]) {
    const auto &left_left_lane_marking = lane_marking_map.at(lane_marking_ids[2]);
    left_left_marking = left_left_lane_marking.line_points;
  }
  lane_markings.emplace_back(left_left_marking);

  std::vector<math::Vec3d> right_right_marking;
  right_right_marking.clear();
  if (lane_marking_valid[3]) {
    const auto &right_right_lane_marking = lane_marking_map.at(lane_marking_ids[3]);
    right_right_marking = right_right_lane_marking.line_points;
  }
  lane_markings.emplace_back(right_right_marking);

  CenterLinesGeneration(left_marking, right_marking, lane_widths[0], center_lines);
  CenterLinesGeneration(left_left_marking, left_marking, lane_widths[1], center_lines);
  CenterLinesGeneration(right_marking, right_right_marking, lane_widths[2], center_lines);
}

void CenterLinesProcess::CenterLinesGeneration(const vector<math::Vec3d> &left_marking,
                                               const vector<math::Vec3d> &right_marking, const double lane_width,
                                               vector<vector<math::Vec3d>> &center_lines) {
  double x_value, y_value;
  vector<math::Vec3d> center_line;
  center_line.clear();
  if ((!left_marking.empty()) && (!right_marking.empty())) {
    const math::Vec3d &left_last = left_marking.back();
    const math::Vec3d &right_last = right_marking.back();
    if ((left_last.x() > 30.0) && (right_last.x() <= 30.0)) {
      math::Vec3d point;
      for (size_t i = 0; i < left_marking.size(); ++i) {
        point.set_x(left_marking[i].x());
        point.set_y(left_marking[i].y() - 0.5 * lane_width);
        point.set_z(left_marking[i].z());
        center_line.push_back(point);
      }
    } else if ((left_last.x() <= 30.0f) && (right_last.x() > 30.0f)) {
      math::Vec3d point;
      for (size_t i = 0; i < right_marking.size(); ++i) {
        point.set_x(right_marking[i].x());
        point.set_y(right_marking[i].y() + 0.5 * lane_width);
        point.set_z(right_marking[i].z());
        center_line.push_back(point);
      }
    } else {
      uint32_t center_line_length =
          std::min(static_cast<uint32_t>(left_marking.size()), static_cast<uint32_t>(right_marking.size()));
      if (center_line_length > 0) {
        std::vector<double> right_x, right_y, right_z, lin_right_y, left_x, left_y, left_z;
        right_x.clear();
        right_y.clear();
        right_z.clear();
        lin_right_y.clear();
        left_x.clear();
        left_y.clear();
        left_z.clear();
        for (size_t i = 0; i < right_marking.size(); ++i) {
          right_x.push_back(right_marking[i].x());
          right_y.push_back(right_marking[i].y());
          right_z.push_back(right_marking[i].z());
        }
        for (size_t i = 0; i < left_marking.size(); ++i) {
          left_x.push_back(left_marking[i].x());
          left_y.push_back(left_marking[i].y());
          left_z.push_back(left_marking[i].z());
        }
        lin_right_y = LinearInterpolation::LinearInterpolation(right_x, right_y, center_line_length, left_x,
                                                               center_line_length);  // 将右车道线对其到左车道线
        right_x.assign(left_x.begin(), left_x.end());
        math::Vec3d point;
        for (int i = 0; i < center_line_length; ++i) {
          point.set_x((left_x[i] + right_x[i]) / 2.0);
          point.set_y((left_y[i] + lin_right_y[i]) / 2.0);
          point.set_z((left_z[i] + right_z[i]) / 2.0);
          center_line.push_back(point);
        }
      }
    }
  }
  center_lines.emplace_back(center_line);
}

void CenterLinesProcess::Update(const double speed, const std::array<bool, 4> &lane_marking_valid,
                                const std::array<int32_t, 4> &lane_marking_ids,
                                const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map) {
  // 左侧车道线
  bool is_curve = true;                          // 默认为弯道，后续可修改
  std::vector<math::Vec3d> lane_marking_buffer;  // 存储上游传输的散点
  lane_marking_buffer.clear();
  if (lane_marking_valid[0]) {
    const auto &left_marking = lane_marking_map.at(lane_marking_ids[0]);
    lane_marking_buffer = left_marking.line_points;
  }
  left_marking_->Update(lane_marking_buffer, speed, is_curve, max_x_value_);
  // 右侧车道线
  lane_marking_buffer.clear();
  if (lane_marking_valid[1]) {
    const auto &right_marking = lane_marking_map.at(lane_marking_ids[1]);
    lane_marking_buffer = right_marking.line_points;
  }
  right_marking_->Update(lane_marking_buffer, speed, is_curve, max_x_value_);
  // 左左侧车道线
  lane_marking_buffer.clear();
  if (lane_marking_valid[2]) {
    const auto &left_left_marking = lane_marking_map.at(lane_marking_ids[2]);
    lane_marking_buffer = left_left_marking.line_points;
  }
  left_left_marking_->Update(lane_marking_buffer, speed, is_curve, max_x_value_);
  // 右右侧车道线
  lane_marking_buffer.clear();
  if (lane_marking_valid[3]) {
    const auto &right_right_marking = lane_marking_map.at(lane_marking_ids[3]);
    lane_marking_buffer = right_right_marking.line_points;
  }
  right_right_marking_->Update(lane_marking_buffer, speed, is_curve, max_x_value_);
}

void CenterLinesProcess::Transform(const double veh_spd, const double yaw_rate) {
  ERT_PLOG_I << "be success in left transform";
  left_marking_->Transform(veh_spd, yaw_rate);
  ERT_PLOG_I << "be success in right transform";
  right_marking_->Transform(veh_spd, yaw_rate);
  ERT_PLOG_I << "be success in lleft transform";
  left_left_marking_->Transform(veh_spd, yaw_rate);
  ERT_PLOG_I << "be success in rright transform";
  right_right_marking_->Transform(veh_spd, yaw_rate);
}

void CenterLinesProcess::Trim() {
  left_marking_->TrimStart();
  right_marking_->TrimStart();
  left_left_marking_->TrimStart();
  right_right_marking_->TrimStart();
}

void CenterLinesProcess::Clear() {
  left_marking_->Clear();
  right_marking_->Clear();
  left_left_marking_->Clear();
  right_right_marking_->Clear();
}

}  // namespace gpal::pnc::planning