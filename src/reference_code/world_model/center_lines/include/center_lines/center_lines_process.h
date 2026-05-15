#pragma once

#ifndef _CENTERLINESPROCESS_H
#define _CENTERLINESPROCESS_H
#include <array>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "lane_marking_process.h"
#include "lane_validate.h"
#include "linear_interpolate.h"
#include "math/vec2d.h"
#include "road_structure.h"

namespace gpal::pnc::planning {
using namespace std;
class CenterLinesProcess {
 private:
  std::shared_ptr<LaneMarkingProcess> left_marking_ = nullptr;
  std::shared_ptr<LaneMarkingProcess> right_marking_ = nullptr;
  std::shared_ptr<LaneMarkingProcess> left_left_marking_ = nullptr;
  std::shared_ptr<LaneMarkingProcess> right_right_marking_ = nullptr;
  std::shared_ptr<LaneValidate> lane_validate_ = nullptr;
  double max_x_value_;
  void Transform(const double veh_spd, const double yaw_rate);
  void Trim();
  void Update(const double speed, const std::array<bool, 4> &lane_marking_valid,
              const std::array<int32_t, 4> &lane_marking_ids,
              const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map);
  void Clear();
  void MergeLanes(const std::array<bool, 4> &lane_marking_valid, const std::array<int32_t, 4> &lane_marking_ids,
                  const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                  const std::array<double, 3> lane_widths, vector<vector<math::Vec3d>> &lane_markings,
                  vector<vector<math::Vec3d>> &center_lines);
  void CenterLinesGeneration(const vector<math::Vec3d> &left_marking, const vector<math::Vec3d> &right_marking,
                             const double lane_width, vector<vector<math::Vec3d>> &center_lines);
  // void RoadStructureToMap(const std::shared_ptr<RoadStructure> &road_structure, std::unordered_map<uint32_t,
  // LaneMarking> &lane_marking_map, std::array<uint32_t, 4> &lane_marking_ids);
 public:
  explicit CenterLinesProcess();
  void CenterLinesMain(const double veh_spd, const double yaw_rate, const array<int32_t, 4> &lane_marking_ids,
                       const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                       vector<vector<math::Vec3d>> &lane_markings, vector<vector<math::Vec3d>> &center_lines);
};
}  // namespace gpal::pnc::planning
#endif