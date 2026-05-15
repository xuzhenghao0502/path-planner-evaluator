#ifndef _LANEVALIDATE_H_
#define _LANEVALIDATE_H_
#include <unordered_map>

#include "cubic_spline_interpolate.h"
#include "math/vec2d.h"
#include "road_structure.h"

namespace gpal::pnc::planning {
class LaneValidate {
 private:
  std::array<int32_t, 4> lane_marking_ids_ = {-1, -1, -1, -1};
  void CheckLaneLength(const double veh_spd, const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                       std::array<bool, 4> &is_length_valid);
  void CheckLaneWidth(const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                      std::array<bool, 4> &is_width_valid, std::array<double, 3> &lane_widths);
  void CheckLaneConsistency(const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                            std::array<bool, 4> &is_consistency_valid);
  void PointOfIndex(const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                    std::array<uint32_t, 4> &start_index, std::array<uint32_t, 4> &end_index,
                    std::array<double, 4> &lane_offset);
  void CheckSingleLaneConsistency(const LaneMarking &left_lane_marking, const LaneMarking &right_lane_marking,
                                  const std::array<uint32_t, 3> &lane_marking_index, bool &lines_consistent);
  void CheckEgoLaneConsistency(const std::array<uint32_t, 4> &start_index, const std::array<uint32_t, 4> &end_index,
                               const LaneMarking &left_lane_marking, const LaneMarking &right_lane_marking,
                               std::array<bool, 4> &is_consistency_valid);
  void CheckLeftLaneConsistency(const std::array<uint32_t, 4> &start_index, const std::array<uint32_t, 4> &end_index,
                                const LaneMarking &left_lane_marking, const LaneMarking &right_lane_marking,
                                std::array<bool, 4> &is_consistency_valid);
  void CheckRightLaneConsistency(const std::array<uint32_t, 4> &start_index, const std::array<uint32_t, 4> &end_index,
                                 const LaneMarking &left_lane_marking, const LaneMarking &right_lane_marking,
                                 std::array<bool, 4> &is_consistency_valid);

 public:
  void Validate(const double veh_spd, const std::array<int32_t, 4> &lane_marking_ids,
                const std::unordered_map<uint32_t, LaneMarking> &lane_marking_map,
                std::array<bool, 4> &lane_marking_valid, std::array<double, 3> &lane_widths);
};
}  // namespace gpal::pnc::planning
#endif  // _LANEVALIDATOR_H_