#ifndef _WORLD_MODEL_LOCAL_VIEW_MEMORIZED_ROUTE_DEFINE_
#define _WORLD_MODEL_LOCAL_VIEW_MEMORIZED_ROUTE_DEFINE_

#include <vector>

#include "basic_algorithm_lib/basic_algorithm_lib.h"
#include "config/memorized_route_config.pb.h"

namespace gpal::pnc::planning {

enum class DrivingDirection : int {
  kDirectionInvalid = 0,      // 无效
  kDirectionForwardOnly = 1,  // 直行
  kDirectionLeftOnly = 2,     // 左转
  kDirectionRightOnly = 3,    // 右转
  kDirectionUTurnOnly = 4,    // 掉头
  kDirectionUnknown = 66      // 未知
};

struct SpeedLimit {
  SpeedLimit() = default;
  SpeedLimit(float _start_s, float _end_s, float _max_speed_limit, float _min_speed_limit, math::Vec3d _start_point,
             math::Vec3d _end_point)
      : start_s(_start_s),
        end_s(_end_s),
        max_speed_limit(_max_speed_limit),
        min_speed_limit(_min_speed_limit),
        start_point(_start_point),
        end_point(_end_point) {}
  float start_s = 0.0;
  float end_s = 0.0;
  float max_speed_limit = 130 * KMH_MS;  //< m/s
  float min_speed_limit = 0.0;           //< m/s
  float recommended_speed = 0.0;         //< m/s
  math::Vec3d start_point;
  math::Vec3d end_point;
};

struct SegmentDirection {
  SegmentDirection() = default;
  SegmentDirection(float _start_s, float _end_s, DrivingDirection _direction, math::Vec3d _start_point,
                   math::Vec3d _end_point)
      : start_s(_start_s), end_s(_end_s), direction(_direction), start_point(_start_point), end_point(_end_point) {}
  float start_s = 0.0;
  float end_s = 0.0;
  DrivingDirection direction = DrivingDirection::kDirectionUnknown;
  math::Vec3d start_point;
  math::Vec3d end_point;
};

class RoutePoint : public math::Vec3d {
 public:
  RoutePoint() = default;
  ~RoutePoint() = default;

  int id() const { return id_; }
  void setId(const int id) { id_ = id; }

  float s() const { return s_; }
  void setS(const float s) { s_ = s; }

  double yaw() const { return yaw_; }
  void setYaw(const double yaw) { yaw_ = yaw; }

  double pitch() const { return pitch_; }
  void setPitch(const double pitch) { pitch_ = pitch; }

  double station() const { return station_; }
  void setStation(const double station) { station_ = station; }

  double curvature() const { return curvature_; }
  void setCurvature(const double curvature) { curvature_ = curvature; }

  double max_speed() const { return max_speed_; }
  void setMaxSpeed(const double max_speed) { max_speed_ = max_speed; }

  bool isRamp() const { return is_ramp_; }
  void setIsRamp(const bool is_ramp) { is_ramp_ = is_ramp; }

  bool isElevatedRoad() const { return is_elevated_road_; }
  void setIsElevatedRoad(const bool is_elevated_road) { is_elevated_road_ = is_elevated_road; }

  bool isJunctionRoad() const { return is_junction_road_; }
  void setIsJunctionRoad(const bool is_junction_road) { is_junction_road_ = is_junction_road; }

  math::Vec3d stopLineCenter() const { return stopline_center_; }
  bool hasStopLineCenter() const { return has_stop_line_center_; }
  void setStopLineCenter(const math::Vec3d stopline_center) {
    stopline_center_ = stopline_center;
    has_stop_line_center_ = true;
  }

  float distToStopLine() const { return dist_to_stopline_; }
  void setDistToStopLine(const float dist_to_stopline) { dist_to_stopline_ = dist_to_stopline; }

  DrivingDirection drivingDirection() const { return driving_direction_; }
  void setDrivingDirection(const DrivingDirection driving_direction) { driving_direction_ = driving_direction; }

  int nextDirectionChangeId() const { return next_direction_change_id_; }
  void setNextDirectionChangeId(const int next_direction_change_id) {
    next_direction_change_id_ = next_direction_change_id;
  }

  float leftBoundaryDistance() const { return left_boundary_distance_; }
  void setLeftBoundaryDistance(const float left_boundary_distance) { left_boundary_distance_ = left_boundary_distance; }

  float rightBoundaryDistance() const { return right_boundary_distance_; }
  void setRightBoundaryDistance(const float right_boundary_distance) {
    right_boundary_distance_ = right_boundary_distance;
  }

  float leftRoadBoundaryDistance() const { return left_road_boundary_distance_; }
  void setLeftRoadBoundaryDistance(const float left_road_boundary_distance) {
    left_road_boundary_distance_ = left_road_boundary_distance;
  }

  float rightRoadBoundaryDistance() const { return right_road_boundary_distance_; }
  void setRightRoadBoundaryDistance(const float right_road_boundary_distance) {
    right_road_boundary_distance_ = right_road_boundary_distance;
  }
  
 protected:
  int id_ = -1;
  float s_ = 0.0;
  double yaw_ = 0.0;
  double pitch_ = 0.0;
  double station_ = 0.0;
  double curvature_ = 0.0;
  double max_speed_ = 0.0;
  bool is_ramp_ = false;           // 是否是匝道
  bool is_elevated_road_ = false;  // 是否是高架道路
  bool is_junction_road_ = false;
  math::Vec3d stopline_center_;
  bool has_stop_line_center_ = false;  // 最近停止线中心点位置（在记忆路线上的点）
  float dist_to_stopline_ = -1.0;      // 最近停止线距离，无效时为-1.0
  DrivingDirection driving_direction_ = DrivingDirection::kDirectionUnknown;  // 行车方向
  int next_direction_change_id_ = -1;
  float left_boundary_distance_ = 0.0;
  float right_boundary_distance_ = 0.0;
  float left_road_boundary_distance_ = 0.0;
  float right_road_boundary_distance_ = 0.0;
};

class SlicedRoute {
 public:
  SlicedRoute() = default;
  SlicedRoute(const std::vector<RoutePoint>& pts);
  SlicedRoute(
      const std::vector<RoutePoint>& pts, const uint64_t& index, const std::string& id,
      const std::shared_ptr<std::vector<RoutePoint>> stop_line_points,
      const std::shared_ptr<std::vector<SpeedLimit>> speed_limits,
      const std::shared_ptr<std::vector<SegmentDirection>> segments_direction,
      const std::shared_ptr<std::vector<std::tuple<bool, float, float, bool, bool>>> navigation_lane_change_ranges,
      const std::shared_ptr<std::vector<std::tuple<bool, float, float>>> lane_follow_ranges);
  ~SlicedRoute() = default;

  uint64_t index() const { return index_; }
  void setIndex(const uint64_t& index) { index_ = index; }

  std::string id() const { return id_; }
  void setId(const std::string& id) { id_ = id; }

  float globalStartS() const { return global_start_s_; }
  void setGlobalStartS(const float& global_start_s) { global_start_s_ = global_start_s; }

  float globalEndS() const { return global_end_s_; }
  void setGlobalEndS(const float& global_end_s) { global_end_s_ = global_end_s; }

  float length() const { return length_; }
  void setLength(const float& length) { length_ = length; }

  const std::vector<RoutePoint>& pts() const { return pts_; }
  std::vector<RoutePoint>* mutablePts() { return &pts_; }

  const std::vector<RoutePoint> stopLinePoints() const { return stop_line_points_; };
  const std::vector<SpeedLimit> speedLimits() const { return speed_limits_; };
  const std::vector<SegmentDirection> segmentsDirection() const { return segments_direction_; };

  bool isParkSlicedRoute() const { return is_park_sliced_route_; }
  void setIsParkSlicedRoute(const bool& is_park_sliced_route) { is_park_sliced_route_ = is_park_sliced_route; }

  bool isEndSlicedRoute() const { return is_end_sliced_route_; }
  void setIsEndSlicedRoute(const bool& is_end_sliced_route) { is_end_sliced_route_ = is_end_sliced_route; }

  std::vector<std::tuple<float, float, bool, bool>> navigationLaneChangeRanges() const {
    return navigation_lane_change_ranges_;
  }
  void setNavigationLaneChangeRanges(
      const std::vector<std::tuple<float, float, bool, bool>>& navigation_lane_change_ranges) {
    navigation_lane_change_ranges_ = navigation_lane_change_ranges;
  }

  std::vector<std::pair<float, float>> laneFollowRanges() const { return lane_follow_ranges_; }
  void setlaneFollowRanges(const std::vector<std::pair<float, float>>& lane_follow_ranges) {
    lane_follow_ranges_ = lane_follow_ranges;
  }

 protected:
  uint64_t index_ = 0;
  std::string id_ = "";
  float global_start_s_ = 0.0;
  float global_end_s_ = 0.0;
  float length_ = 0.0;
  std::vector<RoutePoint> pts_;
  std::vector<RoutePoint> stop_line_points_;
  std::vector<SpeedLimit> speed_limits_;              // start_s, end_s, speed_limit, start_point, end_point
  std::vector<SegmentDirection> segments_direction_;  // start_s, end_s, direction, start_point, end_point
  bool is_park_sliced_route_ = false;
  bool is_end_sliced_route_ = false;
  std::vector<std::tuple<float, float, bool, bool>>
      navigation_lane_change_ranges_;                        // start_s, end_s, is_left_lane_change, is_from_route_file
  std::vector<std::pair<float, float>> lane_follow_ranges_;  // is_valid, start_s, end_s
};

class MemorizedRoute : public StampedBase {
 public:
  MemorizedRoute();
  ~MemorizedRoute() = default;
  friend class MemorizedRouteAdapter;

  void reset();
  void slice();

  void SetValidity(const bool is_valid) { is_valid_ = is_valid; }
  bool IsValid() const { return is_valid_; }

  std::shared_ptr<std::vector<RoutePoint>> mutableMemorizedRoute() { return route_; }
  const std::shared_ptr<std::vector<RoutePoint>>& memorizedRoute() const { return route_; }

  const std::string& id() const { return id_; }
  void setId(const std::string& id) { id_ = id; }

  void setAccumulatedGlobalS(const float accumulated_global_s) { accumulated_global_s_ = accumulated_global_s; }
  float accumulatedGlobalS() const { return accumulated_global_s_; }

  const std::shared_ptr<std::vector<SlicedRoute>>& slicedRoutes() const { return sliced_routes_; }
  std::shared_ptr<std::vector<SlicedRoute>> mutableSlicedRoutes() const { return sliced_routes_; }

  float adcGlobalS() const { return adc_global_s_; }
  void setAdcGlobalS(const float& adc_global_s) { adc_global_s_ = adc_global_s; }

  float lastAdcGlobalS() const { return last_adc_global_s_; }
  void setLastAdcGlobalS(const float& last_adc_global_s) { last_adc_global_s_ = last_adc_global_s; }

  math::Vec3d* mutableRouteOriginPoint() { return &route_origin_point_; }
  const math::Vec3d& routeOriginPoint() const { return route_origin_point_; }

  math::Vec3d* mutableLTMOriginPoint() { return &ltm_origin_point_; }
  const math::Vec3d& lTMOriginPoint() const { return ltm_origin_point_; }

  std::shared_ptr<std::vector<RoutePoint>> mutableStopLinePoints() { return stop_line_points_; }
  const std::shared_ptr<std::vector<RoutePoint>>& stopLinePoints() const { return stop_line_points_; }

  std::shared_ptr<std::vector<SpeedLimit>> mutableSpeedLimits() { return speed_limits_; }
  const std::shared_ptr<std::vector<SpeedLimit>>& speedLimits() const { return speed_limits_; }

  std::shared_ptr<std::vector<SegmentDirection>> mutableSegmentsDirection() { return segments_direction_; }
  const std::shared_ptr<std::vector<SegmentDirection>>& segmentsDirection() const { return segments_direction_; }

  float getRemainDistance() const { return accumulated_global_s_ - adc_global_s_; }

  bool isParkRoute() { return is_park_route_; }
  void setIsParkRoute(const bool& is_park_route) { is_park_route_ = is_park_route; }

  std::shared_ptr<std::vector<std::tuple<bool, float, float, bool, bool>>> mutableNavigationLaneChangeRanges() {
    return navigation_lane_change_ranges_;
  }
  const std::shared_ptr<std::vector<std::tuple<bool, float, float, bool, bool>>>& navigationLaneChangeRanges() const {
    return navigation_lane_change_ranges_;
  }

  std::shared_ptr<std::vector<std::tuple<bool, float, float>>> mutableLaneFollowRanges() { return lane_follow_ranges_; }
  const std::shared_ptr<std::vector<std::tuple<bool, float, float>>>& laneFollowRanges() const {
    return lane_follow_ranges_;
  }
  bool isInLaneFollowRanges() const;

 private:
  bool is_valid_ = false;
  std::shared_ptr<std::vector<RoutePoint>> route_ = nullptr;

  std::string id_ = "memorized_route";
  float accumulated_global_s_ = 0.0;
  std::shared_ptr<std::vector<SlicedRoute>> sliced_routes_ = nullptr;
  float adc_global_s_ = -1.0;
  float last_adc_global_s_ = 0.0;
  math::Vec3d route_origin_point_;
  math::Vec3d ltm_origin_point_;
  std::shared_ptr<std::vector<RoutePoint>> stop_line_points_ = nullptr;
  std::shared_ptr<std::vector<SpeedLimit>> speed_limits_ =
      nullptr;  // start_s, end_s, speed_limit, start_point, end_point
  std::shared_ptr<std::vector<SegmentDirection>> segments_direction_ =
      nullptr;  // start_s, end_s, direction, start_point, end_point
  bool is_park_route_ = false;
  std::shared_ptr<std::vector<std::tuple<bool, float, float, bool, bool>>> navigation_lane_change_ranges_ =
      nullptr;  // is_valid, start_s, end_s, is_left_lane_change, is_from_route_file
  std::shared_ptr<std::vector<std::tuple<bool, float, float>>> lane_follow_ranges_ =
      nullptr;  // is_valid, start_s, end_s

  const double look_backward_dis_ = 100;  // 150.0;
  const double look_forward_dis_ = 300;   // 400.0;
  const double swap_dis_ = 200.0;
};

}  // namespace gpal::pnc::planning

#endif
