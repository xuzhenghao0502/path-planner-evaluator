#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "math/box2d.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
#include "st_point.h"

namespace gpal::pnc::planning {

#define SPEED_LON_DECISION_HORIZON 200.0

class STBoundary : public math::Polygon2d {
 public:
  STBoundary() = default;

  explicit STBoundary(const std::vector<std::pair<STPoint, STPoint>>& point_pairs);

  explicit STBoundary(const math::Box2d& box) = delete;

  explicit STBoundary(std::vector<math::Vec2d> points) = delete;

  static STBoundary createInstance(const std::vector<STPoint>& lower_points, const std::vector<STPoint>& upper_points);

  static std::unique_ptr<STBoundary> createInstance(const std::vector<std::pair<STPoint, STPoint>>& point_pairs);

  ~STBoundary() = default;

  bool IsEmpty() const { return lower_points_.empty(); }
  bool IsPointInBoundary(const STPoint& st_point) const;

  STPoint upper_left_point() const;
  STPoint upper_right_point() const;
  STPoint bottom_left_point() const;
  STPoint bottom_right_point() const;
  STPoint lowest_point() const { return lowest_point_; }

  void set_upper_left_point(STPoint st_point);
  void set_upper_right_point(STPoint st_point);
  void set_bottom_left_point(STPoint st_point);
  void set_bottom_right_point(STPoint st_point);

  void set_lateral_signed_distances(const std::vector<double>& lateral_signed_distances);

  STBoundary expandByS(const double s) const;
  STBoundary expandByT(const double t) const;

  // if you need to add boundary type, make sure you modify
  // getUnblockSRange accordingly.
  enum class BoundaryType { UNKNOWN, STOP, FOLLOW, YIELD, OVERTAKE, KEEP_CLEAR, RISKY };

  static std::string typeName(BoundaryType type);

  BoundaryType boundary_type() const;
  const std::string& id() const;
  double characteristic_length() const;

  void set_id(const std::string& id);
  void setBoundaryType(const BoundaryType& boundary_type);
  void setCharacteristicLength(const double characteristic_length);

  bool getUnblockSRange(const double curr_time, double* s_upper, double* s_lower) const;

  bool getBoundarySRange(const double curr_time, double* s_upper, double* s_lower) const;

  double computeLateralSignedDistance(const double curr_time) const;

  double min_s() const;
  double min_t() const;
  double max_s() const;
  double max_t() const;

  std::vector<STPoint> upper_points() const { return upper_points_; }
  std::vector<STPoint> lower_points() const { return lower_points_; }
  std::vector<double> lateral_signed_distances() const { return lateral_signed_distances_; }

  STBoundary cutOffByT(const double t) const;

  double calcSTUpperBoundProjectedSpeed(double t, double time_window_length) const;

  double calcSTLowerBoundProjectedSpeed(double t, double time_window_length) const;

 private:
  bool isValid(const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;

  bool isPointNear(const math::LineSegment2d& seg, const math::Vec2d& point, const double max_dist);

  void removeRedundantPoints(std::vector<std::pair<STPoint, STPoint>>* point_pairs);

  bool getIndexRange(const std::vector<STPoint>& points, const double t, size_t* left, size_t* right) const;

 private:
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;
  std::vector<double> lateral_signed_distances_;

  std::string id_ = "";
  double characteristic_length_ = 1.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint lowest_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;
};

}  // namespace gpal::pnc::planning
