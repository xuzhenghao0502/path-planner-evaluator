#include "obstacle/st_boundary.h"

#include "base/log.h"
#include "math/math_utils.h"

namespace gpal::pnc::planning {

using math::LineSegment2d;
using math::Vec2d;

STBoundary::STBoundary(const std::vector<std::pair<STPoint, STPoint>>& point_pairs) {
  if (!isValid(point_pairs)) {
    ERT_PLOG_I << "The input point_pairs are NOT valid";
    assert(isValid(point_pairs));
  }

  std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);
  removeRedundantPoints(&reduced_pairs);

  for (const auto& item : reduced_pairs) {
    // use same t for both points
    const double t = item.first.t();
    lower_points_.emplace_back(item.first.s(), t);
    upper_points_.emplace_back(item.second.s(), t);
  }

  for (const auto& point : lower_points_) {
    points_.emplace_back(point.t(), point.s());
  }
  for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); ++rit) {
    points_.emplace_back(rit->t(), rit->s());
  }

  BuildFromPoints();
  double lowest_point_t = 0.0;

  for (const auto& point : lower_points_) {
    if (point.s() < min_s_) {
      lowest_point_t = point.t();
    }
    min_s_ = std::fmin(min_s_, point.s());
  }
  for (const auto& point : upper_points_) {
    max_s_ = std::fmax(max_s_, point.s());
  }
  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();

  // Compute the lowest st-point:
  lowest_point_.set_s(min_s_);
  lowest_point_.set_t(lowest_point_t);
}

bool STBoundary::isPointNear(const math::LineSegment2d& seg, const Vec2d& point, const double max_dist) {
  return seg.DistanceSquareTo(point) < max_dist * max_dist;
}

std::string STBoundary::typeName(BoundaryType type) {
  if (type == BoundaryType::FOLLOW) {
    return "FOLLOW";
  } else if (type == BoundaryType::KEEP_CLEAR) {
    return "KEEP_CLEAR";
  } else if (type == BoundaryType::OVERTAKE) {
    return "OVERTAKE";
  } else if (type == BoundaryType::STOP) {
    return "STOP";
  } else if (type == BoundaryType::YIELD) {
    return "YIELD";
  } else if (type == BoundaryType::UNKNOWN) {
    return "UNKNOWN";
  }
  ERT_PLOG_I << "Unknown boundary type " << static_cast<int>(type) << ", treated as UNKNOWN";
  return "UNKNOWN";
}

void STBoundary::removeRedundantPoints(std::vector<std::pair<STPoint, STPoint>>* point_pairs) {
  if (!point_pairs || point_pairs->size() <= 2) {
    return;
  }

  const double kMaxDist = 0.1;

  LineSegment2d seg(point_pairs->front().first, point_pairs->back().first);
  size_t middle_index = point_pairs->size() / 2;
  if (!isPointNear(seg, point_pairs->at(middle_index).first, kMaxDist)) {
    return;
  }

  size_t i = 0;
  size_t j = 1;

  while (i < point_pairs->size() && j + 1 < point_pairs->size()) {
    LineSegment2d lower_seg(point_pairs->at(i).first, point_pairs->at(j + 1).first);
    LineSegment2d upper_seg(point_pairs->at(i).second, point_pairs->at(j + 1).second);
    if (!isPointNear(lower_seg, point_pairs->at(j).first, kMaxDist) ||
        !isPointNear(upper_seg, point_pairs->at(j).second, kMaxDist)) {
      ++i;
      if (i != j) {
        point_pairs->at(i) = point_pairs->at(j);
      }
    }
    ++j;
  }
  point_pairs->at(++i) = point_pairs->back();
  point_pairs->resize(i + 1);
}

bool STBoundary::isValid(const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const {
  if (point_pairs.size() < 2) {
    ERT_PLOG_I << "point_pairs.size() must > 2. current point_pairs.size() = " << point_pairs.size();
    return false;
  }

  constexpr double kStBoundaryEpsilon = 1e-9;
  constexpr double kMinDeltaT = 1e-6;
  for (size_t i = 0; i < point_pairs.size(); ++i) {
    const auto& curr_lower = point_pairs[i].first;
    const auto& curr_upper = point_pairs[i].second;
    if (curr_upper.s() < curr_lower.s()) {
      ERT_PLOG_I << "s is not increasing";
      for (size_t j = 0; j < point_pairs.size(); ++j) {
        ERT_PLOG_I << j << ": " << point_pairs[j].first.s() << ", " << point_pairs[j].second.s();
      }
      return false;
    }

    if (std::fabs(curr_lower.t() - curr_upper.t()) > kStBoundaryEpsilon) {
      ERT_PLOG_I << "t diff is larger in each STPoint pair";
      return false;
    }

    if (i + 1 != point_pairs.size()) {
      const auto& next_lower = point_pairs[i + 1].first;
      const auto& next_upper = point_pairs[i + 1].second;
      if (std::fmax(curr_lower.t(), curr_upper.t()) + kMinDeltaT >= std::fmin(next_lower.t(), next_upper.t())) {
        ERT_PLOG_I << "t is not increasing";
        ERT_PLOG_I << " curr_lower: " << curr_lower.DebugString();
        ERT_PLOG_I << " curr_upper: " << curr_upper.DebugString();
        ERT_PLOG_I << " next_lower: " << next_lower.DebugString();
        ERT_PLOG_I << " next_upper: " << next_upper.DebugString();
        return false;
      }
    }
  }
  return true;
}

bool STBoundary::IsPointInBoundary(const STPoint& st_point) const {
  if (st_point.t() <= min_t_ || st_point.t() >= max_t_) {
    return false;
  }
  size_t left = 0;
  size_t right = 0;
  if (!getIndexRange(lower_points_, st_point.t(), &left, &right)) {
    ERT_PLOG_I << "failed to get index range.";
    return false;
  }
  const double check_upper = math::CrossProd(st_point, upper_points_[left], upper_points_[right]);
  const double check_lower = math::CrossProd(st_point, lower_points_[left], lower_points_[right]);

  return (check_upper * check_lower < 0);
}

STPoint STBoundary::upper_left_point() const {
  if (upper_points_.empty()) {
    ERT_PLOG_I << "StBoundary has zero points.";
    assert(!upper_points_.empty());
  }
  return upper_points_.front();
}

STPoint STBoundary::upper_right_point() const {
  if (upper_points_.empty()) {
    ERT_PLOG_I << "StBoundary has zero points.";
    assert(!upper_points_.empty());
  }
  return upper_points_.back();
}

STPoint STBoundary::bottom_left_point() const {
  if (lower_points_.empty()) {
    ERT_PLOG_I << "StBoundary has zero points.";
    assert(!lower_points_.empty());
  }
  return lower_points_.front();
}

STPoint STBoundary::bottom_right_point() const {
  if (lower_points_.empty()) {
    ERT_PLOG_I << "StBoundary has zero points.";
    assert(!lower_points_.empty());
  }
  return lower_points_.back();
}

STBoundary STBoundary::expandByS(const double s) const {
  if (lower_points_.empty()) {
    ERT_PLOG_I << "The current st_boundary lower_points_ has NO points.";
    return STBoundary();
  }
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(STPoint(lower_points_[i].s() - s, lower_points_[i].t()),
                             STPoint(upper_points_[i].s() + s, upper_points_[i].t()));
  }
  return STBoundary(std::move(point_pairs));
}

STBoundary STBoundary::expandByT(const double t) const {
  if (lower_points_.size() < 2) {
    ERT_PLOG_I << "The current st_boundary lower_points_.size() < 2.";
    return STBoundary();
  }
  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  const double left_delta_t = lower_points_[1].t() - lower_points_[0].t();
  const double lower_left_delta_s = lower_points_[1].s() - lower_points_[0].s();
  const double upper_left_delta_s = upper_points_[1].s() - upper_points_[0].s();

  point_pairs.emplace_back(
      STPoint(lower_points_[0].s() - t * lower_left_delta_s / left_delta_t, lower_points_[0].t() - t),
      STPoint(upper_points_[0].s() - t * upper_left_delta_s / left_delta_t, upper_points_.front().t() - t));

  const double kMinSEpsilon = 1e-3;
  point_pairs.front().first.set_s(
      std::fmin(point_pairs.front().second.s() - kMinSEpsilon, point_pairs.front().first.s()));

  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(lower_points_[i], upper_points_[i]);
  }

  size_t length = lower_points_.size();
  CHECK_GE(length, 2);

  const double right_delta_t = lower_points_[length - 1].t() - lower_points_[length - 2].t();
  const double lower_right_delta_s = lower_points_[length - 1].s() - lower_points_[length - 2].s();
  const double upper_right_delta_s = upper_points_[length - 1].s() - upper_points_[length - 2].s();

  point_pairs.emplace_back(
      STPoint(lower_points_.back().s() + t * lower_right_delta_s / right_delta_t, lower_points_.back().t() + t),
      STPoint(upper_points_.back().s() + t * upper_right_delta_s / right_delta_t, upper_points_.back().t() + t));
  point_pairs.back().second.set_s(
      std::fmax(point_pairs.back().second.s(), point_pairs.back().first.s() + kMinSEpsilon));

  return STBoundary(std::move(point_pairs));
}

STBoundary::BoundaryType STBoundary::boundary_type() const { return boundary_type_; }

void STBoundary::setBoundaryType(const BoundaryType& boundary_type) { boundary_type_ = boundary_type; }

const std::string& STBoundary::id() const { return id_; }

void STBoundary::set_id(const std::string& id) { id_ = id; }

double STBoundary::characteristic_length() const { return characteristic_length_; }

void STBoundary::setCharacteristicLength(const double characteristic_length) {
  characteristic_length_ = characteristic_length;
}

bool STBoundary::getUnblockSRange(const double curr_time, double* s_upper, double* s_lower) const {
  CHECK_NOTNULL(s_upper);
  CHECK_NOTNULL(s_lower);

  *s_upper = SPEED_LON_DECISION_HORIZON;
  *s_lower = 0.0;
  if (curr_time < min_t_ || curr_time > max_t_) {
    return true;
  }

  size_t left = 0;
  size_t right = 0;
  if (!getIndexRange(lower_points_, curr_time, &left, &right)) {
    ERT_PLOG_I << "Fail to get index range.";
    return false;
  }
  const double r = (curr_time - upper_points_[left].t()) / (upper_points_.at(right).t() - upper_points_.at(left).t());

  double upper_cross_s = upper_points_[left].s() + r * (upper_points_[right].s() - upper_points_[left].s());
  double lower_cross_s = lower_points_[left].s() + r * (lower_points_[right].s() - lower_points_[left].s());

  if (boundary_type_ == BoundaryType::STOP || boundary_type_ == BoundaryType::YIELD ||
      boundary_type_ == BoundaryType::FOLLOW) {
    *s_upper = lower_cross_s;
  } else if (boundary_type_ == BoundaryType::OVERTAKE) {
    *s_lower = std::fmax(*s_lower, upper_cross_s);
  } else {
    ERT_PLOG_I << "boundary_type is not supported. boundary_type: " << static_cast<int>(boundary_type_);
    return false;
  }
  return true;
}

bool STBoundary::getBoundarySRange(const double curr_time, double* s_upper, double* s_lower) const {
  CHECK_NOTNULL(s_upper);
  CHECK_NOTNULL(s_lower);
  if (curr_time < min_t_ || curr_time > max_t_) {
    return false;
  }

  size_t left = 0;
  size_t right = 0;
  if (!getIndexRange(lower_points_, curr_time, &left, &right)) {
    ERT_PLOG_I << "Fail to get index range.";
    return false;
  }
  if (left == right) {
    *s_upper = upper_points_[left].s();
    *s_lower = lower_points_[left].s();
    *s_upper = std::fmin(*s_upper, SPEED_LON_DECISION_HORIZON);
    *s_lower = std::fmax(*s_lower, 0.0);
  } else {
    const double r =
        (curr_time - upper_points_[left].t()) / (upper_points_[right].t() - upper_points_[left].t() + 1E-10);

    *s_upper = upper_points_[left].s() + r * (upper_points_[right].s() - upper_points_[left].s());
    *s_lower = lower_points_[left].s() + r * (lower_points_[right].s() - lower_points_[left].s());

    *s_upper = std::fmin(*s_upper, SPEED_LON_DECISION_HORIZON);
    *s_lower = std::fmax(*s_lower, 0.0);
  }
  return true;
}

double STBoundary::computeLateralSignedDistance(const double curr_time) const {
  if (curr_time < min_t_ || curr_time > max_t_) {
    ERT_PLOG_I << "st_post_process: t out of range0";
    return std::numeric_limits<double>::infinity();
  }

  size_t left = 0;
  size_t right = 0;
  if (!getIndexRange(lower_points_, curr_time, &left, &right)) {
    ERT_PLOG_I << "Fail to get index range.";
    return std::numeric_limits<double>::infinity();
  }

  if (left == right) {
    right = (left + 1UL < upper_points_.size()) ? left + 1UL : left - 1UL;
  }

  const double r = (curr_time - lower_points_[left].t()) / (lower_points_[right].t() - lower_points_[left].t());
  // ERT_PLOG_I << "st_post_process: lat_size: " << lateral_signed_distances_.size() << "  left: " << left << " right: "
  // << right ;
  return lateral_signed_distances_[left] + r * (lateral_signed_distances_[right] - lateral_signed_distances_[left]);
}

double STBoundary::min_s() const { return min_s_; }
double STBoundary::min_t() const { return min_t_; }
double STBoundary::max_s() const { return max_s_; }
double STBoundary::max_t() const { return max_t_; }

bool STBoundary::getIndexRange(const std::vector<STPoint>& points, const double t, size_t* left, size_t* right) const {
  CHECK_NOTNULL(left);
  CHECK_NOTNULL(right);
  if (t < points.front().t() || t > points.back().t()) {
    ERT_PLOG_I << "st_post_process: t is out of range. t = " << t;
    return false;
  }
  auto comp = [](const STPoint& p, const double t) { return p.t() < t; };
  auto first_ge = std::lower_bound(points.begin(), points.end(), t, comp);
  size_t index = std::distance(points.begin(), first_ge);
  if (index == 0) {
    *left = *right = 0;
  } else if (first_ge == points.end()) {
    *left = *right = points.size() - 1;
  } else {
    *left = index - 1;
    *right = index;
  }
  return true;
}

STBoundary STBoundary::createInstance(const std::vector<STPoint>& lower_points,
                                      const std::vector<STPoint>& upper_points) {
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points.size(); ++i) {
    point_pairs.emplace_back(STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
                             STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  return STBoundary(point_pairs);
}

STBoundary STBoundary::cutOffByT(const double t) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  for (size_t i = 0; i < lower_points_.size() && i < upper_points_.size(); ++i) {
    if (lower_points_[i].t() < t) {
      continue;
    }
    lower_points.push_back(lower_points_[i]);
    upper_points.push_back(upper_points_[i]);
  }
  return createInstance(lower_points, upper_points);
}

double STBoundary::calcSTUpperBoundProjectedSpeed(double t, double time_window_length) const {
  size_t left_index = 0, right_index = 0;
  if (!getIndexRange(upper_points_, t, &left_index, &right_index)) {
    return std::numeric_limits<double>::infinity();
  }

  if (upper_points_.size() < 2UL) {
    return std::numeric_limits<double>::infinity();
  }

  if (left_index == upper_points_.size() - 1UL) {
    return calcSTUpperBoundProjectedSpeed(upper_points_[left_index - 1UL].t(), time_window_length);
  } else {
    double s_upper = 0.0, s_lower = 0.0;
    getBoundarySRange(t, &s_upper, &s_lower);
    std::vector<double> t_vector;
    std::vector<double> s_vector;
    if (upper_points_[right_index].t() > t) {
      t_vector.emplace_back(t);
      s_vector.emplace_back(s_upper);
    }
    size_t i = right_index;
    while (upper_points_[i].t() <= t + time_window_length) {
      t_vector.emplace_back(upper_points_[i].t());
      s_vector.emplace_back(upper_points_[i].s());
      i++;
    }

    if (t_vector.size() <= 1UL) {
      return std::numeric_limits<double>::infinity();
    } else {
      return math::Linear1DRegression(t_vector, s_vector).first;
    }
  }
  return std::numeric_limits<double>::infinity();
}

double STBoundary::calcSTLowerBoundProjectedSpeed(double t, double time_window_length) const {
  size_t left_index = 0, right_index = 0;
  if (!getIndexRange(lower_points_, t, &left_index, &right_index)) {
    return std::numeric_limits<double>::infinity();
  }

  if (lower_points_.size() < 2UL) {
    return std::numeric_limits<double>::infinity();
  }

  if (left_index == lower_points_.size() - 1UL) {  // 左侧是最后一个点, 直接返回无穷呗.
    return calcSTLowerBoundProjectedSpeed(lower_points_[left_index - 1UL].t(), time_window_length);
  } else {
    double s_upper = 0.0, s_lower = 0.0;
    getBoundarySRange(t, &s_upper, &s_lower);
    std::vector<double> t_vector;
    std::vector<double> s_vector;
    if (lower_points_[right_index].t() > t) {
      t_vector.emplace_back(t);
      s_vector.emplace_back(s_lower);
    }
    size_t i = right_index;
    double t_instance = t;
    double s_lower_instance = 0.0, s_upper_instance = 0.0;
    while (t_instance <= std::min(t + time_window_length, lower_points_.back().t())) {
      t_vector.emplace_back(t_instance);
      getBoundarySRange(t_instance, &s_upper_instance, &s_lower_instance);
      s_vector.emplace_back(s_lower_instance);
      t_instance += 0.1;
    }

    if (t_vector.size() <= 1UL) {
      return std::numeric_limits<double>::infinity();
    } else {
      return math::Linear1DRegression(t_vector, s_vector).first;
    }
  }
  return std::numeric_limits<double>::infinity();
}

void STBoundary::set_upper_left_point(STPoint st_point) { upper_left_point_ = std::move(st_point); }

void STBoundary::set_upper_right_point(STPoint st_point) { upper_right_point_ = std::move(st_point); }

void STBoundary::set_bottom_left_point(STPoint st_point) { bottom_left_point_ = std::move(st_point); }

void STBoundary::set_bottom_right_point(STPoint st_point) { bottom_right_point_ = std::move(st_point); }

void STBoundary::set_lateral_signed_distances(const std::vector<double>& lateral_signed_distances) {
  lateral_signed_distances_ = lateral_signed_distances;
}
}  // namespace gpal::pnc::planning
