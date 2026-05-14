/**
 * @file st_point.cpp
 **/

#include "obstacle/st_point.h"

#include "base/log.h"
#include "util/string_util.h"

namespace gpal::pnc::planning {

STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const math::Vec2d& vec2d_point) : Vec2d(vec2d_point) {}

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { y_ = s; }

void STPoint::set_t(const double t) { x_ = t; }

std::string STPoint::DebugString() const { return fmt::format("{{ s : {:6f}, t : {:.6f} }}", y_, x_); }

}  // namespace gpal::pnc::planning
