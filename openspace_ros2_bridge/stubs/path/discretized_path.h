#pragma once
#include <vector>
#include "point/path_pt.h"

using PathPtVec = std::vector<PathPt>;

class DiscretizedPath : public PathPtVec {
 public:
  DiscretizedPath() = default;
  explicit DiscretizedPath(std::vector<PathPt> path_points) : PathPtVec(std::move(path_points)) {}

  PathPt evaluate(double s) const {
    if (empty()) return PathPt();
    if (s <= 0) return front();
    if (s >= back().s()) return back();
    for (size_t i = 1; i < size(); ++i) {
      if (s <= (*this)[i].s()) {
        double t = ((*this)[i].s() - (*this)[i-1].s());
        double r = (t > 1e-10) ? (s - (*this)[i-1].s()) / t : 0;
        double x = (*this)[i-1].x() + r * ((*this)[i].x() - (*this)[i-1].x());
        double y = (*this)[i-1].y() + r * ((*this)[i].y() - (*this)[i-1].y());
        double theta = (*this)[i-1].theta() + r * ((*this)[i].theta() - (*this)[i-1].theta());
        return PathPt(x, y, theta);
      }
    }
    return back();
  }

  PathPt evaluateReverse(double s) const { return evaluate(s); }
};

namespace gpal::pnc::planning {
using DiscretizedPath = ::DiscretizedPath;
}  // namespace gpal::pnc::planning
