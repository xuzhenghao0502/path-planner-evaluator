#pragma once
#include <memory>
#include "path/discretized_path.h"
#include "path/path_data.h"
#include "point/path_pt.h"
#include "reference_line_info/reference_line_info.h"
#include "speed/speed_result.h"

namespace gpal::pnc::planning {

class PathPlannerBase {
 public:
  PathPlannerBase() = default;
  virtual ~PathPlannerBase() = default;
  virtual bool init() { return true; }
  virtual bool reset() { return true; }

  virtual PathData::StatusType getPath(const ReferenceLineInfo&, const DiscretizedPath&, const class LocalView&,
                                       const struct DecisionResult&, bool, const StopReason&) {
    return PathData::StatusType::OK;
  }

  static void runRefinePath(std::shared_ptr<PathData>, const PathPt&, PathData*) {}

  PathData::BlockFSInfo collisionCheck(const class Freespace&, const std::vector<PathPt>&, double, double, bool, double,
                                       double, double, double) {
    return PathData::BlockFSInfo{};
  }

 protected:
  void loadKappa(DiscretizedPath* path) {}
  void decideInCurve(const std::vector<PathPt>&, double, double, bool&, bool&) {}
  math::Box2d generateSideCheckBox(const math::Box2d& box, bool, double, double) { return box; }
  PathData::BlockPointDirection getBlockFsPointDirection(const PathPt&, const math::Vec2d&) {
    return PathData::BlockPointDirection::LEFT;
  }
};

}  // namespace gpal::pnc::planning
