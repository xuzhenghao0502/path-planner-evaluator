#pragma once
#include <vector>
#include <string>
#include "point/path_pt.h"
#include "path/discretized_path.h"

namespace gpal::pnc::planning {

class PathData {
 public:
  enum class StatusType : uint8_t { OK = 0, FAILED = 1 };
  enum class BlockPointDirection : uint8_t { LEFT = 0, RIGHT = 1, FRONT = 2, BACK = 3 };
  using BoundsVec3dWithId = std::pair<std::string, std::vector<Eigen::Vector3d>>;

  struct BlockFSInfo {
    bool is_valid = false;
    std::string debug_info;
  };

  const std::vector<PathPt>& discretized_path() const { return discretized_path_; }
  std::vector<PathPt>* mutableDiscretizedPath() { return &discretized_path_; }

 private:
  std::vector<PathPt> discretized_path_;
};

}  // namespace gpal::pnc::planning
