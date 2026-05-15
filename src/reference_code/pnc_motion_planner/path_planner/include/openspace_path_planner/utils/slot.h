#pragma once

#include <path/path_data.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "math/polygon2d.h"

namespace gpal::pnc::planning {
/**
 * 泊车内部角点顺序
 *
 * 水平库位角点编号
 *
 *  2 +————————————————————+ 3
 *    |                    |
 *    | bottom             | top
 *    |                    |
 *  1 +————————————————————+ 0
 *
 *    >>>>>>>>>>>>>>>>>>>> 寻库方向
 *
 *  1 +————————————————————+ 0
 *    |                    |
 *    | bottom             | top
 *    |                    |
 *  2 +————————————————————+ 3
 *
 *
 * 垂直库位角点编号
 *                              寻库方向
 *  2 +————————————————————+ 3     ^     3 +————————————————————+ 2
 *    |                    |       ^       |                    |
 *    | bottom         top |       ^       | top                | bottom
 *    |                    |       ^       |                    |
 *  1 +————————————————————+ 0     ^     0 +————————————————————+ 1
 */

class Slot {
 public:
  enum class SlotType : uint8_t {
    VERTICAL = 0,  // 水平车位
    PARALLEL = 1,  // 垂直车位
    OBLIQUE = 2    // 斜列车位
  };
  enum class SlotDirec : uint8_t {
    LEFT = 0,   // 左侧车位
    RIGHT = 1,  // 右侧车位
  };

 public:
  Slot() = default;
  explicit Slot(const std::vector<PathPt>& _slot_corners, const SlotType& _slot_type = SlotType::VERTICAL,
                const SlotDirec& _slot_direc = SlotDirec::LEFT, const std::string& _slot_id = "")
      : slot_corners_(_slot_corners), slot_type_(_slot_type), slot_direc_(_slot_direc), slot_id_(_slot_id) {
    ValidateCorners();
    top_edge_center_ =
        PathPt(0.5 * (slot_corners_[3].x() + slot_corners_[0].x()), 0.5 * (slot_corners_[3].y() + slot_corners_[0].y()),
               0.5 * (slot_corners_[3].z() + slot_corners_[0].z()));
    bottom_edge_center_ =
        PathPt(0.5 * (slot_corners_[2].x() + slot_corners_[1].x()), 0.5 * (slot_corners_[2].y() + slot_corners_[1].y()),
               0.5 * (slot_corners_[2].z() + slot_corners_[1].z()));
    center_ = PathPt(0.5 * (top_edge_center_.x() + bottom_edge_center_.x()),
                     0.5 * (top_edge_center_.y() + bottom_edge_center_.y()),
                     0.5 * (top_edge_center_.z() + bottom_edge_center_.z()));
    slot_length_ = top_edge_center_.DistanceTo(bottom_edge_center_);
    slot_width_ = slot_corners_[0].DistanceTo(slot_corners_[3]);
    slot_heading_ =
        std::atan2(top_edge_center_.y() - bottom_edge_center_.y(), top_edge_center_.x() - bottom_edge_center_.x());
    slot_door_heading_ = slot_type_ == SlotType::PARALLEL ? std::atan2(slot_corners_[0].y() - slot_corners_[3].y(),
                                                                       slot_corners_[0].x() - slot_corners_[3].x())
                                                          : slot_heading_;
  }

  const std::string& id() const noexcept { return slot_id_; }
  const SlotType type() const { return slot_type_; }
  const SlotDirec direction() const { return slot_direc_; }
  const std::vector<PathPt>& corners() const noexcept { return slot_corners_; }
  const PathPt& corner(size_t index) const {
    if (index >= 4) throw std::out_of_range("角点索引越界");
    return slot_corners_[index];
  }
  double length() const noexcept { return slot_length_; }
  double width() const noexcept { return slot_width_; }
  double heading() const noexcept { return slot_heading_; }
  double doorHeading() const noexcept { return slot_door_heading_; }
  const PathPt& center() const noexcept { return center_; }
  const PathPt& topEdgeCenter() const noexcept { return top_edge_center_; }
  const PathPt& bottomEdgeCenter() const noexcept { return bottom_edge_center_; }

  std::string debugString() const;

  // 判断点是否在库位内
  bool isPointInsideSlot(const PathPt& point) const;

 private:
  std::string slot_id_ = "";
  SlotType slot_type_ = SlotType::VERTICAL;
  SlotDirec slot_direc_ = SlotDirec::LEFT;
  std::vector<PathPt> slot_corners_;
  PathPt top_edge_center_;
  PathPt bottom_edge_center_;
  PathPt center_;
  double slot_length_ = 0.0;
  double slot_width_ = 0.0;
  double slot_heading_ = 0.0;
  double slot_door_heading_ = 0.0;

 private:
  void ValidateCorners() {
    if (slot_corners_.size() != 4) {
      throw std::invalid_argument("车位必须包含4个角点");
    }
  }
};

}  // namespace gpal::pnc::planning