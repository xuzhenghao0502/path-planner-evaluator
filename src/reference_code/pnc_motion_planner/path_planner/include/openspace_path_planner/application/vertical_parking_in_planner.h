/**
 * @copyright Copyright(C): Geometrical-Pal, 2024-2026, All Rights Reserved
 *
 * @file    vertical_parking_in_planner.h
 * @brief   描述
 * @author  WeiTang(weitang@geometricalpal.com)
 * @version 2.0
 * @date    2025-04-17
 */
#include "base_parking_planner.h"

namespace gpal::pnc::planning {

class VerticalParkingInPlanner : public BaseParkingPlanner {
 public:
  VerticalParkingInPlanner() {
    name_ = "vertical_parking_in";
    debug_info_ = "\n\n[" + name_ + "]:\n";
  }
  explicit VerticalParkingInPlanner(const ParkingPlannerKey& key) : BaseParkingPlanner(key) {
    name_ = std::string(parkingModeToStr(key.first));
    debug_info_ = "\n\n[" + name_ + "]:\n";
  }
  ~VerticalParkingInPlanner() = default;

  bool needsFineTune(const PathPt& curr_traj_point) override;
  void generateFineTunePath(const LocalView& local_view) override;
  void slotProcess(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                   const std::vector<math::LineSegment2d>& boundary_seg) override;
  bool generateSearchElements(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                              const std::vector<math::LineSegment2d>& boundary_seg,
                              SearchElements& search_elements) override;
  void customBoundary(const LocalView& local_view, std::vector<Decision::DecisionObject>& select_ods) override;
  void geometricConnect(const LocalView& local_view, const RoiDecideResult& roi) override;
  void trajPostProcess(const LocalView& local_view, const RoiDecideResult& roi) override;
  void calVehicleSlotClearance() override;
  bool isParkingCompleted(const LocalView& local_view, const bool destination_stop_flag) override;
  void parkEnvRecognizer(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                         const std::vector<math::LineSegment2d>& boundary_seg) override;

  /**
   * @brief 将上游角点改成泊车规划内部角点顺序
   */
  void convertSlotCorners(std::vector<PathPt>& slot_corners);

 private:
  void slotCorrection(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods);
  void generateSlot(std::vector<PathPt>& slot_corners);
};

}  // namespace gpal::pnc::planning
