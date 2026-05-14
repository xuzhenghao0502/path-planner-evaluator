/**
 * @copyright Copyright(C): Geometrical-Pal, 2024-2026, All Rights Reserved
 *
 * @file    vertical_parking_out_planner.h
 * @brief   vertical parking out
 * @author  WeiTang(weitang@geometricalpal.com)
 * @version 2.0
 * @date    2025-06-30
 */
#include "base_parking_planner.h"

namespace gpal::pnc::planning {

class VerticalParkingOutPlanner : public BaseParkingPlanner {
 public:
  VerticalParkingOutPlanner() {
    name_ = "vertical_parking_out";
    debug_info_ = "\n\n[" + name_ + "]:\n";
  }
  explicit VerticalParkingOutPlanner(const ParkingPlannerKey& key) : BaseParkingPlanner(key) {
    name_ = std::string(parkingModeToStr(key.first));
    debug_info_ = "\n\n[" + name_ + "]:\n";
  }
  ~VerticalParkingOutPlanner() = default;

  bool generateSearchElements(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                              const std::vector<math::LineSegment2d>& boundary_seg,
                              SearchElements& search_elements) override;
  void geometricConnect(const LocalView& local_view, const RoiDecideResult& roi) override;
  void customBoundary(const LocalView& local_view, std::vector<Decision::DecisionObject>& select_ods) override;
  void trajPostProcess(const LocalView& local_view, const RoiDecideResult& roi) override;
  bool isParkingCompleted(const LocalView& local_view, const bool destination_stop_flag) override;
  void parkEnvRecognizer(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                         const std::vector<math::LineSegment2d>& boundary_seg) override;
  bool uTurnJudge(const ReferenceLine& ref, const double s, const double forward_distance_thrd,
                  const double backward_dist_thrd, double& u_turn_kappa) override;
};

}  // namespace gpal::pnc::planning
