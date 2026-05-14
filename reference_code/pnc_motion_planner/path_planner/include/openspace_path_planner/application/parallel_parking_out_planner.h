/**
 * @copyright Copyright(C): Geometrical-Pal, 2024-2026, All Rights Reserved
 *
 * @file    parallel_parking_out_planner.h
 * @brief   parallel parking out planner
 * @author  WeiTang(weitang@geometricalpal.com)
 * @version 2.0
 * @date    2025-04-16
 */
#include "base_parking_planner.h"

namespace gpal::pnc::planning {

class ParallelParkingOutPlanner : public BaseParkingPlanner {
 public:
  ParallelParkingOutPlanner() {
    name_ = "parallel_parking_out";
    debug_info_ = "\n\n[" + name_ + "]:\n";
  }
  explicit ParallelParkingOutPlanner(const ParkingPlannerKey& key) : BaseParkingPlanner(key) {
    name_ = std::string(parkingModeToStr(key.first));
    debug_info_ = "\n\n[" + name_ + "]:\n";
  }
  ~ParallelParkingOutPlanner() = default;

  bool generateSearchElements(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                              const std::vector<math::LineSegment2d>& boundary_seg,
                              SearchElements& search_elements) override;
  void customBoundary(const LocalView& local_view, std::vector<Decision::DecisionObject>& select_ods) override;
  void trajPostProcess(const LocalView& local_view, const RoiDecideResult& roi) override;
  bool isParkingCompleted(const LocalView& local_view, const bool destination_stop_flag) override;
  void parkEnvRecognizer(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                         const std::vector<math::LineSegment2d>& boundary_seg) override;
};

}  // namespace gpal::pnc::planning
