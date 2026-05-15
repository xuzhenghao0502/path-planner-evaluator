/**
 * @copyright Copyright(C): Geometrical-Pal, 2024-2026, All Rights Reserved
 *
 * @file    escape_planner.h
 * @brief   escape planner
 * @author  WeiTang(weitang@geometricalpal.com)
 * @version 2.0
 * @date    2025-06-30
 */
#include "base_parking_planner.h"

namespace gpal::pnc::planning {

class EscapePlanner : public BaseParkingPlanner {
 public:
  EscapePlanner() {
    name_ = "escape_planner";
    debug_info_ = "\n\n[" + name_ + "]:\n";
  }
  explicit EscapePlanner(const ParkingPlannerKey& key) : BaseParkingPlanner(key) {
    name_ = std::string(parkingModeToStr(key.first));
    debug_info_ = "\n\n[" + name_ + "]:\n";
  }
  ~EscapePlanner() = default;

  bool generateSearchElements(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                              const std::vector<math::LineSegment2d>& boundary_seg,
                              SearchElements& search_elements) override;
  void trajPostProcess(const LocalView& local_view, const RoiDecideResult& roi) override;
  bool isParkingCompleted(const LocalView& local_view, const bool destination_stop_flag) override;
  void parkEnvRecognizer(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                         const std::vector<math::LineSegment2d>& boundary_seg) override;
};

}  // namespace gpal::pnc::planning
