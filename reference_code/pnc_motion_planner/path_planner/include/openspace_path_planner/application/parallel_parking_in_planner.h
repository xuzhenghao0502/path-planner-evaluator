/**
 * @copyright Copyright(C): Geometrical-Pal, 2024-2026, All Rights Reserved
 *
 * @file    parallel_parking_in_planner.h
 * @brief 描述
 * @author  WeiTang(weitang@geometricalpal.com)
 * @version 2.0
 * @date    2025-04-16
 */
#include "base_parking_planner.h"

namespace gpal::pnc::planning {

class ParallelParkingInPlanner : public BaseParkingPlanner {
 public:
  ParallelParkingInPlanner() {
    name_ = "parallel_parking_in";
    debug_info_ = "\n\n[" + name_ + "]:\n";
  }
  explicit ParallelParkingInPlanner(const ParkingPlannerKey& key) : BaseParkingPlanner(key) {
    name_ = std::string(parkingModeToStr(key.first));
    debug_info_ = "\n\n[" + name_ + "]:\n";
  }
  ~ParallelParkingInPlanner() = default;

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
  bool isNeedGeometricReplan() override;
  bool generateGeometricFineTunePath(const LocalView& local_view, const RoiDecideResult& roi, const double& radius = 6,
                                     const int max_shift_num = 8, const double short_path_length = 0.5,
                                     const double short_path_extend_max_length = 2.0) override;
  void parkEnvRecognizer(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                         const std::vector<math::LineSegment2d>& boundary_seg) override;

  /**
   * @brief 将上游角点改成泊车规划内部角点顺序
   */
  void convertSlotCorners(std::vector<PathPt>& slot_corners, const std::pair<bool, float>& slot_direction);

 private:
  void slotCorrection(const LocalView& local_view, const std::vector<Decision::DecisionObject>& select_ods,
                      const std::vector<math::LineSegment2d>& boundary_seg);
  void generateSlot(std::vector<PathPt>& slot_corners, const std::pair<bool, float>& slot_direction);
};

}  // namespace gpal::pnc::planning
