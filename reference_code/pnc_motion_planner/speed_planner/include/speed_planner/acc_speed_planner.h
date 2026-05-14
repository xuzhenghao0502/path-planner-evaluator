#include "speed_planner/speed_planner_base.h"

namespace gpal::pnc::planning {

class AccSpeedPlanner : public SpeedPlannerBase {
 public:
  AccSpeedPlanner() = default;
  ~AccSpeedPlanner() = default;
  gpal::pnc::planning::Status runOnce(const LocalView& local_view, const StageState& stage_state,
                                      const shared_ptr<ReferenceLineInfo> target_reference_line_info,
                                      const shared_ptr<DecisionResult> decision_result,
                                      const shared_ptr<PathData> path_data, int64_t time_stamp,
                                      shared_ptr<SpeedResult> speed_result) override;

 private:
  void speedDataPostProcess(const LocalView& local_view, const ObstacleSet& obstacle_map,
                            const InvasionObstacle& nearest_invasion_obstacle,
                            const std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>>& x_u,
                            SpeedData* speed_data);
};

}  // namespace gpal::pnc::planning