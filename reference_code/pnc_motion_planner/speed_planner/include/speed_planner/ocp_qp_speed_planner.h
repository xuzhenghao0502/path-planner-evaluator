#include "speed_planner/speed_planner_base.h"

namespace gpal::pnc::planning {

class OcpQpSpeedPlanner : public SpeedPlannerBase {
 public:
  OcpQpSpeedPlanner() = default;
  ~OcpQpSpeedPlanner() = default;
  gpal::pnc::planning::Status runOnce(const LocalView& local_view, const StageState& stage_state,
                                      const shared_ptr<ReferenceLineInfo> target_reference_line_info,
                                      const shared_ptr<DecisionResult> decision_result,
                                      const shared_ptr<PathData> path_data, int64_t time_stamp,
                                      shared_ptr<SpeedResult> speed_result) override;

 private:
  void speedDataPostProcess(const LocalView& local_view, const ObstacleSet& obstacle_map,
                            const InvasionObstacle& nearest_invasion_obstacle,
                            const std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>>& x_u,
                            SpeedData* speed_data, StopReason* stop_reason);
  void decideStopBasedOnNearestObstacle(const LocalView& local_view, const ObstacleSet& obstacle_map,
                                        const InvasionObstacle& nearest_invasion_obstacle, SpeedData* speed_data);
  double determineStopDistance(const std::string& obj_id);
  double determineStayStaticDistanceBuffer(const std::string& obj_id);
  bool getDestinationStopFlag(const LocalView& local_view, const shared_ptr<PathData>& path_data);
};

}  // namespace gpal::pnc::planning