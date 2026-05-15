#include "speed_planner/speed_planner_base.h"

namespace gpal::pnc::planning {

class ParkSpeedPlanner : public SpeedPlannerBase {
 public:
  ParkSpeedPlanner() = default;
  ~ParkSpeedPlanner() = default;
  gpal::pnc::planning::Status runOnce(const LocalView& local_view, const StageState& stage_state,
                                      const shared_ptr<ReferenceLineInfo> target_reference_line_info,
                                      const shared_ptr<DecisionResult> decision_result,
                                      const shared_ptr<PathData> path_data, int64_t time_stamp,
                                      shared_ptr<SpeedResult> speed_result) override;

 private:
  void speedDataPostProcess(const LocalView& local_view, const ObstacleSet& obstacle_map, const PathData& path_data,
                            const DecisionResult& decision_result,
                            const std::vector<std::pair<STPoint, std::string>>& local_path_lower_bound,
                            const InvasionObstacle& nearest_invasion_obstacle,
                            const std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>>& x_u,
                            SpeedData* speed_data, StopReason* stop_reason);

  void setConstParkSpeedData(const PathData& path_data, SpeedData* speed_data);
  void setEmpiricalParkSpeedData(const PathData& path_data, const InvasionObstacle& nearest_invasion_obstacle, SpeedData* speed_data);
  void decideStopBasedOnNearestObstacle(const InvasionObstacle& nearest_invasion_obstacle, SpeedData* speed_data);
  void emergencyStopBasedLocalPath(const PathData& path_data,
                                   const std::vector<std::pair<STPoint, std::string>>& local_path_lower_bound,
                                   const DecisionResult& decision_result,
                                   SpeedData* speed_data, StopReason* stop_reason);
  bool getDestinationStopFlag(const LocalView& local_view, const PathData& path_data);
  PathData::BlockFSInfo calculateBlockObsInfo(const DiscretizedPath& local_path, const DecisionResult& decision_result);
  math::Box2d getEgoBox(const PathPt& check_pt, double half_s_buffer, double half_l_buffer) const;

 private:
  double obstacle_remain_distance_ = 10000.0;  ///< 障碍物剩余距离
};

}  // namespace gpal::pnc::planning