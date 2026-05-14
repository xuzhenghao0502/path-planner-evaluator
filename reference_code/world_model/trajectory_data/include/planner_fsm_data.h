#pragma once

namespace gpal::pnc::planning {

enum class PlannerFsmState {
    START,
    PARK_OUT,
    CRUISE,
    CRUISE_AND_PARK_IN,
    CRUISE_AND_PARK_OUT,
    PARK_IN,
    ESCAPE,
    END
};

class PlannerFsmData {
 public:
  PlannerFsmData() = default;
  ~PlannerFsmData() = default;

  PlannerFsmState getState() const { return state_; }
  PlannerFsmState getLastState() const { return last_state_; }
  void setState(PlannerFsmState state) { state_ = state; }
  void setLastState(PlannerFsmState last_state) { last_state_ = last_state; }
  double getNonStopPlanningRemainDist() const { return non_stop_planning_remain_dist_; }
  void setNonStopPlanningRemainDist(double non_stop_planning_remain_dist) {
    non_stop_planning_remain_dist_ = non_stop_planning_remain_dist;
  }

  private:
  PlannerFsmState state_ = PlannerFsmState::START;
  PlannerFsmState last_state_ = PlannerFsmState::START;
  double non_stop_planning_remain_dist_ = 10000.0;  // 非停车规划剩余距离，m
};

}