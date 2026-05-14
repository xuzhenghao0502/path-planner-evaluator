

#pragma once

#include <vector>

#include "speed_optimizer/speed_model_param.h"
#include "speed_common/speed_common.h"
#include "local_view/local_view.h"
#include "math/math_utils.h"
#include "path/discretized_path.h"
#include "config/speed_planner/speed_planner.pb.h"


namespace gpal::pnc::planning {

class EmpiricalSpeedPlan {
 public:
  explicit EmpiricalSpeedPlan(const double& init_s, const double& init_v, const double& init_a, const double& map_speed,
                              const double& speed_dt, const size_t& speed_size, const double& gamma, const double& K_0,
                              const double& safe_distance, const SpeedPlannerConfig& parameter)
      : init_v_(init_v),
        init_s_(init_s),
        init_a_(init_a),
        map_speed_(map_speed),
        speed_dt_(speed_dt),
        speed_size_(speed_size),
        gamma_(gamma),
        K_0_(K_0),
        safe_distance_(safe_distance),
        parameter_(parameter){};
  ~EmpiricalSpeedPlan() = default;

  /** @brief Calculate speed plan result using empirical speed planning. Often used as fallback method for qp speed
   * plan. */
 void calcEmpiricalSpeedPlanningResult(const LocalView& local_view, const DiscretizedPath& local_path,std::shared_ptr<SpeedResult> speed_result, const double& front_obj_s0,
                                                          const double& front_obj_v);

 private:
  void getSpeedLimit(const LocalView& local_view,const DiscretizedPath& local_path, vector<proto::TrajectoryPoint>& path_v_t);

  static double calcEmpiricalAcc(const double& ego_v, const double& delta_dist, const double& delta_v,
                                 const double& front_a, const double& k_0, const double& k_1, const double& k_2);
  double init_v_, init_a_, init_s_ = 0, map_speed_ = 80.0 * KMH_MS, speed_dt_ = 0.1, gamma_ = 0.3, K_0_ = 1.0,
                           safe_distance_;
  size_t speed_size_;
  SpeedPlannerConfig parameter_;
  // SpeedData* last_frame_speed_data_;
};

}  // namespace uto::planning