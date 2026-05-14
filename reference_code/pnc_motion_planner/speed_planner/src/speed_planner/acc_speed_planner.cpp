
#include "speed_planner/acc_speed_planner.h"

namespace gpal::pnc::planning {

  gpal::pnc::planning::Status AccSpeedPlanner::runOnce(const LocalView& local_view, const StageState& stage_state,
                                      const shared_ptr<ReferenceLineInfo> target_reference_line_info,
                                      const shared_ptr<DecisionResult> decision_result,
                                      const shared_ptr<PathData> path_data, int64_t time_stamp,
                                      shared_ptr<SpeedResult> speed_result) {
  speed_result->clear();
  ERT_PLOG_I << "[AccSpeedPlanner] process with context";

  if (path_data->local_path().empty()) {
    ERT_PLOG_I << "[AccSpeedPlanner] Error !!!!!!  local path is invalid  ";
    resetTrajectory(local_view.getLocalizationPtr(), speed_result->mutableTrajectoryResult());
    return Status(ErrorCode::PLANNING_ERROR, " lateral path is empty");
  }
  ERT_PLOG_I << "[AccSpeedPlanner] calculate init state ";

  // 0. 纵向初始u状态计算
  calcBehaviorState(stage_state, path_data->local_path(), speed_result->mutableBehaviorState());
  calcInitState(local_view);

  // 1.速度规划预处理
  // a.横向轨迹校验及预处理
  // b.障碍物预处理（？？）
  // c.限速计算（曲率限速，地图限速等）
  ERT_PLOG_I << "[AccSpeedPlanner] speed preprocessor start ";
  speed_preprocessor_ptr_->AccProcess(local_view, *path_data, *decision_result,
                                      x_0_, speed_result);
  auto lateral_path_group = speed_preprocessor_ptr_->getLateralPathResult();
  auto obstacle_map = speed_result->obstacleSet();

  if (lateral_path_group.local_path_group_.origin_path_.empty()
      || lateral_path_group.local_path_group_.extand_interval_path_.empty()) {
    ERT_PLOG_I << "[AccSpeedPlanner] Error !!!!!!  local path is invalid  ";
    resetTrajectory(local_view.getLocalizationPtr(), speed_result->mutableTrajectoryResult());
    return gpal::pnc::planning::Status::OK();
  }

  // 2.障碍物st图计算及driveboundary生成
  ERT_PLOG_I << "[AccSpeedPlanner] st graph start ";
  st_graph_processor_ptr_->processCipvStBoundary(local_view, *decision_result,
                                                lateral_path_group, time_stamp, obstacle_map,
                                                speed_result);

  // 3.求解器参数计算
  ERT_PLOG_I << "[AccSpeedPlanner] speed constraint calculate ";
  auto speed_param = speed_model_param_ptr_->calculateSpeedModelParam(
      local_view, *decision_result, obstacle_map, lateral_path_group.local_path_group_, x_0_,
      speed_result);

  // 4.优化求解：
  ERT_PLOG_I << "[AccSpeedPlanner] speed ocp qp optimizer start ";
  speed_ocp_qp_optimizer_->runOptimizer(*speed_model_param_ptr_, x_0_);
  auto x_u = speed_ocp_qp_optimizer_->getOcpSolvedResult();
  // 5.结果输出
  ERT_PLOG_I << "[AccSpeedPlanner] speed data post ";
  speedDataPostProcess(local_view, obstacle_map, speed_result->nearestInvasionObstacle(), x_u,
                       speed_result->mutable_speed_data());
  // debug
  //  for (auto& sp : frame->getSpeedResult()->speed_data()) {
  //    ERT_PLOG_I << "speed_result:  s = " << sp.s() << "  v = " << sp.v() << "  a = " << sp.a() << "   t = " << sp.t()
  //    ;
  //  }

  getTrajectory(local_view.getLocalizationPtr(), path_data->local_path(),
                speed_result->speed_data(), speed_result->mutableTrajectoryResult());

  in_first_frame_ = false;


  return gpal::pnc::planning::Status::OK();
}


void AccSpeedPlanner::speedDataPostProcess(const LocalView& local_view, const ObstacleSet& obstacle_map,
                                           const InvasionObstacle& nearest_invasion_obstacle,
                                           const std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>>& x_u,
                                           SpeedData* speed_data) {
  setOcpSpeedData(x_u, speed_data);

  ERT_PLOG_I << " >>>>>>>>>>>>>>>>>>>> nearest_front_obj_id_ =  " << nearest_invasion_obstacle.obj_id_
             << " nearest_front_obj_invasion_s_ =  " << nearest_invasion_obstacle.invasion_s_;
  //------------------------------------------------
  static int static_counter = 0;
  static bool obstacle_static = false;
  // double reset_desire_a = -1.0;
  if (nearest_invasion_obstacle.obj_id_ == "None" || !nearest_invasion_obstacle.is_obstacle_) {
    static_counter = 0;
    obstacle_static = false;
  }
  if (nearest_invasion_obstacle.obj_id_ != "None") {
    bool nearest_obstacle_static = false;
    double front_distance = nearest_invasion_obstacle.invasion_s_;
    double ego_speed = local_view.getChassisPtr()->Speed();
    // auto obs = findObstacle(nearest_front_obj_id_);
    auto iter = obstacle_map.find(nearest_invasion_obstacle.obj_id_);
    if (iter != obstacle_map.end() && nearest_invasion_obstacle.is_obstacle_) {
      auto obs = iter->second;
      if (!obstacle_static && abs(obs->speed()) <= 1.0 * KMH_MS) {
        ++static_counter;
      }
      if (abs(obs->speed()) >= 2.0 * KMH_MS) {
        static_counter--;
      }
      if (static_counter >= 3) {
        static_counter = 3;
        obstacle_static = true;
      } else if (static_counter <= 0) {
        static_counter = 0;
        obstacle_static = false;
      }
      nearest_obstacle_static = obstacle_static;
      front_distance = nearest_invasion_obstacle.invasion_s_;
    } else {
      nearest_obstacle_static = true;
      front_distance = nearest_invasion_obstacle.invasion_s_;
    }

    double stop_distance = 5.0;
    double stay_static_distance_buffer = 3.0;

    // double low_speed_thres_for_park = parameter_.low_speed_thres_for_park();
    double almost_static_speed_thres_for_park = 0.5;
    ERT_PLOG_I << "  >>>>>>>>>>>>>>>>>>>  nearest_obstacle_static = " << (int)nearest_obstacle_static;
    if ((ego_speed <= almost_static_speed_thres_for_park) && nearest_obstacle_static
        && (front_distance <= stop_distance + stay_static_distance_buffer)) {
      resetSpeedData(0.0, 0.0, speed_data);
    }
  }
}


}