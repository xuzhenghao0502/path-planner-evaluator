#include "openspace_path_planner/application/base_parking_planner.h"

#include <eka-rt/base/logger.h>

#include "config_manager/config_manager.h"
namespace gpal::pnc::planning {

bool BaseParkingPlanner::init() {
  if (init_)
    return true;
  init_ = true;
  PARKING_LOG(I, "Init Start !");
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  vehicle_config_ = std::make_shared<VehicleConfig>(config_manager_->vehicle_config());
  auto parking_config = std::make_unique<ParkingPathPlannerConfig>(
      config_manager_->getConfig<ParkingPathPlannerConfig>("ParkingPathPlannerConfig"));
  auto parking_profile = ProfileManager::instance().profile<ParkingPathPlannerProfile>(
      parking_config, parkingTypeToStr(parking_planner_key_.second));  // 目前默认APA,待上游功能丰富后扩展
  planner_profile_ = ProfileManager::instance().profile<PlannerProfile>(parking_profile, name_);
  optimizer_data_ptr_ = std::make_shared<OpenspaceOptimizerData>();
  search_data_ptr_ = std::make_shared<OpenspaceSearchData>();
  geometry_path_generator_.init();
  return planner_profile_ != nullptr;
}

bool BaseParkingPlanner::reset() {
  PARKING_LOG(I, "reset !");
  end_pose_map_bak_ = std::make_pair(false, PathPt());
  start_pose_map_bak_ = std::make_pair(false, PathPt());
  park_env_type_ = ParkEnvType::INVALID;
  replan_direction_ = PathPt::Direction::FORWARD;
  is_ignore_od_and_fs_ = false;
  current_fine_tune_count_ = 0.0;
  non_stop_planning_dist_ = 10000.0;
  gate_wall_ = std::make_pair(false, math::LineSegment2d());
  clear();
  return true;
}

void BaseParkingPlanner::clear() {
  PARKING_LOG(I, "clear !");
  cal_status_ = CalStatus::DEFAULT;
  orin_combined_flu_path_.clear();
  optimizer_segmented_map_paths_.clear();
  realtime_optimizer_segmented_flu_paths_.clear();
  optimizer_segmented_flu_path_.clear();
  current_flu_path_.clear();
  park_roi_decider_.clear();
  optimizer_boundary_vec_.clear();
  planner_status_ = PlannerStatus::NORMAL;
  is_last_traj_ = false;
  remain_dis_ = 0.0;
  debug_info_ = "\n\n[" + name_ + "]:\n";
  // cal_info_ = "";
  is_tuning_ = false;
  optimizer_data_ptr_->clear();
  search_data_ptr_->clear();
  initial_slot_center_pos_ = PathPt();
  target_reference_line_info_ = ReferenceLineInfo();
  driving_discretized_path_ = DiscretizedPath();
  shift_replan_num_count_ = 0;
  forced_stop_replan_num_count_ = 0;
  slot_update_replan_num_count_ = 0;
  forced_stop_time_count_ = 0;
  forced_stop_cancel_time_count_ = 0;
  is_reoptimizing_ = false;
  slot_lateral_error_ = 0.0;
  slot_longti_error_ = 0.0;
  slot_heading_error_ = 0.0;
  corrected_slot_corner_.clear();
  corrected_slot_ = std::make_pair(false, std::vector<PathPt>());
  tf_map_2_ego_search_ = Eigen::Matrix4d::Identity();
  tf_ego_2_map_search_ = Eigen::Matrix4d::Identity();
  driving_phase_splice_path_.clear();
}

PathData::StatusType BaseParkingPlanner::getPath(const ReferenceLineInfo& target_reference_line_info,
                                                 const DiscretizedPath& driving_discretized_path,
                                                 const LocalView& local_view, const DecisionResult& decision_result,
                                                 const bool destination_stop_flag, const StopReason& stop_reason) {
  util::TimerLogger<std::milli> timer(name_, [this](const std::string& str) { PARKING_LOG(I, "getPath: ", str); });
  PARKING_LOG(D, "vehicle_type: ", vehicle_config_->vehicle_type());
  bool destination_stop = destination_stop_flag;
  target_reference_line_info_ = target_reference_line_info;

  if (planner_profile_->dynamic_search_config().enable_dynamic_search()) {
    driving_discretized_path_ = driving_discretized_path;
  }

  PARKING_LOG(I, "destination_stop_flag: ", destination_stop);
  PARKING_LOG(D, "is_ignore_od_and_fs_: ", is_ignore_od_and_fs_);
  if (planner_status_ == PlannerStatus::FINISHED) {
    return PathData::StatusType::FINISHED;
  }
  if (planner_status_ == PlannerStatus::FAILED) {
    return PathData::StatusType::FAILED;
  }
  debug_info_ = "\n\n[" + name_ + "]:\n";

  // 1. roi和搜索要素的确定：特殊场景识别、库位处理、搜索起点终点确定、障碍物边界确定
  RoiDecideResult roi;
  {
    auto roi_timer = timer.tap("1. roi");
    auto roi_res = defineROI(local_view, decision_result, roi);
    if (roi_res == UnitStatus::FAILED) {
      PARKING_LOG(E, "  Generate Roi Failed ! ");
      planner_status_ = PlannerStatus::FAILED;
      return PathData::StatusType::FAILED;
    } else if (roi_res == UnitStatus::WAITING) {
      return PathData::StatusType::RUNNING;
    }
  }

  // 2. 轨迹计算
  {
    auto cal_timer = timer.tap("2. cal");
    auto res = UnitStatus::WAITING;
    switch (cal_status_) {
      case CalStatus::DEFAULT:
        for (auto segs : roi.obstacles_linesegments) {
          for (auto seg : std::get<2>(segs)) {
            PARKING_LOG(I, "roi_boundary: start.x = ", seg.start().x(), " start.y = ", seg.start().y(),
                        " end.x = ", seg.end().x(), " end.y = ", seg.end().y());
          }
        }
        for (auto corner : slot_.slot.corners()) {
          PARKING_LOG(I, "slot_corner: corner.x = ", corner.x(), " corner.y = ", corner.y());
        }
        if (!corrected_slot_corner_.empty()) {
          // 若库位被修正，debug显示原始库位
          for (int i = 0; i < 4; i++) {
            PARKING_LOG(I, "corrected_corner: corner.x = ", local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][0],
                        " corner.y = ", local_view.getParkLotPtr()->data_.gstSpace[0].gfPos[i][1]);
          }
        }
        if (search_elements_.is_region_search) {
          for(auto pt : search_elements_.goal_region.GetAllCorners()){
            PARKING_LOG(I, "goal_region: x = ", pt.x(), " y = ", pt.y());
          }
        }

        // 记录当前库位位置
        {
          initial_slot_center_pos_ = slot_.slot.center();
          auto tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
          transfer::transformPoint(tf_ego_2_map, &initial_slot_center_pos_);
          math::Vec3d map_rpy_vec3d(0.0, 0.0, slot_.slot.heading());
          transfer::transformRPY(tf_ego_2_map, &map_rpy_vec3d);
          initial_slot_center_pos_.set_theta(map_rpy_vec3d.z());
          cal_info_ = debug_info_;
          corrected_slot_ = std::make_pair(!corrected_slot_corner_.empty(), corrected_slot_corner_);
        }

        search_data_ptr_->clear();
        manager_.Clear();
        cal_status_ = CalStatus::SEARCH;
        tf_map_2_ego_search_ = local_view.getLocalizationPtr()->getTfMap2Ego();
        tf_ego_2_map_search_ = local_view.getLocalizationPtr()->getTfEgo2Map();
      case CalStatus::SEARCH:  // 粗轨迹搜索
      {
        auto search_timer = cal_timer.tap("2.1 search");
        res = originalPathSearch(local_view, roi);
      }
        if (res == UnitStatus::FAILED) {
          // 重规划失败，恢复上次轨迹
          if (planner_status_ == PlannerStatus::REPLAN) {
            optimizer_segmented_map_paths_ = optimizer_segmented_map_paths_backup_;
            cal_status_ = CalStatus::FINISH;
            break;
          }
          if (planner_status_ == PlannerStatus::NORMAL) {
            clear();
            is_ignore_od_and_fs_ = true;
            return PathData::StatusType::RUNNING;
          }

          planner_status_ = PlannerStatus::FAILED;
          return PathData::StatusType::FAILED;
        } else if (res == UnitStatus::WAITING) {
          return PathData::StatusType::RUNNING;
        }

        cal_status_ = CalStatus::GEO_CONNECT;
      case CalStatus::GEO_CONNECT:  // 轨迹的几何拼接
      {
        auto geo_connect_timer = cal_timer.tap("2.2 geo_connect");
        geometricConnect(local_view, roi);
        // 转到搜索时刻debug打印
        auto orin_combined_flu_path_search_timestamp = orin_combined_flu_path_;
        trajectoryTimeCompensate(local_view.getLocalizationPtr()->getTfEgo2Map(), tf_map_2_ego_search_,
                                 orin_combined_flu_path_search_timestamp);
        debugPrintTraj(orin_combined_flu_path_search_timestamp, "orin_combined_flu_path_");
      }

        optimizer_data_ptr_->clear();
        manager_.Clear();
        cal_status_ = CalStatus::OPTIMIZE;
      case CalStatus::OPTIMIZE:  // 轨迹平滑优化
      {
        auto ocp_timer = cal_timer.tap("2.3 ocp");
        res = optimizer(local_view, decision_result, roi);
      }
        if (res == UnitStatus::FAILED) {
          // 为REPLAN状态增加失败处理
          if (planner_status_ == PlannerStatus::REPLAN) {
            optimizer_segmented_map_paths_ = optimizer_segmented_map_paths_backup_;
            planner_status_ = PlannerStatus::NORMAL;  // 恢复到正常状态
            cal_status_ = CalStatus::FINISH;
            break;
          }
          planner_status_ = PlannerStatus::FAILED;
          return PathData::StatusType::FAILED;
        } else if (res == UnitStatus::WAITING) {
          return PathData::StatusType::RUNNING;
        }

        cal_status_ = CalStatus::POST_PROCESS;
      case CalStatus::POST_PROCESS:  // 轨迹后处理
      {
        auto post_process_timer = cal_timer.tap("2.4 post_process");
        trajPostProcess(local_view, roi);

        // 转到搜索时刻debug打印
        auto optimizer_segmented_flu_path_search_timestamp = optimizer_segmented_flu_path_;
        for (auto& path : optimizer_segmented_flu_path_search_timestamp) {
          trajectoryTimeCompensate(local_view.getLocalizationPtr()->getTfEgo2Map(), tf_map_2_ego_search_, path);
        }
        debugPrintTraj(optimizer_segmented_flu_path_search_timestamp, "tractor_flu_path");

        // 将结果轨迹转换到map
        optimizer_segmented_map_paths_ = std::move(local2GlobalPaths(local_view, optimizer_segmented_flu_path_));
      }
        cal_info_ += "finished: 1 traj_segs: " + std::to_string(optimizer_segmented_map_paths_.size()) + "\n";
        PARKING_LOG(D, "\n" + cal_info_);
        cal_status_ = CalStatus::FINISH;
        break;
      default:
        cal_status_ = CalStatus::FINISH;
        break;
    }
  }

  // 3. 重规划
  {
    auto replan_timer = timer.tap("3. replan");
    if (isNeedReplan(local_view, destination_stop, stop_reason)) {
      // 选择重规划的方式
      if (isNeedGeometricReplan()
          && generateGeometricFineTunePath(
              local_view, roi, planner_profile_->parking_replan_config().geometric_fine_tune_config().radius(),
              planner_profile_->parking_replan_config().geometric_fine_tune_config().max_shift_num(),
              planner_profile_->parking_replan_config().geometric_fine_tune_config().short_path_length(),
              planner_profile_->parking_replan_config().geometric_fine_tune_config().short_path_extend_max_length())) {
        PARKING_LOG(I, "  产生几何揉库方式");
        corrected_slot_ = std::make_pair(!corrected_slot_corner_.empty(), corrected_slot_corner_);
        destination_stop = false;
      } else {
        int shift_replan_num_count_backup = shift_replan_num_count_;
        int forced_stop_replan_num_count_backup = forced_stop_replan_num_count_;
        int slot_update_replan_num_count_backup = slot_update_replan_num_count_;
        clear();
        shift_replan_num_count_ = shift_replan_num_count_backup;
        forced_stop_replan_num_count_ = forced_stop_replan_num_count_backup;
        slot_update_replan_num_count_ = slot_update_replan_num_count_backup;
        planner_status_ = PlannerStatus::REPLAN;
        return PathData::StatusType::RUNNING;
      }
    }
  }

  // 产品需求：换挡次数过多，判定为泊车失败（从首次切入R档开始计数）
  int shif_nums = optimizer_segmented_map_paths_.size();
  if (!optimizer_segmented_map_paths_.empty() && !optimizer_segmented_map_paths_.front().empty()
      && optimizer_segmented_map_paths_.front().front().direction() == PathPt::Direction::FORWARD) {
    shif_nums -= 1;
  }
  if (shif_nums > planner_profile_->parking_failure_indicators().parking_max_gear_shift_times()) {
    planner_status_ = PlannerStatus::FAILED;
    PARKING_LOG(W, "泊车轨迹换挡次数过多: ", shif_nums, " > ",
                planner_profile_->parking_failure_indicators().parking_max_gear_shift_times());
    return PathData::StatusType::FAILED;
  }

  // 4. 处理轨迹换挡切换、切换前轨迹重优化
  {
    auto handle_shift_timer = timer.tap("4. shift_reoptimize");
    if (handleShiftAndReoptimization(local_view, decision_result, destination_stop)) {
      return PathData::StatusType::RUNNING;
    }
  }
  // ================== 新增：Last Segment 实时滚动优化 ==================
  // 1. 必须是最后一段轨迹 (size == 1)
  // 2. 且未处于微调模式 (!is_tuning_)
  // 3. 且开启了相关配置
  OPENSPACE_LOG(I, "is_tuning_ = ", is_tuning_,
                "enable_realtime_refine = ", planner_profile_->enable_realtime_refine());
  if (optimizer_segmented_map_paths_.size() == 1 && !is_tuning_ && planner_profile_->enable_realtime_refine()) {
    // 内部会判断 remain_dis_ > 3.0
    handleRealTimeRefinement(local_view, decision_result);
  }
  PARKING_LOG(I, "  当前有 ", optimizer_segmented_map_paths_.size(), " 段轨迹");

  // 5. 基于实时车辆map位置，转换轨迹在自车FLU下
  {
    auto map_2_flu_timer = timer.tap("5. map_2_flu");
    realtime_optimizer_segmented_flu_paths_ = std::move(global2LocalPaths(local_view, optimizer_segmented_map_paths_));
  }

  // 6. 当前轨迹裁减、剩余距离计算、最终轨迹生成
  {
    auto traj_result_timer = timer.tap("6. traj_result");
    selectAndClipCurrentTrajectory(local_view);
  }

  // 7. 库位稳定性、轨迹跟踪稳定性计算，供debug
  {
    auto stability_timer = timer.tap("7. stability");
    calculateSlotAndTrackingStability(local_view);
  }

  debug_info_ += fmt::format("replan num: shift: {}, forced_stop: {}, slot_update: {}\n", shift_replan_num_count_,
                             forced_stop_replan_num_count_, slot_update_replan_num_count_);
  debug_info_ += fmt::format("park env type: {}\n", (uint8_t)park_env_type_);

  // 8. 轨迹微调
  {
    auto fine_tune_timer = timer.tap("8. fine_tune");
    if (planner_profile_->fine_tune_config().enable() && !current_flu_path_.empty()
        && needsFineTune(current_flu_path_.front())) {
      PARKING_LOG(I, "[ParkPathPlanner]park fine_tune!");
      is_tuning_ = true;
      generateFineTunePath(local_view);
    }
    if (is_tuning_) {
      debug_info_ += fmt::format("fine tuning: {}\n", current_fine_tune_count_);
    }
    PARKING_LOG(D, "[ParkPathPlanner]当前轨迹段数 = ", optimizer_segmented_map_paths_.size());
  }

  // 9. 泊车完成判断
  {
    auto park_finish_check_timer = timer.tap("9. park_finish_check");
    if (isParkingCompleted(local_view, destination_stop)) {
      if (planner_status_ == PlannerStatus::HOLD_ON) {
        return PathData::StatusType::HOLD_ON;
      }
      return PathData::StatusType::FINISHED;
    }
  }
  return PathData::StatusType::RUNNING;
}

BaseParkingPlanner::UnitStatus BaseParkingPlanner::defineROI(const LocalView& local_view,
                                                             const DecisionResult& decision_result,
                                                             RoiDecideResult& roi) {
  park_roi_decider_.clear();
  // 0. 选择感兴趣类型障碍物
  auto seclect_ods = obstacleSelector(local_view, decision_result);

  // 1. 获取道路边界
  std::vector<math::LineSegment2d> boundary_seg;
  mapBoundary(local_view, boundary_seg);

  // 2. 特殊泊车场景识别
  parkEnvRecognizer(local_view, seclect_ods, boundary_seg);

  // 3. 库位处理
  slotProcess(local_view, seclect_ods, boundary_seg);

  // 4. 确定搜索起点和终点
  if (!generateSearchElements(local_view, seclect_ods, boundary_seg, search_elements_))
    return UnitStatus::FAILED;

  // 忽略障碍物
  if (is_ignore_od_and_fs_) {
    seclect_ods.clear();
  }

  // 5. 自定义约束border
  customBoundary(local_view, seclect_ods);

  // 6. 接入上游决策OD，产生最终ROI范围
  park_roi_decider_.setBoundary(search_elements_.search_boundary);
  if (!park_roi_decider_.generateRoi(local_view, seclect_ods))
    return UnitStatus::FAILED;
  park_roi_decider_.getRoiDecideResuilt(roi);

  return UnitStatus::SUCCEEDED;
}

BaseParkingPlanner::UnitStatus BaseParkingPlanner::optimizer(const LocalView& local_view,
                                                             const DecisionResult& decision_result,
                                                             const RoiDecideResult& roi) {
  PARKING_LOG(D, "Enter optimizer !!!!");
  const std::string optimizer_name = "ipm_ocp";
  if (optimizer_data_ptr_->model_name_ != "parking_general") {
    optimizer_data_ptr_->model_name_ = "parking_general";
    PARKING_LOG(D, "[ParkPathPlanner] change optimizer model to ", optimizer_data_ptr_->model_name_);
    optimizer_data_ptr_->origin_path_ = orin_combined_flu_path_;
    if (is_ignore_od_and_fs_) {
      optimizer_data_ptr_->freespace_ptr_ = null_freespace_;
    } else {
      optimizer_data_ptr_->freespace_ptr_ = local_view.getFreespacePtr();
    }
    optimizer_data_ptr_->oringin_vehicle_point_ = local_view.getLocalizationPtr()->vehicleAlignPosePoint();
    optimizer_data_ptr_->tf_ego_2_map_ = local_view.getLocalizationPtr()->getTfEgo2Map();
    optimizer_data_ptr_->tf_map_2_ego_ = local_view.getLocalizationPtr()->getTfMap2Ego();
    optimizer_data_ptr_->decision_object_map_ = decision_result.getOdDecisions();
    optimizer_data_ptr_->piecewise_point_ = slot_.slot.topEdgeCenter();
    if (optimizer_data_ptr_->obstacles_linesegments_ == nullptr) {
      optimizer_data_ptr_->obstacles_linesegments_ = std::make_unique<std::vector<ObstaclesLinesegment>>();
    }
    *optimizer_data_ptr_->obstacles_linesegments_ = roi.obstacles_linesegments_map;

    auto optimizer_profiles =
        ProfileManager::instance().profile<ParkingPathOptimizerProfile>(planner_profile_, optimizer_name);
    optimizer_data_ptr_->optimizer_profiles_ = std::move(optimizer_profiles);
  }
  std::any optimizer_data = optimizer_data_ptr_;

  manager_.Register(optimizer_name);
  const auto& optimizer_status = manager_.Execute(optimizer_data);

  if (optimizer_status != OpenspacePathOptimizer::OpenspaceStatus::FINISH)
    return UnitStatus::WAITING;
  std::shared_ptr<OpenspaceOptimizerData> optimizer_data_ptr =
      std::any_cast<std::shared_ptr<OpenspaceOptimizerData>>(optimizer_data);
  optimizer_segmented_flu_path_ = std::move(optimizer_data_ptr->optimizer_flu_path_);

  cal_info_ += optimizer_data_ptr->debug_info_;
  optimizer_boundary_vec_ = std::move(optimizer_data_ptr->boundarys_vis_);
  if (optimizer_segmented_flu_path_.empty()) {
    PARKING_LOG(E, " optimizer_segmented_flu_path_ empty !!!!! ");
    return UnitStatus::FAILED;
  }

  // 轨迹时间补偿
  for (auto& path : optimizer_segmented_flu_path_) {
    trajectoryTimeCompensate(optimizer_data_ptr_->tf_ego_2_map_, local_view.getLocalizationPtr()->getTfMap2Ego(), path);
  }

  optimizer_data_ptr_->clear();
  manager_.Clear();
  return UnitStatus::SUCCEEDED;
}

std::vector<Decision::DecisionObject> BaseParkingPlanner::obstacleSelector(const LocalView& local_view,
                                                                           const DecisionResult& decision_result) {
  std::vector<Decision::DecisionObject> selected_obstacles;
  if (planner_profile_->enable_decision_object()) {
    for (auto [id, decision_od] : *decision_result.getOdDecisions()) {
      PARKING_LOG(D, "decision_od type = ", static_cast<int>(decision_od.type), " id = ", decision_od.id,
                  " isStatic() =", decision_od.is_static);
      // 排查移动的障碍物
      if (!decision_od.is_static)
        continue;

      // 忽略行人
      if (decision_od.type == Decision::ObjectType::PEDESTRIAN)
        continue;

      // 排除远距离的障碍物
      auto veh_align_pose = local_view.getLocalizationPtr()->vehicleAlignPosePoint();
      if (math::Vec2d(veh_align_pose.x(), veh_align_pose.y()).DistanceTo(math::Vec2d(decision_od.x, decision_od.y))
          > (planner_profile_->decision_object_distance_filter() + decision_od.length / 2))
        continue;
      // todo：挑选关注的OD

      // 将障碍物box转换到ego坐标系
      Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
      auto box_center_3d_ego = math::Vec3d(decision_od.cur_box.center(), veh_align_pose.z());
      transfer::transformPoint(tf_map_2_ego, &box_center_3d_ego);
      math::Vec3d box_center_3d_rpy(0.0, 0.0, decision_od.cur_box.heading());
      transfer::transformRPY(tf_map_2_ego, &box_center_3d_rpy);
      decision_od.cur_box = math::Box2d(box_center_3d_ego, box_center_3d_rpy.z(), decision_od.cur_box.length(),
                                        decision_od.cur_box.width());
      selected_obstacles.emplace_back(decision_od);
    }
  }
  return selected_obstacles;
}

BaseParkingPlanner::UnitStatus BaseParkingPlanner::originalPathSearch(const LocalView& local_view,
                                                                      const RoiDecideResult& roi) {
  PARKING_LOG(D, "Enter Hybrid A Star Search !!!!");
  const std::string search_name = "hybrid_astar";
  search_data_ptr_->roi_ = roi;
  search_data_ptr_->start_pose_ = search_elements_.start_pose;
  search_data_ptr_->end_pose_ = search_elements_.end_pose;
  search_data_ptr_->is_region_search_ = search_elements_.is_region_search;
  if (search_elements_.is_region_search) {
    search_data_ptr_->goal_region_ = search_elements_.goal_region;
    search_data_ptr_->goal_region_heading_ = search_elements_.goal_region_heading;
    search_data_ptr_->goal_region_heading_tolerance_ = search_elements_.goal_region_heading_tolerance;
  }

  if (is_ignore_od_and_fs_) {
    search_data_ptr_->freespace_ptr_ = null_freespace_;
  } else {
    search_data_ptr_->freespace_ptr_ = local_view.getFreespacePtr();
  }
  search_data_ptr_->search_config_name_ = planner_profile_->search_model_name();

  // scenario_tags优先级： dead_end > replan
  if (planner_status_ == PlannerStatus::REPLAN) {
    search_data_ptr_->scenario_tags_.push_back("replan");
    search_data_ptr_->start_direction_ = replan_direction_;
  }
  if (park_env_type_ == ParkEnvType::DEAD_END) {
    search_data_ptr_->scenario_tags_.push_back("dead_end");
  }

  std::any search_data = search_data_ptr_;

  manager_.Register(search_name);
  const auto& search_status = manager_.Execute(search_data);

  if (search_status == BaseOpenspacePathPlanner::OpenspaceStatus::FAILED) {
    PARKING_LOG(E, "Hybrid A Star Search Failed !!!!! ");
    return UnitStatus::FAILED;
  }
  if (search_status != BaseOpenspacePathPlanner::OpenspaceStatus::FINISH)
    return UnitStatus::WAITING;
  std::shared_ptr<OpenspaceSearchData> search_data_ptr =
      std::any_cast<std::shared_ptr<OpenspaceSearchData>>(search_data);
  orin_combined_flu_path_ = std::move(DiscretizedPath(search_data_ptr->search_path_));
  cal_info_ += search_data_ptr_->debug_info_;
  if (orin_combined_flu_path_.empty())
    return UnitStatus::FAILED;

  // 轨迹时间补偿
  trajectoryTimeCompensate(tf_ego_2_map_search_, local_view.getLocalizationPtr()->getTfMap2Ego(),
                           orin_combined_flu_path_);

  search_data_ptr->clear();
  manager_.Clear();
  return UnitStatus::SUCCEEDED;
}

void BaseParkingPlanner::selectAndClipCurrentTrajectory(const LocalView& local_view) {
  current_flu_path_.clear();
  if (!realtime_optimizer_segmented_flu_paths_.empty()) {
    currentTrajSelectAndCut(realtime_optimizer_segmented_flu_paths_, current_flu_path_, remain_dis_);

    PARKING_LOG(I, "剩余的轨迹点数 = ", current_flu_path_.size(), " 剩余距离： = ", remain_dis_);
  }
  debug_info_ = cal_info_
                + fmt::format("gear: {}  {}/{}  |  {} {:.2f}\n",
                              (current_flu_path_.front().direction() == PathPt::Direction::BACKWARD ? "R" : "D"),
                              std::to_string(realtime_optimizer_segmented_flu_paths_.size()),
                              std::to_string(optimizer_segmented_flu_path_.size()),
                              std::to_string(current_flu_path_.size()), remain_dis_);

  if (is_tuning_ && planner_profile_->fine_tune_config().enable_follow_up()) {
    float factor = 0;
    for (auto& pt : current_flu_path_) {
      PathPt pedal_pt = CalculatePedalPoint(pt, slot_.slot.topEdgeCenter(), slot_.slot.bottomEdgeCenter(), factor);
      pt.set_x(pedal_pt.x());
      pt.set_y(pedal_pt.y());
      pt.set_theta(slot_.slot.heading());
    }
  }

  ReCalculateLineLength(current_flu_path_);
  debugPrintTraj(current_flu_path_, "current_flu_path_");
}

void BaseParkingPlanner::debugPrintTraj(DiscretizedPath traj, string name) {
  for (auto pt : traj) {
    PARKING_LOG(D, name, " ", pt.x(), " ", pt.y(), " ", pt.theta() * RAD2ANG, " ", pt.kappa(), " ", pt.s(), " ",
                (int)pt.direction());
  }
}

void BaseParkingPlanner::debugPrintTraj(vector<DiscretizedPath> trajs, string name) {
  for (auto path : trajs) {
    debugPrintTraj(path, name);
  }
}

vector<DiscretizedPath> BaseParkingPlanner::local2GlobalPaths(const LocalView& local_view,
                                                              const std::vector<DiscretizedPath>& local_paths) {
  Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();

  vector<DiscretizedPath> global_paths;
  for (auto path : local_paths) {
    DiscretizedPath global_path;
    for (auto pt : path) {
      transfer::transformPoint(tf_ego_2_map, &pt);
      math::Vec3d map_rpy_vec3d(0.0, 0.0, pt.theta());
      transfer::transformRPY(tf_ego_2_map, &map_rpy_vec3d);
      pt.set_theta(map_rpy_vec3d.z());
      global_path.push_back(pt);
    }
    global_paths.push_back(global_path);
  }

  return global_paths;
}

std::vector<DiscretizedPath> BaseParkingPlanner::global2LocalPaths(const LocalView& local_view,
                                                                   const std::vector<DiscretizedPath>& global_paths) {
  Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();

  vector<DiscretizedPath> local_paths;
  for (auto path : global_paths) {
    DiscretizedPath local_path;
    for (auto pt : path) {
      transfer::transformPoint(tf_map_2_ego, &pt);
      math::Vec3d ego_rpy_vec3d(0.0, 0.0, pt.theta());
      transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
      pt.set_theta(ego_rpy_vec3d.z());
      local_path.push_back(pt);
    }
    local_paths.push_back(local_path);
  }

  return local_paths;
}

PathData::BlockFSInfo BaseParkingPlanner::calPathBlockFS(const LocalView& local_view, const DiscretizedPath& path_map) {
  PathData::BlockFSInfo block_fs_info;
  if (path_map.empty() || !planner_profile_->path_block_fs_config().enable_fs_collision_check()) {
    return block_fs_info;
  }
  // path: map->ego
  std::vector<PathPt> ego_path;
  Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
  for (const auto& path_pt : path_map) {
    math::Vec3d flu_position_vec3d(path_pt.x(), path_pt.y(), path_pt.z());
    transfer::transformPoint(tf_map_2_ego, &flu_position_vec3d);
    math::Vec3d flu_rpy_vec3d(0.0, 0.0, path_pt.theta());
    transfer::transformRPY(tf_map_2_ego, &flu_rpy_vec3d);

    ego_path.emplace_back(path_pt);
    ego_path.back().set_x(flu_position_vec3d.x());
    ego_path.back().set_y(flu_position_vec3d.y());
    ego_path.back().set_z(flu_position_vec3d.z());
    ego_path.back().set_theta(flu_rpy_vec3d.z());
  }
  block_fs_info = collisionCheck(*local_view.getFreespacePtr(), ego_path,
                                 planner_profile_->path_block_fs_config().collision_check_buffer(),
                                 planner_profile_->path_block_fs_config().corner_width(),
                                 planner_profile_->path_block_fs_config().enable_curve_decide_process(),
                                 planner_profile_->path_block_fs_config().curve_look_ahead_distance(),
                                 planner_profile_->path_block_fs_config().curve_kappa_thresold(),
                                 planner_profile_->path_block_fs_config().side_box_length(),
                                 planner_profile_->path_block_fs_config().side_box_width());
  if (block_fs_info.is_valid) {
    block_fs_info.path_type = PathData::PathType::LATERAL;
    block_fs_info.block_point_direction =
        getBlockFsPointDirection(block_fs_info.check_point, block_fs_info.block_point);

    // ego->map
    Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
    math::Vec3d map_check_point(block_fs_info.check_point.x(), block_fs_info.check_point.y(),
                                block_fs_info.check_point.z());
    transfer::transformPoint(tf_ego_2_map, &map_check_point);
    block_fs_info.check_point = PathPt(map_check_point.x(), map_check_point.y(), map_check_point.z());
    math::Vec3d map_block_point(block_fs_info.block_point.x(), block_fs_info.block_point.y(), 0.0);
    transfer::transformPoint(tf_ego_2_map, &map_block_point);
    block_fs_info.block_point = math::Vec2d(map_block_point.x(), map_block_point.y());
    std::vector<math::Vec2d> map_vis_pts;
    for (const auto& vis_pt : block_fs_info.vis_pts) {
      math::Vec3d map_vis_pt(vis_pt.x(), vis_pt.y(), 0.0);
      transfer::transformPoint(tf_ego_2_map, &map_vis_pt);
      map_vis_pts.emplace_back(map_vis_pt.x(), map_vis_pt.y());
    }
    block_fs_info.vis_pts = std::move(map_vis_pts);
  }

  return block_fs_info;
}
void BaseParkingPlanner::calculateSlotAndTrackingStability(const LocalView& local_view) {
  // 计算跟踪横向误差、航向误差、与终点航向误差
  auto road_error = CalculateRoadError(PathPt(0, 0, 0), current_flu_path_);
  auto head_error = current_flu_path_.empty() ? 0.0 : current_flu_path_.front().theta();
  auto terminal_head_error = current_flu_path_.empty() ? 0.0 : current_flu_path_.back().theta();
  PARKING_LOG(D, "road_error: ", road_error.f_rear_road_error, " head_error: ", head_error * RAD2ANG,
              " terminal_head_error: ", terminal_head_error * RAD2ANG);
  debug_info_ += fmt::format("tracking_stability: RE: {:.2f} HE: {:.2f} THE: {:.2f}\n", road_error.f_rear_road_error,
                             head_error * RAD2ANG, terminal_head_error * RAD2ANG);

  // 计算自车各个方位到库位边线的距离
  calVehicleSlotClearance();

  // 计算库位横向偏移、纵向偏移、角度偏移
  PathPt cur_slot_center_pos = slot_.slot.center();
  auto tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
  transfer::transformPoint(tf_ego_2_map, &cur_slot_center_pos);
  math::Vec3d map_rpy_vec3d(0.0, 0.0, slot_.slot.heading());
  transfer::transformRPY(tf_ego_2_map, &map_rpy_vec3d);
  cur_slot_center_pos.set_theta(map_rpy_vec3d.z());

  math::Vec2d unit_vec2d = initial_slot_center_pos_.CreateUnitVec2d(initial_slot_center_pos_.theta());
  math::Vec2d relative_vec2d = cur_slot_center_pos - initial_slot_center_pos_;

  slot_lateral_error_ = unit_vec2d.CrossProd(relative_vec2d);
  slot_longti_error_ = unit_vec2d.InnerProd(relative_vec2d);

  slot_heading_error_ = math::NormalizeAngle(cur_slot_center_pos.theta() - initial_slot_center_pos_.theta()) * RAD2ANG;
  PARKING_LOG(D, "slot_lateral_error: ", slot_lateral_error_);
  PARKING_LOG(D, "slot_longti_error: ", slot_longti_error_);
  PARKING_LOG(D, "slot_heading_error: ", slot_heading_error_);
  debug_info_ += fmt::format("slot_stability: lat: {:.2f} lon: {:.2f} ang: {:.2f}\n", slot_lateral_error_,
                             slot_longti_error_, slot_heading_error_);
}

bool BaseParkingPlanner::isNeedReplan(const LocalView& local_view, const bool destination_stop,
                                      const StopReason& stop_reason) {
  if (!planner_profile_->parking_replan_config().enable_replan()) {
    PARKING_LOG(I, "  重规划 关闭");
    return false;
  }

  if (is_tuning_) {
    PARKING_LOG(I, "  is_tuning_ 重规划 不触发");
    return false;
  }

  if (current_flu_path_.empty()) {
    return false;
  }

  replan_direction_ = !current_flu_path_.empty() && current_flu_path_.front().direction() == PathPt::Direction::FORWARD
                          ? PathPt::Direction::BACKWARD
                          : PathPt::Direction::FORWARD;
  if (optimizer_segmented_map_paths_.size() > 1 && destination_stop) {
    // 换挡处重规划触发
    if (planner_profile_->parking_replan_config().enable_shift_trigger()) {
      auto road_error = CalculateRoadError(PathPt(0, 0, 0), current_flu_path_);
      double angle_error = current_flu_path_.empty() ? 0.0 : current_flu_path_.front().theta();
      if (abs(road_error.f_rear_road_error)
              > planner_profile_->parking_replan_config().shift_trigger_lateral_deviation_threshold()
          || abs(angle_error)
                 > planner_profile_->parking_replan_config().shift_trigger_angle_deviation_threshold() * ANG2RAD) {
        if (shift_replan_num_count_ >= planner_profile_->parking_replan_config().shift_replan_num_limit()) {
          PARKING_LOG(W, "  换挡处重规划次数 超限");
        } else {
          PARKING_LOG(I, "  Need replan!!! RE: ", abs(road_error.f_rear_road_error),
                      " HE: ", abs(angle_error) * RAD2ANG);
          shift_replan_num_count_++;
          optimizer_segmented_map_paths_backup_ = optimizer_segmented_map_paths_;
          if (!optimizer_segmented_map_paths_backup_.empty()) {
            optimizer_segmented_map_paths_backup_.erase(optimizer_segmented_map_paths_backup_.begin());
          }
          return true;
        }
      }
    }

    // 库位更新重规划触发
    if (planner_profile_->parking_replan_config().enable_slot_update_trigger()
        && optimizer_segmented_map_paths_.size() > 1) {
      if (abs(slot_lateral_error_)
              > planner_profile_->parking_replan_config().slot_update_trigger_lateral_deviation_threshold()
          || abs(slot_longti_error_)
                 > planner_profile_->parking_replan_config().slot_update_trigger_longti_deviation_threshold()
          || abs(slot_heading_error_)
                 > planner_profile_->parking_replan_config().slot_update_trigger_angle_deviation_threshold()) {
        if (slot_update_replan_num_count_ >= planner_profile_->parking_replan_config().slot_update_replan_num_limit()) {
          PARKING_LOG(W, "  库位更新重规划次数 超限");
        } else {
          PARKING_LOG(I, "  Need replan!!! slot update lat: ", abs(slot_lateral_error_),
                      " lon: ", abs(slot_longti_error_), " ang: ", abs(slot_heading_error_));
          slot_update_replan_num_count_++;
          optimizer_segmented_map_paths_backup_ = optimizer_segmented_map_paths_;
          if (!optimizer_segmented_map_paths_backup_.empty()) {
            optimizer_segmented_map_paths_backup_.erase(optimizer_segmented_map_paths_backup_.begin());
          }
          return true;
        }
      }
    }
  }

  // 逼停后重规划触发
  if (planner_profile_->parking_replan_config().enable_forced_stop_trigger()) {
    if (fabs(local_view.getChassisPtr()->Speed()) < 0.01
        && (stop_reason.stop_reason_type == StopReason::StopReasonType::BLOCK_OD
            || stop_reason.stop_reason_type == StopReason::StopReasonType::BLOCK_FS)) {
      forced_stop_cancel_time_count_ = 0;
      forced_stop_time_count_++;
      if (forced_stop_time_count_ < planner_profile_->parking_replan_config().forced_stop_trigger_time_threshold()) {
        PARKING_LOG(I, "  forced stop replan!!! Waiting: ", forced_stop_time_count_);
        return false;
      }

      // 在忽略障碍物的规划模式中，若被障碍物逼停，则直接反馈泊车失败
      if (is_ignore_od_and_fs_) {
        planner_status_ = PlannerStatus::FAILED;
        return false;
      }

      if (forced_stop_replan_num_count_ >= planner_profile_->parking_replan_config().forced_stop_replan_num_limit()) {
        PARKING_LOG(W, "  逼停重规划次数 超限");
        return false;
      }
      PARKING_LOG(I, "  Need replan!!! stop_reason: ", stop_reason.stop_reason_type);
      forced_stop_replan_num_count_++;
      optimizer_segmented_map_paths_backup_ = optimizer_segmented_map_paths_;
      return true;
    } else {
      if (forced_stop_time_count_ > 0) {
        forced_stop_cancel_time_count_++;
      }
      if (forced_stop_cancel_time_count_
          > planner_profile_->parking_replan_config().forced_stop_cancel_time_threshold()) {
        forced_stop_time_count_ = 0;
      }
    }
  }

  return false;
}

BaseParkingPlanner::UnitStatus BaseParkingPlanner::reOptimizer(const LocalView& local_view,
                                                               const DecisionResult& decision_result,
                                                               const DiscretizedPath& segment_to_optimize) {
  // --- 首次调用：初始化并开始优化 ---
  if (!is_reoptimizing_) {
    PARKING_LOG(I, "[ReOptimizer] Initiating asynchronous re-optimization.");

    if (segment_to_optimize.empty()) {
      PARKING_LOG(W, "[ReOptimizer] Input segment is empty.");
      return UnitStatus::FAILED;
    }

    // 设置状态标志位
    is_reoptimizing_ = true;

    // 初始化数据和管理器
    reopt_data_ptr_ = std::make_shared<OpenspaceOptimizerData>();
    reopt_manager_ = std::make_unique<OpenspaceCoreManager>();

    // --- 填充优化器所需数据 ---
    Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
    DiscretizedPath flu_segment;
    for (auto pt : segment_to_optimize) {
      transfer::transformPoint(tf_map_2_ego, &pt);
      math::Vec3d ego_rpy_vec3d(0.0, 0.0, pt.theta());
      transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
      pt.set_theta(ego_rpy_vec3d.z());
      flu_segment.push_back(pt);
    }
    ReCalculateLineLength(flu_segment);

    OPENSPACE_LOG(D, "[ReOptimizer] Segment to optimize size: ", flu_segment.size());

    for (const auto& pt : flu_segment) {
      OPENSPACE_LOG(D, "[ReOptimizer] segment_point = ", pt.x(), ", ", pt.y(), ", ", pt.theta(), ", ", pt.s(), ", ",
                    static_cast<int>(pt.direction()));
    }

    reopt_data_ptr_->model_name_ = "reoptimizing";
    reopt_data_ptr_->origin_path_ = flu_segment;
    reopt_data_ptr_->freespace_ptr_ = local_view.getFreespacePtr();
    reopt_data_ptr_->oringin_vehicle_point_ = local_view.getLocalizationPtr()->vehicleAlignPosePoint();
    reopt_data_ptr_->tf_ego_2_map_ = local_view.getLocalizationPtr()->getTfEgo2Map();
    reopt_data_ptr_->tf_map_2_ego_ = tf_map_2_ego;
    reopt_data_ptr_->decision_object_map_ = decision_result.getOdDecisions();

    RoiDecideResult roi;
    park_roi_decider_.getRoiDecideResuilt(roi);
    if (reopt_data_ptr_->obstacles_linesegments_ == nullptr) {
      reopt_data_ptr_->obstacles_linesegments_ = std::make_unique<std::vector<ObstaclesLinesegment>>();
    }
    *reopt_data_ptr_->obstacles_linesegments_ = roi.obstacles_linesegments_map;
    const std::string optimizer_name = "ipm_ocp";
    auto optimizer_profiles =
        ProfileManager::instance().profile<ParkingPathOptimizerProfile>(planner_profile_, optimizer_name);
    reopt_data_ptr_->optimizer_profiles_ = std::move(optimizer_profiles);
    // --- 数据填充结束 ---

    reopt_manager_->Register(optimizer_name);
  }

  // --- 持续调用：轮询优化状态 ---
  PARKING_LOG(D, "[ReOptimizer] Executing re-optimization step.");
  std::any optimizer_data = reopt_data_ptr_;
  const auto& optimizer_status = reopt_manager_->Execute(optimizer_data);

  switch (optimizer_status) {
    case OpenspacePathOptimizer::OpenspaceStatus::FINISH:
      PARKING_LOG(I, "[ReOptimizer] Re-optimization finished successfully.");
      return UnitStatus::SUCCEEDED;
    case OpenspacePathOptimizer::OpenspaceStatus::FAILED:
      PARKING_LOG(E, "[ReOptimizer] Re-optimization failed.");
      return UnitStatus::FAILED;
    case OpenspacePathOptimizer::OpenspaceStatus::WAITING:
    default:
      PARKING_LOG(D, "[ReOptimizer] Re-optimization is waiting for next cycle.");
      return UnitStatus::WAITING;
  }
}

bool BaseParkingPlanner::handleShiftAndReoptimization(const LocalView& local_view,
                                                      const DecisionResult& decision_result, bool& destination_stop) {
  bool ready_for_next_segment = optimizer_segmented_map_paths_.size() > 1
                                && (remain_dis_ <= planner_profile_->shift_min_dis_require()) && destination_stop;

  if (is_reoptimizing_ || ready_for_next_segment) {
    // 1. 轮询处理：如果已经在重优化状态，则持续处理
    if (is_reoptimizing_) {
      PARKING_LOG(D, "Continuing re-optimization process...");
      const auto& segment_to_optimize = optimizer_segmented_map_paths_[1];
      auto reopt_status = reOptimizer(local_view, decision_result, segment_to_optimize);

      if (reopt_status == UnitStatus::SUCCEEDED) {
        // 优化成功，提取结果，替换轨迹，然后清理状态
        std::shared_ptr<OpenspaceOptimizerData> result_data_ptr =
            std::any_cast<std::shared_ptr<OpenspaceOptimizerData>>(reopt_data_ptr_);

        if (!result_data_ptr->optimizer_flu_path_.empty() && !result_data_ptr->optimizer_flu_path_.front().empty()) {
          // 将优化好的FLU轨迹转换回map坐标系
          auto optimized_flu_segment = result_data_ptr->optimizer_flu_path_.front();
          DiscretizedPath optimized_map_segment;
          Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
          for (auto pt : optimized_flu_segment) {
            transfer::transformPoint(tf_ego_2_map, &pt);
            math::Vec3d map_rpy_vec3d(0.0, 0.0, pt.theta());
            transfer::transformRPY(tf_ego_2_map, &map_rpy_vec3d);
            pt.set_theta(map_rpy_vec3d.z());
            optimized_map_segment.push_back(pt);
          }
          // 替换第二段轨迹
          optimizer_segmented_map_paths_[1] = optimized_map_segment;
          PARKING_LOG(D, "Re-optimized segment has been updated.");
        } else {
          PARKING_LOG(W, "Re-optimizer returned success but path is empty. Using original path.");
        }

        // 清理并退出重优化模式
        is_reoptimizing_ = false;
        reopt_data_ptr_.reset();
        reopt_manager_.reset();
        optimizer_segmented_map_paths_.erase(optimizer_segmented_map_paths_.begin());
        destination_stop = false;  // 消耗标志位
        PARKING_LOG(D, "重优化成功并重置状态，已消耗destination_stop标志位。");

      } else if (reopt_status == UnitStatus::FAILED) {
        // 优化失败，清理状态，使用原始轨迹继续
        PARKING_LOG(W, "Re-optimization failed. Proceeding with the original segment.");
        is_reoptimizing_ = false;
        reopt_data_ptr_.reset();
        reopt_manager_.reset();
        optimizer_segmented_map_paths_.erase(optimizer_segmented_map_paths_.begin());
        destination_stop = false;  // 消耗标志位
        PARKING_LOG(W, "重优化失败并重置状态，已消耗destination_stop标志位。");

      } else {  // reopt_status == UnitStatus::WAITING
        // 优化仍在进行中，直接返回，等待下一帧
        return true;  // 返回true，表示 getPath 需要提前终止
      }
    }
    // 2. 触发：如果满足换挡条件且当前未处于重优化状态，则触发
    else if (ready_for_next_segment) {
      if (planner_profile_->enable_reoptimize_at_shift()) {
        PARKING_LOG(D, "Shift point reached. Triggering asynchronous re-optimization.");
        const auto& segment_to_optimize = optimizer_segmented_map_paths_[1];
        // 调用reOptimizer以启动流程 (内部会将 is_reoptimizing_ 置为 true)
        reOptimizer(local_view, decision_result, segment_to_optimize);
        return true;  // 返回true，表示 getPath 需要提前终止
      } else {
        // 如果不启用重优化，执行原始逻辑
        optimizer_segmented_map_paths_.erase(optimizer_segmented_map_paths_.begin());
        destination_stop = false;
      }
    }
  }

  // 如果没有触发、或者任务已完成，返回false，让 getPath 继续执行
  return false;
}

void BaseParkingPlanner::mapBoundary(const LocalView& local_view, std::vector<math::LineSegment2d>& boundary_seg) {
  auto getPointInLineSegment2d = [=](const math::LineSegment2d& segment, const double s) {
    math::Vec2d res = math::Vec2d(0.0, 0.0);
    double inter_s = std::max(0.0, std::min(segment.length(), s));

    PathPt start_pt = PathPt(math::Vec3d(segment.start(), 0.0));
    PathPt end_pt = PathPt(math::Vec3d(segment.end(), 0.0));
    start_pt.set_s(0.0);
    end_pt.set_s(segment.length());
    PathPt interpolate_pt = math::interpolateUsingLinearApproximation(start_pt, end_pt, inter_s);
    res = math::Vec2d(interpolate_pt);
    return res;
  };

  if (!target_reference_line_info_.isValid() || !planner_profile_->custom_boundary_config().enable_map_boundary()) {
    return;
  }
  // 构建地图边界
  Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
  const double min_length = 2.0;
  const double max_heading_err = 0.17;
  const double map_boundary_range = 40.0;
  auto ref = target_reference_line_info_.ref_line();
  math::Vec3d adc_pose = math::Vec3d(ref.adcLocalization().x(), ref.adcLocalization().y(), ref.adcLocalization().z());

  std::vector<std::vector<math::LineSegment2d>> boundary_segs;
  for (const auto& lane_marking : local_view.getEnvRoadCognitionPtr()->getLaneMarkings()) {
    std::vector<math::Vec2d> current_segment_pts;

    for (const auto& pt : lane_marking.points) {
      double distance = pt.DistanceTo(adc_pose);

      if (distance <= map_boundary_range) {
        // 近距离点：添加到当前段
        math::Vec3d vec3d = pt;
        transfer::transformPoint(tf_map_2_ego, &vec3d);
        current_segment_pts.emplace_back(vec3d.x(), vec3d.y());
      } else if (!current_segment_pts.empty()) {
        // 从近距离切换到远距离：处理已累积的段
        std::vector<math::LineSegment2d> temp_seg;
        appendLineSegments(current_segment_pts, temp_seg, min_length, max_heading_err);
        boundary_segs.emplace_back(temp_seg);
        current_segment_pts.clear();
      }
    }

    // 处理最后一个段（如果存在）
    if (!current_segment_pts.empty()) {
      std::vector<math::LineSegment2d> temp_seg;
      appendLineSegments(current_segment_pts, temp_seg, min_length, max_heading_err);
      boundary_segs.emplace_back(temp_seg);
    }
  }

  // 忽略与自车干涉的地图边界
  const double buff = 0.4;
  math::Box2d ego_box(math::Vec2d(0.0, 0.0), 0.0, vehicle_config_->vehicle_param().front_edge_to_ego() + buff,
                      vehicle_config_->vehicle_param().rear_edge_to_ego() + buff,
                      vehicle_config_->vehicle_param().width() + 2 * buff);
  math::Polygon2d polygon(ego_box);
  for (auto& segs : boundary_segs) {
    std::vector<double> accumulated_s{0.0};
    for (auto segment : segs) {
      accumulated_s.emplace_back(accumulated_s.back() + segment.length());
    }
    auto [in_range, start_s, end_s, start_index, end_index] = getOverlapRange(polygon, segs, accumulated_s);

    math::LineSegment2d segments_start, segments_end;
    if (in_range) {
      double s = start_s - accumulated_s.at(start_index);
      auto pt_in_start_segment = getPointInLineSegment2d(segs.at(start_index), s);
      segments_start = math::LineSegment2d(segs.at(start_index).start(), pt_in_start_segment);

      s = end_s - accumulated_s.at(end_index);
      auto pt_in_end_segment = getPointInLineSegment2d(segs.at(end_index), s);
      segments_end = math::LineSegment2d(pt_in_end_segment, segs.at(end_index).end());

      segs.erase(segs.begin() + start_index, segs.begin() + end_index + 1);
      int add_index = start_index;
      if (segments_start.length() > 0.1) {
        segs.insert(segs.begin() + add_index, segments_start);
        add_index++;
      }
      if (segments_end.length() > 0.1) {
        segs.insert(segs.begin() + add_index, segments_end);
      }

      // 将干涉部分推离自车一定距离
      math::LineSegment2d seg_overlap(pt_in_start_segment, pt_in_end_segment);
      math::Vec2d unit_dir = seg_overlap.unit_direction();
      math::Vec2d left_normal(-unit_dir.y(), unit_dir.x());   // 左垂直方向
      math::Vec2d right_normal(unit_dir.y(), -unit_dir.x());  // 右垂直方向
      const double step = 0.1;
      const int max_index = 50;

      for (int i = 1; i < max_index; i++) {
        math::Vec2d left_new_start = seg_overlap.start() + left_normal * step * i;
        math::Vec2d left_new_end = seg_overlap.end() + left_normal * step * i;
        math::LineSegment2d left_test_seg(left_new_start, left_new_end);
        if (!ego_box.HasOverlap(left_test_seg)) {
          boundary_seg.emplace_back(left_test_seg);
          break;
        }

        math::Vec2d right_new_start = seg_overlap.start() + right_normal * step * i;
        math::Vec2d right_new_end = seg_overlap.end() + right_normal * step * i;
        math::LineSegment2d right_test_seg(right_new_start, right_new_end);
        if (!ego_box.HasOverlap(right_test_seg)) {
          boundary_seg.emplace_back(right_test_seg);
          break;
        }
      }
    }
  }

  for (auto& segs : boundary_segs) {
    boundary_seg.insert(boundary_seg.begin(), segs.begin(), segs.end());
  }

  park_roi_decider_.addBoundary(std::make_tuple("roadside", OpenspaceObjectType::ROAD_SIDE, boundary_seg));
}

void BaseParkingPlanner::handleRealTimeRefinement(const LocalView& local_view, const DecisionResult& decision_result) {
  // 1. 距离硬约束检查：剩余距离小于3米时，停止一切重优化，防止末端抖动
  if (remain_dis_ < 3.0) {
    // 如果之前正在跑，强制中断并清理，保证不再更新
    if (is_realtime_optimizing_) {
      PARKING_LOG(I, "[RealTime] Remain dist < 3.0m, stop optimizing.");
      is_realtime_optimizing_ = false;
      if (realtime_manager_)
        realtime_manager_->Clear();
    }
    return;
  }

  // 2. 如果当前没有正在跑的优化任务，则发起一个新的 (Start)
  if (!is_realtime_optimizing_) {
    PARKING_LOG(D, "[RealTime] Start new frame optimization.");

    // 初始化资源
    if (!realtime_manager_) {
      realtime_manager_ = std::make_unique<OpenspaceCoreManager>();
      realtime_data_ptr_ = std::make_shared<OpenspaceOptimizerData>();
    }

    // 填充数据：基于当前车辆位置 + 当前剩余轨迹作为初值
    prepareRealTimeData(local_view, decision_result);

    // 注册并启动
    const std::string optimizer_name = "ipm_ocp";  // 或使用专门的 "realtime_ocp" 配置
    realtime_manager_->Register(optimizer_name);

    is_realtime_optimizing_ = true;
  }

  // 3. 轮询优化器状态 (Execute)
  // 注意：如果是同步优化器，这里会阻塞直到算完；如果是异步，这里会返回 WAITING
  std::any optimizer_data = realtime_data_ptr_;
  const auto& status = realtime_manager_->Execute(optimizer_data);

  if (status == OpenspacePathOptimizer::OpenspaceStatus::FINISH) {
    // --- 优化成功 ---
    PARKING_LOG(I, "[RealTime] Optimization FINISHED. Updating trajectory.");

    std::shared_ptr<OpenspaceOptimizerData> result_ptr =
        std::any_cast<std::shared_ptr<OpenspaceOptimizerData>>(optimizer_data);

    if (!result_ptr->optimizer_flu_path_.empty() && !result_ptr->optimizer_flu_path_.front().empty()) {
      // 1. 获取优化后的 FLU 轨迹
      auto& optimized_flu_path = result_ptr->optimizer_flu_path_.front();

      // 2. 将 FLU 转回 Map 坐标系 (使用当时发起优化时的 TF 还是当前的 TF?
      //    通常 OCP 输出是在车辆坐标系下的，直接用当前的 TF 转回 Map 替换即可)
      DiscretizedPath optimized_map_path;
      Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();

      for (auto pt : optimized_flu_path) {
        transfer::transformPoint(tf_ego_2_map, &pt);
        math::Vec3d map_rpy_vec3d(0.0, 0.0, pt.theta());
        transfer::transformRPY(tf_ego_2_map, &map_rpy_vec3d);
        pt.set_theta(map_rpy_vec3d.z());
        optimized_map_path.push_back(pt);
      }

      // 3. 无缝替换：直接更新 optimizer_segmented_map_paths_[0]
      //    下游的 Step 5 (global2LocalPaths) 和 Step 6 (selectAndClip) 会在当前帧
      //    自动处理裁剪和拼接，保证控制模块拿到的是基于新轨迹的路径。
      optimizer_segmented_map_paths_[0] = std::move(optimized_map_path);
    }

    // 4. 重置状态，允许下一帧立即发起新的优化
    is_realtime_optimizing_ = false;
    realtime_data_ptr_->clear();
    realtime_manager_->Clear();

  } else if (status == OpenspacePathOptimizer::OpenspaceStatus::FAILED) {
    // --- 优化失败 ---
    PARKING_LOG(W, "[RealTime] Optimization FAILED. Keeping original trajectory.");
    // 失败也重置，允许下一帧重试
    is_realtime_optimizing_ = false;
    realtime_data_ptr_->clear();
    realtime_manager_->Clear();
  }
}

void BaseParkingPlanner::prepareRealTimeData(const LocalView& local_view, const DecisionResult& decision_result) {
  // 基础配置
  realtime_data_ptr_->model_name_ = "refine";
  realtime_data_ptr_->freespace_ptr_ = local_view.getFreespacePtr();
  realtime_data_ptr_->decision_object_map_ = decision_result.getOdDecisions();

  // 起点设置为当前车辆位置
  realtime_data_ptr_->oringin_vehicle_point_ = local_view.getLocalizationPtr()->vehicleAlignPosePoint();

  // 使用当前剩余的轨迹作为初值
  if (!current_flu_path_.empty()) {
    realtime_data_ptr_->origin_path_ = std::move(global2LocalPaths(local_view, optimizer_segmented_map_paths_).front());
    auto dir = current_flu_path_.back().direction();

    realtime_data_ptr_->destination_straight_line_length_ =
        planner_profile_->trajectory_post_process_config().last_segment_splice_straight_line_length();
    PathPt straight_line_point = slot_.destination_point;
    if (dir == PathPt::Direction::BACKWARD) {
      straight_line_point.set_x(slot_.destination_point.x()
                                + cos(slot_.slot.heading()) * realtime_data_ptr_->destination_straight_line_length_);
      straight_line_point.set_y(slot_.destination_point.y()
                                + sin(slot_.slot.heading()) * realtime_data_ptr_->destination_straight_line_length_);
    } else {
      straight_line_point.set_x(slot_.destination_point.x()
                                - cos(slot_.slot.heading()) * realtime_data_ptr_->destination_straight_line_length_);
      straight_line_point.set_y(slot_.destination_point.y()
                                - sin(slot_.slot.heading()) * realtime_data_ptr_->destination_straight_line_length_);
    }
    realtime_data_ptr_->destination_point_ = straight_line_point;
    realtime_data_ptr_->destination_point_.set_theta(slot_.slot.heading());
    realtime_data_ptr_->destination_point_.set_direction(dir);
  }
  // 坐标系
  realtime_data_ptr_->tf_ego_2_map_ = local_view.getLocalizationPtr()->getTfEgo2Map();
  realtime_data_ptr_->tf_map_2_ego_ = local_view.getLocalizationPtr()->getTfMap2Ego();

  // ROI 和 障碍物
  RoiDecideResult roi;
  park_roi_decider_.getRoiDecideResuilt(roi);
  if (realtime_data_ptr_->obstacles_linesegments_ == nullptr) {
    realtime_data_ptr_->obstacles_linesegments_ = std::make_unique<std::vector<ObstaclesLinesegment>>();
  }
  *realtime_data_ptr_->obstacles_linesegments_ = roi.obstacles_linesegments_map;

  // 配置加载
  const std::string optimizer_name = "ipm_ocp";
  auto optimizer_profiles =
      ProfileManager::instance().profile<ParkingPathOptimizerProfile>(planner_profile_, optimizer_name);
  realtime_data_ptr_->optimizer_profiles_ = std::move(optimizer_profiles);
}

bool BaseParkingPlanner::endPointMoveToSafePosition(PathPt& end_pose, const LocalView& local_view,
                                                    const std::vector<Decision::DecisionObject>& select_ods,
                                                    const std::vector<math::LineSegment2d>& boundary_seg) {
  /** 找到安全可靠的搜索终点位置 **/
  /**
   * 1. 根据预设距离，确定默认搜索终点
   * 2. 判断当前点是否可达,若可达则直接返回
   * 3. 若当前点不可达，则向左、右侧方向探索，直到找到一个安全位置（注：不能超过地图边界<若有>）
   * 4. 若仍未找到安全位置，则向回（近端）探索一个位置（存在/不存在参考线：沿参考线反方向/沿当前点反方向），返回第2步
   * 5. 若多次向回仍未找到安全位置，超过向回极限阈值，则返回默认搜索终点
   */
  if (!planner_profile_->end_point_select_config().enable_end_pose_select()) {
    return false;
  }
  Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
  Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();
  math::Box2d vehicle_box =
      math::Box2d(end_pose, end_pose.theta(),
                  vehicle_config_->vehicle_param().front_edge_to_ego()
                      + planner_profile_->end_point_select_config().longit_safe_dist_threshold(),
                  vehicle_config_->vehicle_param().rear_edge_to_ego()
                      + planner_profile_->end_point_select_config().longit_safe_dist_threshold(),
                  vehicle_config_->vehicle_param().width()
                      + planner_profile_->end_point_select_config().lateral_safe_dist_threshold());

  // 先向前，再向左右侧，循环遍历出一个不碰撞的box
  const int explore_max_longit_num = planner_profile_->end_point_select_config().explore_max_longit_num();
  const double explore_longit_step = planner_profile_->end_point_select_config().explore_longit_step();
  const double explore_longit_heading = end_pose.theta();
  const int explore_max_lateral_num = planner_profile_->end_point_select_config().explore_max_lateral_num();
  const double explore_lateral_step = planner_profile_->end_point_select_config().explore_lateral_step();
  const double explore_lateral_heading = math::NormalizeAngle(end_pose.theta() + M_PI_2);

  bool is_found = false;
  auto current_box = vehicle_box;
  PathPt sample_pt;
  double lateral_shift = 0.0;
  double longit_shift = 0.0;

  bool sl_veh_point_valid = false;
  FrenetFramePoint sl_veh_point;
  for (int i = 0; i <= explore_max_longit_num; ++i) {
    // 向回移动
    current_box = vehicle_box;
    longit_shift = -explore_longit_step * i;

    // 若存在参考线，则沿着参考线采样
    if (target_reference_line_info_.isValid()) {
      auto ref = target_reference_line_info_.ref_line();

      if (!sl_veh_point_valid) {
        auto end_pose_map = end_pose;
        transfer::transformPoint(tf_ego_2_map, &end_pose_map);
        math::Vec3d ego_rpy_vec3d(0.0, 0.0, end_pose_map.theta());
        transfer::transformRPY(tf_ego_2_map, &ego_rpy_vec3d);
        end_pose_map.set_theta(ego_rpy_vec3d.z());
        sl_veh_point = ref.getFrenetPoint(PathPt(end_pose_map.x(), end_pose_map.y(), end_pose_map.z()));
        sl_veh_point_valid = true;
      }

      if (sl_veh_point.s() + longit_shift < 0.0) {
        break;
      }
      PARKING_LOG(D, "sl_veh_point: ", sl_veh_point.DebugString());
      PARKING_LOG(D, "longit_shift: ", longit_shift);
      auto sample_ref_pt = ref.getNearestReferencePoint(sl_veh_point.s() + longit_shift);
      sample_pt = PathPt(sample_ref_pt);
      sample_pt.set_theta(sample_ref_pt.heading());

      transfer::transformPoint(tf_map_2_ego, &sample_pt);
      math::Vec3d ego_rpy_vec3d = math::Vec3d(0.0, 0.0, sample_pt.theta());
      transfer::transformRPY(tf_map_2_ego, &ego_rpy_vec3d);
      sample_pt.set_theta(ego_rpy_vec3d.z());

      current_box = math::Box2d(sample_pt, sample_pt.theta(),
                                vehicle_config_->vehicle_param().front_edge_to_ego()
                                    + planner_profile_->end_point_select_config().longit_safe_dist_threshold(),
                                vehicle_config_->vehicle_param().rear_edge_to_ego()
                                    + planner_profile_->end_point_select_config().longit_safe_dist_threshold(),
                                vehicle_config_->vehicle_param().width()
                                    + planner_profile_->end_point_select_config().lateral_safe_dist_threshold());
    } else {
      current_box.Shift(math::Vec2d(longit_shift * std::cos(explore_longit_heading),
                                    longit_shift * std::sin(explore_longit_heading)));
    }
    PARKING_LOG(D, "current_box: ", current_box.DebugString());

    // 横向向左搜索
    auto left_box = current_box;
    for (int j = 0; j <= explore_max_lateral_num; ++j) {
      if (j > 0) {
        left_box.Shift(math::Vec2d(explore_lateral_step * std::cos(explore_lateral_heading),
                                   explore_lateral_step * std::sin(explore_lateral_heading)));
      }

      PARKING_LOG(D, "left_search_box: ", left_box.DebugString());
      if (boxIsCollided(boundary_seg, left_box)) {
        break;
      }
      if (!boxIsCollided(local_view, select_ods, left_box)) {
        lateral_shift = j * explore_lateral_step;
        is_found = true;
        break;
      }
    }

    // 横向向右搜索
    auto right_box = current_box;
    for (int j = -1; j >= -explore_max_lateral_num; --j) {
      right_box.Shift(math::Vec2d(-explore_lateral_step * std::cos(explore_lateral_heading),
                                  -explore_lateral_step * std::sin(explore_lateral_heading)));

      PARKING_LOG(D, "right_search_box: ", right_box.DebugString());
      if (boxIsCollided(boundary_seg, right_box)) {
        break;
      }
      if (!boxIsCollided(local_view, select_ods, right_box)) {
        if (is_found) {
          if (lateral_shift > -j * explore_lateral_step) {
            lateral_shift = j * explore_lateral_step;
          }
        }
        is_found = true;
        break;
      }
    }
    if (is_found) {
      break;
    }
  }

  if (!is_found) {
    debug_info_ += "WARN: 找不到安全的泊出终点\n";
    PARKING_LOG(D, "找不到安全的泊出终点");
    return false;
  } else {
    PARKING_LOG(D, "longit_shift: ", longit_shift);
    PARKING_LOG(D, "lateral_shift: ", lateral_shift);
    if (target_reference_line_info_.isValid()) {
      end_pose.set_x(sample_pt.x() + lateral_shift * std::cos(explore_lateral_heading));
      end_pose.set_y(sample_pt.y() + lateral_shift * std::sin(explore_lateral_heading));
    } else {
      end_pose.set_x(end_pose.x() + longit_shift * std::cos(explore_longit_heading)
                     + lateral_shift * std::cos(explore_lateral_heading));
      end_pose.set_y(end_pose.y() + longit_shift * std::sin(explore_longit_heading)
                     + lateral_shift * std::sin(explore_lateral_heading));
    }
  }
  return true;
}

}  // namespace gpal::pnc::planning