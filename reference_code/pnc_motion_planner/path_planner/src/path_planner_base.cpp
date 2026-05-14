/**
 * @file path_planner_base.cpp
 * @brief 路径规划器基类实现文件
 * @details 该文件实现了路径规划器的核心功能，包括路径初始化、路径优化、参考线加载、碰撞检测等。
 */

#include "path_planner/path_planner_base.h"
#include "base/singleton.h"
#include "config_manager/config_manager.h"

namespace gpal::pnc::planning {

/**
 * @brief 初始化路径规划器
 * @details 该函数用于初始化路径规划器，主要完成车辆配置的加载。通过单例模式获取配置管理器，并从中加载车辆配置信息。
 * 
 * @par 输入参数说明:
 * - 无显式输入参数
 * 
 * @par 关键变量说明:
 * - config_manager (ConfigManager*): 配置管理器的单例实例
 * - vehicle_config_ (std::shared_ptr<VehicleConfig>): 车辆配置信息的智能指针
 * 
 * @par 初始化流程:
 * 1. 获取配置管理器的单例实例
 * 2. 从配置管理器中加载车辆配置信息
 * 3. 返回初始化成功状态
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取配置管理器单例实例;
 * :加载车辆配置信息;
 * :返回初始化成功状态;
 * stop
 * @enduml
 * 
 * @return bool 初始化是否成功，true表示成功，false表示失败
 * 
 * @note 该函数应在路径规划器启动时调用，确保车辆配置信息正确加载
 * 
 * @warning 需确保配置管理器已正确初始化，且包含有效的车辆配置信息
 */
bool PathPlannerBase::init() {
  auto config_manager = Singleton<ConfigManager>::get_instance();
  vehicle_config_ = std::make_shared<VehicleConfig>(config_manager->vehicle_config());
  return true;
}

/**
 * @brief 优化路径
 * @details 该函数用于根据上一帧的路径数据和当前起点，优化生成新的路径。通过计算起点与上一帧路径的最近点，并检查航向角误差，确保路径的连续性和平滑性。
 * 
 * @param[in] prev_path_data 上一帧的路径数据，包含上一帧的路径点信息
 * @param[in] start_pt 当前起点，包含起点的位置和航向信息
 * @param[in,out] path_data 路径数据，用于存储优化后的路径
 * 
 * @par 关键变量说明:
 * - min_dist (double): 起点与上一帧路径的最近距离
 * - fpt (PathPt): 上一帧路径中与起点最近的点
 * - max_heading_error (double): 最大允许的航向角误差
 * - theta_diff_abs (double): 起点与最近点的航向角差值
 * - refine_path_points (std::vector<PathPt>): 优化后的路径点集合
 * 
 * @par 优化流程:
 * 1. 检查上一帧路径数据是否有效，若无效则直接返回
 * 2. 计算起点与上一帧路径的最近点
 * 3. 检查航向角误差是否超过最大允许值，若超过则直接返回
 * 4. 从最近点开始，沿上一帧路径生成新的路径点
 * 5. 将优化后的路径点存储到路径数据中
 * 
 * @par 流程图:
 * @startuml
 * start
 * if (上一帧路径数据是否有效?) then (否)
 *   :直接返回;
 * endif
 * :计算起点与上一帧路径的最近点;
 * if (航向角误差是否超过最大允许值?) then (是)
 *   :直接返回;
 * endif
 * :从最近点开始，沿上一帧路径生成新的路径点;
 * :将优化后的路径点存储到路径数据中;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径的连续性和平滑性
 * 
 * @warning 需确保输入参数有效，特别是prev_path_data和start_pt
 */
void PathPlannerBase::runRefinePath(const std::shared_ptr<PathData> prev_path_data, const TrajectoryPt& start_pt,
                                    PathData* path_data) {
  ERT_PLOG_I << "[PathPlannerBase::runRefinePath]: runRefinePath" ;
  if (prev_path_data == nullptr) return;
  if (prev_path_data->discretized_path().empty()) return;
  
  path_data->setPathLabel("prev");
  path_data->mutableDiscretizedPath()->clear(); 

  double min_dist = std::numeric_limits<double>::max();
  auto fpt = prev_path_data->discretized_path().getNearestPoint(start_pt.path_pt(), min_dist);
  constexpr double max_heading_error = M_PI / 4.0;
  double theta_diff_abs = std::abs(math::NormalizeAngle(fpt.theta() - start_pt.path_pt().theta()));
  if (theta_diff_abs >= max_heading_error) {
    std::cerr << "[PathPlannerBase::runRefinePath]: theta_diff_abs = " << theta_diff_abs 
              << " >= max_heading_error, so discretized_path_ is empty!" ;
    return;
  }

  std::vector<PathPt> refine_path_points(1, fpt);
  refine_path_points.front().set_s(0.0);
  for (double curr_length = 1.0; curr_length < 200.0; curr_length += 1.0) {
    double curr_s = fpt.s() + curr_length;
    if (curr_s > prev_path_data->discretized_path().back().s()) {
      refine_path_points.emplace_back(prev_path_data->discretized_path().back());
      refine_path_points.back().set_s(prev_path_data->discretized_path().back().s() - fpt.s());
      break;
    }
    refine_path_points.emplace_back(prev_path_data->discretized_path().evaluate(curr_s));
    refine_path_points.back().set_s(curr_length);
  }
  *path_data->mutableDiscretizedPath() = DiscretizedPath(refine_path_points);
}

/**
 * @brief 计算并加载路径点的曲率
 * @details 该函数用于计算路径点的曲率，并将曲率值加载到路径点中。通过三点法计算路径点的曲率，确保路径的平滑性和连续性。
 * 
 * @param[in,out] path 路径数据，包含路径点的位置信息
 * 
 * @par 关键变量说明:
 * - points (Eigen::Matrix2Xd): 路径点的位置矩阵，用于存储路径点的坐标
 * - params (std::vector<double>): 路径点的参数集合，用于存储路径点的曲率
 * - kappa (double): 路径点的曲率值
 * 
 * @par 计算流程:
 * 1. 检查路径点数量是否足够，若不足则直接返回
 * 2. 遍历路径点，通过三点法计算曲率
 * 3. 将曲率值加载到路径点中
 * 
 * @par 流程图:
 * @startuml
 * start
 * if (路径点数量是否足够?) then (否)
 *   :直接返回;
 * endif
 * :遍历路径点;
 * :通过三点法计算曲率;
 * :将曲率值加载到路径点中;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径点的曲率正确计算和加载
 * 
 * @warning 需确保输入参数有效，特别是path
 */
void PathPlannerBase::loadKappa(DiscretizedPath* path) {
  if (path->size() < 3) {
    return;
  }
  Eigen::Matrix2Xd points(2, path->size());
  std::vector<double> params;
  for (int i = 1; i + 1 < path->size(); i++) {
    double kappa = CalculateThreePointsCurvature(path->at(i - 1), path->at(i), path->at(i + 1));
    path->at(i - 1).set_kappa(kappa);
    if (i + 2 == path->size()) {
      path->at(i).set_kappa(kappa);
      path->at(i + 1).set_kappa(kappa);
    }
  }
}

/**
 * @brief 加载参考线并生成路径数据
 * @details 该函数用于将参考线加载到路径数据中，并根据起点生成离散化的路径点。通过将起点转换到Frenet坐标系，并沿参考线插值生成路径点，确保路径的连续性和平滑性。
 * 
 * @param[in] reference_line 参考线，包含参考线的位置和曲率信息
 * @param[in] start_pt 起点，包含起点的位置和航向信息
 * @param[in,out] path_data 路径数据，用于存储生成的路径点
 * 
 * @par 关键变量说明:
 * - fpt (std::pair<std::vector<double>, std::vector<double>>): 起点在Frenet坐标系中的位置
 * - start_s (double): 起点在参考线上的s坐标
 * - end_s (double): 路径的结束s坐标
 * - rpts (std::vector<ReferencePoint>): 沿参考线插值生成的路径点
 * 
 * @par 加载流程:
 * 1. 清空路径数据中的离散化路径点
 * 2. 将起点转换到Frenet坐标系
 * 3. 计算路径的结束s坐标
 * 4. 沿参考线插值生成路径点
 * 5. 将生成的路径点存储到路径数据中
 * 6. 如果参考线类型为RAW，则计算并加载路径点的曲率
 * 
 * @par 流程图:
 * @startuml
 * start
 * :清空路径数据中的离散化路径点;
 * :将起点转换到Frenet坐标系;
 * :计算路径的结束s坐标;
 * :沿参考线插值生成路径点;
 * :将生成的路径点存储到路径数据中;
 * if (参考线类型是否为RAW?) then (是)
 *   :计算并加载路径点的曲率;
 * endif
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保参考线正确加载到路径数据中
 * 
 * @warning 需确保输入参数有效，特别是reference_line和start_pt
 */
void PathPlannerBase::runLoadRefLine(const ReferenceLine& reference_line, const TrajectoryPt& start_pt,
                                     PathData* path_data) {
  ERT_PLOG_I << "[PathPlannerBase::runLoadRefLine]: runLoadRefLine" ;
  
  path_data->setPathLabel("reference_line");
  path_data->setReferenceLine(&reference_line);
  path_data->mutableDiscretizedPath()->clear(); 

  auto fpt = reference_line.toFrenetFrame(start_pt);
  double start_s = fpt.first[0];
  // float speed_limit = reference_line.GetSpeedLimitFromS(start_s) * MS_KMH;
  DrivingDirection segment_direction = reference_line.getDirectionFromS(start_s).direction;
  const float path_length_thresold = 150.0;
  double end_s = std::fmin(start_s + path_length_thresold, reference_line.length());
  // ERT_PLOG_I << "sl projection: s = " << fpt.first[0] << " l = "<< fpt.second[0]
  //           << "  start_s = " << start_s << " end_s = "<< end_s
  //           << "  speed_limit = " << speed_limit
  //           << "  direction = " << static_cast<int>(segment_direction)
  //           << "  start_pt_x = " << start_pt.path_pt().x() << "  start_pt_y = " << start_pt.path_pt().y() 
  //           << "  start_pt_theta = " << start_pt.path_pt().theta() ;
  const auto& rpts = reference_line.getInterpolatedRefPoints(start_s, end_s, 1.0);
  for (const auto& rpt : rpts) {
    path_data->mutableDiscretizedPath()->emplace_back(rpt.x(), rpt.y(), rpt.z(), rpt.slope(), rpt.heading(), rpt.kappa(),
                                                      rpt.local_s() - rpts.front().local_s(), rpt.dkappa(), 0.0);
    path_data->mutableDiscretizedPath()->back().set_ref_s(rpt.local_s());
  }
  if (reference_line.smooth_type() == ReferenceLine::SmoothType::RAW) {
    loadKappa(path_data->mutableDiscretizedPath());
  }
  
  // ERT_PLOG_I << "[runLoadRefLine]: reference_line.id() = " <<reference_line.id()
  //           << "  global_start_s = " <<reference_line.global_start_s()
  //           << "  length = " << reference_line.length()
  //           << "  num_points = " << reference_line.num_points();  
  // if(reference_line.reference_points().size() > 0) {
  //   for(int i = 0; i < std::fmin(20, reference_line.reference_points().size()); ++i) {
  //     ERT_PLOG_I << "i = "<< i
  //               << " x = " << reference_line.reference_points().at(i).x()
  //               << " y = " << reference_line.reference_points().at(i).y()
  //               << " theta = " << reference_line.reference_points().at(i).heading()
  //               << " local_s = " << reference_line.reference_points().at(i).local_s()
  //               << " kappa = " << reference_line.reference_points().at(i).kappa()
  //               ;
  //   }   
  // }

  // ERT_PLOG_I << "[runLoadRefLine]: discretized_path.size() = " <<path_data->discretized_path().size();
  // if(path_data->discretized_path().size() > 0) {
  //   for(int i = 0; i < std::min<int>(10, path_data->discretized_path().size()); ++i) {
  //     ERT_PLOG_I << "i = "<< i
  //               << " x = " << path_data->discretized_path().at(i).x()
  //               << " y = " << path_data->discretized_path().at(i).y()
  //               << " theta = " << path_data->discretized_path().at(i).theta()
  //               << " s = " << path_data->discretized_path().at(i).s()
  //               << " kappa = " << path_data->discretized_path().at(i).kappa()
  //               << " dkappa = " << path_data->discretized_path().at(i).dkappa()
  //               << " direction = " << static_cast<int>(path_data->discretized_path().at(i).direction())
  //               ;
  //   }   
  //}
}

/**
 * @brief 碰撞检测
 * @details 该函数用于检测路径点是否与自由空间中的障碍物发生碰撞。通过遍历路径点，生成车辆的包围盒，并检查包围盒是否与自由空间中的障碍物发生碰撞，确保路径的安全性。
 * 
 * @param[in] freespace 自由空间，包含障碍物的位置信息
 * @param[in] path 路径点集合，包含路径点的位置和航向信息
 * @param[in] collision_check_buffer 碰撞检测缓冲区，用于扩展车辆的包围盒
 * @param[in] corner_width 角落宽度，用于扩展车辆的包围盒
 * @param[in] enable_curve_decide_process 是否启用曲线决策过程
 * @param[in] curve_look_ahead_distance 曲线检测的前瞻距离
 * @param[in] curve_kappa_thresold 曲线检测的曲率阈值
 * @param[in] side_box_length 侧边检测盒的长度
 * @param[in] side_box_width 侧边检测盒的宽度
 * 
 * @par 关键变量说明:
 * - block_fs_info (PathData::BlockFSInfo): 碰撞检测结果，包含碰撞点的位置和方向信息
 * - length (double): 车辆的长度
 * - width (double): 车辆的宽度
 * - l_center (double): 车辆中心到后轴的距离
 * - is_left_turn (bool): 是否左转
 * - is_right_turn (bool): 是否右转
 * - box (math::Box2d): 车辆的包围盒
 * - key_vec (math::Vec2d): 碰撞点的位置
 * - side_box (math::Box2d): 侧边检测盒
 * - side_key_vec (math::Vec2d): 侧边碰撞点的位置
 * 
 * @par 检测流程:
 * 1. 检查路径点是否为空，若为空则直接返回
 * 2. 初始化碰撞检测结果
 * 3. 遍历路径点，生成车辆的包围盒
 * 4. 检查包围盒是否与自由空间中的障碍物发生碰撞
 * 5. 如果启用曲线决策过程，则生成侧边检测盒并检查是否发生碰撞
 * 6. 返回碰撞检测结果
 * 
 * @par 流程图:
 * @startuml
 * start
 * if (路径点是否为空?) then (是)
 *   :直接返回;
 * endif
 * :初始化碰撞检测结果;
 * :遍历路径点;
 * :生成车辆的包围盒;
 * if (包围盒是否与障碍物发生碰撞?) then (是)
 *   :记录碰撞点信息;
 * else if (是否启用曲线决策过程?) then (是)
 *   :生成侧边检测盒;
 *   if (侧边检测盒是否与障碍物发生碰撞?) then (是)
 *     :记录侧边碰撞点信息;
 *   endif
 * endif
 * :返回碰撞检测结果;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径的安全性
 * 
 * @warning 需确保输入参数有效，特别是freespace和path
 */
PathData::BlockFSInfo PathPlannerBase::collisionCheck(const Freespace& freespace, const std::vector<PathPt>& path,
                                                      const double& collision_check_buffer,
                                                      const double& corner_width,
                                                      const bool& enable_curve_decide_process,
                                                      const double& curve_look_ahead_distance,
                                                      const double& curve_kappa_thresold,                                                    
                                                      const double& side_box_length,
                                                      const double& side_box_width) {
  PathData::BlockFSInfo block_fs_info;
  if (path.empty()) {
    return block_fs_info;
  }

  double length = vehicle_config_->vehicle_param().length();
  double width = vehicle_config_->vehicle_param().width();
  double l_center = length / 2 - vehicle_config_->vehicle_param().rear_edge_to_ego();

  bool is_left_turn = false;
  bool is_right_turn = false;
  decideInCurve(path, curve_look_ahead_distance, curve_kappa_thresold, is_left_turn, is_right_turn);

  PathPt last_pt = path.front();
  for (auto& path_pt : path) {
    math::Vec2d center(path_pt.x() + l_center * cos(path_pt.theta()), path_pt.y() + l_center * sin(path_pt.theta()));
    math::Box2d box(center, path_pt.theta(), length, width + collision_check_buffer);
    math::Vec2d key_vec;
    if (freespace.isOccupied(box, &key_vec, corner_width)) {
      const double step = 0.01;
      double dis = last_pt.DistanceTo(path_pt);
      int inter_num = std::max(1, static_cast<int>(dis / step));
      for (int i = 0; i <= inter_num; ++i) {
        PathPt temp_pt = interpolateUsingLinearApproximation(last_pt, path_pt, last_pt.s() + (double)i / inter_num * dis);
        center =
            math::Vec2d(temp_pt.x() + l_center * cos(temp_pt.theta()), temp_pt.y() + l_center * sin(temp_pt.theta()));
        box = math::Box2d(center, temp_pt.theta(), length, width + collision_check_buffer);
        if (freespace.isOccupied(box, &key_vec, corner_width)) {
          // fill in freespace block info
          block_fs_info.is_valid = true;
          block_fs_info.s = temp_pt.s();
          block_fs_info.check_point = temp_pt;
          block_fs_info.block_point = key_vec;

          block_fs_info.vis_pts = box.GetAllCorners();
          block_fs_info.vis_pts.emplace_back(block_fs_info.vis_pts.front());
          block_fs_info.vis_pts.emplace_back(key_vec);
          block_fs_info.vis_pts.emplace_back(temp_pt.x(), temp_pt.y());
          return block_fs_info;
        }
      }
    } else if (enable_curve_decide_process && (is_right_turn || is_left_turn)) {
      // add more checks for curve
      bool is_right = is_right_turn ? true : false;
      auto side_box = generateSideCheckBox(box, is_right, side_box_length, side_box_width);
      math::Vec2d side_key_vec;
      if (freespace.isOccupied(side_box, &side_key_vec)) {
        const double step = 0.01;
        double dis = last_pt.DistanceTo(path_pt);
        int inter_num = std::max(1, static_cast<int>(dis / step));
        for (int i = 0; i <= inter_num; ++i) {
          PathPt temp_pt = interpolateUsingLinearApproximation(last_pt, path_pt, last_pt.s() + (double)i / inter_num * dis);
          center =
              math::Vec2d(temp_pt.x() + l_center * cos(temp_pt.theta()), temp_pt.y() + l_center * sin(temp_pt.theta()));
          box = math::Box2d(center, temp_pt.theta(), length, width + collision_check_buffer);
          side_box = generateSideCheckBox(box, is_right, side_box_length, side_box_width);
          if (freespace.isOccupied(side_box, &side_key_vec)) {
            block_fs_info.is_valid = true;
            block_fs_info.s = temp_pt.s();
            block_fs_info.check_point = temp_pt;
            block_fs_info.block_point = side_key_vec;    
    
            block_fs_info.vis_pts = box.GetAllCorners();
            block_fs_info.vis_pts.emplace_back(block_fs_info.vis_pts.front()); 
            auto side_box_pts = side_box.GetAllCorners();
            block_fs_info.vis_pts.insert(block_fs_info.vis_pts.end(), side_box_pts.begin(), side_box_pts.end());
            block_fs_info.vis_pts.emplace_back(side_box_pts.front());
            block_fs_info.vis_pts.emplace_back(side_key_vec);
            block_fs_info.vis_pts.emplace_back(temp_pt.x(), temp_pt.y());
            return block_fs_info;
          }
        }
      }
    }
    last_pt = path_pt;
  }

  return block_fs_info;
}

PathData::BlockFSInfo PathPlannerBase::collisionCheck(const std::vector<math::LineSegment2d>& boundary_segs,
                                                      const std::vector<PathPt>& path,
                                                      const double& collision_check_buffer,
                                                      const bool& enable_curve_decide_process,
                                                      const double& curve_look_ahead_distance,
                                                      const double& curve_kappa_thresold,                                                    
                                                      const double& side_box_length,
                                                      const double& side_box_width) {
  auto calSegmentsCollision = [&](const math::Box2d& box) -> bool {
    for(const auto& seg : boundary_segs){
      if(box.HasOverlap(seg)){
        return true;
      }
    }
    return false;
  };

  PathData::BlockFSInfo block_fs_info;
  if (path.empty()) {
    return block_fs_info;
  }

  double length = vehicle_config_->vehicle_param().length();
  double width = vehicle_config_->vehicle_param().width();
  double l_center = length / 2 - vehicle_config_->vehicle_param().rear_edge_to_ego();

  bool is_left_turn = false;
  bool is_right_turn = false;
  decideInCurve(path, curve_look_ahead_distance, curve_kappa_thresold, is_left_turn, is_right_turn);

  PathPt last_pt = path.front();
  for (auto& path_pt : path) {
    math::Vec2d center(path_pt.x() + l_center * cos(path_pt.theta()), path_pt.y() + l_center * sin(path_pt.theta()));
    math::Box2d box(center, path_pt.theta(), length, width + collision_check_buffer);
    if (calSegmentsCollision(box)) {
      const double step = 0.01;
      double dis = last_pt.DistanceTo(path_pt);
      int inter_num = std::max(1, static_cast<int>(dis / step));
      for (int i = 0; i <= inter_num; ++i) {
        PathPt temp_pt = interpolateUsingLinearApproximation(last_pt, path_pt, last_pt.s() + (double)i / inter_num * dis);
        center =
            math::Vec2d(temp_pt.x() + l_center * cos(temp_pt.theta()), temp_pt.y() + l_center * sin(temp_pt.theta()));
        box = math::Box2d(center, temp_pt.theta(), length, width + collision_check_buffer);
        if (calSegmentsCollision(box)) {
          // fill in freespace block info
          block_fs_info.is_valid = true;
          block_fs_info.s = temp_pt.s();
          block_fs_info.check_point = temp_pt;
          block_fs_info.block_point = temp_pt;

          block_fs_info.vis_pts = box.GetAllCorners();
          block_fs_info.vis_pts.emplace_back(block_fs_info.vis_pts.front());
          block_fs_info.vis_pts.emplace_back(temp_pt.x(), temp_pt.y());
          return block_fs_info;
        }
      }
    } else if (enable_curve_decide_process && (is_right_turn || is_left_turn)) {
      // add more checks for curve
      bool is_right = is_right_turn ? true : false;
      auto side_box = generateSideCheckBox(box, is_right, side_box_length, side_box_width);
      if (calSegmentsCollision(side_box)) {
        const double step = 0.01;
        double dis = last_pt.DistanceTo(path_pt);
        int inter_num = std::max(1, static_cast<int>(dis / step));
        for (int i = 0; i <= inter_num; ++i) {
          PathPt temp_pt = interpolateUsingLinearApproximation(last_pt, path_pt, last_pt.s() + (double)i / inter_num * dis);
          center =
              math::Vec2d(temp_pt.x() + l_center * cos(temp_pt.theta()), temp_pt.y() + l_center * sin(temp_pt.theta()));
          box = math::Box2d(center, temp_pt.theta(), length, width + collision_check_buffer);
          side_box = generateSideCheckBox(box, is_right, side_box_length, side_box_width);
          if (calSegmentsCollision(side_box)) {
            block_fs_info.is_valid = true;
            block_fs_info.s = temp_pt.s();
            block_fs_info.check_point = temp_pt;
            block_fs_info.block_point = temp_pt;    
    
            block_fs_info.vis_pts = box.GetAllCorners();
            block_fs_info.vis_pts.emplace_back(block_fs_info.vis_pts.front()); 
            auto side_box_pts = side_box.GetAllCorners();
            block_fs_info.vis_pts.insert(block_fs_info.vis_pts.end(), side_box_pts.begin(), side_box_pts.end());
            block_fs_info.vis_pts.emplace_back(side_box_pts.front());
            block_fs_info.vis_pts.emplace_back(temp_pt.x(), temp_pt.y());
            return block_fs_info;
          }
        }
      }
    }
    last_pt = path_pt;
  }

  return block_fs_info;
}

/**
 * @brief 判断车辆是否处于转弯状态
 * @details 根据路径点的曲率信息判断车辆是否处于左转或右转状态。通过分析路径点在前瞻距离内的曲率值，判断车辆的转向意图。
 * 
 * @param[in] path 路径点集合，包含路径点的位置和曲率信息
 * @param[in] curve_look_ahead_distance 曲线检测的前瞻距离，范围[0, +∞)，单位米
 * @param[in] curve_kappa_thresold 曲线检测的曲率阈值，范围[0, +∞)，单位1/米
 * @param[out] is_left_turn 是否左转，true表示左转，false表示非左转
 * @param[out] is_right_turn 是否右转，true表示右转，false表示非右转
 * 
 * @par 输入参数说明:
 * - path: 必须包含至少2个路径点，且路径点需包含有效的曲率信息
 * - curve_look_ahead_distance: 建议取值范围[10, 50]米
 * - curve_kappa_thresold: 建议取值范围[0.01, 0.1] 1/米
 * 
 * @par 关键变量说明:
 * - curr_ind (size_t): 当前路径点索引，用于定位前瞻距离内的路径点
 * - kappa (double): 路径点的曲率值，范围[-∞, +∞]，单位1/米
 * 
 * @par 判断条件:
 * - 左转条件: kappa > curve_kappa_thresold
 * - 右转条件: kappa < -curve_kappa_thresold
 * 
 * @par 流程图:
 * @startuml
 * start
 * if (路径点数量是否足够?) then (否)
 *   :直接返回;
 * endif
 * :初始化curr_ind;
 * partition 前瞻距离检测 {
 * while (遍历路径点?) is (是)
 * if (路径点s值 > curve_look_ahead_distance?) then (是)
 *   :更新curr_ind;
 *   :跳出循环;
 * endif
 * endwhile
 * }
 * if (curr_ind点曲率 > curve_kappa_thresold?) then (是)
 *   :标记为左转;
 * else if (curr_ind点曲率 < -curve_kappa_thresold?) then (是)
 *   :标记为右转;
 * else (否)
 *   :检查起点曲率;
 *   if (起点曲率 > curve_kappa_thresold?) then (是)
 *     :标记为左转;
 *   else if (起点曲率 < -curve_kappa_thresold?) then (是)
 *     :标记为右转;
 *   endif
 * endif
 * stop
 * @enduml
 * 
 * @note 包含两种检测逻辑:
 * 1. 前瞻距离检测：检测curve_look_ahead_distance距离内的曲率
 * 2. 起点检测：当前瞻距离内未检测到转向时，检测起点曲率
 * 
 * @warning 需确保path参数有效，且包含足够的路径点信息
 */
void PathPlannerBase::decideInCurve(const std::vector<PathPt>& path, 
                                    const double& curve_look_ahead_distance,
                                    const double& curve_kappa_thresold,
                                    bool& is_left_turn, bool& is_right_turn) {
  if (path.size() < 2) {
    return;
  }

  size_t curr_ind = 0;
  for (size_t i = 0; i < path.size(); i++) {
    if (path[i].s() > curve_look_ahead_distance) {
      curr_ind = i;
      break;
    }
  }
  if (path.at(curr_ind).kappa() > curve_kappa_thresold) {
    is_left_turn = true;
  } else if (path.at(curr_ind).kappa() < -curve_kappa_thresold) {
    is_right_turn = true;
  }
  if (is_left_turn || is_right_turn) {
    return;
  }

  if (path.at(0).kappa() > curve_kappa_thresold) {
    is_left_turn = true;
  } else if (path.at(0).kappa() < -curve_kappa_thresold) {
    is_right_turn = true;
  }
}

/**
 * @brief 生成侧边检测盒
 * @details 该函数用于在车辆包围盒的基础上生成侧边检测盒，用于在转弯时进行额外的碰撞检测。通过计算侧边检测盒的中心位置和方向，确保检测盒能够覆盖车辆的侧边区域。
 * 
 * @param[in] box 车辆的主包围盒，包含中心位置、方向和尺寸信息
 * @param[in] is_right 是否生成右侧检测盒，true表示右侧，false表示左侧
 * @param[in] length 侧边检测盒的长度，范围[0, +∞)，单位米
 * @param[in] width 侧边检测盒的宽度，范围[0, +∞)，单位米
 * 
 * @par 输入参数说明:
 * - box: 必须包含有效的中心位置、方向和尺寸信息
 * - is_right: 用于确定生成左侧还是右侧检测盒
 * - length: 建议取值范围[1, 5]米
 * - width: 建议取值范围[0.5, 2]米
 * 
 * @par 关键变量说明:
 * - box_width (double): 主包围盒的宽度，单位米
 * - box_length (double): 主包围盒的长度，单位米
 * - x (double): 主包围盒中心的x坐标，单位米
 * - y (double): 主包围盒中心的y坐标，单位米
 * - theta (double): 主包围盒的方向角，范围[-π, π]，单位弧度
 * - cos_theta (double): 方向角的余弦值
 * - sin_theta (double): 方向角的正弦值
 * - dy (double): 侧边检测盒在y轴方向的偏移量，单位米
 * - dx (double): 侧边检测盒在x轴方向的偏移量，单位米
 * - center_x (double): 侧边检测盒中心的x坐标，单位米
 * - center_y (double): 侧边检测盒中心的y坐标，单位米
 * 
 * @par 计算流程:
 * 1. 获取主包围盒的尺寸和方向信息
 * 2. 根据is_right参数计算侧边检测盒的偏移量
 * 3. 计算侧边检测盒的中心位置
 * 4. 生成并返回侧边检测盒
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取主包围盒尺寸和方向信息;
 * if (是否生成右侧检测盒?) then (是)
 *   :计算右侧偏移量;
 * else (否)
 *   :计算左侧偏移量;
 * endif
 * :计算侧边检测盒中心位置;
 * :生成侧边检测盒;
 * stop
 * @enduml
 * 
 * @note 该函数主要用于在车辆转弯时生成额外的检测盒，以提高碰撞检测的准确性
 * 
 * @warning 需确保输入参数有效，特别是box参数必须包含有效的中心位置和方向信息
 */
math::Box2d PathPlannerBase::generateSideCheckBox(const math::Box2d& box, const bool& is_right, 
                                                  const double& length, const double& width) {
  double box_width = box.width();
  double box_length = box.length();
  double x = box.center_x();
  double y = box.center_y();
  double theta = box.heading();
  double cos_theta = box.cos_heading();
  double sin_theta = box.sin_heading();

  double dy = 0;
  if (is_right) {
    dy = box_width / 2 + width / 2;
  } else {
    dy = -box_width / 2 - width / 2;
  }
  double dx = -box_length / 2 + length / 2;

  // calculate the center of check box
  double center_x = x + dx * cos_theta - dy * sin_theta;
  double center_y = y + dx * sin_theta + dy * cos_theta;
  math::Vec2d center(center_x, center_y);
  math::Box2d check_box(center, theta, length, width);
  return check_box;
}

/**
 * @brief 获取碰撞点的方向
 * @details 该函数用于确定碰撞点相对于检查点的方向（左侧或右侧）。通过将碰撞点转换到以检查点为原点的局部坐标系，并根据y坐标的正负判断碰撞方向。
 * 
 * @param[in] check_point 检查点，包含位置和航向信息
 * @param[in] collision_point 碰撞点，包含位置信息
 * 
 * @par 输入参数说明:
 * - check_point: 必须包含有效的位置和航向信息
 * - collision_point: 必须包含有效的位置信息
 * 
 * @par 关键变量说明:
 * - x_shifted (double): 碰撞点在全局坐标系中相对于检查点的x偏移量，单位米
 * - y_shifted (double): 碰撞点在全局坐标系中相对于检查点的y偏移量，单位米
 * - check_point_heading (double): 检查点的航向角，范围[-π, π]，单位弧度
 * - x_rotated (double): 碰撞点在局部坐标系中的x坐标，单位米
 * - y_rotated (double): 碰撞点在局部坐标系中的y坐标，单位米
 * 
 * @par 计算流程:
 * 1. 计算碰撞点相对于检查点的偏移量
 * 2. 将碰撞点转换到以检查点为原点的局部坐标系
 * 3. 根据局部坐标系中的y坐标判断碰撞方向
 * 
 * @par 流程图:
 * @startuml
 * start
 * :计算碰撞点相对于检查点的偏移量;
 * :将碰撞点转换到局部坐标系;
 * if (局部坐标系y坐标 < 0?) then (是)
 *   :返回右侧碰撞;
 * else (否)
 *   :返回左侧碰撞;
 * endif
 * stop
 * @enduml
 * 
 * @note 该函数主要用于在碰撞检测后确定碰撞点的相对方向，以便进行后续处理
 * 
 * @warning 需确保输入参数有效，特别是check_point和collision_point
 */
PathData::BlockPointDirection PathPlannerBase::getBlockFsPointDirection(const PathPt& check_point,
                                                                        const math::Vec2d& collision_point) {
  // 将collision_point转换至以check_point为坐标原点的坐标系中.
  const double x_shifted = collision_point.x() - check_point.x();
  const double y_shifted = collision_point.y() - check_point.y();
  const double check_point_heading = check_point.theta();
  const double x_rotated = std::cos(check_point_heading) * x_shifted + std::sin(check_point_heading) * y_shifted;
  const double y_rotated = -1.0 * std::sin(check_point_heading) * x_shifted + std::cos(check_point_heading) * y_shifted;
  if (y_rotated < 0.0) { 
    // collision_point在y轴右侧，即右侧发生碰撞.
    return PathData::BlockPointDirection::RIGHT; // collision point is on the right, so proc the right bound
  } else {
    // collision_point在y轴左侧，即左侧发生碰撞.
    return PathData::BlockPointDirection::LEFT;
  }
}


}
