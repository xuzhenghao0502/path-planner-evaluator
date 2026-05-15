/**
 * @file reference_line.cpp
 * @brief 参考线模块实现文件
 * @details
 * 该文件实现了参考线的核心功能，包括参考线的初始化、更新、查询等操作。参考线用于描述车辆行驶的路径，包含路径的几何信息、速度限制、停止线等。
 */

#include "reference_line/reference_line.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

#include "base/log.h"
#include "boost/math/tools/minima.hpp"
#include "math/angle.h"
#include "math/cartesian_frenet_conversion.h"
#include "math/linear_interpolation.h"
#include "math/vec2d.h"
#include "proto/common/pnc_point.pb.h"
#include "util/string_util.h"
#include "util/util.h"

namespace gpal::pnc::planning {

using gpal::pnc::SLPoint;
using math::CartesianFrenetConverter;
using math::Vec3d;
using util::DistanceXY;

/**
 * @brief ReferenceLine构造函数
 * @details 根据给定的三维点集初始化参考线对象，并计算参考线的几何信息（如长度、曲率等）
 *
 * @param[in] xyz_points 三维点集，用于构建参考线
 * @par 输入参数说明:
 * - xyz_points: 必须包含至少2个点，类型为std::vector<math::Vec3d>
 *
 * @par 关键变量说明:
 * - num_points_ (int): 参考线的点数，范围[2, +∞)
 * - reference_points_ (std::vector<ReferencePoint>): 参考点列表，包含每个点的位置、航向、曲率等信息
 * - accumulated_s_ (std::vector<double>): 累计距离数组，存储每个点到起点的累计距离
 * - line_segments_ (std::vector<math::LineSegment2d>): 线段列表，由相邻点生成
 * - length_ (double): 参考线总长度，范围[0, +∞)
 * - num_line_segments_ (int): 线段数量，范围[1, +∞)
 *
 * @par 判断条件:
 * - 如果点集为空或点数小于2，则无法构建参考线
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化成员变量;
 * :遍历点集;
 * :计算累计距离;
 * :生成参考点;
 * :计算航向角;
 * :生成线段;
 * :更新参考线长度;
 * :清空停止线、速度限制等信息;
 * :更新驾驶管缓冲区;
 * :计算曲率和曲率变化率;
 * stop
 * @enduml
 *
 * @note 该函数用于根据点集初始化参考线，适用于从外部数据源（如地图、感知等）构建参考线
 *
 * @warning 需确保输入的点集包含至少2个点，否则无法正确初始化参考线
 */
ReferenceLine::ReferenceLine(const std::vector<math::Vec3d>& xyz_points) {
  num_points_ = static_cast<int>(xyz_points.size());
  reference_points_.clear();
  reference_points_.reserve(num_points_);
  accumulated_s_.clear();
  accumulated_s_.reserve(num_points_);
  line_segments_.clear();
  line_segments_.reserve(num_points_);
  is_memorized_park_route_ = false;
  is_memorized_end_route_ = false;
  is_terminal_enforced_ = false;
  is_parallel_virtual_ = false;
  is_bound_modified_by_lane_ = false;

  double s = 0.0;
  for (int i = 0; i < num_points_; ++i) {
    accumulated_s_.push_back(s);
    reference_points_.emplace_back(xyz_points[i], 0.0, 0.0, 0.0, 0.0, 1.7, -1.7, 1.7, -1.7, 0.0, s);
    math::Vec3d heading;
    if (i + 1 >= num_points_) {
      if (i - 1 >= 0) {
        heading = xyz_points[i] - xyz_points[i - 1];
      }
    } else {
      line_segments_.emplace_back(xyz_points[i], xyz_points[i + 1]);
      heading = xyz_points[i + 1] - xyz_points[i];
      s += heading.Length();
    }
    heading.Normalize();
    reference_points_.back().setHeading(heading.Angle());
  }
  length_ = s;
  num_line_segments_ = num_points_ - 1;

  stop_lines_.clear();
  speed_limits_.clear();
  directions_.clear();
  source_infos_.clear();
  navigation_lane_change_info_ = {-1.0, 0, 0};
  boundary_types_.clear();
  gates_.clear();
  left_reference_lines_.clear();
  right_reference_lines_.clear();
  areas_.clear();
  road_info_.clear();
  merge_fork_ranges_.clear();
  navigation_lane_change_ranges_.clear();
  lane_follow_ranges_.clear();
  navigation_scores_.clear();
}

/**
 * @brief ReferenceLine构造函数
 * @details 根据给定的参考点集初始化参考线对象，并计算参考线的几何信息（如长度、曲率等）
 *
 * @param[in] reference_points 参考点集，用于构建参考线
 * @par 输入参数说明:
 * - reference_points: 必须包含至少2个点，类型为std::vector<ReferencePoint>
 *
 * @par 关键变量说明:
 * - num_points_ (int): 参考线的点数，范围[2, +∞)
 * - reference_points_ (std::vector<ReferencePoint>): 参考点列表，包含每个点的位置、航向、曲率等信息
 * - accumulated_s_ (std::vector<double>): 累计距离数组，存储每个点到起点的累计距离
 * - line_segments_ (std::vector<math::LineSegment2d>): 线段列表，由相邻点生成
 * - length_ (double): 参考线总长度，范围[0, +∞)
 * - num_line_segments_ (int): 线段数量，范围[1, +∞)
 *
 * @par 判断条件:
 * - 如果点集为空或点数小于2，则无法构建参考线
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化成员变量;
 * :遍历参考点集;
 * :计算累计距离;
 * :生成线段;
 * :更新参考线长度;
 * :清空停止线、速度限制等信息;
 * :更新驾驶管缓冲区;
 * :计算曲率和曲率变化率;
 * stop
 * @enduml
 *
 * @note 该函数用于根据参考点集初始化参考线，适用于从外部数据源（如地图、感知等）构建参考线
 *
 * @warning 需确保输入的参考点集包含至少2个点，否则无法正确初始化参考线
 */
ReferenceLine::ReferenceLine(const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points) {
  num_points_ = static_cast<int>(reference_points.size());
  accumulated_s_.clear();
  accumulated_s_.reserve(num_points_);
  line_segments_.clear();
  line_segments_.reserve(num_points_);
  is_memorized_park_route_ = false;
  is_memorized_end_route_ = false;
  is_terminal_enforced_ = false;
  is_parallel_virtual_ = false;
  is_bound_modified_by_lane_ = false;

  double s = 0.0;
  for (int i = 0; i < num_points_; ++i) {
    accumulated_s_.push_back(s);
    reference_points_[i].setLocalS(s);
    Vec3d heading;
    if (i + 1 >= num_points_) {
      if (i - 1 >= 0) {
        heading = reference_points[i] - reference_points[i - 1];
      }
    } else {
      line_segments_.emplace_back(reference_points[i], reference_points[i + 1]);
      heading = reference_points[i + 1] - reference_points[i];
      s += heading.Length();
    }
    heading.Normalize();
  }
  length_ = s;
  num_line_segments_ = num_points_ - 1;

  stop_lines_.clear();
  speed_limits_.clear();
  directions_.clear();
  source_infos_.clear();
  navigation_lane_change_info_ = {-1.0, 0, 0};
  boundary_types_.clear();
  gates_.clear();
  left_reference_lines_.clear();
  right_reference_lines_.clear();
  areas_.clear();
  road_info_.clear();
  merge_fork_ranges_.clear();
  navigation_lane_change_ranges_.clear();
  lane_follow_ranges_.clear();
  navigation_scores_.clear();
}

/**
 * @brief ReferenceLine构造函数
 * @details 根据给定的切片路线初始化参考线对象，并计算参考线的几何信息（如长度、曲率等）
 *
 * @param[in] route 切片路线指针，用于构建参考线
 * @par 输入参数说明:
 * - route: 必须为非空指针，类型为SlicedRoute*
 *
 * @par 关键变量说明:
 * - id_ (std::string): 参考线ID，全局唯一标识
 * - global_start_s_ (double): 全局起始s值，范围[0, +∞)
 * - reference_points_ (std::vector<ReferencePoint>): 参考点列表，包含每个点的位置、航向、曲率等信息
 * - accumulated_s_ (std::vector<double>): 累计距离数组，存储每个点到起点的累计距离
 * - line_segments_ (std::vector<math::LineSegment2d>): 线段列表，由相邻点生成
 * - length_ (double): 参考线总长度，范围[0, +∞)
 * - num_line_segments_ (int): 线段数量，范围[1, +∞)
 * - line_type_ (LineType): 参考线类型，枚举值
 * - is_memorized_park_route_ (bool): 是否为记忆停车路线
 * - is_memorized_end_route_ (bool): 是否为记忆终点路线
 * - is_terminal_enforced_ (bool): 是否为强制终点
 * - is_parallel_virtual_ (bool): 是否为平行虚拟线
 * - is_bound_modified_by_lane_ (bool): 是否被车道边界修改
 *
 * @par 判断条件:
 * - 如果切片路线为空，则无法构建参考线
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化成员变量;
 * :遍历切片路线的点集;
 * :计算累计距离;
 * :生成参考点;
 * :计算航向角;
 * :生成线段;
 * :更新参考线长度;
 * :清空停止线、速度限制等信息;
 * :更新速度限制;
 * :更新方向信息;
 * :更新停止线;
 * :更新车道变换范围;
 * :更新车道线巡线范围;
 * :更新车道类型;
 * :更新车道区域类型;
 * :更新区域重叠信息;
 * :更新驾驶管缓冲区;
 * :计算曲率和曲率变化率;
 * stop
 * @enduml
 *
 * @note 该函数用于根据切片路线初始化参考线，适用于从记忆路线构建参考线
 *
 * @warning 需确保输入的切片路线指针非空，否则无法正确初始化参考线
 */
ReferenceLine::ReferenceLine(const SlicedRoute* route) {
  CHECK_NOTNULL(route);
  id_ = route->id();
  global_start_s_ = route->globalStartS();
  const auto& route_points = route->pts();
  num_points_ = static_cast<int>(route_points.size());
  reference_points_.clear();
  reference_points_.reserve(num_points_);
  accumulated_s_.clear();
  accumulated_s_.reserve(num_points_);
  line_segments_.clear();
  line_segments_.reserve(num_points_);
  line_type_ = LineType::MEMORIZED_ROUTE;
  is_memorized_park_route_ = route->isParkSlicedRoute();
  is_memorized_end_route_ = route->isEndSlicedRoute();
  is_terminal_enforced_ = is_memorized_park_route_ && is_memorized_end_route_;
  is_parallel_virtual_ = false;
  is_bound_modified_by_lane_ = false;

  double s = 0.0;
  for (int i = 0; i < num_points_; ++i) {
    accumulated_s_.push_back(s);
    reference_points_.emplace_back(route_points[i], 0.0, route_points[i].yaw(), route_points[i].curvature(), 0.0,
                                   route_points[i].leftBoundaryDistance(), -route_points[i].rightBoundaryDistance(),
                                   route_points[i].leftRoadBoundaryDistance(), -route_points[i].rightRoadBoundaryDistance(), 0.0, s);
    // ERT_PLOG_I << "id = " << id_ << "  i = " << i << "  s = " << s << "  left = " <<
    // route_points[i].leftBoundaryDistance()
    //      << "  right = " << route_points[i].rightBoundaryDistance() ;
    math::Vec3d heading;
    if (i + 1 >= num_points_) {
      if (i - 1 >= 0) {
        heading = route_points[i] - route_points[i - 1];
      }
    } else {
      line_segments_.emplace_back(route_points[i], route_points[i + 1]);
      heading = route_points[i + 1] - route_points[i];
      s += heading.Length();
    }
    heading.Normalize();
  }
  length_ = s;
  num_line_segments_ = num_points_ - 1;

  stop_lines_.clear();
  speed_limits_.clear();
  directions_.clear();
  source_infos_.clear();
  navigation_lane_change_info_ = {-1.0, 0, 0};
  boundary_types_.clear();
  gates_.clear();
  left_reference_lines_.clear();
  right_reference_lines_.clear();
  areas_.clear();
  road_info_.clear();
  merge_fork_ranges_.clear();
  navigation_lane_change_ranges_.clear();
  lane_follow_ranges_.clear();
  navigation_scores_.clear();
  shape_type_ = ShapeType::REGULAR;
  if (!id_.empty()) {
    // speed_limits
    SpeedLimit sp;
    for (auto pt : route->speedLimits()) {
      if (speed_limits_.empty()) {
        sp.start_s = 0.0;
        if (!reference_points_.empty()) {
          sp.start_point =
              math::Vec3d(reference_points_.front().x(), reference_points_.front().y(), reference_points_.front().z());
        } else {
          break;
        }
      } else {
        sp.start_s = speed_limits_.back().end_s;
        sp.start_point = speed_limits_.back().end_point;
      }
      SLPoint sl;
      xy2sl(math::Vec3d(pt.end_point.x(), pt.end_point.y(), pt.end_point.z()), &sl);
      sp.end_s = sl.s();
      sp.max_speed_limit = pt.max_speed_limit;
      sp.min_speed_limit = pt.min_speed_limit;
      sp.end_point = pt.end_point;
      speed_limits_.emplace_back(sp);
    }

    // segments_direction
    SegmentDirection sd;
    for (auto pt : route->segmentsDirection()) {
      if (directions_.empty()) {
        sd.start_s = 0.0;
        if (!reference_points_.empty()) {
          sd.start_point =
              math::Vec3d(reference_points_.front().x(), reference_points_.front().y(), reference_points_.front().z());
        } else {
          break;
        }
      } else {
        sd.start_s = directions_.back().end_s;
        sd.start_point = directions_.back().end_point;
      }
      SLPoint sl;
      xy2sl(math::Vec3d(pt.end_point.x(), pt.end_point.y(), pt.end_point.z()), &sl);
      sd.end_s = sl.s();
      sd.direction = pt.direction;
      sd.end_point = pt.end_point;
      directions_.emplace_back(sd);

      ShapeType shape_type = ShapeType::REGULAR;
      if (sd.direction == DrivingDirection::kDirectionUTurnOnly) {
        shape_type = ShapeType::U_TURN;
      } else if (sd.direction == DrivingDirection::kDirectionLeftOnly) {
        shape_type = ShapeType::LEFT_TURN;
      } else if (sd.direction == DrivingDirection::kDirectionRightOnly) {
        shape_type = ShapeType::RIGHT_TURN;
      } else {
        shape_type = ShapeType::REGULAR;
      }
      if (shape_type > shape_type_) {
        shape_type_ = shape_type;
      }
    }

    // stop_line_points
    for (auto pt : route->stopLinePoints()) {
      SLPoint sl;
      xy2sl(math::Vec3d(pt.x(), pt.y(), pt.z()), &sl);
      stop_lines_.emplace_back(pt.x(), pt.y(), pt.z(), sl.s(), pt.drivingDirection());
      AddSpeedLimit(sl.s(), 40.0 * KMH_MS, math::Vec3d(pt.x(), pt.y(), pt.z()));
    }

    // navigation_lane_change_ranges_
    for (const auto& range : route->navigationLaneChangeRanges()) {
      float start_s = std::fmin(length_, std::fmax(0.0, std::get<0>(range) - global_start_s_));
      float end_s = std::fmin(length_, std::fmax(0.0, std::get<1>(range) - global_start_s_));
      navigation_lane_change_ranges_.emplace_back(std::move(start_s), std::move(end_s), std::get<2>(range),
                                                  std::get<3>(range));
    }

    // lane_follow_ranges_
    for (const auto& range : route->laneFollowRanges()) {
      float start_s = std::fmin(length_, std::fmax(0.0, range.first - global_start_s_));
      float end_s = std::fmin(length_, std::fmax(0.0, range.second - global_start_s_));
      lane_follow_ranges_.emplace_back(start_s, end_s);
    }
  }
  // ERT_PLOG_I << "[ReferenceLine::ReferenceLine]: global_start_s_ = " << global_start_s_<< " size = " <<
  // route_points.size() << " length_ = " << length_ ;
}

/**
 * @brief ReferenceLine构造函数
 * @details 根据感知车道信息初始化参考线对象，并计算参考线的几何信息（如长度、曲率等）
 *
 * @param[in] lane 感知车道指针，用于构建参考线
 * @par 输入参数说明:
 * - lane: 必须为非空指针，类型为PerceptionLane*
 *
 * @par 关键变量说明:
 * - id_ (std::string): 参考线ID，全局唯一标识
 * - adc_loc_ (MapPoint): 自车位置信息
 * - global_start_s_ (double): 全局起始s值，范围[0, +∞)
 * - reference_points_ (std::vector<ReferencePoint>): 参考点列表，包含每个点的位置、航向、曲率等信息
 * - accumulated_s_ (std::vector<double>): 累计距离数组，存储每个点到起点的累计距离
 * - line_segments_ (std::vector<math::LineSegment2d>): 线段列表，由相邻点生成
 * - length_ (double): 参考线总长度，范围[0, +∞)
 * - num_points_ (int): 参考线的点数，范围[2, +∞)
 * - num_line_segments_ (int): 线段数量，范围[1, +∞)
 * - line_type_ (LineType): 参考线类型，枚举值
 * - is_memorized_park_route_ (bool): 是否为记忆停车路线
 * - is_memorized_end_route_ (bool): 是否为记忆终点路线
 * - is_terminal_enforced_ (bool): 是否为强制终点
 * - is_parallel_virtual_ (bool): 是否为平行虚拟线
 * - is_bound_modified_by_lane_ (bool): 是否被车道边界修改
 * - adc_s_ (double): 自车在参考线上的s值，范围[0, length_]
 * - adc_l_ (double): 自车在参考线上的横向偏移，范围[-∞, +∞]
 *
 * @par 判断条件:
 * - 如果感知车道为空，则无法构建参考线
 * - 如果车道段的中心线点数和偏移点数不一致，则无法构建参考线
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化成员变量;
 * :遍历车道段;
 * :检查中心线点数和偏移点数;
 * :计算累计距离;
 * :生成参考点;
 * :计算航向角;
 * :生成线段;
 * :更新参考线长度;
 * :计算自车位置;
 * :清空停止线、速度限制等信息;
 * :更新速度限制;
 * :更新车道类型;
 * :更新车道区域类型;
 * :更新区域重叠信息;
 * :更新驾驶管缓冲区;
 * :计算曲率和曲率变化率;
 * stop
 * @enduml
 *
 * @note 该函数用于根据感知车道初始化参考线，适用于从感知模块构建参考线
 *
 * @warning 需确保输入的感知车道指针非空，否则无法正确初始化参考线
 */
ReferenceLine::ReferenceLine(const PerceptionLane* lane) {
  CHECK_NOTNULL(lane);
  id_ = lane->id;
  adc_loc_ = lane->adc_loc;
  global_start_s_ = 0.0;
  reference_points_.clear();
  accumulated_s_.clear();
  line_segments_.clear();
  line_type_ = LineType::PERCEPTION_LANE;
  is_memorized_park_route_ = false;
  is_memorized_end_route_ = false;
  is_terminal_enforced_ = false;
  is_parallel_virtual_ = false;
  is_bound_modified_by_lane_ = false;

  double s = 0.0;
  adc_s_ = 0.0;
  adc_l_ = 0.0;
  float min_dis = std::numeric_limits<float>::max();
  for (auto& lane_seg : lane->lane_segments) {
    int center_line_pt_num = lane_seg.center_line.size();
    int offset_num = lane_seg.offset.size();
    if (center_line_pt_num != offset_num) {
      std::cerr << "[PerceptionLane][ReferenceLine::ReferenceLine]: PerceptionLane rebuild Failed!"
                << "  lane_id = " << id_ << "  seg_id = " << lane_seg.segment_id
                << "  center_line_pt_num = " << center_line_pt_num << "  offset_num = " << offset_num;
      return;
    }
    for (int i = 0; i < center_line_pt_num; ++i) {
      math::Vec3d heading;
      if (!reference_points_.empty()) {
        line_segments_.emplace_back(reference_points_.back(), lane_seg.center_line.at(i));
        heading = lane_seg.center_line.at(i) - reference_points_.back();
        s += heading.Length();
      }
      heading.Normalize();

      if (reference_points_.size() == 1) {
        reference_points_.front().setHeading(heading.Angle());
      }

      accumulated_s_.push_back(s);
      reference_points_.emplace_back(lane_seg.center_line.at(i), 0.0, heading.Angle(), 0.0, 0.0, lane_seg.offset.at(i),
                                     -lane_seg.offset.at(i), lane_seg.offset.at(i), -lane_seg.offset.at(i), 0.0, s);

      float dis =
          std::sqrt((lane_seg.center_line.at(i).x() - adc_loc_.x()) * (lane_seg.center_line.at(i).x() - adc_loc_.x()) +
                    (lane_seg.center_line.at(i).y() - adc_loc_.y()) * (lane_seg.center_line.at(i).y() - adc_loc_.y()));
      if (dis < min_dis) {
        min_dis = dis;
        adc_s_ = s;
        adc_l_ = dis;
      }
    }
  }
  length_ = s;
  num_points_ = static_cast<int>(reference_points_.size());
  num_line_segments_ = static_cast<int>(line_segments_.size());
  // ERT_PLOG_I << "[PerceptionLane] lane_id = " << id_
  //           << "  num_line_segments_ = " << num_line_segments_
  //           << "  num_points_ = " << num_points_
  //           << "  s = " << s
  //           ;

  stop_lines_.clear();
  speed_limits_.clear();
  directions_.clear();
  source_infos_.clear();
  navigation_lane_change_info_ = {-1.0, 0, 0};
  boundary_types_.clear();
  gates_.clear();
  left_reference_lines_.clear();
  right_reference_lines_.clear();
  areas_.clear();
  road_info_.clear();
  merge_fork_ranges_.clear();
  navigation_lane_change_ranges_.clear();
  lane_follow_ranges_.clear();
  navigation_scores_.clear();
  if (!id_.empty()) {
    SpeedLimit sp;
    math::Vec3d vec;
    sp.start_s = reference_points_.empty() ? 0.0 : reference_points_.front().local_s();
    sp.end_s = reference_points_.empty() ? 0.0 : reference_points_.back().local_s();
    sp.max_speed_limit = kMaxSpeedMS;
    sp.min_speed_limit = kMaxSpeedMS;
    sp.start_point = reference_points_.empty() ? vec : reference_points_.front();
    sp.end_point = reference_points_.empty() ? vec : reference_points_.back();
    speed_limits_.emplace_back(sp);
  }
}

/**
 * @brief ReferenceLine构造函数
 * @details 根据给定的主路径历史信息初始化参考线对象，并计算参考线的几何信息（如长度、曲率等）
 *
 * @param[in] path_history 主路径历史指针，用于构建参考线
 * @par 输入参数说明:
 * - path_history: 必须为非空指针，类型为MainPathHistory*
 *
 * @par 关键变量说明:
 * - id_ (std::string): 参考线ID，全局唯一标识
 * - adc_loc_ (MapPoint): 自车位置信息
 * - global_start_s_ (double): 全局起始s值，范围[0, +∞)
 * - reference_points_ (std::vector<ReferencePoint>): 参考点列表，包含每个点的位置、航向、曲率等信息
 * - accumulated_s_ (std::vector<double): 累计距离数组，存储每个点到起点的累计距离
 * - line_segments_ (std::vector<math::LineSegment2d>): 线段列表，由相邻点生成
 * - length_ (double): 参考线总长度，范围[0, +∞)
 * - num_points_ (int): 参考线的点数，范围[2, +∞)
 * - num_line_segments_ (int): 线段数量，范围[1, +∞)
 * - is_memorized_park_route_ (bool): 是否为记忆停车路线
 * - is_memorized_end_route_ (bool): 是否为记忆终点路线
 * - is_terminal_enforced_ (bool): 是否为强制终点
 * - is_parallel_virtual_ (bool): 是否为平行虚拟线
 * - is_bound_modified_by_lane_ (bool): 是否被车道边界修改
 * - adc_s_ (double): 自车在参考线上的s值，范围[0, length_]
 * - adc_l_ (double): 自车在参考线上的横向偏移，范围[-∞, +∞]
 *
 * @par 判断条件:
 * - 如果主路径历史为空，则无法构建参考线
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化成员变量;
 * :遍历主路径历史点集;
 * :计算累计距离;
 * :生成参考点;
 * :计算航向角;
 * :生成线段;
 * :更新参考线长度;
 * :计算自车位置;
 * :清空停止线、速度限制等信息;
 * :更新速度限制;
 * :更新车道类型;
 * :更新车道区域类型;
 * :更新区域重叠信息;
 * :更新驾驶管缓冲区;
 * :计算曲率和曲率变化率;
 * stop
 * @enduml
 *
 * @note 该函数用于根据主路径历史初始化参考线，适用于从历史路径构建参考线
 *
 * @warning 需确保输入的主路径历史指针非空，否则无法正确初始化参考线
 */
ReferenceLine::ReferenceLine(const MainPathHistory* path_history) {
  CHECK_NOTNULL(path_history);
  id_ = path_history->id;
  adc_loc_ = path_history->adc_loc;
  global_start_s_ = 0.0;
  reference_points_.clear();
  accumulated_s_.clear();
  line_segments_.clear();
  // line_type_ = LineType::PERCEPTION_LANE;
  is_memorized_park_route_ = false;
  is_memorized_end_route_ = false;
  is_terminal_enforced_ = false;
  is_parallel_virtual_ = false;
  is_bound_modified_by_lane_ = false;
  double s = 0.0;
  adc_s_ = 0.0;
  adc_l_ = 0.0;
  float min_dis = std::numeric_limits<float>::max();
  int center_line_pt_num = path_history->values.size();
  const auto& center_line_pt = path_history->values;
  for (int i = 0; i < center_line_pt_num; ++i) {
    math::Vec3d heading;
    if (!reference_points_.empty()) {
      line_segments_.emplace_back(reference_points_.back(), center_line_pt.at(i));
      heading = center_line_pt.at(i) - reference_points_.back();
      s += heading.Length();
    }
    heading.Normalize();
    if (reference_points_.size() == 1) {
      reference_points_.front().setHeading(heading.Angle());
    }
    accumulated_s_.push_back(s);
    reference_points_.emplace_back(center_line_pt.at(i), 0.0, heading.Angle(), 0.0, 0.0, 1.7, -1.7, 1.7, -1.7, 0.0, s);
    float dis = std::sqrt((center_line_pt.at(i).x() - adc_loc_.x()) * (center_line_pt.at(i).x() - adc_loc_.x()) +
                          (center_line_pt.at(i).y() - adc_loc_.y()) * (center_line_pt.at(i).y() - adc_loc_.y()));
    if (dis < min_dis) {
      min_dis = dis;
      adc_s_ = s;
      adc_l_ = dis;
    }
  }
  length_ = s;
  num_points_ = static_cast<int>(reference_points_.size());
  num_line_segments_ = static_cast<int>(line_segments_.size());
  // ERT_PLOG_I << "[PerceptionLane] lane_id = " << id_
  //           << "  num_line_segments_ = " << num_line_segments_
  //           << "  num_points_ = " << num_points_
  //           << "  s = " << s
  //           ;
  stop_lines_.clear();
  speed_limits_.clear();
  directions_.clear();
  source_infos_.clear();
  navigation_lane_change_info_ = {-1.0, 0, 0};
  boundary_types_.clear();
  gates_.clear();
  left_reference_lines_.clear();
  right_reference_lines_.clear();
  areas_.clear();
  road_info_.clear();
  merge_fork_ranges_.clear();
  navigation_lane_change_ranges_.clear();
  lane_follow_ranges_.clear();
  navigation_scores_.clear();
  if (!id_.empty()) {
    SpeedLimit sp;
    math::Vec3d vec;
    sp.start_s = reference_points_.empty() ? 0.0 : reference_points_.front().local_s();
    sp.end_s = reference_points_.empty() ? 0.0 : reference_points_.back().local_s();
    sp.max_speed_limit = kMaxSpeedMS;
    sp.min_speed_limit = kMaxSpeedMS;
    sp.start_point = reference_points_.empty() ? vec : reference_points_.front();
    sp.end_point = reference_points_.empty() ? vec : reference_points_.back();
    speed_limits_.emplace_back(sp);
  }
}

/**
 * @brief ReferenceLine构造函数
 * @details 根据车道线标记信息初始化参考线对象，并计算参考线的几何信息（如长度、曲率等）
 *
 * @param[in] lane_line_marking 车道线标记指针，用于构建参考线
 * @par 输入参数说明:
 * - lane_line_marking: 必须为非空指针，类型为adas::LaneLineMarking*
 *
 * @par 关键变量说明:
 * - id_ (std::string): 参考线ID，全局唯一标识
 * - adc_loc_ (MapPoint): 自车位置信息
 * - global_start_s_ (double): 全局起始s值，范围[0, +∞)
 * - reference_points_ (std::vector<ReferencePoint>): 参考点列表，包含每个点的位置、航向、曲率等信息
 * - accumulated_s_ (std::vector<double>): 累计距离数组，存储每个点到起点的累计距离
 * - line_segments_ (std::vector<math::LineSegment2d>): 线段列表，由相邻点生成
 * - length_ (double): 参考线总长度，范围[0, +∞)
 * - num_points_ (int): 参考线的点数，范围[2, +∞)
 * - num_line_segments_ (int): 线段数量，范围[1, +∞)
 * - is_memorized_park_route_ (bool): 是否为记忆停车路线
 * - is_memorized_end_route_ (bool): 是否为记忆终点路线
 * - is_terminal_enforced_ (bool): 是否为强制终点
 * - is_parallel_virtual_ (bool): 是否为平行虚拟线
 * - is_bound_modified_by_lane_ (bool): 是否被车道边界修改
 * - adc_s_ (double): 自车在参考线上的s值，范围[0, length_]
 * - adc_l_ (double): 自车在参考线上的横向偏移，范围[-∞, +∞]
 *
 * @par 判断条件:
 * - 如果车道线标记为空，则无法构建参考线
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化成员变量;
 * :遍历车道线点集;
 * :计算累计距离;
 * :生成参考点;
 * :计算航向角;
 * :生成线段;
 * :更新参考线长度;
 * :计算自车位置;
 * :清空停止线、速度限制等信息;
 * :更新速度限制;
 * :更新车道类型;
 * :更新车道区域类型;
 * :更新区域重叠信息;
 * :更新驾驶管缓冲区;
 * :计算曲率和曲率变化率;
 * stop
 * @enduml
 *
 * @note 该函数用于根据车道线标记初始化参考线，适用于从感知模块构建参考线
 *
 * @warning 需确保输入的车道线标记指针非空，否则无法正确初始化参考线
 */
ReferenceLine::ReferenceLine(const adas::LaneLineMarking* lane_line_marking) {
  CHECK_NOTNULL(lane_line_marking);
  id_ = std::to_string(lane_line_marking->id);
  adc_loc_ = lane_line_marking->adc_loc;
  global_start_s_ = 0.0;
  reference_points_.clear();
  accumulated_s_.clear();
  line_segments_.clear();
  // line_type_ = LineType::PERCEPTION_LANE;
  is_memorized_park_route_ = false;
  is_memorized_end_route_ = false;
  is_terminal_enforced_ = false;
  is_parallel_virtual_ = false;
  is_bound_modified_by_lane_ = false;

  double s = 0.0;
  adc_s_ = 0.0;
  adc_l_ = 0.0;
  float min_dis = std::numeric_limits<float>::max();
  const auto& line_points = lane_line_marking->line_points;
  int line_points_num = lane_line_marking->line_points.size();
  for (int i = 0; i < line_points_num; ++i) {
    math::Vec3d heading;
    if (!reference_points_.empty()) {
      line_segments_.emplace_back(reference_points_.back(), line_points.at(i));
      heading = line_points.at(i) - reference_points_.back();
      s += heading.Length();
    }
    heading.Normalize();

    if (reference_points_.size() == 1) {
      reference_points_.front().setHeading(heading.Angle());
    }
    accumulated_s_.push_back(s);
    reference_points_.emplace_back(line_points.at(i), 0.0, heading.Angle(), 0.0, 0.0, 1.7, -1.7, 1.7, -1.7, 0.0, s);

    float dis = std::sqrt((line_points.at(i).x() - adc_loc_.x()) * (line_points.at(i).x() - adc_loc_.x()) +
                          (line_points.at(i).y() - adc_loc_.y()) * (line_points.at(i).y() - adc_loc_.y()));
    if (dis < min_dis) {
      min_dis = dis;
      adc_s_ = s;
      adc_l_ = dis;
    }
  }
  length_ = s;
  num_points_ = static_cast<int>(reference_points_.size());
  num_line_segments_ = static_cast<int>(line_segments_.size());
  // ERT_PLOG_I << "[PerceptionLane] lane_id = " << id_
  //           << "  num_line_segments_ = " << num_line_segments_
  //           << "  num_points_ = " << num_points_
  //           << "  s = " << s
  //           ;
  stop_lines_.clear();
  speed_limits_.clear();
  directions_.clear();
  source_infos_.clear();
  navigation_lane_change_info_ = {-1.0, 0, 0};
  boundary_types_.clear();
  gates_.clear();
  left_reference_lines_.clear();
  right_reference_lines_.clear();
  areas_.clear();
  road_info_.clear();
  merge_fork_ranges_.clear();
  navigation_lane_change_ranges_.clear();
  lane_follow_ranges_.clear();
  navigation_scores_.clear();
  if (!id_.empty()) {
    SpeedLimit sp;
    math::Vec3d vec;
    sp.start_s = reference_points_.empty() ? 0.0 : reference_points_.front().local_s();
    sp.end_s = reference_points_.empty() ? 0.0 : reference_points_.back().local_s();
    sp.max_speed_limit = kMaxSpeedMS;
    sp.min_speed_limit = kMaxSpeedMS;
    sp.start_point = reference_points_.empty() ? vec : reference_points_.front();
    sp.end_point = reference_points_.empty() ? vec : reference_points_.back();
    speed_limits_.emplace_back(sp);
  }
}

/**
 * @brief ReferenceLine构造函数
 * @details 根据本地路径信息初始化参考线对象，并计算参考线的几何信息（如长度、曲率等）
 *
 * @param[in] local_route 本地路径指针，用于构建参考线
 * @par 输入参数说明:
 * - local_route: 必须为非空指针，类型为LocalRoute*
 *
 * @par 关键变量说明:
 * - id_ (std::string): 参考线ID，全局唯一标识
 * - is_current_reference_line_ (bool): 是否为当前参考线
 * - adc_loc_ (MapPoint): 自车位置信息
 * - global_start_s_ (double): 全局起始s值，范围[0, +∞)
 * - reference_points_ (std::vector<ReferencePoint>): 参考点列表，包含每个点的位置、航向、曲率等信息
 * - accumulated_s_ (std::vector<double>): 累计距离数组，存储每个点到起点的累计距离
 * - line_segments_ (std::vector<math::LineSegment2d>): 线段列表，由相邻点生成
 * - length_ (double): 参考线总长度，范围[0, +∞)
 * - num_points_ (int): 参考线的点数，范围[2, +∞)
 * - num_line_segments_ (int): 线段数量，范围[1, +∞)
 * - line_type_ (LineType): 参考线类型，枚举值
 * - is_memorized_park_route_ (bool): 是否为记忆停车路线
 * - is_memorized_end_route_ (bool): 是否为记忆终点路线
 * - is_terminal_enforced_ (bool): 是否为强制终点
 * - is_parallel_virtual_ (bool): 是否为平行虚拟线
 * - is_bound_modified_by_lane_ (bool): 是否被车道边界修改
 * - adc_s_ (double): 自车在参考线上的s值，范围[0, length_]
 * - adc_l_ (double): 自车在参考线上的横向偏移，范围[-∞, +∞]
 *
 * @par 判断条件:
 * - 如果本地路径为空，则无法构建参考线
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化成员变量;
 * :遍历本地路径点集;
 * :计算累计距离;
 * :生成参考点;
 * :计算航向角;
 * :生成线段;
 * :更新参考线长度;
 * :计算自车位置;
 * :清空停止线、速度限制等信息;
 * :更新速度限制;
 * :更新方向信息;
 * :更新停止线;
 * :更新车道变换范围;
 * :更新车道线巡线范围;
 * :更新车道类型;
 * :更新车道区域类型;
 * :更新区域重叠信息;
 * :更新驾驶管缓冲区;
 * :计算曲率和曲率变化率;
 * stop
 * @enduml
 *
 * @note 该函数用于根据本地路径初始化参考线，适用于从本地路径构建参考线
 *
 * @warning 需确保输入的本地路径指针非空，否则无法正确初始化参考线
 */
ReferenceLine::ReferenceLine(const LocalRoute* local_route) {
  CHECK_NOTNULL(local_route);
  id_ = std::to_string(local_route->id());
  is_current_reference_line_ = local_route->isCurrentLocalRoute();
  adc_loc_ = local_route->adcLocalization();
  const auto& local_route_points = local_route->guideLine();
  reference_points_.clear();
  accumulated_s_.clear();
  line_segments_.clear();
  line_type_ = LineType::PERCEPTION_ROAD_STRUCTURE;
  is_memorized_park_route_ = false;
  is_memorized_end_route_ = false;
  is_terminal_enforced_ = false;
  is_parallel_virtual_ = false;
  is_bound_modified_by_lane_ = false;

  double front_length_thresold = 400.0;
  double back_length_thresold = 150.0;
  double local_route_start_s =
      std::fmax(std::get<1>(local_route->validRange()), std::fmax(0.0, local_route->adcS() - back_length_thresold));
  double local_route_end_s = std::fmin(std::get<2>(local_route->validRange()),
                                       std::fmin(local_route->length(), local_route->adcS() + front_length_thresold));
  global_start_s_ = local_route->globalStartS() + local_route_start_s;
  int local_route_guide_line_size = local_route->guideLine().size();
  double s = 0.0;
  float min_s = std::numeric_limits<float>::max();
  adc_s_ = 0.0;
  adc_l_ = local_route->adcL();
  double ref_start_s_in_local_route = 0.0;
  double ref_end_s_in_local_route = 0.0;
  for (int i = 0; i < local_route_guide_line_size; ++i) {
    if (local_route_points[i].s() < local_route_start_s) {
      continue;
    }
    if (local_route_points[i].s() > local_route_end_s) {
      break;
    }

    float delta_s = std::fabs(local_route_points[i].s() - local_route->adcS());
    if (delta_s < min_s) {
      min_s = delta_s;
      adc_s_ = s;
    }

    accumulated_s_.push_back(s);
    reference_points_.emplace_back(
        math::Vec3d(local_route_points[i].x(), local_route_points[i].y(), local_route_points[i].z()),
        local_route_points[i].slope(), local_route_points[i].theta(), 0.0, 0.0, local_route_points[i].leftLaneWidth(),
        -local_route_points[i].rightLaneWidth(), local_route_points[i].leftRoadWidth(),
        -local_route_points[i].rightRoadWidth(), 0.0, s);
    if (reference_points_.size() == 1) {
      ref_start_s_in_local_route = local_route_points[i].s();
    }
    ref_end_s_in_local_route = local_route_points[i].s();

    math::Vec3d heading;
    if (i + 1 >= local_route_guide_line_size) {
      if (i - 1 >= 0) {
        heading =
            math::Vec3d(local_route_points[i].x(), local_route_points[i].y(), local_route_points[i].z()) -
            math::Vec3d(local_route_points[i - 1].x(), local_route_points[i - 1].y(), local_route_points[i - 1].z());
      }
    } else {
      line_segments_.emplace_back(
          math::Vec3d(local_route_points[i].x(), local_route_points[i].y(), local_route_points[i].z()),
          math::Vec3d(local_route_points[i + 1].x(), local_route_points[i + 1].y(), local_route_points[i + 1].z()));
      heading =
          math::Vec3d(local_route_points[i + 1].x(), local_route_points[i + 1].y(), local_route_points[i + 1].z()) -
          math::Vec3d(local_route_points[i].x(), local_route_points[i].y(), local_route_points[i].z());
      s += heading.Length();
    }
    heading.Normalize();
  }
  length_ = s;
  num_points_ = reference_points_.size();
  num_line_segments_ = num_points_ - 1;

  stop_lines_.clear();
  speed_limits_.clear();
  directions_.clear();
  source_infos_.clear();
  navigation_lane_change_info_ = {-1.0, 0, 0};
  boundary_types_.clear();
  gates_.clear();
  left_reference_lines_.clear();
  right_reference_lines_.clear();
  areas_.clear();
  road_info_.clear();
  merge_fork_ranges_.clear();
  navigation_lane_change_ranges_.clear();
  lane_follow_ranges_.clear();
  navigation_scores_.clear();
  shape_type_ = ShapeType::REGULAR;
  if (!id_.empty()) {
    // speed_limits
    SpeedLimit sp;
    for (const auto& pt : local_route->speedLimits()) {
      if (pt.end_s < ref_start_s_in_local_route) {
        continue;
      } else if (pt.start_s > ref_end_s_in_local_route) {
        break;
      }

      if (speed_limits_.empty()) {
        sp.start_s = 0.0;
        if (!reference_points_.empty()) {
          sp.start_point =
              math::Vec3d(reference_points_.front().x(), reference_points_.front().y(), reference_points_.front().z());
        } else {
          break;
        }
      } else {
        sp.start_s = speed_limits_.back().end_s;
        sp.start_point = speed_limits_.back().end_point;
      }
      SLPoint sl;
      xy2sl(math::Vec3d(pt.end_point.x(), pt.end_point.y(), pt.end_point.z()), &sl);
      if (sl.s() > sp.start_s + (1e-2)) {
        if (sl.s() > length_ - (1e-2)) {
          sl.set_s(length_);
        }
        sp.end_s = sl.s();
        sp.max_speed_limit = pt.max_speed_limit;
        sp.min_speed_limit = pt.min_speed_limit;
        sp.end_point = pt.end_point;
        speed_limits_.emplace_back(sp);
      }
      if (sl.s() > length_ - (1e-2)) {
        break;
      }
    }

    // segments_direction
    SegmentDirection sd;
    for (const auto& direction : local_route->directions()) {
      if (direction.end_s < ref_start_s_in_local_route) {
        continue;
      } else if (direction.start_s > ref_end_s_in_local_route) {
        break;
      }

      if (directions_.empty()) {
        sd.start_s = 0.0;
        if (!reference_points_.empty()) {
          sd.start_point =
              math::Vec3d(reference_points_.front().x(), reference_points_.front().y(), reference_points_.front().z());
        } else {
          break;
        }
      } else {
        sd.start_s = directions_.back().end_s;
        sd.start_point = directions_.back().end_point;
      }
      SLPoint sl;
      xy2sl(math::Vec3d(direction.end_point.x(), direction.end_point.y(), direction.end_point.z()), &sl);
      if (sl.s() > sd.start_s + (1e-2)) {
        if (sl.s() > length_ - (1e-2)) {
          sl.set_s(length_);
        }
        sd.end_s = sl.s();
        sd.direction = direction.direction;
        sd.end_point = direction.end_point;
        directions_.emplace_back(sd);

        ShapeType shape_type = ShapeType::REGULAR;
        if (sd.direction == DrivingDirection::kDirectionUTurnOnly) {
          shape_type = ShapeType::U_TURN;
        } else if (sd.direction == DrivingDirection::kDirectionLeftOnly) {
          shape_type = ShapeType::LEFT_TURN;
        } else if (sd.direction == DrivingDirection::kDirectionRightOnly) {
          shape_type = ShapeType::RIGHT_TURN;
        } else {
          shape_type = ShapeType::REGULAR;
        }
        if (shape_type > shape_type_) {
          shape_type_ = shape_type;
        }
      }
      if (sl.s() > length_ - (1e-2)) {
        break;
      }
    }

    // source_infos_
    for (const auto& source_info : local_route->sourceInfos()) {
      if (std::get<2>(source_info) < ref_start_s_in_local_route) {
        continue;
      } else if (std::get<1>(source_info) > ref_end_s_in_local_route) {
        break;
      }

      std::tuple<LineSourceType, float, float> res;
      std::get<0>(res) = std::get<0>(source_info);
      std::get<1>(res) = std::fmin(std::fmax(std::get<1>(source_info) - ref_start_s_in_local_route, 0.0), length_);
      std::get<2>(res) = std::fmin(std::fmax(std::get<2>(source_info) - ref_start_s_in_local_route, 0.0), length_);

      if (source_infos_.empty()) {
        source_infos_.emplace_back(std::move(res));
      } else {
        if (std::get<0>(source_infos_.back()) == std::get<0>(res)) {
          std::get<2>(source_infos_.back()) = std::get<2>(res);
        } else {
          source_infos_.emplace_back(std::get<0>(res), std::get<2>(source_infos_.back()), std::get<2>(res));
        }
      }
    }
    if (!source_infos_.empty()) {
      std::get<2>(source_infos_.back()) = length_;
    }

    // navigation_lane_change_info_
    navigation_lane_change_info_ = local_route->navigationLaneChangeInfo();

    // segments_boundary_type
    SegmentBoundaryType sb;
    for (const auto& boundary_type : local_route->boundaryTypes()) {
      if (boundary_type.end_s < ref_start_s_in_local_route) {
        continue;
      } else if (boundary_type.start_s > ref_end_s_in_local_route) {
        break;
      }

      if (boundary_types_.empty()) {
        sb.start_s = 0.0;
        if (!reference_points_.empty()) {
          sb.start_point =
              math::Vec3d(reference_points_.front().x(), reference_points_.front().y(), reference_points_.front().z());
        } else {
          break;
        }
      } else {
        sb.start_s = boundary_types_.back().end_s;
        sb.start_point = boundary_types_.back().end_point;
      }
      SLPoint sl;
      xy2sl(math::Vec3d(boundary_type.end_point.x(), boundary_type.end_point.y(), boundary_type.end_point.z()), &sl);
      if (sl.s() > sb.start_s + (1e-2)) {
        if (sl.s() > length_ - (1e-2)) {
          sl.set_s(length_);
        }
        sb.end_s = sl.s();
        sb.left_type = boundary_type.left_type;
        sb.right_type = boundary_type.right_type;
        sb.end_point = boundary_type.end_point;
        boundary_types_.emplace_back(sb);
      }
      if (sl.s() > length_ - (1e-2)) {
        break;
      }
    }

    // stop_lines
    for (const auto& pt : local_route->stopLines()) {
      if (pt.s < ref_start_s_in_local_route) {
        continue;
      } else if (pt.s > ref_end_s_in_local_route) {
        break;
      }

      SLPoint sl;
      xy2sl(math::Vec3d(pt.x, pt.y, pt.z), &sl);
      if (sl.s() > 0 && sl.s() < length_) {
        stop_lines_.emplace_back(pt.id, pt.is_virtual, pt.x, pt.y, pt.z, sl.s(), pt.traffic_light_id, pt.direction);
        if (pt.is_virtual) {
          stop_lines_.back().type = proto::perception::StopLine_LineType_kTypeVirtual;
        }
        stop_lines_.back().is_exist = true;
      }
    }

    // gates
    for (const auto& pt : local_route->gates()) {
      if (pt.s < ref_start_s_in_local_route) {
        continue;
      } else if (pt.s > ref_end_s_in_local_route) {
        break;
      }

      SLPoint sl;
      xy2sl(math::Vec3d(pt.x, pt.y, pt.z), &sl);
      if (sl.s() > 0 && sl.s() < length_) {
        gates_.emplace_back(pt.id, static_cast<proto::perception::Gate::GateStatus>(pt.gate_status), pt.x, pt.y, pt.z,
                            sl.s(), pt.head_stop_distance);
      }
    }

    // left neighbor
    auto left_neighbors = local_route->getLeftNeighborLocalRouteBySRange(0.0, length_);
    // // print info
    // ERT_PLOG_I << "[ReferenceLine-LocalRoute]: local_route id = " << local_route->id()
    //           << "  left_neighbors.size() = " << left_neighbors.size()
    //           << "  check range <0.0, " << length_ << ">" ;
    // for(const auto& neighbor : left_neighbors) {
    //   ERT_PLOG_I << "  key = " << neighbor.first
    //             << "  neighbor local route id = " << neighbor.second.id
    //             << "  element size = " << neighbor.second.elements.size() ;
    //   for(const auto& element : neighbor.second.elements) {
    //     ERT_PLOG_I << "    bound_type = " << static_cast<int>(element.bound_type)
    //               << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range)<< ", " <<
    //               std::get<2>(element.range)
    //               << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size()
    //               ;
    //   }
    // }
    for (const auto& neighbor : left_neighbors) {
      left_reference_lines_[std::to_string(neighbor.first)].id = neighbor.second.id;
      left_reference_lines_[std::to_string(neighbor.first)].elements.clear();
      for (const auto& element : neighbor.second.elements) {
        if (std::get<0>(element.range)) {
          if (std::get<2>(element.range) < ref_start_s_in_local_route) {
            continue;
          } else if (std::get<1>(element.range) > ref_end_s_in_local_route) {
            break;
          }

          NeighborLocalRoute::Element el = element;
          SLPoint start_sl, end_sl;
          xy2sl(math::Vec3d(element.points.first.x(), element.points.first.y(), element.points.first.z()), &start_sl);
          xy2sl(math::Vec3d(element.points.second.x(), element.points.second.y(), element.points.second.z()), &end_sl);
          float start_s = std::fmax(0.0, start_sl.s());
          float end_s = std::fmin(end_sl.s(), length_);
          if (end_s > start_s + (1e-2)) {
            std::get<0>(el.range) = true;
            std::get<1>(el.range) = start_s;
            std::get<2>(el.range) = end_s;
            left_reference_lines_[std::to_string(neighbor.first)].elements.emplace_back(std::move(el));
          }
        }
      }
      if (left_reference_lines_[std::to_string(neighbor.first)].elements.empty()) {
        left_reference_lines_.erase(std::to_string(neighbor.first));
      }
    }
    // // print info
    // ERT_PLOG_I << "[ReferenceLine-LocalRoute]: reference_line id = " << id_
    //           << "  left_reference_lines_.size() = " << left_reference_lines_.size() ;
    // for(const auto& neighbor : left_reference_lines_) {
    //   ERT_PLOG_I << "  key = " << neighbor.first
    //             << "  neighbor local route id = " << neighbor.second.id
    //             << "  element size = " << neighbor.second.elements.size() ;
    //   for(const auto& element : neighbor.second.elements) {
    //     ERT_PLOG_I << "    bound_type = " << static_cast<int>(element.bound_type)
    //               << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range)<< ", " <<
    //               std::get<2>(element.range)
    //               << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size()
    //               ;
    //   }
    // }

    // right neighbor
    auto right_neighbors = local_route->getRightNeighborLocalRouteBySRange(0.0, length_);
    // // print info
    // ERT_PLOG_I << "[ReferenceLine-LocalRoute]: local_route id = " << local_route->id()
    //           << "  right_neighbors.size() = " << right_neighbors.size()
    //           << "  check range <0.0, " << length_ << ">" ;
    // for(const auto& neighbor : right_neighbors) {
    //   ERT_PLOG_I << "  key = " << neighbor.first
    //             << "  neighbor local route id = " << neighbor.second.id
    //             << "  element size = " << neighbor.second.elements.size() ;
    //   for(const auto& element : neighbor.second.elements) {
    //     ERT_PLOG_I << "    bound_type = " << static_cast<int>(element.bound_type)
    //               << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range)<< ", " <<
    //               std::get<2>(element.range)
    //               << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size()
    //               ;
    //   }
    // }
    for (const auto& neighbor : right_neighbors) {
      right_reference_lines_[std::to_string(neighbor.first)].id = neighbor.second.id;
      right_reference_lines_[std::to_string(neighbor.first)].elements.clear();
      for (const auto& element : neighbor.second.elements) {
        if (std::get<0>(element.range)) {
          if (std::get<2>(element.range) < ref_start_s_in_local_route) {
            continue;
          } else if (std::get<1>(element.range) > ref_end_s_in_local_route) {
            break;
          }

          NeighborLocalRoute::Element el;
          el = element;
          SLPoint start_sl, end_sl;
          xy2sl(math::Vec3d(element.points.first.x(), element.points.first.y(), element.points.first.z()), &start_sl);
          xy2sl(math::Vec3d(element.points.second.x(), element.points.second.y(), element.points.second.z()), &end_sl);
          float start_s = std::fmax(0.0, start_sl.s());
          float end_s = std::fmin(end_sl.s(), length_);
          if (end_s > start_s + (1e-2)) {
            std::get<0>(el.range) = true;
            std::get<1>(el.range) = start_s;
            std::get<2>(el.range) = end_s;
            right_reference_lines_[std::to_string(neighbor.first)].elements.emplace_back(std::move(el));
          }
        }
      }
      if (right_reference_lines_[std::to_string(neighbor.first)].elements.empty()) {
        right_reference_lines_.erase(std::to_string(neighbor.first));
      }
    }
    // // print info
    // ERT_PLOG_I << "[ReferenceLine-LocalRoute]: reference_line id = " << id_
    //           << "  right_reference_lines_.size() = " << right_reference_lines_.size() ;
    // for(const auto& neighbor : right_reference_lines_) {
    //   ERT_PLOG_I << "  key = " << neighbor.first
    //             << "  neighbor local route id = " << neighbor.second.id
    //             << "  element size = " << neighbor.second.elements.size() ;
    //   for(const auto& element : neighbor.second.elements) {
    //     ERT_PLOG_I << "    bound_type = " << static_cast<int>(element.bound_type)
    //               << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range)<< ", " <<
    //               std::get<2>(element.range)
    //               << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size()
    //               ;
    //   }
    // }

    // merge and fork info
    for (const auto& range : local_route->mergeForkRanges()) {
      if (range.s() < ref_start_s_in_local_route) {
        continue;
      } else if (range.s() > ref_end_s_in_local_route) {
        break;
      }

      MergeForkRange res = range;
      SLPoint sl;
      xy2sl(math::Vec3d(range.point().x(), range.point().y(), range.point().z()), &sl);
      if (sl.s() > 0 && sl.s() < length_) {
        res.setS(sl.s());
        merge_fork_ranges_.emplace_back(std::move(res));
      }
    }

    // navigation_scores_
    for (const auto& lane_segs : local_route->lanesSegments()) {
      bool flag = false;
      for (const auto& lane_seg : lane_segs.second) {
        if (std::get<0>(lane_seg.range())) {
          if (std::get<2>(lane_seg.range()) < ref_start_s_in_local_route) {
            continue;
          } else if (std::get<1>(lane_seg.range()) > ref_end_s_in_local_route) {
            flag = true;
            break;
          }

          SLPoint sl;
          xy2sl(math::Vec3d(lane_seg.rangePoints().second.x(), lane_seg.rangePoints().second.y(),
                            lane_seg.rangePoints().second.z()),
                &sl);
          if (navigation_scores_.empty()) {
            navigation_scores_.emplace_back(lane_seg.navigationScore(), 0.0, sl.s());
          } else {
            if (fabs(std::get<0>(navigation_scores_.back()) - lane_seg.navigationScore()) < 1e-6) {
              std::get<2>(navigation_scores_.back()) = sl.s();
            } else {
              navigation_scores_.emplace_back(lane_seg.navigationScore(), std::get<2>(navigation_scores_.back()),
                                              sl.s());
            }
          }
        }
      }
      if (flag) {
        break;
      }
    }
    if (!navigation_scores_.empty()) {
      std::get<2>(navigation_scores_.back()) = length_;
    }

    // areas
    areas_ = local_route->areas();

    // road_info_
    road_info_ = local_route->roadInfo();
  }
  // ERT_PLOG_I << "[ReferenceLine::ReferenceLine]: global_start_s_ = " << global_start_s_<< " size = " <<
  // local_route_points.size() << " length_ = " << length_ ;
}

/**
 * @brief ReferenceLine构造函数
 * @details 根据本地路径参考线信息初始化参考线对象，并计算参考线的几何信息（如长度、曲率等）
 *
 * @param[in] local_route_ref 本地路径参考线信息，用于构建参考线
 * @param[in] tf 坐标变换信息，包含地图到自车坐标系的变换矩阵
 * @param[in] road_info 道路信息，包含道路ID、道路类型等
 * @param[in] adc_loc 自车位置信息
 * @par 输入参数说明:
 * - local_route_ref: 本地路径参考线信息，类型为proto::LocalRouteReferenceLine
 * - tf: 坐标变换信息，包含地图到自车坐标系的变换矩阵，类型为std::tuple<Eigen::Matrix4d, Eigen::Matrix4d, math::Vec3d>
 * - road_info: 道路信息，包含道路ID、道路类型等，类型为proto::LocalRouteRoadInfo
 * - adc_loc: 自车位置信息，类型为MapPoint
 *
 * @par 关键变量说明:
 * - id_ (std::string): 参考线ID，全局唯一标识
 * - is_current_reference_line_ (bool): 是否为当前参考线
 * - reference_points_ (std::vector<ReferencePoint>): 参考点列表，包含每个点的位置、航向、曲率等信息
 * - accumulated_s_ (std::vector<double>): 累计距离数组，存储每个点到起点的累计距离
 * - line_segments_ (std::vector<math::LineSegment2d>): 线段列表，由相邻点生成
 * - length_ (double): 参考线总长度，范围[0, +∞)
 * - num_points_ (int): 参考线的点数，范围[2, +∞)
 * - num_line_segments_ (int): 线段数量，范围[1, +∞)
 * - line_type_ (LineType): 参考线类型，枚举值
 * - is_memorized_park_route_ (bool): 是否为记忆停车路线
 * - is_memorized_end_route_ (bool): 是否为记忆终点路线
 * - is_terminal_enforced_ (bool): 是否为强制终点
 * - is_parallel_virtual_ (bool): 是否为平行虚拟线
 * - is_bound_modified_by_lane_ (bool): 是否被车道边界修改
 * - adc_loc_ (MapPoint): 自车位置信息
 * - adc_s_ (double): 自车在参考线上的s值，范围[0, length_]
 * - adc_l_ (double): 自车在参考线上的横向偏移，范围[-∞, +∞]
 *
 * @par 判断条件:
 * - 如果本地路径参考线信息为空，则无法构建参考线
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化成员变量;
 * :遍历本地路径参考线点集;
 * :计算累计距离;
 * :生成参考点;
 * :计算航向角;
 * :生成线段;
 * :更新参考线长度;
 * :计算自车位置;
 * :清空停止线、速度限制等信息;
 * :更新速度限制;
 * :更新方向信息;
 * :更新停止线;
 * :更新车道变换范围;
 * :更新车道线巡线范围;
 * :更新车道类型;
 * :更新车道区域类型;
 * :更新区域重叠信息;
 * :更新驾驶管缓冲区;
 * :计算曲率和曲率变化率;
 * stop
 * @enduml
 *
 * @note 该函数用于根据本地路径参考线信息初始化参考线，适用于从本地路径构建参考线
 *
 * @warning 需确保输入的本地路径参考线信息非空，否则无法正确初始化参考线
 */
ReferenceLine::ReferenceLine(const proto::LocalRouteReferenceLine& local_route_ref,
                             const std::tuple<Eigen::Matrix4d, Eigen::Matrix4d, math::Vec3d>& tf,
                             const proto::LocalRouteRoadInfo& road_info, const std::string& frame_id,
                             const MapPoint& adc_loc) {
  id_ = local_route_ref.id();
  is_current_reference_line_ = false;
  reference_points_.clear();
  accumulated_s_.clear();
  line_segments_.clear();
  line_type_ = LineType::LOCAL_ROUTE;
  is_memorized_park_route_ = false;
  is_memorized_end_route_ = false;
  is_terminal_enforced_ = false;
  is_parallel_virtual_ = false;
  is_bound_modified_by_lane_ = false;
  smooth_type_ = static_cast<SmoothType>(local_route_ref.smooth_type());
  global_start_s_ = 0.0;

  int local_route_guide_line_size = local_route_ref.guide_points().size();
  for (int i = 0; i < local_route_guide_line_size; ++i) {
    accumulated_s_.push_back(local_route_ref.guide_points().at(i).s());

    math::Vec3d middle_pos(local_route_ref.guide_points().at(i).point().x(),
                           local_route_ref.guide_points().at(i).point().y(),
                           local_route_ref.guide_points().at(i).point().z());
    math::Vec3d middle_rpy(0.0, 0.0, local_route_ref.guide_points().at(i).theta());
    if (frame_id == "map") {
      transfer::getPoseMap2Odom(std::get<0>(tf), std::get<1>(tf), std::get<2>(tf), &middle_pos, &middle_rpy);
    } else {
      transfer::transformPoint(std::get<1>(tf), &middle_pos);
      transfer::transformRPY(std::get<1>(tf), &middle_rpy);
    }

    if (reference_points_.size() > 0) {
      line_segments_.emplace_back(reference_points_.back(), middle_pos);
    }
    reference_points_.emplace_back(
        middle_pos, local_route_ref.guide_points().at(i).slope(), middle_rpy.z(),
        local_route_ref.guide_points().at(i).kappa(), local_route_ref.guide_points().at(i).dkappa(),
        local_route_ref.guide_points().at(i).lane_left_width(),
        -local_route_ref.guide_points().at(i).lane_right_width(),
        local_route_ref.guide_points().at(i).road_left_width(),
        -local_route_ref.guide_points().at(i).road_right_width(), 0.0, local_route_ref.guide_points().at(i).s());
  }
  length_ = local_route_ref.length();
  num_points_ = reference_points_.size();
  num_line_segments_ = num_points_ - 1;

  adc_loc_ = adc_loc;
  SLPoint sl_point;
  xy2sl(math::Vec3d(adc_loc_.x(), adc_loc_.y(), adc_loc_.z()), adc_loc_.yaw(), &sl_point);
  adc_s_ = sl_point.s();
  adc_l_ = sl_point.l();

  stop_lines_.clear();
  speed_limits_.clear();
  directions_.clear();
  source_infos_.clear();
  navigation_lane_change_info_ = {-1.0, 0, 0};
  boundary_types_.clear();
  gates_.clear();
  left_reference_lines_.clear();
  right_reference_lines_.clear();
  areas_.clear();
  road_info_.clear();
  merge_fork_ranges_.clear();
  navigation_lane_change_ranges_.clear();
  lane_follow_ranges_.clear();
  navigation_scores_.clear();
  shape_type_ = ShapeType::REGULAR;
  if (!id_.empty()) {
    // speed_limits
    for (const auto& pt : local_route_ref.speed_limits()) {
      SpeedLimit sp;
      sp.start_s = pt.start_s();
      sp.end_s = pt.end_s();
      sp.max_speed_limit = pt.max_speed_limit();
      sp.min_speed_limit = pt.min_speed_limit();
      speed_limits_.emplace_back(std::move(sp));
    }

    // segments_direction
    for (const auto& direction : local_route_ref.directions()) {
      SegmentDirection sd;
      sd.start_s = direction.start_s();
      sd.end_s = direction.end_s();
      sd.direction = static_cast<DrivingDirection>(direction.direction());
      ShapeType shape_type = ShapeType::REGULAR;
      if (sd.direction == DrivingDirection::kDirectionUTurnOnly) {
        shape_type = ShapeType::U_TURN;
      } else if (sd.direction == DrivingDirection::kDirectionLeftOnly) {
        shape_type = ShapeType::LEFT_TURN;
      } else if (sd.direction == DrivingDirection::kDirectionRightOnly) {
        shape_type = ShapeType::RIGHT_TURN;
      } else {
        shape_type = ShapeType::REGULAR;
      }
      if (shape_type > shape_type_) {
        shape_type_ = shape_type;
      }
      directions_.emplace_back(std::move(sd));
    }

    // source_infos_
    for (const auto& source_info : local_route_ref.line_sources()) {
      std::tuple<LineSourceType, float, float> res;
      std::get<0>(res) = source_info.type();
      std::get<1>(res) = source_info.start_s();
      std::get<2>(res) = source_info.end_s();
      source_infos_.emplace_back(std::move(res));
    }

    // navigation_lane_change_info_
    navigation_lane_change_info_ = {local_route_ref.navigation_lane_change_info().s(),
                                    local_route_ref.navigation_lane_change_info().direction(),
                                    local_route_ref.navigation_lane_change_info().times()};

    // segments_boundary_type
    if (local_route_ref.left_boundaries().size() == local_route_ref.right_boundaries().size()) {
      for (int m = 0; m < local_route_ref.left_boundaries().size(); m++) {
        SegmentBoundaryType sb;
        sb.start_s = local_route_ref.left_boundaries().at(m).start_s();
        sb.end_s = local_route_ref.left_boundaries().at(m).end_s();
        sb.left_type =
            static_cast<LocalRouteSegment::BoundaryType>(local_route_ref.left_boundaries().at(m).bound_type());
        sb.right_type =
            static_cast<LocalRouteSegment::BoundaryType>(local_route_ref.right_boundaries().at(m).bound_type());
        boundary_types_.emplace_back(std::move(sb));
      }
    }

    // stop_lines
    for (const auto& pt : local_route_ref.stop_lines()) {
      StopLine stop_line;
      stop_line.is_exist = true;
      stop_line.id = pt.id();
      stop_line.is_virtual = pt.is_virtual();
      if (stop_line.is_virtual) {
        stop_line.type = proto::perception::StopLine_LineType_kTypeVirtual;
      }
      stop_line.s = pt.s();
      stop_line.direction = static_cast<DrivingDirection>(pt.direction());
      stop_lines_.emplace_back(std::move(stop_line));
    }

    // gates
    for (const auto& pt : local_route_ref.gates()) {
      Gate gate;
      gate.is_exist = true;
      gate.id = pt.id();
      gate.gate_status = static_cast<proto::perception::Gate::GateStatus>(pt.gate_status());
      gate.s = pt.s();
      gate.head_stop_distance = pt.head_stop_distance();
      gates_.emplace_back(std::move(gate));
    }

    // left neighbor
    for (const auto& neighbor : local_route_ref.left_reference_lines()) {
      left_reference_lines_[neighbor.neighbor_id()].id = neighbor.neighbor_id();
      left_reference_lines_[neighbor.neighbor_id()].elements.clear();
      for (const auto& element : neighbor.boundaries()) {
        NeighborLocalRoute::Element el;
        el.bound_type = static_cast<LocalRouteSegment::BoundaryType>(element.bound_type());
        std::get<0>(el.range) = true;
        std::get<1>(el.range) = element.start_s();
        std::get<2>(el.range) = element.end_s();
        left_reference_lines_[neighbor.neighbor_id()].elements.emplace_back(std::move(el));
      }
      if (left_reference_lines_[neighbor.neighbor_id()].elements.empty()) {
        left_reference_lines_.erase(neighbor.neighbor_id());
      }
    }
    // // print info
    // ERT_PLOG_I << "[ReferenceLine-LocalRoute]: reference_line id = " << id_
    //           << "  left_reference_lines_.size() = " << left_reference_lines_.size() ;
    // for(const auto& neighbor : left_reference_lines_) {
    //   ERT_PLOG_I << "  key = " << neighbor.first
    //             << "  neighbor local route id = " << neighbor.second.id
    //             << "  element size = " << neighbor.second.elements.size() ;
    //   for(const auto& element : neighbor.second.elements) {
    //     ERT_PLOG_I << "    bound_type = " << static_cast<int>(element.bound_type)
    //               << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range)<< ", " <<
    //               std::get<2>(element.range)
    //               << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size()
    //               ;
    //   }
    // }

    // right neighbor
    for (const auto& neighbor : local_route_ref.right_reference_lines()) {
      right_reference_lines_[neighbor.neighbor_id()].id = neighbor.neighbor_id();
      right_reference_lines_[neighbor.neighbor_id()].elements.clear();
      for (const auto& element : neighbor.boundaries()) {
        NeighborLocalRoute::Element el;
        el.bound_type = static_cast<LocalRouteSegment::BoundaryType>(element.bound_type());
        std::get<0>(el.range) = true;
        std::get<1>(el.range) = element.start_s();
        std::get<2>(el.range) = element.end_s();
        right_reference_lines_[neighbor.neighbor_id()].elements.emplace_back(std::move(el));
      }
      if (right_reference_lines_[neighbor.neighbor_id()].elements.empty()) {
        right_reference_lines_.erase(neighbor.neighbor_id());
      }
    }
    // // print info
    // ERT_PLOG_I << "[ReferenceLine-LocalRoute]: reference_line id = " << id_
    //           << "  right_reference_lines_.size() = " << right_reference_lines_.size() ;
    // for(const auto& neighbor : right_reference_lines_) {
    //   ERT_PLOG_I << "  key = " << neighbor.first
    //             << "  neighbor local route id = " << neighbor.second.id
    //             << "  element size = " << neighbor.second.elements.size() ;
    //   for(const auto& element : neighbor.second.elements) {
    //     ERT_PLOG_I << "    bound_type = " << static_cast<int>(element.bound_type)
    //               << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range)<< ", " <<
    //               std::get<2>(element.range)
    //               << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size()
    //               ;
    //   }
    // }

    // merge and fork info
    for (const auto& range : local_route_ref.merge_fork_ranges()) {
      MergeForkRange res;
      res.setType(static_cast<MergeForkRange::MergeForkType>(range.type()));
      res.setS(range.s());
      std::tuple<std::string, std::string, std::vector<std::string>> relation;
      for (const auto& id : range.related_local_route_ids()) {
        std::get<2>(relation).emplace_back(id);
      }
      res.mutableRelationIdsInfo()->emplace_back(std::move(relation));
      merge_fork_ranges_.emplace_back(std::move(res));
    }

    // navigation_scores_
    for (const auto& score : local_route_ref.navigation_scores()) {
      navigation_scores_.emplace_back(score.navigation_score(), score.start_s(), score.end_s());
    }

    // areas
    for (const auto& id : local_route_ref.related_area_ids()) {
      PerceptionArea area;
      area.id = id;
      areas_.emplace_back(std::move(area));
    }

    // road_info_
    road_info_.road_id = road_info.road_id();
    road_info_.road_class = static_cast<RoadClass>(road_info.road_class());
    for (const auto& type : road_info.road_types()) {
      road_info_.road_kind_types.emplace_back(static_cast<RoadType>(type));
    }
  }
}

ReferenceLine::ReferenceLine(const proto::road_cognition::ReferenceLine& env_road_cognition_ref,
                             const MapPoint& adc_loc) {
  id_ = env_road_cognition_ref.id();
  reference_points_.clear();
  accumulated_s_.clear();
  line_segments_.clear();
  line_type_ = LineType::ENV_ROAD_COGNITION;
  smooth_type_ = static_cast<SmoothType>(env_road_cognition_ref.smooth_type());
  lane_direction_ = env_road_cognition_ref.road_direction();

  int env_ref_pts_size = env_road_cognition_ref.reference_points().size();
  for (int i = 0; i < env_ref_pts_size; ++i) {
    accumulated_s_.push_back(env_road_cognition_ref.reference_points().at(i).s());

    math::Vec3d middle_pos(env_road_cognition_ref.reference_points().at(i).point().x(),
                           env_road_cognition_ref.reference_points().at(i).point().y(),
                           env_road_cognition_ref.reference_points().at(i).point().z());
    math::Vec3d middle_rpy(0.0, 0.0, env_road_cognition_ref.reference_points().at(i).theta());

    if (reference_points_.size() > 0) {
      line_segments_.emplace_back(reference_points_.back(), middle_pos);
    }
    reference_points_.emplace_back(middle_pos, env_road_cognition_ref.reference_points().at(i).slope(), middle_rpy.z(),
                                   env_road_cognition_ref.reference_points().at(i).kappa(),
                                   env_road_cognition_ref.reference_points().at(i).dkappa(),
                                   env_road_cognition_ref.reference_points().at(i).lane_left_width(),
                                   -env_road_cognition_ref.reference_points().at(i).lane_right_width(),
                                   env_road_cognition_ref.reference_points().at(i).road_left_width(),
                                   -env_road_cognition_ref.reference_points().at(i).road_right_width(), 0.0,
                                   env_road_cognition_ref.reference_points().at(i).s());
  }
  length_ = env_road_cognition_ref.length();
  num_points_ = reference_points_.size();
  num_line_segments_ = num_points_ - 1;

  adc_loc_ = adc_loc;
  SLPoint sl_point;
  xy2sl(math::Vec3d(adc_loc_.x(), adc_loc_.y(), adc_loc_.z()), adc_loc_.yaw(), &sl_point);
  adc_s_ = sl_point.s();
  adc_l_ = sl_point.l();

  stop_lines_.clear();
  speed_limits_.clear();
  directions_.clear();
  line_attribute_ranges_.clear();
  left_lane_boundaries_.clear();
  right_lane_boundaries_.clear();
  left_road_boundaries_.clear();
  right_road_boundaries_.clear();
  related_risk_boundaries_ids_.clear();
  related_god_boundaries_.clear();
  gates_.clear();
  left_ref_lines_.clear();
  right_ref_lines_.clear();
  areas_.clear();
  road_ranges_.clear();
  key_points_.clear();
  navigation_lane_change_info_ = {-1.0, 0, 0};
  navigation_scores_.clear();
  shape_type_ = ShapeType::REGULAR;
  if (!id_.empty()) {
    // speed_limits
    for (const auto& pt : env_road_cognition_ref.speed_limits()) {
      SpeedLimit sp;
      sp.start_s = pt.start_s();
      sp.end_s = pt.end_s();
      sp.max_speed_limit = pt.max_speed_limit();
      sp.min_speed_limit = pt.min_speed_limit();
      sp.recommended_speed = pt.recommended_speed();
      speed_limits_.emplace_back(std::move(sp));
    }

    // directions_
    for (const auto& direction : env_road_cognition_ref.directions()) {
      SegmentDirection sd;
      sd.start_s = direction.start_s();
      sd.end_s = direction.end_s();
      sd.direction = static_cast<DrivingDirection>(direction.direction());
      ShapeType shape_type = ShapeType::REGULAR;
      if (sd.direction == DrivingDirection::kDirectionUTurnOnly) {
        shape_type = ShapeType::U_TURN;
      } else if (sd.direction == DrivingDirection::kDirectionLeftOnly) {
        shape_type = ShapeType::LEFT_TURN;
      } else if (sd.direction == DrivingDirection::kDirectionRightOnly) {
        shape_type = ShapeType::RIGHT_TURN;
      } else {
        shape_type = ShapeType::REGULAR;
      }
      if (shape_type > shape_type_) {
        shape_type_ = shape_type;
      }
      directions_.emplace_back(std::move(sd));
    }

    // line_attribute_ranges_
    for (const auto& line_attribute : env_road_cognition_ref.line_attributes()) {
      LineAttributeRange res;
      res.start_s = line_attribute.start_s();
      res.end_s = line_attribute.end_s();
      res.source_type = line_attribute.source_type();
      res.line_type = line_attribute.line_type();
      line_attribute_ranges_.emplace_back(std::move(res));
    }

    // left_lane_boundaries_
    for (const auto& boundary : env_road_cognition_ref.left_lane_boundaries()) {
      BoundaryRange res;
      res.start_s = boundary.start_s();
      res.end_s = boundary.end_s();
      res.type = boundary.bound_type();
      res.shape = boundary.bound_shape();
      res.color = boundary.bound_color();
      for (const auto& id : boundary.related_lane_markings_ids()) {
        res.related_lane_markings_ids.emplace_back(id);
      }
      left_lane_boundaries_.emplace_back(std::move(res));
    }
    // right_lane_boundaries_
    for (const auto& boundary : env_road_cognition_ref.right_lane_boundaries()) {
      BoundaryRange res;
      res.start_s = boundary.start_s();
      res.end_s = boundary.end_s();
      res.type = boundary.bound_type();
      res.shape = boundary.bound_shape();
      res.color = boundary.bound_color();
      for (const auto& id : boundary.related_lane_markings_ids()) {
        res.related_lane_markings_ids.emplace_back(id);
      }
      right_lane_boundaries_.emplace_back(std::move(res));
    }
    // left_road_boundaries_
    for (const auto& boundary : env_road_cognition_ref.left_road_boundaries()) {
      BoundaryRange res;
      res.start_s = boundary.start_s();
      res.end_s = boundary.end_s();
      res.type = boundary.bound_type();
      res.shape = boundary.bound_shape();
      res.color = boundary.bound_color();
      for (const auto& id : boundary.related_lane_markings_ids()) {
        res.related_lane_markings_ids.emplace_back(id);
      }
      left_road_boundaries_.emplace_back(std::move(res));
    }
    // right_road_boundaries_
    for (const auto& boundary : env_road_cognition_ref.right_road_boundaries()) {
      BoundaryRange res;
      res.start_s = boundary.start_s();
      res.end_s = boundary.end_s();
      res.type = boundary.bound_type();
      res.shape = boundary.bound_shape();
      res.color = boundary.bound_color();
      for (const auto& id : boundary.related_lane_markings_ids()) {
        res.related_lane_markings_ids.emplace_back(id);
      }
      right_road_boundaries_.emplace_back(std::move(res));
    }

    // related_risk_boundaries_ids_
    for (const auto& id : env_road_cognition_ref.related_risk_boundaries_ids()) {
      related_risk_boundaries_ids_.emplace_back(id);
    }

    // related_god_boundaries_
    for (const auto& boundary : env_road_cognition_ref.related_god_boundaries()) {
      GodBoundary res;
      res.type = boundary.type();
      for (int i = 0; i < std::min(boundary.s_arr().size(), boundary.l_arr().size()); i++) {
        res.boundary.emplace_back(boundary.s_arr().at(i), boundary.l_arr().at(i));
      }
      if (!res.boundary.empty()) {
        related_god_boundaries_.emplace_back(std::move(res));
      }
    }
    // std::cout << "related_god_boundaries_.size() = " << related_god_boundaries_.size() << std::endl;

    // stop_lines
    for (const auto& pt : env_road_cognition_ref.stop_lines()) {
      StopLine stop_line;
      stop_line.is_exist = true;
      stop_line.id = pt.id();
      stop_line.type = pt.type();
      stop_line.is_virtual = stop_line.type == proto::perception::StopLine_LineType_kTypeVirtual;
      stop_line.s = pt.s();
      stop_line.direction = static_cast<DrivingDirection>(pt.direction());
      stop_line.traffic_light_id = pt.related_traffic_light_id();
      stop_line.traffic_light_priority = pt.related_traffic_light_priority();
      stop_lines_.emplace_back(std::move(stop_line));
    }

    // gates
    for (const auto& pt : env_road_cognition_ref.gates()) {
      Gate gate;
      gate.is_exist = true;
      gate.id = pt.id();
      gate.type = pt.gate_type();
      gate.gate_status = pt.gate_status();
      gate.s = pt.s();
      gate.head_stop_distance = pt.head_stop_distance();
      gates_.emplace_back(std::move(gate));
    }

    // left neighbor
    for (const auto& neighbor : env_road_cognition_ref.left_reference_lines()) {
      left_ref_lines_[neighbor.neighbor_id()].id = neighbor.neighbor_id();
      left_ref_lines_[neighbor.neighbor_id()].boundaries.clear();
      for (const auto& boundary : neighbor.boundaries()) {
        BoundaryRange res;
        res.start_s = boundary.start_s();
        res.end_s = boundary.end_s();
        res.type = boundary.bound_type();
        res.shape = boundary.bound_shape();
        res.color = boundary.bound_color();
        for (const auto& id : boundary.related_lane_markings_ids()) {
          res.related_lane_markings_ids.emplace_back(id);
        }
        left_ref_lines_[neighbor.neighbor_id()].boundaries.emplace_back(std::move(res));
      }
      if (left_ref_lines_[neighbor.neighbor_id()].boundaries.empty()) {
        left_ref_lines_.erase(neighbor.neighbor_id());
      }
    }
    // right neighbor
    for (const auto& neighbor : env_road_cognition_ref.right_reference_lines()) {
      right_ref_lines_[neighbor.neighbor_id()].id = neighbor.neighbor_id();
      right_ref_lines_[neighbor.neighbor_id()].boundaries.clear();
      for (const auto& boundary : neighbor.boundaries()) {
        BoundaryRange res;
        res.start_s = boundary.start_s();
        res.end_s = boundary.end_s();
        res.type = boundary.bound_type();
        res.shape = boundary.bound_shape();
        res.color = boundary.bound_color();
        for (const auto& id : boundary.related_lane_markings_ids()) {
          res.related_lane_markings_ids.emplace_back(id);
        }
        right_ref_lines_[neighbor.neighbor_id()].boundaries.emplace_back(std::move(res));
      }
      if (right_ref_lines_[neighbor.neighbor_id()].boundaries.empty()) {
        right_ref_lines_.erase(neighbor.neighbor_id());
      }
    }

    // key_points_
    for (const auto& pt : env_road_cognition_ref.key_points()) {
      KeyPoint res;
      res.id = pt.id();
      res.type = pt.type();
      res.s = pt.s();
      res.pt.set_x(pt.point().x());
      res.pt.set_y(pt.point().y());
      res.pt.set_z(pt.point().z());
      for (const auto& id : pt.related_reference_lines_ids()) {
        res.related_reference_lines_ids.emplace_back(id);
      }
      key_points_.emplace_back(std::move(res));
    }

    // lane_marking_cross_attributes_
    for (const auto& lane_marking_cross_attribute : env_road_cognition_ref.lane_marking_cross_attributes()) {
      LaneMarkingCrossAttributeRange attribute;
      attribute.is_left_cross = lane_marking_cross_attribute.is_left_cross();
      attribute.lane_marking_id = lane_marking_cross_attribute.lane_marking_id();
      attribute.before_cross_ref_start_s = lane_marking_cross_attribute.before_cross_ref_start_s();
      attribute.before_cross_ref_end_s = lane_marking_cross_attribute.before_cross_ref_end_s();
      attribute.after_cross_ref_start_s = lane_marking_cross_attribute.after_cross_ref_start_s();
      attribute.after_cross_ref_end_s = lane_marking_cross_attribute.after_cross_ref_end_s();
      attribute.is_merge = lane_marking_cross_attribute.is_merge();
      attribute.related_ref_id = lane_marking_cross_attribute.related_ref_id();
      attribute.before_cross_related_ref_start_s = lane_marking_cross_attribute.before_cross_related_ref_start_s();
      attribute.before_cross_related_ref_end_s = lane_marking_cross_attribute.before_cross_related_ref_end_s();
      attribute.after_cross_related_ref_start_s = lane_marking_cross_attribute.after_cross_related_ref_start_s();
      attribute.after_cross_related_ref_end_s = lane_marking_cross_attribute.after_cross_related_ref_end_s();

      lane_marking_cross_attributes_.emplace_back(std::move(attribute));
    }

    // navigation_lane_change_info_
    navigation_lane_change_info_ = {env_road_cognition_ref.navigation_info().lane_change_distance(),
                                    env_road_cognition_ref.navigation_info().lane_change_direction(),
                                    env_road_cognition_ref.navigation_info().lane_change_times()};

    // navigation_scores_
    for (const auto& score : env_road_cognition_ref.navigation_info().navigation_scores()) {
      navigation_scores_.emplace_back(score.score(), score.start_s(), score.end_s());
    }

    // areas
    for (const auto& range : env_road_cognition_ref.areas()) {
      PerceptionArea area;
      area.id = range.area_id();
      area.start_s = range.start_s();
      area.end_s = range.end_s();
      areas_.emplace_back(std::move(area));
    }

    // road_ranges_
    for (const auto& range : env_road_cognition_ref.roads()) {
      RoadRange road;
      road.id = range.road_id();
      road.start_s = range.start_s();
      road.end_s = range.end_s();
      road_ranges_.emplace_back(std::move(road));
    }
  }
}

/**
 * @brief 获取参考线上某一点的横向偏移量
 * @details 根据给定的s值，获取参考线上对应点的横向偏移量，并将其存储在l_offset中
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[out] l_offset 存储横向偏移量的指针
 * @return bool 返回操作是否成功
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - l_offset (double*): 存储横向偏移量的指针
 *
 * @par 判断条件:
 * - 如果s值超出参考线范围，则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查s值是否在有效范围内;
 * if (s值无效?) then (yes)
 *   :返回false;
 * else (no)
 *   :获取s值对应的参考点;
 *   :将参考点的偏移量赋值给l_offset;
 *   :返回true;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上某一点的横向偏移量
 *
 * @warning 需确保输入的s值在参考线范围内，否则无法正确获取偏移量
 */
bool ReferenceLine::offset(const double s, double* l_offset) const {
  auto ref_point = getReferencePoint(s);
  *l_offset = ref_point.offset();
  return true;
}

/**
 * @brief 根据给定的(x, y)坐标获取参考线上最近的点
 * @details 通过将(x, y)坐标转换为SL坐标系，找到参考线上对应的参考点
 *
 * @param[in] x 输入点的x坐标
 * @param[in] y 输入点的y坐标
 * @return ReferencePoint 返回参考线上对应的参考点
 *
 * @par 关键变量说明:
 * - x (double): 输入点的x坐标
 * - y (double): 输入点的y坐标
 * - sl_point (SLPoint): 存储转换后的SL坐标
 *
 * @par 判断条件:
 * - 如果参考点列表为空，则返回默认构造的参考点
 * - 如果坐标转换失败，则输出错误日志并返回默认构造的参考点
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考点列表是否为空;
 * if (参考点列表为空?) then (yes)
 *   :返回默认构造的参考点;
 * else (no)
 *   :将(x, y)坐标转换为SL坐标;
 *   if (转换成功?) then (yes)
 *     :根据SL坐标获取参考点;
 *     :返回参考点;
 *   else (no)
 *     :输出错误日志;
 *     :返回默认构造的参考点;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于根据(x, y)坐标获取参考线上最近的点
 *
 * @warning 需确保输入的(x, y)坐标在参考线附近，否则可能无法正确获取参考点
 */
ReferencePoint ReferenceLine::getReferencePoint(const double x, const double y) const {
  CHECK_GE(reference_points_.size(), 0);
  SLPoint sl_point;
  if (!xy2sl({x, y, 0.0}, &sl_point)) {
    PERROR << "Failed find reference point.";
  }
  return getReferencePoint(sl_point.s());
}

/**
 * @brief 根据给定的s值获取参考线上对应的参考点
 * @details 通过s值在参考线上进行插值计算，获取对应点的位置、航向、曲率等信息
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return ReferencePoint 返回参考线上对应的参考点
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - upper_index (int): s值对应的上界索引
 * - index (int): s值对应的下界索引
 * - next_index (int): s值对应的上界索引
 * - p0 (ReferencePoint): 下界索引对应的参考点
 * - p1 (ReferencePoint): 上界索引对应的参考点
 * - s0 (double): 下界索引对应的累计s值
 * - s1 (double): 上界索引对应的累计s值
 *
 * @par 判断条件:
 * - 如果s值小于参考线起点，则返回第一个参考点
 * - 如果s值大于参考线终点，则返回最后一个参考点
 * - 如果s值在参考线范围内，则通过插值计算获取参考点
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查s值是否小于参考线起点;
 * if (s < accumulated_s_.front()) then (yes)
 *   :返回第一个参考点;
 * else (no)
 *   :检查s值是否大于参考线终点;
 *   if (s > accumulated_s_.back()) then (yes)
 *     :返回最后一个参考点;
 *   else (no)
 *     :获取s值对应的上界索引;
 *     :计算下界索引和上界索引;
 *     :获取下界和上界对应的参考点;
 *     :获取下界和上界对应的累计s值;
 *     :通过插值计算参考点的位置、航向、曲率等信息;
 *     :返回插值后的参考点;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于根据s值获取参考线上对应的参考点，适用于在参考线上进行插值计算
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取参考点
 */
ReferencePoint ReferenceLine::getReferencePoint(const double s) const {
  CHECK_GE(reference_points_.size(), 0);
  if (s < accumulated_s_.front() + math::kMathEpsilon) {
    return reference_points_.front();
  }
  if (s > accumulated_s_.back() - math::kMathEpsilon) {
    return reference_points_.back();
  }

  int upper_index = getUpperIndexFromS(s);
  int index = std::max(0, upper_index - 1);
  int next_index = upper_index;
  if (next_index >= reference_points_.size()) {
    next_index = reference_points_.size() - 1;
  }

  const auto& p0 = reference_points_[index];
  const auto& p1 = reference_points_[next_index];

  const double s0 = accumulated_s_[index];
  const double s1 = accumulated_s_[next_index];

  if (std::fabs(s0 - s1) < math::kMathEpsilon) {
    return p0;
  }
  const double x = math::lerp(p0.x(), s0, p1.x(), s1, s);
  const double y = math::lerp(p0.y(), s0, p1.y(), s1, s);
  const double z = math::lerp(p0.z(), s0, p1.z(), s1, s);
  const float slope = math::lerp(p0.slope(), s0, p1.slope(), s1, s);
  const double heading = math::slerp(p0.heading(), s0, p1.heading(), s1, s);

  const double kappa = math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);
  const double lw = math::lerp(p0.left_bound(), s0, p1.left_bound(), s1, s);
  const double rw = math::lerp(p0.right_bound(), s0, p1.right_bound(), s1, s);
  const double rlw = math::lerp(p0.road_left_bound(), s0, p1.road_left_bound(), s1, s);
  const double rrw = math::lerp(p0.road_right_bound(), s0, p1.road_right_bound(), s1, s);
  const double rdlw = math::lerp(p0.driving_tube_left_buffer(), s0, p1.driving_tube_left_buffer(), s1, s);
  const double rdrw = math::lerp(p0.driving_tube_right_buffer(), s0, p1.driving_tube_right_buffer(), s1, s);

  ReferencePoint ref_point(Vec3d(x, y, z), slope, heading, kappa, dkappa, lw, rw, rlw, rrw, 0.0, s);
  ref_point.setDrivingTubeBuffer(rdlw, rdrw);
  return ref_point;
}

/**
 * @brief 获取参考线上指定s值范围内的参考点
 * @details 根据给定的起始s值和结束s值，返回参考线上对应范围内的参考点列表
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::vector<ReferencePoint> 返回参考点列表
 *
 * @par 关键变量说明:
 * - start_s (double): 起始s值，范围[0, length_]
 * - end_s (double): 结束s值，范围[0, length_]
 * - ref_points (std::vector<ReferencePoint>): 存储返回的参考点列表
 *
 * @par 判断条件:
 * - 如果起始s值小于0，则将其设置为0
 * - 如果结束s值大于参考线长度，则将其设置为参考线长度
 * - 如果参考点列表为空，则返回空列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考点列表是否为空;
 * if (参考点列表为空?) then (yes)
 *   :返回空列表;
 * else (no)
 *   :检查起始s值是否小于0;
 *   if (start_s < 0) then (yes)
 *     :将start_s设置为0;
 *   endif
 *   :检查结束s值是否大于参考线长度;
 *   if (end_s > length()) then (yes)
 *     :将end_s设置为length();
 *   endif
 *   :获取起始s值对应的索引;
 *   :获取结束s值对应的索引;
 *   if (起始索引 < 结束索引) then (yes)
 *     :返回起始索引到结束索引之间的参考点;
 *   else (no)
 *     :返回空列表;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值范围内的参考点，适用于需要获取参考线某一段的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取参考点
 */
std::vector<ReferencePoint> ReferenceLine::getReferencePoints(double start_s, double end_s) const {
  std::vector<ReferencePoint> ref_points;
  if (reference_points_.empty()) {
    ERT_PLOG_I << "[ReferenceLine::getReferencePoints]: reference_points_ is empty!";
    return ref_points;
  }

  if (start_s < 0.0) {
    start_s = 0.0;
  }
  if (end_s > length()) {
    end_s = length();
  }
  auto start_index = getNearestReferenceIndex(start_s);
  auto end_index = getNearestReferenceIndex(end_s);
  if (start_index < end_index) {
    ref_points.assign(reference_points_.begin() + start_index, reference_points_.begin() + end_index);
  }
  return ref_points;
}

/**
 * @brief 获取参考线上指定s值范围内的插值参考点
 * @details 根据给定的起始s值、结束s值和点间隔，返回参考线上对应范围内的插值参考点列表
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @param[in] point_interval 点间隔，范围(0, +∞)
 * @return std::vector<ReferencePoint> 返回插值参考点列表
 *
 * @par 关键变量说明:
 * - start_s (double): 起始s值，范围[0, length_]
 * - end_s (double): 结束s值，范围[0, length_]
 * - point_interval (double): 点间隔，范围(0, +∞)
 * - ref_points_Interpolated (std::vector<ReferencePoint>): 存储返回的插值参考点列表
 *
 * @par 判断条件:
 * - 如果起始s值小于0，则将其设置为0
 * - 如果结束s值大于参考线长度，则将其设置为参考线长度
 * - 如果点间隔小于等于0，则返回空列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查起始s值是否小于0;
 * if (start_s < 0) then (yes)
 *   :将start_s设置为0;
 * endif
 * :检查结束s值是否大于参考线长度;
 * if (end_s > length()) then (yes)
 *   :将end_s设置为length();
 * endif
 * :检查点间隔是否小于等于0;
 * if (point_interval <= 0) then (yes)
 *   :返回空列表;
 * else (no)
 *   :从start_s开始，以point_interval为步长遍历到end_s;
 *   :获取每个s值对应的参考点;
 *   :将参考点加入插值参考点列表;
 * endif
 * :返回插值参考点列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值范围内的插值参考点，适用于需要均匀采样参考点的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取参考点
 */
std::vector<ReferencePoint> ReferenceLine::getInterpolatedRefPoints(double start_s, double end_s,
                                                                    double point_interval) const {
  if (start_s < 0.0) {
    start_s = 0.0;
  }
  if (end_s > length()) {
    end_s = length();
  }

  std::vector<ReferencePoint> ref_points_Interpolated;

  // lxy 20240812 modify
  // double pieces = (end_s - start_s) / point_interval + 1.0;
  // double s = start_s;
  // for (size_t i = 0; i < pieces; i++, s = std::fmin(s + point_interval, end_s)) {
  //   ReferencePoint ref_point = getReferencePoint(s);
  //   ref_points_Interpolated.emplace_back(ref_point);
  // }
  for (double s = start_s; s < end_s; s += point_interval) {
    ReferencePoint ref_point = getReferencePoint(s);
    ref_points_Interpolated.emplace_back(ref_point);
  }

  return ref_points_Interpolated;
}

/**
 * @brief 获取记忆路线的参考点
 * @details 根据给定的原点坐标和自车位置，更新记忆路线的参考点，并计算自车在参考线上的位置
 *
 * @param[in] origin 原点坐标，用于将参考点从地图坐标系转换到自车坐标系
 * @param[in] loc 自车位置信息，包含自车的x、y、z坐标
 *
 * @par 关键变量说明:
 * - origin (math::Vec3d): 原点坐标，用于坐标转换
 * - loc (MapPoint): 自车位置信息
 * - min_dis (float): 自车到参考点的最小距离
 * - adc_s_ (double): 自车在参考线上的s值
 * - adc_l_ (double): 自车在参考线上的横向偏移
 *
 * @par 判断条件:
 * - 如果参考点列表为空，则直接返回
 * - 如果自车位置与参考点的距离小于当前最小距离，则更新最小距离和自车位置
 *
 * @par 流程图:
 * @startuml
 * start
 * :清空线段列表;
 * :初始化自车位置和s值;
 * :遍历参考点列表;
 * :将参考点坐标转换到自车坐标系;
 * :计算自车到参考点的距离;
 * if (距离 < 最小距离) then (yes)
 *   :更新最小距离;
 *   :更新自车s值和横向偏移;
 * endif
 * :生成线段;
 * :根据参考线ID更新记忆路线ID;
 * stop
 * @enduml
 *
 * @note 该函数用于获取记忆路线的参考点，适用于记忆路线的场景
 *
 * @warning 需确保输入的origin和loc参数有效，否则可能无法正确更新参考点
 */
void ReferenceLine::getMemorizedRouteReferencePoints(const math::Vec3d& origin, const MapPoint& loc) {
  line_segments_.clear();
  adc_loc_ = loc;
  adc_s_ = 0.0;
  adc_l_ = 0.0;
  float min_dis = std::numeric_limits<float>::max();
  for (int i = 0; i < reference_points_.size(); ++i) {
    reference_points_.at(i).set_x(reference_points_.at(i).x() + origin.x());
    reference_points_.at(i).set_y(reference_points_.at(i).y() + origin.y());
    float dis = std::sqrt((reference_points_.at(i).x() - loc.x()) * (reference_points_.at(i).x() - loc.x()) +
                          (reference_points_.at(i).y() - loc.y()) * (reference_points_.at(i).y() - loc.y()));
    if (dis < min_dis) {
      min_dis = dis;
      adc_s_ = reference_points_.at(i).local_s();
      adc_l_ = dis;
    }

    if (i > 0) {
      line_segments_.emplace_back(reference_points_.at(i - 1), reference_points_.at(i));
    }
  }

  if (id_.find("left") != std::string::npos) {
    id_ = "memorized_route_sliced_left";
  } else if (id_.find("right") != std::string::npos) {
    id_ = "memorized_route_sliced_right";
  } else {
    id_ = "memorized_route_sliced_ego";
  }
}

/**
 * @brief 根据感知车道信息修改记忆参考线的边界
 * @details 通过比较感知车道与记忆参考线的偏差，调整记忆参考线的左右边界，使其与感知车道对齐
 *
 * @param[in] lane 感知车道信息，包含车道线、边界等信息
 * @param[in] theta_deviation_thresold 航向角偏差阈值，范围[0, +∞)
 * @param[in] lateral_deviation_thresold 横向偏移偏差阈值，范围[0, +∞)
 *
 * @par 关键变量说明:
 * - lane (PerceptionLane): 感知车道信息
 * - theta_deviation_thresold (float): 航向角偏差阈值
 * - lateral_deviation_thresold (float): 横向偏移偏差阈值
 * - bounds (util::AbstractTable1d<float, float, float>): 存储调整后的边界信息
 *
 * @par 判断条件:
 * - 如果航向角偏差或横向偏移偏差超过阈值，则不对边界进行调整
 * - 如果调整后的边界信息为空，则不对记忆参考线进行修改
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化边界修改标志为false;
 * :遍历感知车道的参考点;
 * :计算感知车道参考点的半车道宽度;
 * :将感知车道参考点坐标转换为SL坐标;
 * :获取记忆参考线上对应s值的参考点;
 * :计算航向角偏差和横向偏移;
 * if (航向角偏差 <= 阈值 && 横向偏移 <= 阈值) then (yes)
 *   :根据感知车道调整记忆参考线的左右边界;
 *   :将调整后的边界信息加入bounds;
 * else (no)
 *   :清空bounds;
 *   :返回;
 * endif
 * :对bounds进行排序;
 * if (bounds不为空) then (yes)
 *   :遍历记忆参考线的参考点;
 *   :根据bounds插值调整参考点的左右边界;
 *   :设置边界修改标志为true;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于根据感知车道信息调整记忆参考线的边界，适用于记忆参考线与感知车道对齐的场景
 *
 * @warning 需确保输入的感知车道信息有效，否则可能无法正确调整参考线边界
 */
void ReferenceLine::modifyMemorizedReferenceLineBoundByPerceptionLane(const PerceptionLane& lane,
                                                                      float theta_deviation_thresold,
                                                                      float lateral_deviation_thresold) {
  is_bound_modified_by_lane_ = false;
  ReferenceLine lane_ref_line(&lane);
  util::AbstractTable1d<float, float, float> bounds;
  // ERT_PLOG_I << "[useLaneBounds]: Begin-------------------------------------------" ;
  for (float s = lane_ref_line.adcS(); s <= lane_ref_line.length(); s += 1.0) {
    ReferencePoint lane_pt = lane_ref_line.getReferencePoint(s);
    float half_lane_width = 0.5 * (lane_pt.left_bound() - lane_pt.right_bound());
    SLPoint sl;
    xy2sl(math::Vec3d(lane_pt.x(), lane_pt.y(), lane_pt.z()), &sl);

    ReferencePoint mem_pt = getReferencePoint(sl.s());
    float delta_theta = math::NormalizeAngle(lane_pt.heading() - mem_pt.heading());
    float left_bound = mem_pt.left_bound();
    float right_bound = mem_pt.right_bound();
    // ERT_PLOG_I << "[useLaneBounds]: before: delta_theta = " << delta_theta * RAD2ANG << "  sl.l() = " << sl.l() << "
    // sl.s() = " << sl.s() << "  left_bound = " << left_bound << "  right_bound = " << right_bound ;
    if (std::fabs(delta_theta) <= theta_deviation_thresold && std::fabs(sl.l()) <= lateral_deviation_thresold) {
      left_bound = sl.l() + half_lane_width / std::cos(delta_theta);
      right_bound = sl.l() - half_lane_width / std::cos(delta_theta);
      // ERT_PLOG_I << "[useLaneBounds]:     modify: left_bound = " << left_bound << "  right_bound = " << right_bound ;
      bounds.emplace_back(sl.s(), right_bound, left_bound);
    } else {
      // ERT_PLOG_I << "[useLaneBounds]:can not modify!!!" ;
      bounds.clear();
      return;
    }
  }
  // ERT_PLOG_I << "[useLaneBounds]: End-------------------------------------------" ;

  // get memorized reference line interpolated bounds by local_s
  bounds.sort();
  if (!bounds.empty()) {
    is_bound_modified_by_lane_ = true;
    for (auto& pt : reference_points_) {
      auto [bound, has_bound] = bounds.interpolate(pt.local_s());
      if (!has_bound) {
        std::get<1>(bound) = pt.right_bound();
        std::get<2>(bound) = pt.left_bound();
      }
      pt.setLeftBound(std::get<2>(bound));
      pt.setRightBound(std::get<1>(bound));
      pt.setRoadLeftBound(std::get<2>(bound));
      pt.setRoadRightBound(std::get<1>(bound));
    }
  }
}

/**
 * @brief 获取参考线上距离给定s值最近的参考点索引
 * @details 根据给定的s值，在参考线的累计s值列表中查找最近的参考点索引
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return size_t 返回最近的参考点索引
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - accumulated_s_ (std::vector<double>): 参考线的累计s值列表
 *
 * @par 判断条件:
 * - 如果参考点列表为空，则返回0
 * - 如果s值小于参考线起点，则返回第一个参考点索引
 * - 如果s值大于参考线终点，则返回最后一个参考点索引
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考点列表是否为空;
 * if (参考点列表为空?) then (yes)
 *   :返回0;
 * else (no)
 *   :检查s值是否小于参考线起点;
 *   if (s < accumulated_s_.front()) then (yes)
 *     :返回第一个参考点索引;
 *   else (no)
 *     :检查s值是否大于参考线终点;
 *     if (s > accumulated_s_.back()) then (yes)
 *       :返回最后一个参考点索引;
 *     else (no)
 *       :使用lower_bound查找s值对应的上界索引;
 *       :返回上界索引;
 *     endif
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上距离给定s值最近的参考点索引，适用于需要快速定位参考点的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取参考点索引
 */
size_t ReferenceLine::getNearestReferenceIndex(const double s) const {
  if (reference_points_.empty()) {
    ERT_PLOG_I << "[ReferenceLine::getNearestReferenceIndex]: reference_points_ is empty!";
    return 0;
  }
  if (s < accumulated_s_.front()) {
    return 0;
  }
  if (s > accumulated_s_.back()) {
    return (reference_points_.size() - 1);
  }
  auto it_lower = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  return std::distance(accumulated_s_.begin(), it_lower);
}

/**
 * @brief 获取参考线上距离给定坐标点最近的参考点
 * @details 通过计算给定坐标点与参考线上所有参考点的距离，找到距离最近的参考点
 *
 * @param[in] xyz 输入点的三维坐标
 * @return ReferencePoint 返回参考线上距离最近的参考点
 *
 * @par 关键变量说明:
 * - xyz (math::Vec3d): 输入点的三维坐标
 * - min_dist (double): 当前最小距离，初始值为最大值
 * - min_index (size_t): 当前最小距离对应的参考点索引
 *
 * @par 判断条件:
 * - 如果参考点列表为空，则返回默认构造的参考点
 * - 如果找到更近的参考点，则更新最小距离和索引
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考点列表是否为空;
 * if (参考点列表为空?) then (yes)
 *   :返回默认构造的参考点;
 * else (no)
 *   :初始化最小距离为最大值;
 *   :遍历参考点列表;
 *   :计算输入点与当前参考点的距离;
 *   if (距离 < 最小距离) then (yes)
 *     :更新最小距离和索引;
 *   endif
 *   :返回最小距离对应的参考点;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上距离给定坐标点最近的参考点，适用于需要快速定位参考点的场景
 *
 * @warning 需确保输入的坐标点在参考线附近，否则可能无法正确获取参考点
 */
ReferencePoint ReferenceLine::getNearestReferencePoint(const math::Vec3d& xyz) const {
  if (reference_points_.empty()) {
    ERT_PLOG_I << "[ReferenceLine::getNearestReferencePoint]: reference_points_ is empty!";
    return ReferencePoint();
  }
  double min_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;
  for (size_t i = 0; i < reference_points_.size(); ++i) {
    const double distance = DistanceXY(xyz, reference_points_[i]);
    if (distance < min_dist) {
      min_dist = distance;
      min_index = i;
    }
  }
  return reference_points_[min_index];
}

/**
 * @brief 获取参考线上距离给定s值最近的参考点
 * @details 根据给定的s值，在参考线的累计s值列表中查找最近的参考点
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return ReferencePoint 返回参考线上距离最近的参考点
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - accumulated_s_ (std::vector<double>): 参考线的累计s值列表
 * - upper_index (int): s值对应的上界索引
 * - index (int): s值对应的下界索引
 * - next_index (int): s值对应的上界索引
 * - p0 (ReferencePoint): 下界索引对应的参考点
 * - p1 (ReferencePoint): 上界索引对应的参考点
 * - s0 (double): 下界索引对应的累计s值
 * - s1 (double): 上界索引对应的累计s值
 *
 * @par 判断条件:
 * - 如果s值小于参考线起点，则返回第一个参考点
 * - 如果s值大于参考线终点，则返回最后一个参考点
 * - 如果s值在参考线范围内，则通过插值计算获取参考点
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查s值是否小于参考线起点;
 * if (s < accumulated_s_.front()) then (yes)
 *   :返回第一个参考点;
 * else (no)
 *   :检查s值是否大于参考线终点;
 *   if (s > accumulated_s_.back()) then (yes)
 *     :返回最后一个参考点;
 *   else (no)
 *     :获取s值对应的上界索引;
 *     :计算下界索引和上界索引;
 *     :获取下界和上界对应的参考点;
 *     :获取下界和上界对应的累计s值;
 *     :通过插值计算参考点的位置、航向、曲率等信息;
 *     :返回插值后的参考点;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于根据s值获取参考线上距离最近的参考点，适用于需要快速定位参考点的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取参考点
 */
ReferencePoint ReferenceLine::getNearestReferencePoint(const double s) const {
  if (reference_points_.empty()) {
    ERT_PLOG_I << "[ReferenceLine::getNearestReferencePoint]: reference_points_ is empty!";
    return ReferencePoint();
  }
  if (s < accumulated_s_.front() - 1e-2) {
    return reference_points_.front();
  }
  if (s > accumulated_s_.back() + 1e-2) {
    return reference_points_.back();
  }
  auto it_lower = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  if (it_lower == accumulated_s_.begin()) {
    return reference_points_.front();
  }
  auto index = std::distance(accumulated_s_.begin(), it_lower);
  if (std::fabs(accumulated_s_[index - 1] - s) < std::fabs(accumulated_s_[index] - s)) {
    return reference_points_[index - 1];
  }
  return reference_points_[index];
}

/**
 * @brief 获取参考线上给定s值对应的上界索引
 * @details 根据给定的s值，在参考线的累计s值列表中查找上界索引
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return int 返回s值对应的上界索引
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - accumulated_s_ (std::vector<double>): 参考线的累计s值列表
 *
 * @par 判断条件:
 * - 如果s值小于等于0，则返回第一个参考点索引
 * - 如果s值大于等于参考线长度，则返回最后一个参考点索引
 * - 如果s值在参考线范围内，则使用upper_bound查找上界索引
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查s值是否小于等于0;
 * if (s <= 0) then (yes)
 *   :返回第一个参考点索引;
 * else (no)
 *   :检查s值是否大于等于参考线长度;
 *   if (s >= length_) then (yes)
 *     :返回最后一个参考点索引;
 *   else (no)
 *     :使用upper_bound查找s值对应的上界索引;
 *     :返回上界索引;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上给定s值对应的上界索引，适用于需要快速定位参考点的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取上界索引
 */
int ReferenceLine::getUpperIndexFromS(double s) const {
  if (s <= 1e-2) {
    return 0;
  }
  if (s >= length_ - 1e-2) {
    return num_points_ - 1;
  }
  int pos = std::upper_bound(accumulated_s_.begin(), accumulated_s_.end(), s) - accumulated_s_.begin();
  return pos;
}

/**
 * @brief 将路径点从笛卡尔坐标系转换到Frenet坐标系
 * @details 通过将路径点的(x, y, z)坐标转换为SL坐标，并计算横向偏移的一阶和二阶导数，生成Frenet坐标系下的点
 *
 * @param[in] path_point 输入的路径点，包含位置、航向、曲率等信息
 * @return FrenetFramePoint 返回Frenet坐标系下的点
 *
 * @par 关键变量说明:
 * - path_point (planning::PathPt): 输入的路径点
 * - sl_point (SLPoint): 存储转换后的SL坐标
 * - frenet_frame_point (FrenetFramePoint): 存储生成的Frenet坐标系点
 * - theta (double): 路径点的航向角
 * - kappa (double): 路径点的曲率
 * - l (double): 路径点的横向偏移
 * - ref_point (ReferencePoint): 参考线上对应s值的参考点
 * - theta_ref (double): 参考点的航向角
 * - kappa_ref (double): 参考点的曲率
 * - dkappa_ref (double): 参考点的曲率变化率
 * - dl (double): 横向偏移的一阶导数
 * - ddl (double): 横向偏移的二阶导数
 *
 * @par 判断条件:
 * - 如果参考点列表为空，则返回默认构造的Frenet坐标系点
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考点列表是否为空;
 * if (参考点列表为空?) then (yes)
 *   :返回默认构造的Frenet坐标系点;
 * else (no)
 *   :将路径点坐标转换为SL坐标;
 *   :获取参考线上对应s值的参考点;
 *   :计算横向偏移的一阶和二阶导数;
 *   :生成Frenet坐标系点;
 *   :返回Frenet坐标系点;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于将路径点从笛卡尔坐标系转换到Frenet坐标系，适用于需要将路径点映射到参考线的场景
 *
 * @warning 需确保输入的路径点在参考线附近，否则可能无法正确生成Frenet坐标系点
 */
FrenetFramePoint ReferenceLine::getFrenetPoint(const planning::PathPt& path_point) const {
  if (reference_points_.empty()) {
    ERT_PLOG_I << "[ReferenceLine::getFrenetPoint]: reference_points_ is empty!";
    return FrenetFramePoint();
  }

  SLPoint sl_point;
  xy2sl({path_point.x(), path_point.y(), path_point.z()}, &sl_point);
  FrenetFramePoint frenet_frame_point;
  frenet_frame_point.set_s(sl_point.s());
  frenet_frame_point.set_l(sl_point.l());

  const double theta = path_point.theta();
  const double kappa = path_point.kappa();
  const double l = frenet_frame_point.l();

  ReferencePoint ref_point = getReferencePoint(frenet_frame_point.s());

  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = CartesianFrenetConverter::CalculateLateralDerivative(theta_ref, theta, l, kappa_ref);
  const double ddl = CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(theta_ref, theta, kappa_ref, kappa,
                                                                                     dkappa_ref, l);
  frenet_frame_point.set_dl(dl);
  frenet_frame_point.set_ddl(ddl);
  return frenet_frame_point;
}

/**
 * @brief 将轨迹点从笛卡尔坐标系转换到Frenet坐标系
 * @details 通过将轨迹点的(x, y, z)坐标转换为SL坐标，并计算速度、加速度等条件，生成Frenet坐标系下的状态
 *
 * @param[in] traj_point 输入的轨迹点，包含位置、速度、加速度、航向、曲率等信息
 * @return std::pair<std::array<double, 3>, std::array<double, 3>> 返回Frenet坐标系下的状态，包含s条件和l条件
 *
 * @par 关键变量说明:
 * - traj_point (planning::TrajectoryPt): 输入的轨迹点
 * - sl_point (SLPoint): 存储转换后的SL坐标
 * - s_condition (std::array<double, 3>): 存储s方向的状态，包含s值、s方向速度、s方向加速度
 * - l_condition (std::array<double, 3>): 存储l方向的状态，包含l值、l方向速度、l方向加速度
 * - ref_point (ReferencePoint): 参考线上对应s值的参考点
 *
 * @par 判断条件:
 * - 如果参考点列表为空，则直接返回默认构造的Frenet坐标系状态
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考点列表是否为空;
 * if (参考点列表为空?) then (yes)
 *   :返回默认构造的Frenet坐标系状态;
 * else (no)
 *   :将轨迹点坐标转换为SL坐标;
 *   :获取参考线上对应s值的参考点;
 *   :计算s方向和l方向的状态;
 *   :返回Frenet坐标系状态;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于将轨迹点从笛卡尔坐标系转换到Frenet坐标系，适用于需要将轨迹点映射到参考线的场景
 *
 * @warning 需确保输入的轨迹点在参考线附近，否则可能无法正确生成Frenet坐标系状态
 */
std::pair<std::array<double, 3>, std::array<double, 3>> ReferenceLine::toFrenetFrame(
    const planning::TrajectoryPt& traj_point) const {
  CHECK(!reference_points_.empty());

  SLPoint sl_point;
  xy2sl({traj_point.path_pt().x(), traj_point.path_pt().y(), traj_point.path_pt().z()}, &sl_point);

  // ERT_PLOG_I<<"[ReferenceLine::toFrenetFrame]: sl_point : s = "<<sl_point.s()<<"  l = "<<sl_point.l();

  std::array<double, 3> s_condition;
  std::array<double, 3> l_condition;
  ReferencePoint ref_point = getReferencePoint(sl_point.s());
  CartesianFrenetConverter::cartesian_to_frenet(
      sl_point.s(), ref_point.x(), ref_point.y(), ref_point.heading(), ref_point.kappa(), ref_point.dkappa(),
      traj_point.path_pt().x(), traj_point.path_pt().y(), traj_point.v(), traj_point.a(), traj_point.path_pt().theta(),
      traj_point.path_pt().kappa(), &s_condition, &l_condition);

  return std::make_pair(s_condition, l_condition);
}

/**
 * @brief 获取参考线上指定s值范围内的导航评分
 * @details 根据给定的s值、前向距离阈值和后向距离阈值，返回参考线上对应范围内的导航评分列表
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_distance_thrd 前向距离阈值，范围[0, +∞)
 * @param[in] backward_dist_thrd 后向距离阈值，范围[0, +∞)
 * @return std::vector<std::tuple<float, float, float>> 返回导航评分列表，每个元素包含评分、起始s值和结束s值
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - forward_distance_thrd (double): 前向距离阈值，范围[0, +∞)
 * - backward_dist_thrd (double): 后向距离阈值，范围[0, +∞)
 * - res (std::vector<std::tuple<float, float, float>>): 存储返回的导航评分列表
 *
 * @par 判断条件:
 * - 如果参考线类型为记忆路线且为平行虚拟线，则返回默认评分
 * - 如果参考线类型为记忆路线且不为平行虚拟线，则返回默认评分
 * - 如果参考线类型不为记忆路线，则遍历导航评分列表，返回符合条件的评分
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考线类型是否为记忆路线;
 * if (参考线类型为记忆路线?) then (yes)
 *   :检查是否为平行虚拟线;
 *   if (是平行虚拟线?) then (yes)
 *     :返回默认评分;
 *   else (no)
 *     :返回默认评分;
 *   endif
 * else (no)
 *   :遍历导航评分列表;
 *   :检查评分是否在指定范围内;
 *   if (评分在指定范围内?) then (yes)
 *     :将评分加入返回列表;
 *   endif
 * endif
 * :返回导航评分列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值范围内的导航评分，适用于需要根据导航评分进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取导航评分
 */
std::vector<std::tuple<float, float, float>> ReferenceLine::getNavigationScoresFromSRange(
    const double s, const double forward_distance_thrd, const double backward_dist_thrd) const {
  return getNavigationScoresFromSRange(s - backward_dist_thrd, s + forward_distance_thrd);
}

/**
 * @brief 获取参考线上指定s值范围内的导航评分
 * @details 根据给定的起始s值和结束s值，返回参考线上对应范围内的导航评分列表
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::vector<std::tuple<float, float, float>> 返回导航评分列表，每个元素包含评分、起始s值和结束s值
 *
 * @par 关键变量说明:
 * - start_s (float): 起始s值，范围[0, length_]
 * - end_s (float): 结束s值，范围[0, length_]
 * - res (std::vector<std::tuple<float, float, float>>): 存储返回的导航评分列表
 *
 * @par 判断条件:
 * - 如果参考线类型为记忆路线且为平行虚拟线，则返回默认评分
 * - 如果参考线类型为记忆路线且不为平行虚拟线，则返回默认评分
 * - 如果参考线类型不为记忆路线，则遍历导航评分列表，返回符合条件的评分
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考线类型是否为记忆路线;
 * if (参考线类型为记忆路线?) then (yes)
 *   :检查是否为平行虚拟线;
 *   if (是平行虚拟线?) then (yes)
 *     :返回默认评分;
 *   else (no)
 *     :返回默认评分;
 *   endif
 * else (no)
 *   :遍历导航评分列表;
 *   :检查评分是否在指定范围内;
 *   if (评分在指定范围内?) then (yes)
 *     :将评分加入返回列表;
 *   endif
 * endif
 * :返回导航评分列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值范围内的导航评分，适用于需要根据导航评分进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取导航评分
 */
std::vector<std::tuple<float, float, float>> ReferenceLine::getNavigationScoresFromSRange(const float& start_s,
                                                                                          const float& end_s) const {
  std::vector<std::tuple<float, float, float>> res;
  if (line_type_ == LineType::MEMORIZED_ROUTE) {
    if (is_parallel_virtual_) {
      res.emplace_back(0.0, start_s, end_s);
    } else {
      res.emplace_back(1.0, start_s, end_s);
    }
  } else {
    for (const auto& score : navigation_scores_) {
      if (std::get<2>(score) < start_s) {
        continue;
      } else if (std::get<1>(score) > end_s) {
        break;
      }
      res.emplace_back(score);
    }
  }
  return res;
}

/**
 * @brief 获取参考线上指定s值对应的导航评分
 * @details 根据给定的s值，返回参考线上对应位置的导航评分
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return float 返回导航评分，范围[0, 1]
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - line_type_ (LineType): 参考线类型，用于判断是否为记忆路线
 * - is_parallel_virtual_ (bool): 是否为平行虚拟线，用于判断记忆路线的评分
 * - navigation_scores_ (std::vector<std::tuple<float, float, float>>):
 * 存储导航评分列表，每个元素包含评分、起始s值和结束s值
 *
 * @par 判断条件:
 * - 如果参考线类型为记忆路线且为平行虚拟线，则返回默认评分0.0
 * - 如果参考线类型为记忆路线且不为平行虚拟线，则返回默认评分1.0
 * - 如果参考线类型不为记忆路线，则遍历导航评分列表，返回符合条件的评分
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考线类型是否为记忆路线;
 * if (参考线类型为记忆路线?) then (yes)
 *   :检查是否为平行虚拟线;
 *   if (是平行虚拟线?) then (yes)
 *     :返回默认评分0.0;
 *   else (no)
 *     :返回默认评分1.0;
 *   endif
 * else (no)
 *   :遍历导航评分列表;
 *   :检查s值是否在评分范围内;
 *   if (s值在评分范围内?) then (yes)
 *     :返回对应评分;
 *   endif
 * endif
 * :返回默认评分0.0;
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值对应的导航评分，适用于需要根据导航评分进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取导航评分
 */
float ReferenceLine::getNavigationScoreFromS(const double s) const {
  if (line_type_ == LineType::MEMORIZED_ROUTE) {
    if (is_parallel_virtual_) {
      return 0.0;
    } else {
      return 1.0;
    }
  } else {
    for (const auto& score : navigation_scores_) {
      if (s >= std::get<1>(score) && s <= std::get<2>(score)) {
        return std::get<0>(score);
      }
    }
  }

  return 0.0;
}

/**
 * @brief 检查自车是否在车道线巡线范围内
 * @details 根据自车的s值，检查是否在车道线巡线范围内
 *
 * @return bool 返回true表示自车在车道线巡线范围内，否则返回false
 *
 * @par 关键变量说明:
 * - adc_s_ (double): 自车的s值
 * - lane_follow_ranges_ (std::vector<std::pair<float, float>>): 车道线巡线范围列表，每个元素包含起始s值和结束s值
 *
 * @par 判断条件:
 * - 如果自车的s值在任一车道线巡线范围内，则返回true
 * - 如果自车的s值不在任何车道线巡线范围内，则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历车道线巡线范围列表;
 * :检查自车s值是否在当前范围内;
 * if (自车s值在当前范围内?) then (yes)
 *   :返回true;
 * else (no)
 *   :继续遍历;
 * endif
 * :返回false;
 * stop
 * @enduml
 *
 * @note 该函数用于检查自车是否在车道线巡线范围内，适用于需要判断自车是否在特定车道范围内的场景
 *
 * @warning 需确保自车的s值在参考线范围内，否则可能无法正确判断
 */
bool ReferenceLine::isInLaneFollowRanges() const {
  for (const auto& range : lane_follow_ranges_) {
    if (adc_s_ >= range.first && adc_s_ <= range.second) {
      return true;
    }
  }
  return false;
}

/**
 * @brief 获取参考线上指定s值范围内的最大曲率
 * @details 根据给定的起始s值和结束s值，返回参考线上对应范围内的最大曲率
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return double 返回最大曲率值
 *
 * @par 关键变量说明:
 * - start_s (double): 起始s值，范围[0, length_]
 * - end_s (double): 结束s值，范围[0, length_]
 * - kEps (constexpr double): 容差值，用于判断s值是否在参考线范围内
 * - max_curvature (double): 当前最大曲率值
 * - reference_points_ (std::vector<ReferencePoint>): 参考点列表，包含每个参考点的s值和曲率
 *
 * @par 判断条件:
 * - 如果起始s值大于结束s值，或者起始s值大于参考线终点，或者结束s值小于参考线起点，则返回0
 * - 如果参考点列表为空，则返回0
 * - 遍历参考点列表，找到指定s值范围内的最大曲率
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输入s值是否有效;
 * if (输入s值无效?) then (yes)
 *   :返回0;
 * else (no)
 *   :初始化最大曲率为0;
 *   :遍历参考点列表;
 *   :检查参考点s值是否在指定范围内;
 *   if (参考点s值在指定范围内?) then (yes)
 *     :更新最大曲率;
 *   endif
 *   :返回最大曲率;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值范围内的最大曲率，适用于需要根据曲率进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取最大曲率
 */
double ReferenceLine::getMaxCurvatureInRange(double start_s, double end_s) const {
  constexpr double kEps = 1e-2;
  if (end_s < start_s || start_s > accumulated_s_.back() || end_s < accumulated_s_.front()) {
    PERROR << "invalid input";
    return 0;
  }
  // if (end_s > accumulated_s_.back()) {
  //   end_s = accumulated_s_.back();
  // }
  // if (start_s < accumulated_s_.front()) {
  //   start_s = accumulated_s_.front();
  // }
  double max_curvature = 0.0;
  for (const auto& p : reference_points_) {
    if (p.local_s() >= start_s && p.local_s() <= end_s) {
      if (std::fabs(p.kappa()) > max_curvature) {
        max_curvature = std::fabs(p.kappa());
      }
    }
    if (p.local_s() > end_s) {
      break;
    }
  }
  return max_curvature;
}

/**
 * @brief 获取参考线上指定s值范围内的最大和最小曲率
 * @details 根据给定的起始s值和结束s值，返回参考线上对应范围内的最大和最小曲率
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::pair<double, double> 返回最大和最小曲率值
 *
 * @par 关键变量说明:
 * - start_s (double): 起始s值，范围[0, length_]
 * - end_s (double): 结束s值，范围[0, length_]
 * - kEps (constexpr double): 容差值，用于判断s值是否在参考线范围内
 * - max_curvature (double): 当前最大曲率值
 * - min_curvature (double): 当前最小曲率值
 * - reference_points_ (std::vector<ReferencePoint>): 参考点列表，包含每个参考点的s值和曲率
 *
 * @par 判断条件:
 * - 如果起始s值大于结束s值，或者起始s值大于参考线终点，或者结束s值小于参考线起点，则返回{0, 0}
 * - 如果参考点列表为空，则返回{0, 0}
 * - 遍历参考点列表，找到指定s值范围内的最大和最小曲率
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输入s值是否有效;
 * if (输入s值无效?) then (yes)
 *   :返回{0, 0};
 * else (no)
 *   :初始化最大曲率为最小值;
 *   :初始化最小曲率为最大值;
 *   :遍历参考点列表;
 *   :检查参考点s值是否在指定范围内;
 *   if (参考点s值在指定范围内?) then (yes)
 *     :更新最大曲率;
 *     :更新最小曲率;
 *   endif
 *   :返回最大和最小曲率;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值范围内的最大和最小曲率，适用于需要根据曲率进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取曲率
 */
std::pair<double, double> ReferenceLine::getMaxAndMinCurvatureInRange(double start_s, double end_s) const {
  constexpr double kEps = 1e-2;
  if (end_s < start_s || start_s > accumulated_s_.back() || end_s < accumulated_s_.front()) {
    PERROR << "invalid input";
    return {0, 0};
  }
  // if (end_s > accumulated_s_.back()) {
  //   end_s = accumulated_s_.back();
  // }
  // if (start_s < accumulated_s_.front()) {
  //   start_s = accumulated_s_.front();
  // }
  double max_curvature = std::numeric_limits<double>::min();
  double min_curvature = std::numeric_limits<double>::max();
  for (const auto& p : reference_points_) {
    if (p.local_s() >= start_s && p.local_s() <= end_s) {
      if (p.kappa() > max_curvature) {
        max_curvature = p.kappa();
      }
      if (p.kappa() < min_curvature) {
        min_curvature = p.kappa();
      }
    }
    if (p.local_s() > end_s) {
      break;
    }
  }
  return {max_curvature, min_curvature};
}

/**
 * @brief 获取参考线上指定s值处的车道边界
 * @details 根据给定的s值，返回参考线上对应位置的车道左边界和右边界
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[out] lane_left_bound 存储车道左边界的指针
 * @param[out] lane_right_bound 存储车道右边界的指针
 * @return bool 返回true表示成功获取车道边界，否则返回false
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - lane_left_bound (double*): 存储车道左边界的指针
 * - lane_right_bound (double*): 存储车道右边界的指针
 * - ref_point (ReferencePoint): 参考线上对应s值的参考点
 *
 * @par 判断条件:
 * - 如果参考点列表为空，则返回false
 * - 如果成功获取参考点，则返回true
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考点列表是否为空;
 * if (参考点列表为空?) then (yes)
 *   :返回false;
 * else (no)
 *   :获取参考线上对应s值的参考点;
 *   :获取车道左边界和右边界;
 *   :返回true;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值处的车道边界，适用于需要根据车道边界进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取车道边界
 */
bool ReferenceLine::getLaneBound(const double s, double* const lane_left_bound, double* const lane_right_bound) const {
  auto ref_point = getReferencePoint(s);
  *lane_left_bound = ref_point.left_bound();
  *lane_right_bound = ref_point.right_bound();
  return true;
}
std::pair<float, float> ReferenceLine::getAverageLaneBoundFromRange(const float &start_s, const float &end_s,
                                                                    const float &interval) const {
  float average_left_bound = 0.0, average_right_bound = 0.0;
  float sum_left_bound = 0.0, sum_right_bound = 0.0;
  int count = 0;
  for (float s = start_s; s < end_s; s += interval) {
    sum_left_bound += getReferencePoint(s).left_bound();
    sum_right_bound += getReferencePoint(s).right_bound();
    count++;
  }
  average_left_bound = count == 0 ? 0 : sum_left_bound / count;
  average_right_bound = count == 0 ? 0 : sum_right_bound / count;
  return std::make_pair(average_left_bound, average_right_bound);
}

/**
 * @brief 获取参考线上指定s值处的道路边界
 * @details 根据给定的s值，返回参考线上对应位置的道路左边界和右边界
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[out] road_left_bound 存储道路左边界的指针
 * @param[out] road_right_bound 存储道路右边界的指针
 * @return bool 返回true表示成功获取道路边界，否则返回false
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - road_left_bound (double*): 存储道路左边界的指针
 * - road_right_bound (double*): 存储道路右边界的指针
 * - ref_point (ReferencePoint): 参考线上对应s值的参考点
 *
 * @par 判断条件:
 * - 如果参考点列表为空，则返回false
 * - 如果成功获取参考点，则返回true
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考点列表是否为空;
 * if (参考点列表为空?) then (yes)
 *   :返回false;
 * else (no)
 *   :获取参考线上对应s值的参考点;
 *   :获取道路左边界和右边界;
 *   :返回true;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值处的道路边界，适用于需要根据道路边界进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取道路边界
 */
bool ReferenceLine::getRoadBound(const double s, double* const road_left_bound, double* const road_right_bound) const {
  auto ref_point = getReferencePoint(s);
  *road_left_bound = ref_point.road_left_bound();
  *road_right_bound = ref_point.road_right_bound();
  return true;
}

std::pair<float, float> ReferenceLine::getAverageRoadBoundFromRange(const float &start_s, const float &end_s,
                                                                    const float &interval) const {
  float average_left_bound = 0.0, average_right_bound = 0.0;
  float sum_left_bound = 0.0, sum_right_bound = 0.0;
  int count = 0;
  for (float s = start_s; s < end_s; s += interval) {
    sum_left_bound += getReferencePoint(s).road_left_bound();
    sum_right_bound += getReferencePoint(s).road_right_bound();
    count++;
  }
  average_left_bound = count == 0 ? 0 : sum_left_bound / count;
  average_right_bound = count == 0 ? 0 : sum_right_bound / count;
  return std::make_pair(average_left_bound, average_right_bound);
}

/**
 * @brief 获取自车在参考线上的可行驶边界
 * @details 根据自车的SL边界，计算并返回自车在参考线上的可行驶边界，包括起始s值、左边界和右边界
 *
 * @param[in] sl_boundary 自车的SL边界，包含起始s值、结束s值、起始l值和结束l值
 * @return std::tuple<float, float, float> 返回可行驶边界，包含起始s值、左边界和右边界
 *
 * @par 关键变量说明:
 * - sl_boundary (gpal::pnc::SLBoundary): 自车的SL边界
 * - lane_left_bound (double): 车道左边界
 * - lane_right_bound (double): 车道右边界
 * - left_width (float): 自车左边界到车道左边界的宽度
 * - right_width (float): 自车右边界到车道右边界的宽度
 * - driving_bound (std::tuple<float, float, float>): 存储返回的可行驶边界
 *
 * @par 判断条件:
 * - 如果自车左边界到车道左边界的宽度大于0且大于等于右边界宽度，则返回左边界
 * - 如果自车右边界到车道右边界的宽度大于0且大于左边界宽度，则返回右边界
 * - 如果以上条件都不满足，则返回车道左右边界的中间值
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取自车SL边界的起始s值;
 * :获取车道左边界和右边界;
 * :计算自车左边界到车道左边界的宽度;
 * :计算自车右边界到车道右边界的宽度;
 * if (左边界宽度 > 0 且 >= 右边界宽度?) then (yes)
 *   :返回左边界;
 * else (no)
 *   if (右边界宽度 > 0 且 > 左边界宽度?) then (yes)
 *     :返回右边界;
 *   else (no)
 *     :返回车道左右边界的中间值;
 *   endif
 * endif
 * :返回可行驶边界;
 * stop
 * @enduml
 *
 * @note 该函数用于获取自车在参考线上的可行驶边界，适用于需要根据自车位置进行路径规划的场景
 *
 * @warning 需确保输入的SL边界在参考线范围内，否则可能无法正确获取可行驶边界
 */
std::tuple<float, float, float> ReferenceLine::getDrivingBound(const gpal::pnc::SLBoundary& sl_boundary) const {
  std::tuple<float, float, float> driving_bound{sl_boundary.start_s(), 0.0, 0.0};
  double lane_left_bound = 0.0;
  double lane_right_bound = 0.0;
  getLaneBound(sl_boundary.start_s(), &lane_left_bound, &lane_right_bound);

  float left_width = lane_left_bound - sl_boundary.end_l();
  float right_width = sl_boundary.start_l() - lane_right_bound;
  if (left_width > 0 && left_width >= right_width) {
    std::get<1>(driving_bound) = sl_boundary.end_l();
    std::get<2>(driving_bound) = lane_left_bound;
  } else if (right_width > 0 && right_width > left_width) {
    std::get<1>(driving_bound) = lane_right_bound;
    std::get<2>(driving_bound) = sl_boundary.start_l();
  } else {
    std::get<1>(driving_bound) = 0.5 * (lane_left_bound + lane_right_bound);
    std::get<2>(driving_bound) = 0.5 * (lane_left_bound + lane_right_bound);
  }
  return driving_bound;
}

/**
 * @brief 获取参考线上指定s值处的可行驶管道边界
 * @details 根据给定的s值，返回参考线上对应位置的可行驶管道左边界和右边界
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return std::pair<double, double> 返回可行驶管道边界，包含左边界和右边界
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - rpt (ReferencePoint): 参考线上对应s值的参考点
 *
 * @par 判断条件:
 * - 如果参考点列表为空，则返回默认边界{0, 0}
 * - 如果成功获取参考点，则返回可行驶管道边界
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考点列表是否为空;
 * if (参考点列表为空?) then (yes)
 *   :返回默认边界{0, 0};
 * else (no)
 *   :获取参考线上对应s值的参考点;
 *   :获取可行驶管道左边界和右边界;
 *   :返回可行驶管道边界;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值处的可行驶管道边界，适用于需要根据可行驶管道边界进行路径规划的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取可行驶管道边界
 */
std::pair<double, double> ReferenceLine::getDrivingTubeBound(const double s) const {
  auto rpt = getReferencePoint(s);
  return std::make_pair<double, double>(rpt.driving_tube_left_buffer(), rpt.driving_tube_right_buffer());
}

/**
 * @brief 判断SL点是否在车道范围内
 * @details 根据给定的SL点，判断其是否在当前参考线的车道范围内
 *
 * @param[in] sl_point 输入的SL点，包含s值和l值
 * @return bool 返回true表示SL点在车道范围内，否则返回false
 *
 * @par 关键变量说明:
 * - sl_point (SLPoint): 输入的SL点，包含s值和l值
 * - left_bound (double): 车道左边界
 * - right_bound (double): 车道右边界
 *
 * @par 判断条件:
 * - 如果SL点的s值不在参考线范围内（s <= 0 或 s > length_），则返回false
 * - 如果无法获取车道边界，则返回false
 * - 如果SL点的l值在车道左右边界之间，则返回true，否则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查SL点的s值是否在参考线范围内;
 * if (s值不在参考线范围内?) then (yes)
 *   :返回false;
 * else (no)
 *   :获取车道左边界和右边界;
 *   if (无法获取车道边界?) then (yes)
 *     :返回false;
 *   else (no)
 *     :检查SL点的l值是否在车道边界内;
 *     if (l值在车道边界内?) then (yes)
 *       :返回true;
 *     else (no)
 *       :返回false;
 *     endif
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于判断SL点是否在车道范围内，适用于需要根据SL点位置进行决策的场景
 *
 * @warning 需确保输入的SL点的s值在参考线范围内，否则可能无法正确判断
 */
bool ReferenceLine::isOnLane(const SLPoint& sl_point) const {
  if (sl_point.s() <= 0 || sl_point.s() > length_) {
    return false;
  }

  double left_bound = 0.0;
  double right_bound = 0.0;
  if (!getLaneBound(sl_point.s(), &left_bound, &right_bound)) {
    return false;
  }

  return sl_point.l() >= right_bound && sl_point.l() <= left_bound;
}

/**
 * @brief 判断笛卡尔坐标系下的点是否在车道范围内
 * @details 根据给定的笛卡尔坐标点，将其转换为SL坐标后，判断其是否在当前参考线的车道范围内
 *
 * @param[in] vec3d_point 输入的笛卡尔坐标点，包含x、y、z坐标
 * @return bool 返回true表示点在车道范围内，否则返回false
 *
 * @par 关键变量说明:
 * - vec3d_point (math::Vec3d): 输入的笛卡尔坐标点
 * - sl_point (SLPoint): 存储转换后的SL坐标
 *
 * @par 判断条件:
 * - 如果无法将笛卡尔坐标点转换为SL坐标，则返回false
 * - 如果转换后的SL点在车道范围内，则返回true，否则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :将笛卡尔坐标点转换为SL坐标;
 * if (转换成功?) then (yes)
 *   :判断SL点是否在车道范围内;
 *   if (SL点在车道范围内?) then (yes)
 *     :返回true;
 *   else (no)
 *     :返回false;
 *   endif
 * else (no)
 *   :返回false;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于判断笛卡尔坐标系下的点是否在车道范围内，适用于需要根据点位置进行决策的场景
 *
 * @warning 需确保输入的笛卡尔坐标点在参考线附近，否则可能无法正确判断
 */
bool ReferenceLine::isOnLane(const math::Vec3d& vec3d_point) const {
  SLPoint sl_point;
  if (!xy2sl(vec3d_point, &sl_point)) {
    return false;
  }
  return isOnLane(sl_point);
}

/**
 * @brief 判断SL边界是否在车道范围内
 * @details 根据给定的SL边界，判断其是否在当前参考线的车道范围内
 *
 * @param[in] sl_boundary 输入的SL边界，包含起始s值、结束s值、起始l值和结束l值
 * @return bool 返回true表示SL边界在车道范围内，否则返回false
 *
 * @par 关键变量说明:
 * - sl_boundary (gpal::pnc::SLBoundary): 输入的SL边界
 * - middle_s (double): SL边界的中间s值
 * - lane_left_bound (double): 车道左边界
 * - lane_right_bound (double): 车道右边界
 *
 * @par 判断条件:
 * - 如果SL边界的结束s值小于0或起始s值大于参考线长度，则返回false
 * - 计算SL边界的中间s值，并获取该位置的车道左右边界
 * - 如果SL边界的起始l值小于等于车道左边界且结束l值大于等于车道右边界，则返回true，否则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查SL边界的s值是否在参考线范围内;
 * if (s值不在参考线范围内?) then (yes)
 *   :返回false;
 * else (no)
 *   :计算SL边界的中间s值;
 *   :获取车道左边界和右边界;
 *   :检查SL边界的l值是否在车道边界内;
 *   if (l值在车道边界内?) then (yes)
 *     :返回true;
 *   else (no)
 *     :返回false;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于判断SL边界是否在车道范围内，适用于需要根据SL边界进行决策的场景
 *
 * @warning 需确保输入的SL边界的s值在参考线范围内，否则可能无法正确判断
 */
bool ReferenceLine::isOnLane(const gpal::pnc::SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double lane_left_bound = 0.0;
  double lane_right_bound = 0.0;
  getLaneBound(middle_s, &lane_left_bound, &lane_right_bound);
  return sl_boundary.start_l() <= lane_left_bound && sl_boundary.end_l() >= lane_right_bound;
}

/**
 * @brief 判断SL边界是否在车道范围内（带缓冲区）
 * @details 根据给定的SL边界，判断其是否在当前参考线的车道范围内，并考虑缓冲区的影响
 *
 * @param[in] sl_boundary 输入的SL边界，包含起始s值、结束s值、起始l值和结束l值
 * @param[out] lane_range 存储车道左右边界的范围
 * @param[in] buffer 缓冲区大小，用于扩展车道边界范围
 * @return bool 返回true表示SL边界在车道范围内，否则返回false
 *
 * @par 关键变量说明:
 * - sl_boundary (gpal::pnc::SLBoundary): 输入的SL边界
 * - lane_range (std::pair<double, double>): 存储车道左右边界的范围
 * - buffer (double): 缓冲区大小，用于扩展车道边界范围
 * - middle_s (double): SL边界的中间s值
 * - lane_left_bound (double): 车道左边界
 * - lane_right_bound (double): 车道右边界
 *
 * @par 判断条件:
 * - 如果SL边界的结束s值小于0或起始s值大于参考线长度，则返回false
 * - 计算SL边界的中间s值，并获取该位置的车道左右边界
 * - 如果SL边界的起始l值小于等于车道左边界加上缓冲区且结束l值大于等于车道右边界减去缓冲区，则返回true，否则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查SL边界的s值是否在参考线范围内;
 * if (s值不在参考线范围内?) then (yes)
 *   :返回false;
 * else (no)
 *   :计算SL边界的中间s值;
 *   :获取车道左边界和右边界;
 *   :存储车道边界范围;
 *   :检查SL边界的l值是否在车道边界内（考虑缓冲区）;
 *   if (l值在车道边界内?) then (yes)
 *     :返回true;
 *   else (no)
 *     :返回false;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于判断SL边界是否在车道范围内，适用于需要根据SL边界进行决策的场景，且考虑缓冲区的影响
 *
 * @warning 需确保输入的SL边界的s值在参考线范围内，否则可能无法正确判断
 */
bool ReferenceLine::isOnLane(const gpal::pnc::SLBoundary& sl_boundary, std::pair<double, double>& lane_range,
                             const double buffer) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double lane_left_bound = 0.0;
  double lane_right_bound = 0.0;
  getLaneBound(middle_s, &lane_left_bound, &lane_right_bound);
  lane_range.first = lane_right_bound;
  lane_range.second = lane_left_bound;
  return sl_boundary.start_l() <= lane_left_bound + buffer && sl_boundary.end_l() >= lane_right_bound - buffer;
}

/**
 * @brief 判断SL点是否在道路范围内
 * @details 根据给定的SL点，判断其是否在当前参考线的道路范围内
 *
 * @param[in] sl_point 输入的SL点，包含s值和l值
 * @return bool 返回true表示SL点在道路范围内，否则返回false
 *
 * @par 关键变量说明:
 * - sl_point (SLPoint): 输入的SL点，包含s值和l值
 * - road_left_bound (double): 道路左边界
 * - road_right_bound (double): 道路右边界
 *
 * @par 判断条件:
 * - 如果SL点的s值不在参考线范围内（s <= 0 或 s > length_），则返回false
 * - 如果无法获取道路边界，则返回false
 * - 如果SL点的l值在道路左右边界之间，则返回true，否则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查SL点的s值是否在参考线范围内;
 * if (s值不在参考线范围内?) then (yes)
 *   :返回false;
 * else (no)
 *   :获取道路左边界和右边界;
 *   if (无法获取道路边界?) then (yes)
 *     :返回false;
 *   else (no)
 *     :检查SL点的l值是否在道路边界内;
 *     if (l值在道路边界内?) then (yes)
 *       :返回true;
 *     else (no)
 *       :返回false;
 *     endif
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于判断SL点是否在道路范围内，适用于需要根据SL点位置进行决策的场景
 *
 * @warning 需确保输入的SL点的s值在参考线范围内，否则可能无法正确判断
 */
bool ReferenceLine::isOnRoad(const SLPoint& sl_point) const {
  if (sl_point.s() <= 0 || sl_point.s() > length_) {
    return false;
  }
  double road_left_bound = 0.0;
  double road_right_bound = 0.0;

  if (!getRoadBound(sl_point.s(), &road_left_bound, &road_right_bound)) {
    return false;
  }

  return sl_point.l() >= road_right_bound && sl_point.l() <= road_left_bound;
}

/**
 * @brief 判断笛卡尔坐标系下的点是否在道路范围内
 * @details 根据给定的笛卡尔坐标点，将其转换为SL坐标后，判断其是否在当前参考线的道路范围内
 *
 * @param[in] vec3d_point 输入的笛卡尔坐标点，包含x、y、z坐标
 * @return bool 返回true表示点在道路范围内，否则返回false
 *
 * @par 关键变量说明:
 * - vec3d_point (math::Vec3d): 输入的笛卡尔坐标点
 * - sl_point (SLPoint): 存储转换后的SL坐标
 *
 * @par 判断条件:
 * - 如果无法将笛卡尔坐标点转换为SL坐标，则返回false
 * - 如果转换后的SL点在道路范围内，则返回true，否则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :将笛卡尔坐标点转换为SL坐标;
 * if (转换成功?) then (yes)
 *   :判断SL点是否在道路范围内;
 *   if (SL点在道路范围内?) then (yes)
 *     :返回true;
 *   else (no)
 *     :返回false;
 *   endif
 * else (no)
 *   :返回false;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于判断笛卡尔坐标系下的点是否在道路范围内，适用于需要根据点位置进行决策的场景
 *
 * @warning 需确保输入的笛卡尔坐标点在参考线附近，否则可能无法正确判断
 */
bool ReferenceLine::isOnRoad(const math::Vec3d& vec3d_point) const {
  SLPoint sl_point;
  return xy2sl(vec3d_point, &sl_point) && isOnRoad(sl_point);
}

/**
 * @brief 判断SL边界是否在道路范围内
 * @details 根据给定的SL边界，判断其是否在当前参考线的道路范围内
 *
 * @param[in] sl_boundary 输入的SL边界，包含起始s值、结束s值、起始l值和结束l值
 * @return bool 返回true表示SL边界在道路范围内，否则返回false
 *
 * @par 关键变量说明:
 * - sl_boundary (gpal::pnc::SLBoundary): 输入的SL边界
 * - middle_s (double): SL边界的中间s值
 * - road_left_bound (double): 道路左边界
 * - road_right_bound (double): 道路右边界
 *
 * @par 判断条件:
 * - 如果SL边界的结束s值小于0或起始s值大于参考线长度，则返回false
 * - 计算SL边界的中间s值，并获取该位置的道路左右边界
 * - 如果SL边界的起始l值小于等于道路左边界且结束l值大于等于道路右边界，则返回true，否则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查SL边界的s值是否在参考线范围内;
 * if (s值不在参考线范围内?) then (yes)
 *   :返回false;
 * else (no)
 *   :计算SL边界的中间s值;
 *   :获取道路左边界和右边界;
 *   :检查SL边界的l值是否在道路边界内;
 *   if (l值在道路边界内?) then (yes)
 *     :返回true;
 *   else (no)
 *     :返回false;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于判断SL边界是否在道路范围内，适用于需要根据SL边界进行决策的场景
 *
 * @warning 需确保输入的SL边界的s值在参考线范围内，否则可能无法正确判断
 */
bool ReferenceLine::isOnRoad(const gpal::pnc::SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double road_left_bound = 0.0;
  double road_right_bound = 0.0;
  getRoadBound(middle_s, &road_left_bound, &road_right_bound);
  return sl_boundary.start_l() <= road_left_bound && sl_boundary.end_l() >= road_right_bound;
}

/**
 * @brief 判断矩形框是否与参考线重叠
 * @details 根据给定的矩形框，将其转换为SL边界后，判断其是否与当前参考线重叠
 *
 * @param[in] box 输入的矩形框，包含位置、大小和方向信息
 * @return bool 返回true表示矩形框与参考线重叠，否则返回false
 *
 * @par 关键变量说明:
 * - box (math::Box2d): 输入的矩形框
 * - sl_boundary (gpal::pnc::SLBoundary): 存储转换后的SL边界
 * - lane_left_bound (double): 车道左边界
 * - lane_right_bound (double): 车道右边界
 *
 * @par 判断条件:
 * - 如果无法将矩形框转换为SL边界，则返回false
 * - 如果SL边界的结束s值小于0或起始s值大于参考线长度，则返回false
 * - 如果SL边界的起始l值和结束l值符号相反，则返回true（表示矩形框跨越车道中心线）
 * - 计算SL边界的中间s值，并获取该位置的车道左右边界
 * - 如果SL边界的起始l值大于0且小于车道左边界，或者结束l值小于0且大于车道右边界，则返回true，否则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :将矩形框转换为SL边界;
 * if (转换成功?) then (yes)
 *   :检查SL边界的s值是否在参考线范围内;
 *   if (s值不在参考线范围内?) then (yes)
 *     :返回false;
 *   else (no)
 *     :检查SL边界的l值是否跨越车道中心线;
 *     if (l值跨越车道中心线?) then (yes)
 *       :返回true;
 *     else (no)
 *       :计算SL边界的中间s值;
 *       :获取车道左边界和右边界;
 *       :检查SL边界是否在车道边界内;
 *       if (SL边界在车道边界内?) then (yes)
 *         :返回true;
 *       else (no)
 *         :返回false;
 *       endif
 *     endif
 *   endif
 * else (no)
 *   :返回false;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于判断矩形框是否与参考线重叠，适用于需要根据障碍物位置进行决策的场景
 *
 * @warning 需确保输入的矩形框在参考线附近，否则可能无法正确判断
 */
bool ReferenceLine::hasOverlap(const math::Box2d& box) const {
  gpal::pnc::SLBoundary sl_boundary;
  if (!getSLBoundary(box, &sl_boundary)) {
    ERT_PLOG_I << "[ReferenceLine::hasOverlap]: Failed to get sl boundary for box: " << box.DebugString();
    return false;
  }
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > length()) {
    return false;
  }
  if (sl_boundary.start_l() * sl_boundary.end_l() < 0) {
    // 240805 lxy modify: origin: return false;
    return true;
  }

  double lane_left_bound = 0.0;
  double lane_right_bound = 0.0;
  const double mid_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  if (mid_s < 0 || mid_s > length()) {
    return false;
  }
  if (!getLaneBound(mid_s, &lane_left_bound, &lane_right_bound)) {
    ERT_PLOG_I << "[ReferenceLine::hasOverlap]: Failed to get bound at s = " << mid_s;
    return false;
  }
  if (sl_boundary.start_l() > 0) {
    return sl_boundary.start_l() < lane_left_bound;
  } else {
    return sl_boundary.end_l() > lane_right_bound;
  }
}

/**
 * @brief 判断SL边界是否与自车相关
 * @details 根据给定的SL边界，判断其是否与自车相关，并返回车道范围和自车范围
 *
 * @param[in] sl_boundary 输入的SL边界，包含起始s值、结束s值、起始l值和结束l值
 * @param[out] lane_range 存储车道左右边界的范围
 * @param[out] adc_range 存储自车左右边界的范围
 * @param[in] buffer 缓冲区大小，用于扩展边界范围
 * @return bool 返回true表示SL边界与自车相关，否则返回false
 *
 * @par 关键变量说明:
 * - sl_boundary (gpal::pnc::SLBoundary): 输入的SL边界
 * - lane_range (std::pair<double, double>): 存储车道左右边界的范围
 * - adc_range (std::pair<double, double>): 存储自车左右边界的范围
 * - buffer (double): 缓冲区大小，用于扩展边界范围
 * - middle_s (double): SL边界的中间s值
 * - rpt (ReferencePoint): 参考线上对应s值的参考点
 *
 * @par 判断条件:
 * - 如果SL边界的结束s值小于0或起始s值大于参考线长度，则返回false
 * - 计算SL边界的中间s值，并获取该位置的车道边界和自车边界
 * -
 * 如果SL边界的起始l值小于等于车道左边界或自车左边界加上缓冲区，且结束l值大于等于车道右边界或自车右边界减去缓冲区，则返回true，否则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查SL边界的s值是否在参考线范围内;
 * if (s值不在参考线范围内?) then (yes)
 *   :返回false;
 * else (no)
 *   :计算SL边界的中间s值;
 *   :获取车道边界和自车边界;
 *   :检查SL边界的l值是否在车道或自车边界内（考虑缓冲区）;
 *   if (l值在车道或自车边界内?) then (yes)
 *     :返回true;
 *   else (no)
 *     :返回false;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于判断SL边界是否与自车相关，适用于需要根据SL边界进行决策的场景，且考虑缓冲区的影响
 *
 * @warning 需确保输入的SL边界的s值在参考线范围内，否则可能无法正确判断
 */
bool ReferenceLine::isRelavent(const gpal::pnc::SLBoundary& sl_boundary, std::pair<double, double>& lane_range,
                               std::pair<double, double>& adc_range, const double buffer) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  auto rpt = getReferencePoint(middle_s);
  lane_range.first = rpt.right_bound();
  lane_range.second = rpt.left_bound();
  adc_range.first = rpt.driving_tube_right_buffer();
  adc_range.second = rpt.driving_tube_left_buffer();
  return sl_boundary.start_l() <= std::max(lane_range.second, adc_range.second) + buffer &&
         sl_boundary.end_l() >= std::min(lane_range.first, adc_range.first) - buffer;
}

/**
 * @brief 根据给定的s值获取速度限制
 * @details 根据输入的s值，查找参考线上对应位置的速度限制，并考虑减速需求
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return double 返回速度限制值，单位为m/s
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - speed_limits_ (std::vector<SpeedLimit>): 存储速度限制的数组
 * - acc (float): 加速度，用于计算减速需求
 *
 * @par 判断条件:
 * - 如果s值在某个速度限制的s范围内，则返回该速度限制的最大值
 * - 如果当前速度限制与下一个速度限制的差值大于1.0，则需要减速
 * - 根据减速距离和加速度计算新的速度限制
 * - 如果s值不在任何速度限制的s范围内，则返回默认速度限制130 km/h
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历速度限制数组;
 * if (s值在某个速度限制的s范围内?) then (yes)
 *   if (存在下一个速度限制?) then (yes)
 *     if (当前速度限制与下一个速度限制的差值 > 1.0?) then (yes)
 *       :计算减速距离;
 *       :根据减速距离和加速度计算新的速度限制;
 *       if (新的速度限制 >= 当前速度限制?) then (yes)
 *         :返回当前速度限制;
 *       else (no)
 *         :返回新的速度限制;
 *       endif
 *     else (no)
 *       :返回当前速度限制;
 *     endif
 *   else (no)
 *     :返回当前速度限制;
 *   endif
 * else (no)
 *   :返回默认速度限制130 km/h;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取参考线上指定s值处的速度限制，适用于需要根据速度限制进行路径规划的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取速度限制
 */
double ReferenceLine::GetSpeedLimitFromS(const double s) const {
  constexpr float acc = 1.0f;
  for (int i = 0; i < speed_limits_.size(); ++i) {
    if (s >= speed_limits_.at(i).start_s && s <= speed_limits_.at(i).end_s) {
      if (i + 1 < speed_limits_.size()) {
        if (speed_limits_.at(i).max_speed_limit - speed_limits_.at(i + 1).max_speed_limit > 1.0) {
          // need decelerate
          float max_speed_limit = speed_limits_.at(i).max_speed_limit;
          float min_speed_limit = speed_limits_.at(i + 1).max_speed_limit;
          float dis = speed_limits_.at(i + 1).start_s - s;
          float speed_limit = std::sqrt(min_speed_limit * min_speed_limit + 2.0 * acc * dis);
          if (speed_limit >= max_speed_limit) {
            speed_limit = max_speed_limit;
          }
          return speed_limit;
        } else {
          return speed_limits_.at(i).max_speed_limit;
        }
      } else {
        return speed_limits_.at(i).max_speed_limit;
      }
    }
  }

  double speed_limit = kMaxSpeedMS;
  return speed_limit;
}

/**
 * @brief 添加速度限制
 * @details 将传入的速度限制对象添加到参考线的速度限制列表中
 *
 * @param[in] speed_limit 速度限制对象，包含起始s值、结束s值、最大速度限制、最小速度限制、起始点和结束点
 *
 * @par 关键变量说明:
 * - speed_limit (SpeedLimit): 速度限制对象，包含以下成员：
 *   - start_s (double): 速度限制的起始s值
 *   - end_s (double): 速度限制的结束s值
 *   - max_speed_limit (double): 最大速度限制
 *   - min_speed_limit (double): 最小速度限制
 *   - start_point (math::Vec3d): 速度限制的起始点
 *   - end_point (math::Vec3d): 速度限制的结束点
 *
 * @par 处理逻辑:
 * - 调用AddSpeedLimit(double start_s, double end_s, double speed_limit, math::Vec3d start_point, math::Vec3d
 * end_point)函数， 将速度限制对象的各个成员作为参数传入
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用AddSpeedLimit函数;
 * :传入速度限制对象的各个成员;
 * stop
 * @enduml
 *
 * @note 该函数用于将速度限制对象添加到参考线的速度限制列表中，适用于需要根据速度限制进行路径规划的场景
 *
 * @warning 需确保传入的速度限制对象的s值在参考线范围内，否则可能无法正确添加
 */
void ReferenceLine::AddSpeedLimit(SpeedLimit speed_limit) {
  AddSpeedLimit(speed_limit.start_s, speed_limit.end_s, speed_limit.max_speed_limit, speed_limit.start_point,
                speed_limit.end_point);
}

/**
 * @brief 添加速度限制
 * @details 根据给定的s值、速度限制和点，将速度限制添加到参考线的速度限制列表中
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] speed_limit 速度限制值，单位为m/s
 * @param[in] point 速度限制的起始点，包含x、y、z坐标
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - speed_limit (double): 速度限制值，单位为m/s
 * - point (math::Vec3d): 速度限制的起始点，包含x、y、z坐标
 *
 * @par 处理逻辑:
 * - 调用AddSpeedLimit(double start_s, double end_s, double speed_limit, math::Vec3d start_point, math::Vec3d
 * end_point)函数， 将s值、s + 1.0、speed_limit、point和point作为参数传入
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用AddSpeedLimit函数;
 * :传入s值、s + 1.0、speed_limit、point和point;
 * stop
 * @enduml
 *
 * @note 该函数用于将速度限制添加到参考线的速度限制列表中，适用于需要根据速度限制进行路径规划的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确添加
 */
void ReferenceLine::AddSpeedLimit(double s, double speed_limit, math::Vec3d point) {
  AddSpeedLimit(s, s + 1.0, speed_limit, point, point);
}

/**
 * @brief 添加速度限制
 * @details 根据给定的起始s值、结束s值、速度限制、起始点和结束点，将速度限制添加到参考线的速度限制列表中
 *
 * @param[in] start_s 速度限制的起始s值，范围[0, length_]
 * @param[in] end_s 速度限制的结束s值，范围[0, length_]
 * @param[in] speed_limit 速度限制值，单位为m/s
 * @param[in] start_point 速度限制的起始点，包含x、y、z坐标
 * @param[in] end_point 速度限制的结束点，包含x、y、z坐标
 *
 * @par 关键变量说明:
 * - start_s (double): 速度限制的起始s值，范围[0, length_]
 * - end_s (double): 速度限制的结束s值，范围[0, length_]
 * - speed_limit (double): 速度限制值，单位为m/s
 * - start_point (math::Vec3d): 速度限制的起始点，包含x、y、z坐标
 * - end_point (math::Vec3d): 速度限制的结束点，包含x、y、z坐标
 * - new_speed_limit (std::vector<SpeedLimit>): 存储新的速度限制列表
 *
 * @par 处理逻辑:
 * - 如果起始s值小于0，则将其设置为0，并将起始点设置为参考线的第一个点
 * - 如果起始s值大于参考线长度，则将其设置为参考线长度，并将起始点设置为参考线的最后一个点
 * - 如果结束s值小于0，则将其设置为0，并将结束点设置为参考线的第一个点
 * - 如果结束s值大于参考线长度，则将其设置为参考线长度，并将结束点设置为参考线的最后一个点
 * - 遍历现有的速度限制列表，根据起始s值和结束s值与现有速度限制的关系，将新的速度限制插入到合适的位置
 * - 清空现有的速度限制列表，并将新的速度限制列表赋值给现有的速度限制列表
 * - 对速度限制列表进行排序，确保速度限制按照起始s值、结束s值和最大速度限制的顺序排列
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查并修正起始s值和结束s值;
 * :遍历现有的速度限制列表;
 * if (起始s值 >= 现有速度限制的结束s值 或 结束s值 <= 现有速度限制的起始s值?) then (yes)
 *   :将现有速度限制添加到新的速度限制列表;
 * else (no)
 *   :计算新的速度限制与现有速度限制的交集;
 *   :将交集部分的速度限制添加到新的速度限制列表;
 * endif
 * :清空现有的速度限制列表;
 * :将新的速度限制列表赋值给现有的速度限制列表;
 * :对速度限制列表进行排序;
 * stop
 * @enduml
 *
 * @note 该函数用于将速度限制添加到参考线的速度限制列表中，适用于需要根据速度限制进行路径规划的场景
 *
 * @warning 需确保输入的起始s值和结束s值在参考线范围内，否则可能无法正确添加
 */
void ReferenceLine::AddSpeedLimit(double start_s, double end_s, double speed_limit, math::Vec3d start_point,
                                  math::Vec3d end_point) {
  if (start_s < 0) {
    start_s = 0;
    start_point = reference_points_.front();
  }
  if (start_s > length_) {
    start_s = length_;
    start_point = reference_points_.back();
  }
  if (end_s < 0) {
    end_s = 0;
    end_point = reference_points_.front();
  }
  if (end_s > length_) {
    end_s = length_;
    end_point = reference_points_.back();
  }

  std::vector<SpeedLimit> new_speed_limit;
  for (const auto& limit : speed_limits_) {
    if (start_s >= limit.end_s || end_s <= limit.start_s) {
      new_speed_limit.emplace_back(limit);
    } else {
      double min_speed = std::fmin(limit.max_speed_limit, speed_limit);
      if (start_s >= limit.start_s) {
        new_speed_limit.emplace_back(limit.start_s, start_s, limit.max_speed_limit, limit.max_speed_limit,
                                     limit.start_point, start_point);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(start_s, end_s, min_speed, min_speed, start_point, end_point);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.max_speed_limit, limit.max_speed_limit, end_point,
                                       limit.end_point);
        } else {
          new_speed_limit.emplace_back(start_s, limit.end_s, min_speed, min_speed, start_point, limit.end_point);
        }
      } else {
        new_speed_limit.emplace_back(start_s, limit.start_s, speed_limit, speed_limit, start_point, limit.start_point);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(limit.start_s, end_s, min_speed, min_speed, limit.start_point, end_point);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.max_speed_limit, limit.max_speed_limit, end_point,
                                       limit.end_point);
        } else {
          new_speed_limit.emplace_back(limit.start_s, limit.end_s, min_speed, min_speed, limit.start_point,
                                       limit.end_point);
        }
      }
      start_s = limit.end_s;
      start_point = limit.end_point;
      if (end_s <= start_s) {
        end_s = start_s;
        end_point = start_point;
      }
    }
  }

  speed_limits_.clear();
  for (const auto& limit : new_speed_limit) {
    if (limit.start_s < limit.end_s) {
      speed_limits_.emplace_back(limit);
    }
  }
  std::sort(speed_limits_.begin(), speed_limits_.end(), [](const SpeedLimit& a, const SpeedLimit& b) {
    if (a.start_s != b.start_s) {
      return a.start_s < b.start_s;
    }
    if (a.end_s != b.end_s) {
      return a.end_s < b.end_s;
    }
    return a.max_speed_limit < b.max_speed_limit;
  });
}

/**
 * @brief 获取记忆路线中的速度限制
 * @details 根据给定的原点坐标，将参考线上的速度限制转换为全局坐标系下的速度限制
 *
 * @param[in] origin 原点坐标，用于将局部坐标系下的速度限制转换为全局坐标系
 * @return std::vector<SpeedLimit> 返回全局坐标系下的速度限制列表
 *
 * @par 关键变量说明:
 * - origin (math::Vec3d): 原点坐标，包含x、y、z坐标
 * - res (std::vector<SpeedLimit>): 存储转换后的速度限制列表
 * - pt (SpeedLimit): 临时存储单个速度限制对象
 *
 * @par 处理逻辑:
 * - 遍历参考线的速度限制列表
 * - 将每个速度限制的起始点和结束点的坐标加上原点坐标，转换为全局坐标系
 * - 将转换后的速度限制添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历速度限制列表;
 * :将起始点和结束点的坐标加上原点坐标;
 * :将转换后的速度限制添加到结果列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取全局坐标系下的速度限制，适用于需要根据记忆路线进行路径规划的场景
 *
 * @warning 需确保传入的原点坐标正确，否则可能无法正确转换坐标系
 */
std::vector<SpeedLimit> ReferenceLine::getMemorizedRouteSpeedLimits(const math::Vec3d& origin) const {
  std::vector<SpeedLimit> res;
  SpeedLimit pt;
  for (int i = 0; i < speed_limits_.size(); ++i) {
    pt.start_s = speed_limits_.at(i).start_s;
    pt.end_s = speed_limits_.at(i).end_s;
    pt.max_speed_limit = speed_limits_.at(i).max_speed_limit;
    pt.min_speed_limit = speed_limits_.at(i).min_speed_limit;
    pt.recommended_speed = speed_limits_.at(i).recommended_speed;
    pt.start_point.set_x(speed_limits_.at(i).start_point.x() + origin.x());
    pt.start_point.set_y(speed_limits_.at(i).start_point.y() + origin.y());
    pt.start_point.set_z(speed_limits_.at(i).start_point.z());
    pt.end_point.set_x(speed_limits_.at(i).end_point.x() + origin.x());
    pt.end_point.set_y(speed_limits_.at(i).end_point.y() + origin.y());
    pt.end_point.set_z(speed_limits_.at(i).end_point.z());

    res.emplace_back(pt);
  }
  return res;
}

/**
 * @brief 获取指定s值范围内的停止线
 * @details 根据给定的s值、前向距离阈值和后向距离阈值，返回参考线上该范围内的所有停止线
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_distance_thrd 前向距离阈值，用于确定s值范围的上限
 * @param[in] backward_dist_thrd 后向距离阈值，用于确定s值范围的下限
 * @return std::vector<StopLine> 返回符合条件的停止线列表
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - forward_distance_thrd (double): 前向距离阈值，用于确定s值范围的上限
 * - backward_dist_thrd (double): 后向距离阈值，用于确定s值范围的下限
 * - res (std::vector<StopLine>): 存储符合条件的停止线列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的停止线列表
 * - 如果停止线的s值小于s - backward_dist_thrd，则跳过该停止线
 * - 如果停止线的s值大于s + forward_distance_thrd，则停止遍历
 * - 将符合条件的停止线添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历停止线列表;
 * if (停止线的s值 < s - backward_dist_thrd?) then (yes)
 *   :跳过该停止线;
 * else if (停止线的s值 > s + forward_distance_thrd?) then (yes)
 *   :停止遍历;
 * else (no)
 *   :将停止线添加到结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的停止线，适用于需要根据停止线位置进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取停止线
 */
std::vector<StopLine> ReferenceLine::getStopLinesFromSRange(const double& s, const double forward_distance_thrd,
                                                            const double backward_dist_thrd) const {
  std::vector<StopLine> res;
  for (auto& pt : stop_lines_) {
    if (pt.s < s - backward_dist_thrd) {
      continue;
    } else if (pt.s > s + forward_distance_thrd) {
      break;
    }
    res.emplace_back(pt);
  }
  return res;
}

/**
 * @brief 获取记忆路线中的停止线
 * @details 根据给定的原点坐标，将参考线上的停止线转换为全局坐标系下的停止线
 *
 * @param[in] origin 原点坐标，用于将局部坐标系下的停止线转换为全局坐标系
 * @return std::vector<StopLine> 返回全局坐标系下的停止线列表
 *
 * @par 关键变量说明:
 * - origin (math::Vec3d): 原点坐标，包含x、y、z坐标
 * - res (std::vector<StopLine>): 存储转换后的停止线列表
 * - pt (StopLine): 临时存储单个停止线对象
 *
 * @par 处理逻辑:
 * - 遍历参考线的停止线列表
 * - 将每个停止线的坐标加上原点坐标，转换为全局坐标系
 * - 将转换后的停止线添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历停止线列表;
 * :将停止线的坐标加上原点坐标;
 * :将转换后的停止线添加到结果列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取全局坐标系下的停止线，适用于需要根据记忆路线进行路径规划的场景
 *
 * @warning 需确保传入的原点坐标正确，否则可能无法正确转换坐标系
 */
std::vector<StopLine> ReferenceLine::getMemorizedRouteStopLines(const math::Vec3d& origin) const {
  std::vector<StopLine> res;
  StopLine pt;
  for (int i = 0; i < stop_lines_.size(); ++i) {
    pt.is_exist = stop_lines_.at(i).is_exist;
    pt.id = stop_lines_.at(i).id;
    pt.is_virtual = stop_lines_.at(i).is_virtual;
    pt.type = stop_lines_.at(i).type;
    pt.x = stop_lines_.at(i).x + origin.x();
    pt.y = stop_lines_.at(i).y + origin.y();
    pt.z = stop_lines_.at(i).z;
    pt.s = stop_lines_.at(i).s;
    pt.traffic_light_id = stop_lines_.at(i).traffic_light_id;
    pt.direction = stop_lines_.at(i).direction;
    res.emplace_back(pt);
  }
  return res;
}

/**
 * @brief 获取记忆路线中的停止线
 * @details
 * 根据给定的原点坐标、s值、前向距离阈值和后向距离阈值，返回参考线上该范围内的所有停止线，并将其转换为全局坐标系
 *
 * @param[in] origin 原点坐标，用于将局部坐标系下的停止线转换为全局坐标系
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_distance_thrd 前向距离阈值，用于确定s值范围的上限
 * @param[in] backward_dist_thrd 后向距离阈值，用于确定s值范围的下限
 * @return std::vector<StopLine> 返回全局坐标系下的停止线列表
 *
 * @par 关键变量说明:
 * - origin (math::Vec3d): 原点坐标，包含x、y、z坐标
 * - s (double): 参考线上的s值，范围[0, length_]
 * - forward_distance_thrd (double): 前向距离阈值，用于确定s值范围的上限
 * - backward_dist_thrd (double): 后向距离阈值，用于确定s值范围的下限
 * - res (std::vector<StopLine>): 存储符合条件的停止线列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的停止线列表
 * - 如果停止线的s值小于s - backward_dist_thrd，则跳过该停止线
 * - 如果停止线的s值大于s + forward_distance_thrd，则停止遍历
 * - 将符合条件的停止线添加到结果列表中
 * - 遍历结果列表，将每个停止线的坐标加上原点坐标，转换为全局坐标系
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历停止线列表;
 * if (停止线的s值 < s - backward_dist_thrd?) then (yes)
 *   :跳过该停止线;
 * else if (停止线的s值 > s + forward_distance_thrd?) then (yes)
 *   :停止遍历;
 * else (no)
 *   :将停止线添加到结果列表;
 * endif
 * :遍历结果列表;
 * :将停止线的坐标加上原点坐标;
 * stop
 * @enduml
 *
 * @note 该函数用于获取全局坐标系下的停止线，适用于需要根据记忆路线进行路径规划的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取停止线
 */
std::vector<StopLine> ReferenceLine::getMemorizedRouteStopLines(const math::Vec3d& origin, const double s,
                                                                const double forward_distance_thrd,
                                                                const double backward_dist_thrd) const {
  std::vector<StopLine> res;
  for (auto& pt : stop_lines_) {
    if (pt.s < s - backward_dist_thrd) {
      continue;
    } else if (pt.s > s + forward_distance_thrd) {
      break;
    }
    res.emplace_back(pt);
  }
  for (int i = 0; i < res.size(); ++i) {
    res.at(i).x = res.at(i).x + origin.x();
    res.at(i).y = res.at(i).y + origin.y();
  }
  return res;
}

/**
 * @brief 获取指定s值范围内的闸门
 * @details 根据给定的s值、前向距离阈值和后向距离阈值，返回参考线上该范围内的所有闸门
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_distance_thrd 前向距离阈值，用于确定s值范围的上限
 * @param[in] backward_dist_thrd 后向距离阈值，用于确定s值范围的下限
 * @return std::vector<Gate> 返回符合条件的闸门列表
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - forward_distance_thrd (double): 前向距离阈值，用于确定s值范围的上限
 * - backward_dist_thrd (double): 后向距离阈值，用于确定s值范围的下限
 * - res (std::vector<Gate>): 存储符合条件的闸门列表
 *
 * @par 处理逻辑:
 * - 调用getGatesFromSRange(s - backward_dist_thrd, s + forward_distance_thrd)函数，
 *   将s - backward_dist_thrd和s + forward_distance_thrd作为参数传入
 * - 返回指定s值范围内的闸门列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用getGatesFromSRange函数;
 * :传入s - backward_dist_thrd和s + forward_distance_thrd;
 * :返回闸门列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的闸门，适用于需要根据闸门位置进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取闸门
 */
std::vector<Gate> ReferenceLine::getGatesFromSRange(const double s, const double forward_distance_thrd,
                                                    const double backward_dist_thrd) const {
  return getGatesFromSRange(s - backward_dist_thrd, s + forward_distance_thrd);
}

/**
 * @brief 获取指定s值范围内的闸门
 * @details 根据给定的起始s值和结束s值，返回参考线上该范围内的所有闸门
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::vector<Gate> 返回符合条件的闸门列表
 *
 * @par 关键变量说明:
 * - start_s (float): 起始s值，范围[0, length_]
 * - end_s (float): 结束s值，范围[0, length_]
 * - res (std::vector<Gate>): 存储符合条件的闸门列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的闸门列表
 * - 如果闸门的s值小于起始s值，则跳过该闸门
 * - 如果闸门的s值大于结束s值，则停止遍历
 * - 将符合条件的闸门添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历闸门列表;
 * if (闸门的s值 < 起始s值?) then (yes)
 *   :跳过该闸门;
 * else if (闸门的s值 > 结束s值?) then (yes)
 *   :停止遍历;
 * else (no)
 *   :将闸门添加到结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的闸门，适用于需要根据闸门位置进行决策的场景
 *
 * @warning 需确保输入的起始s值和结束s值在参考线范围内，否则可能无法正确获取闸门
 */
std::vector<Gate> ReferenceLine::getGatesFromSRange(const float& start_s, const float& end_s) const {
  std::vector<Gate> res;
  for (auto& pt : gates_) {
    if (pt.s < start_s) {
      continue;
    } else if (pt.s > end_s) {
      break;
    }
    res.emplace_back(pt);
  }
  return res;
}

/**
 * @brief 获取指定s值范围内的方向段
 * @details 根据给定的s值、前向距离阈值和后向距离阈值，返回参考线上该范围内的所有方向段
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_distance_thrd 前向距离阈值，用于确定s值范围的上限
 * @param[in] backward_dist_thrd 后向距离阈值，用于确定s值范围的下限
 * @return std::vector<SegmentDirection> 返回符合条件的方向段列表
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - forward_distance_thrd (double): 前向距离阈值，用于确定s值范围的上限
 * - backward_dist_thrd (double): 后向距离阈值，用于确定s值范围的下限
 * - res (std::vector<SegmentDirection>): 存储符合条件的方向段列表
 *
 * @par 处理逻辑:
 * - 调用getDirectionsFromSRange(s - backward_dist_thrd, s + forward_distance_thrd)函数，
 *   将s - backward_dist_thrd和s + forward_distance_thrd作为参数传入
 * - 返回指定s值范围内的方向段列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用getDirectionsFromSRange函数;
 * :传入s - backward_dist_thrd和s + forward_distance_thrd;
 * :返回方向段列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的方向段，适用于需要根据方向段进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取方向段
 */
std::vector<SegmentDirection> ReferenceLine::getDirectionsFromSRange(const double s, const double forward_distance_thrd,
                                                                     const double backward_dist_thrd) const {
  return getDirectionsFromSRange(s - backward_dist_thrd, s + forward_distance_thrd);
}

/**
 * @brief 获取指定s值范围内的方向段
 * @details 根据给定的起始s值和结束s值，返回参考线上该范围内的所有方向段
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::vector<SegmentDirection> 返回符合条件的方向段列表
 *
 * @par 关键变量说明:
 * - start_s (float): 起始s值，范围[0, length_]
 * - end_s (float): 结束s值，范围[0, length_]
 * - res (std::vector<SegmentDirection>): 存储符合条件的方向段列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的方向段列表
 * - 如果方向段的结束s值小于起始s值，则跳过该方向段
 * - 如果方向段的起始s值大于结束s值，则停止遍历
 * - 将符合条件的方向段添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历方向段列表;
 * if (方向段的结束s值 < 起始s值?) then (yes)
 *   :跳过该方向段;
 * else if (方向段的起始s值 > 结束s值?) then (yes)
 *   :停止遍历;
 * else (no)
 *   :将方向段添加到结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的方向段，适用于需要根据方向段进行决策的场景
 *
 * @warning 需确保输入的起始s值和结束s值在参考线范围内，否则可能无法正确获取方向段
 */
std::vector<SegmentDirection> ReferenceLine::getDirectionsFromSRange(const float& start_s, const float& end_s) const {
  std::vector<SegmentDirection> res;
  for (auto& direction : directions_) {
    if (direction.end_s < start_s) {
      continue;
    } else if (direction.start_s > end_s) {
      break;
    }
    res.emplace_back(direction);
  }
  return res;
}

/**
 * @brief 获取指定s值处的方向段
 * @details 根据给定的s值，返回参考线上该s值所在的方向段
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return SegmentDirection 返回包含该s值的方向段
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 *
 * @par 处理逻辑:
 * - 遍历参考线的方向段列表
 * - 如果s值在某个方向段的起始s值和结束s值之间，则返回该方向段
 * - 如果遍历结束后仍未找到匹配的方向段，则返回一个默认构造的SegmentDirection对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历方向段列表;
 * if (s值 >= 方向段的起始s值 且 s值 <= 方向段的结束s值?) then (yes)
 *   :返回该方向段;
 * else (no)
 *   :继续遍历;
 * endif
 * :返回默认的SegmentDirection对象;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值处的方向段，适用于需要根据方向段进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取方向段
 */
SegmentDirection ReferenceLine::getDirectionFromS(const double s) const {
  for (int i = 0; i < directions_.size(); ++i) {
    if (s >= directions_.at(i).start_s && s <= directions_.at(i).end_s) {
      return directions_.at(i);
    }
  }

  return SegmentDirection();
}

/**
 * @brief 获取指定行驶方向的方向段范围
 * @details 根据给定的行驶方向，返回参考线上所有符合该方向的方向段范围
 *
 * @param[in] direction 行驶方向，包含正向、逆向和双向
 * @return std::vector<std::pair<float, float>> 返回符合条件的方向段范围列表，每个范围包含起始s值和结束s值
 *
 * @par 关键变量说明:
 * - direction (DrivingDirection): 行驶方向，包含以下枚举值：
 *   - kStartToEnd: 正向
 *   - kEndToStart: 逆向
 *   - kBothDirection: 双向
 * - res (std::vector<std::pair<float, float>>): 存储符合条件的方向段范围列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的方向段列表
 * - 如果方向段的行驶方向与传入的行驶方向匹配，则将该方向段的起始s值和结束s值添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历方向段列表;
 * if (方向段的行驶方向 == 传入的行驶方向?) then (yes)
 *   :将方向段的起始s值和结束s值添加到结果列表;
 * else (no)
 *   :继续遍历;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定行驶方向的方向段范围，适用于需要根据行驶方向进行路径规划的场景
 *
 * @warning 需确保传入的行驶方向有效，否则可能无法正确获取方向段范围
 */
std::vector<std::pair<float, float>> ReferenceLine::getSpecifiedDirectionRanges(
    const DrivingDirection& direction) const {
  std::vector<std::pair<float, float>> res;
  for (int i = 0; i < directions_.size(); ++i) {
    if (directions_.at(i).direction == direction) {
      res.emplace_back(directions_.at(i).start_s, directions_.at(i).end_s);
    }
  }

  return res;
}

/**
 * @brief 获取记忆路线中的方向段
 * @details 根据给定的原点坐标，将参考线上的方向段转换为全局坐标系下的方向段
 *
 * @param[in] origin 原点坐标，用于将局部坐标系下的方向段转换为全局坐标系
 * @return std::vector<SegmentDirection> 返回全局坐标系下的方向段列表
 *
 * @par 关键变量说明:
 * - origin (math::Vec3d): 原点坐标，包含x、y、z坐标
 * - res (std::vector<SegmentDirection>): 存储转换后的方向段列表
 * - pt (SegmentDirection): 临时存储单个方向段对象
 *
 * @par 处理逻辑:
 * - 遍历参考线的方向段列表
 * - 将每个方向段的起始点和结束点的坐标加上原点坐标，转换为全局坐标系
 * - 将转换后的方向段添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历方向段列表;
 * :将起始点和结束点的坐标加上原点坐标;
 * :将转换后的方向段添加到结果列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取全局坐标系下的方向段，适用于需要根据记忆路线进行路径规划的场景
 *
 * @warning 需确保传入的原点坐标正确，否则可能无法正确转换坐标系
 */
std::vector<SegmentDirection> ReferenceLine::getMemorizedRouteSegmentsDirection(const math::Vec3d& origin) const {
  std::vector<SegmentDirection> res;
  SegmentDirection pt;
  for (int i = 0; i < directions_.size(); ++i) {
    pt.start_s = directions_.at(i).start_s;
    pt.end_s = directions_.at(i).end_s;
    pt.direction = directions_.at(i).direction;
    pt.start_point.set_x(directions_.at(i).start_point.x() + origin.x());
    pt.start_point.set_y(directions_.at(i).start_point.y() + origin.y());
    pt.start_point.set_z(directions_.at(i).start_point.z());
    pt.end_point.set_x(directions_.at(i).end_point.x() + origin.x());
    pt.end_point.set_y(directions_.at(i).end_point.y() + origin.y());
    pt.end_point.set_z(directions_.at(i).end_point.z());

    res.emplace_back(pt);
  }
  return res;
}

/**
 * @brief 获取指定s值范围内的来源信息
 * @details 根据给定的起始s值和结束s值，返回参考线上该范围内的所有来源信息
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::vector<std::tuple<LineSourceType, float, float>>
 * 返回符合条件的来源信息列表，每个元素包含来源类型、起始s值和结束s值
 *
 * @par 关键变量说明:
 * - start_s (float): 起始s值，范围[0, length_]
 * - end_s (float): 结束s值，范围[0, length_]
 * - res (std::vector<std::tuple<LineSourceType, float, float>>): 存储符合条件的来源信息列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的来源信息列表
 * - 如果来源信息的结束s值小于起始s值，则跳过该来源信息
 * - 如果来源信息的起始s值大于结束s值，则停止遍历
 * - 将符合条件的来源信息添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历来源信息列表;
 * if (来源信息的结束s值 < 起始s值?) then (yes)
 *   :跳过该来源信息;
 * else if (来源信息的起始s值 > 结束s值?) then (yes)
 *   :停止遍历;
 * else (no)
 *   :将来源信息添加到结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的来源信息，适用于需要根据来源信息进行决策的场景
 *
 * @warning 需确保输入的起始s值和结束s值在参考线范围内，否则可能无法正确获取来源信息
 */
std::vector<std::tuple<LineSourceType, float, float>> ReferenceLine::getSourceInfosFromSRange(
    const float& start_s, const float& end_s) const {
  std::vector<std::tuple<LineSourceType, float, float>> res;
  for (const auto& source_info : source_infos_) {
    if (std::get<2>(source_info) < start_s) {
      continue;
    } else if (std::get<1>(source_info) > end_s) {
      break;
    }
    res.emplace_back(source_info);
  }
  return res;
}

/**
 * @brief 获取指定s值范围内的来源信息
 * @details 根据给定的s值、前向距离阈值和后向距离阈值，返回参考线上该范围内的所有来源信息
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_distance_thrd 前向距离阈值，用于确定s值范围的上限
 * @param[in] backward_dist_thrd 后向距离阈值，用于确定s值范围的下限
 * @return std::vector<std::tuple<LineSourceType, float, float>>
 * 返回符合条件的来源信息列表，每个元素包含来源类型、起始s值和结束s值
 *
 * @par 关键变量说明:
 * - s (float): 参考线上的s值，范围[0, length_]
 * - forward_distance_thrd (float): 前向距离阈值，用于确定s值范围的上限
 * - backward_dist_thrd (float): 后向距离阈值，用于确定s值范围的下限
 *
 * @par 处理逻辑:
 * - 调用getSourceInfosFromSRange(s - backward_dist_thrd, s + forward_distance_thrd)函数，
 *   将s - backward_dist_thrd和s + forward_distance_thrd作为参数传入
 * - 返回指定s值范围内的来源信息列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用getSourceInfosFromSRange函数;
 * :传入s - backward_dist_thrd和s + forward_distance_thrd;
 * :返回来源信息列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的来源信息，适用于需要根据来源信息进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取来源信息
 */
std::vector<std::tuple<LineSourceType, float, float>> ReferenceLine::getSourceInfosFromSRange(
    const float& s, const float& forward_distance_thrd, const float& backward_dist_thrd) const {
  return getSourceInfosFromSRange(s - backward_dist_thrd, s + forward_distance_thrd);
}

/**
 * @brief 获取指定s值处的来源信息
 * @details 根据给定的s值，返回参考线上该s值所在的来源信息
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return std::tuple<LineSourceType, float, float> 返回包含该s值的来源信息，包含来源类型、起始s值和结束s值
 *
 * @par 关键变量说明:
 * - s (float): 参考线上的s值，范围[0, length_]
 *
 * @par 处理逻辑:
 * - 遍历参考线的来源信息列表
 * - 如果s值在某个来源信息的起始s值和结束s值之间，则返回该来源信息
 * - 如果遍历结束后仍未找到匹配的来源信息，则返回一个默认构造的std::tuple对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历来源信息列表;
 * if (s值 >= 来源信息的起始s值 且 s值 <= 来源信息的结束s值?) then (yes)
 *   :返回该来源信息;
 * else (no)
 *   :继续遍历;
 * endif
 * :返回默认的std::tuple对象;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值处的来源信息，适用于需要根据来源信息进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取来源信息
 */
std::tuple<LineSourceType, float, float> ReferenceLine::getSourceInfoFromS(const float& s) const {
  for (const auto& source_info : source_infos_) {
    if (s >= std::get<1>(source_info) && s <= std::get<2>(source_info)) {
      return source_info;
    }
  }
  return std::make_tuple<LineSourceType, float, float>(
      LineSourceType::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeInvalid, 0.0, 0.0);
}

/**
 * @brief 获取指定类型的来源信息
 * @details 根据给定的来源类型，返回参考线上所有符合该类型的来源信息
 *
 * @param[in] type 来源类型，包含以下枚举值：
 *   - LocalRouteReferenceLine_LineSource_LineSourceType_kTypeInvalid: 无效类型
 *   - LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception: 感知来源
 *   - LocalRouteReferenceLine_LineSource_LineSourceType_kTypeMap: 地图来源
 *   - LocalRouteReferenceLine_LineSource_LineSourceType_kTypePrediction: 预测来源
 * @return std::vector<std::tuple<LineSourceType, float, float>>
 * 返回符合条件的来源信息列表，每个元素包含来源类型、起始s值和结束s值
 *
 * @par 关键变量说明:
 * - type (LineSourceType): 来源类型，用于筛选来源信息
 * - res (std::vector<std::tuple<LineSourceType, float, float>>): 存储符合条件的来源信息列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的来源信息列表
 * - 如果来源信息的类型与传入的类型匹配，则将该来源信息添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历来源信息列表;
 * if (来源信息的类型 == 传入的类型?) then (yes)
 *   :将来源信息添加到结果列表;
 * else (no)
 *   :继续遍历;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定类型的来源信息，适用于需要根据来源类型进行决策的场景
 *
 * @warning 需确保传入的来源类型有效，否则可能无法正确获取来源信息
 */
std::vector<std::tuple<LineSourceType, float, float>> ReferenceLine::getSpecifiedSourceInfos(
    const LineSourceType& type) const {
  std::vector<std::tuple<LineSourceType, float, float>> res;
  for (int i = 0; i < source_infos_.size(); i++) {
    if (type == std::get<0>(source_infos_.at(i))) {
      res.emplace_back(source_infos_.at(i));
    }
  }
  return res;
}

/**
 * @brief 获取指定s值范围内的边界类型
 * @details 根据给定的起始s值和结束s值，返回参考线上该范围内的所有边界类型
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::vector<SegmentBoundaryType> 返回符合条件的边界类型列表
 *
 * @par 关键变量说明:
 * - start_s (float): 起始s值，范围[0, length_]
 * - end_s (float): 结束s值，范围[0, length_]
 * - res (std::vector<SegmentBoundaryType>): 存储符合条件的边界类型列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的边界类型列表
 * - 如果边界类型的结束s值小于起始s值，则跳过该边界类型
 * - 如果边界类型的起始s值大于结束s值，则停止遍历
 * - 将符合条件的边界类型添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历边界类型列表;
 * if (边界类型的结束s值 < 起始s值?) then (yes)
 *   :跳过该边界类型;
 * else if (边界类型的起始s值 > 结束s值?) then (yes)
 *   :停止遍历;
 * else (no)
 *   :将边界类型添加到结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的边界类型，适用于需要根据边界类型进行决策的场景
 *
 * @warning 需确保输入的起始s值和结束s值在参考线范围内，否则可能无法正确获取边界类型
 */
std::vector<SegmentBoundaryType> ReferenceLine::getBoundaryTypesFromSRange(const float& start_s,
                                                                           const float& end_s) const {
  std::vector<SegmentBoundaryType> res;
  for (const auto& boundary_type : boundary_types_) {
    if (boundary_type.end_s < start_s) {
      continue;
    } else if (boundary_type.start_s > end_s) {
      break;
    }
    res.emplace_back(boundary_type);
  }
  return res;
}

/**
 * @brief 获取指定s值范围内的边界类型
 * @details 根据给定的s值、前向距离阈值和后向距离阈值，返回参考线上该范围内的所有边界类型
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_distance_thrd 前向距离阈值，用于确定s值范围的上限
 * @param[in] backward_dist_thrd 后向距离阈值，用于确定s值范围的下限
 * @return std::vector<SegmentBoundaryType> 返回符合条件的边界类型列表
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 * - forward_distance_thrd (double): 前向距离阈值，用于确定s值范围的上限
 * - backward_dist_thrd (double): 后向距离阈值，用于确定s值范围的下限
 *
 * @par 处理逻辑:
 * - 调用getBoundaryTypesFromSRange(s - backward_dist_thrd, s + forward_distance_thrd)函数，
 *   将s - backward_dist_thrd和s + forward_distance_thrd作为参数传入
 * - 返回指定s值范围内的边界类型列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用getBoundaryTypesFromSRange函数;
 * :传入s - backward_dist_thrd和s + forward_distance_thrd;
 * :返回边界类型列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的边界类型，适用于需要根据边界类型进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取边界类型
 */
std::vector<SegmentBoundaryType> ReferenceLine::getBoundaryTypesFromSRange(const double s,
                                                                           const double forward_distance_thrd,
                                                                           const double backward_dist_thrd) const {
  return getBoundaryTypesFromSRange(s - backward_dist_thrd, s + forward_distance_thrd);
}

/**
 * @brief 获取指定s值处的边界类型
 * @details 根据给定的s值，返回参考线上该s值所在的边界类型
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return SegmentBoundaryType 返回包含该s值的边界类型
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值，范围[0, length_]
 *
 * @par 处理逻辑:
 * - 遍历参考线的边界类型列表
 * - 如果s值在某个边界类型的起始s值和结束s值之间，则返回该边界类型
 * - 如果遍历结束后仍未找到匹配的边界类型，则返回一个默认构造的SegmentBoundaryType对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历边界类型列表;
 * if (s值 >= 边界类型的起始s值 且 s值 <= 边界类型的结束s值?) then (yes)
 *   :返回该边界类型;
 * else (no)
 *   :继续遍历;
 * endif
 * :返回默认的SegmentBoundaryType对象;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值处的边界类型，适用于需要根据边界类型进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取边界类型
 */
SegmentBoundaryType ReferenceLine::getBoundaryTypeFromS(const double s) const {
  for (const auto& boundary_type : boundary_types_) {
    if (s >= boundary_type.start_s && s <= boundary_type.end_s) {
      return boundary_type;
    }
  }
  return SegmentBoundaryType();
}

std::vector<LineAttributeRange> ReferenceLine::getLineAttributeRangesFromSRange(const float& start_s,
                                                                                const float& end_s) const {
  std::vector<LineAttributeRange> res;
  for (auto& range : line_attribute_ranges_) {
    if (range.end_s < start_s) {
      continue;
    } else if (range.start_s > end_s) {
      break;
    }
    res.emplace_back(range);
  }
  return res;
}

std::vector<LineAttributeRange> ReferenceLine::getLineAttributeRangesFromSRange(const float& s,
                                                                                const float& forward_distance_thrd,
                                                                                const float& backward_dist_thrd) const {
  return getLineAttributeRangesFromSRange(s - backward_dist_thrd, s + forward_distance_thrd);
}

LineAttributeRange ReferenceLine::getLineAttributeRangeFromS(const float& s) const {
  for (int i = 0; i < line_attribute_ranges_.size(); ++i) {
    if (s >= line_attribute_ranges_.at(i).start_s && s <= line_attribute_ranges_.at(i).end_s) {
      return line_attribute_ranges_.at(i);
    }
  }

  return LineAttributeRange();
}

std::vector<LineAttributeRange> ReferenceLine::getSpecifiedLineAttributeRanges(
    const proto::road_cognition::LineAttributeRange::LineSourceType& source_type) const {
  std::vector<LineAttributeRange> res;
  for (int i = 0; i < line_attribute_ranges_.size(); ++i) {
    if (line_attribute_ranges_.at(i).source_type == source_type) {
      res.emplace_back(line_attribute_ranges_.at(i));
    }
  }

  return res;
}

std::vector<LineAttributeRange> ReferenceLine::getSpecifiedLineAttributeRanges(
    const proto::perception::CenterLine::TypeRange::Type& line_type) const {
  std::vector<LineAttributeRange> res;
  for (int i = 0; i < line_attribute_ranges_.size(); ++i) {
    if (line_attribute_ranges_.at(i).line_type == line_type) {
      res.emplace_back(line_attribute_ranges_.at(i));
    }
  }

  return res;
}

/**
 * @brief 获取指定s值范围内的边界类型
 * @details
 * 根据给定的s值、前向距离阈值和后向距离阈值，返回参考线上该范围内的所有边界类型
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_distance_thrd 前向距离阈值，用于确定s值范围的上限
 * @param[in] backward_dist_thrd 后向距离阈值，用于确定s值范围的下限
 * @return std::vector<BoundaryRange> 返回符合条件的边界类型列表
 *
 * @par 关键变量说明:
 * - s (float): 参考线上的s值，范围[0, length_]
 * - forward_distance_thrd (float): 前向距离阈值，用于确定s值范围的上限
 * - backward_dist_thrd (float): 后向距离阈值，用于确定s值范围的下限
 *
 * @par 处理逻辑:
 * - 调用getBoundaryRangesFromSRange(s - backward_dist_thrd, s +
 * forward_distance_thrd)函数， 将s - backward_dist_thrd和s +
 * forward_distance_thrd作为参数传入
 * - 返回指定s值范围内的边界类型列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用getBoundaryRangesFromSRange函数;
 * :传入s - backward_dist_thrd和s + forward_distance_thrd;
 * :返回边界类型列表;
 * stop
 * @enduml
 *
 * @note
 * 该函数用于获取指定s值范围内的边界类型，适用于需要根据边界类型进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取边界类型
 */
std::vector<BoundaryRange> ReferenceLine::getBoundaryRangesFromSRange(const std::vector<BoundaryRange>& boundaries,
                                                                      const float& s,
                                                                      const float& forward_distance_thrd,
                                                                      const float& backward_dist_thrd) const {
  return getBoundaryRangesFromSRange(boundaries, s - backward_dist_thrd, s + forward_distance_thrd);
}

/**
 * @brief 获取指定s值范围内的边界类型
 * @details 根据给定的起始s值和结束s值，返回参考线上该范围内的所有边界类型
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::vector<BoundaryRange> 返回符合条件的边界类型列表
 *
 * @par 关键变量说明:
 * - start_s (float): 起始s值，范围[0, length_]
 * - end_s (float): 结束s值，范围[0, length_]
 * - res (std::vector<BoundaryRange>): 存储符合条件的边界类型列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的边界类型列表
 * - 如果边界类型的结束s值小于起始s值，则跳过该边界类型
 * - 如果边界类型的起始s值大于结束s值，则停止遍历
 * - 将符合条件的边界类型添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历边界类型列表;
 * if (边界类型的结束s值 < 起始s值?) then (yes)
 *   :跳过该边界类型;
 * else if (边界类型的起始s值 > 结束s值?) then (yes)
 *   :停止遍历;
 * else (no)
 *   :将边界类型添加到结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note
 * 该函数用于获取指定s值范围内的边界类型，适用于需要根据边界类型进行决策的场景
 *
 * @warning
 * 需确保输入的起始s值和结束s值在参考线范围内，否则可能无法正确获取边界类型
 */
std::vector<BoundaryRange> ReferenceLine::getBoundaryRangesFromSRange(const std::vector<BoundaryRange>& boundaries,
                                                                      const float& start_s, const float& end_s) const {
  std::vector<BoundaryRange> res;
  for (const auto& boundary : boundaries) {
    if (boundary.end_s < start_s) {
      continue;
    } else if (boundary.start_s > end_s) {
      break;
    }
    res.emplace_back(boundary);
  }
  return res;
}

/**
 * @brief 获取指定s值处的边界类型
 * @details 根据给定的s值，返回参考线上该s值所在的边界类型
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @return BoundaryRange 返回包含该s值的边界类型
 *
 * @par 关键变量说明:
 * - s (float): 参考线上的s值，范围[0, length_]
 *
 * @par 处理逻辑:
 * - 遍历参考线的边界类型列表
 * - 如果s值在某个边界类型的起始s值和结束s值之间，则返回该边界类型
 * -
 * 如果遍历结束后仍未找到匹配的边界类型，则返回一个默认构造的BoundaryRange对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历边界类型列表;
 * if (s值 >= 边界类型的起始s值 且 s值 <= 边界类型的结束s值?) then (yes)
 *   :返回该边界类型;
 * else (no)
 *   :继续遍历;
 * endif
 * :返回默认的BoundaryRange对象;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值处的边界类型，适用于需要根据边界类型进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取边界类型
 */
BoundaryRange ReferenceLine::getBoundaryRangeFromS(const std::vector<BoundaryRange>& boundaries, const float& s) const {
  for (const auto& boundary : boundaries) {
    if (s >= boundary.start_s && s <= boundary.end_s) {
      return boundary;
    }
  }
  return BoundaryRange();
}

std::vector<std::string> ReferenceLine::getNeighborReferenceLineIdsFromS(const float &s,
                                                                         const bool &is_left_neighbor) const {
  std::vector<std::string> neighbor_ref_ids;
  const auto &neighbor_refs = is_left_neighbor ? left_ref_lines_ : right_ref_lines_;
  for (const auto &ref : neighbor_refs) {
    for (const auto &range : ref.second.boundaries) {
      if (s >= range.start_s && s <= range.end_s) {
        neighbor_ref_ids.emplace_back(ref.second.id);
        break;
      }
    }
  }

  return neighbor_ref_ids;
}

/**
 * @brief 生成平行参考线
 * @details 根据给定的偏移量和ID，生成一条与当前参考线平行的新参考线
 *
 * @param[in] offset 偏移量，用于确定新参考线与原参考线的距离
 * @param[in] id 新参考线的ID
 * @return std::pair<bool, ReferenceLine> 返回一个包含生成结果和新参考线的pair对象，其中bool表示生成是否成功
 *
 * @par 关键变量说明:
 * - offset (float): 偏移量，用于确定新参考线与原参考线的距离
 * - id (std::string): 新参考线的ID
 * - res (std::pair<bool, ReferenceLine>): 存储生成结果和新参考线
 * - pts (std::vector<math::Vec3d>): 存储新参考线的点集
 *
 * @par 处理逻辑:
 * - 检查原参考线的点集是否为空，如果为空则返回失败
 * - 遍历原参考线的点集，根据偏移量计算新参考线的点集
 * - 如果新参考线的点集不为空，则创建新参考线对象并设置相关属性
 * - 将原参考线的速度限制、方向段、导航变道范围、车道线巡线范围等信息复制到新参考线
 * - 返回生成结果和新参考线
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查原参考线的点集是否为空;
 * if (原参考线的点集为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :遍历原参考线的点集;
 *   :根据偏移量计算新参考线的点集;
 *   :创建新参考线对象;
 *   :设置新参考线的相关属性;
 *   :复制原参考线的速度限制、方向段等信息;
 *   :返回生成结果和新参考线;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于生成一条与当前参考线平行的新参考线，适用于需要根据偏移量生成虚拟参考线的场景
 *
 * @warning 需确保传入的偏移量合理，否则可能无法正确生成平行参考线
 */
std::pair<bool, ReferenceLine> ReferenceLine::generateParallelReferenceLine(const float& offset,
                                                                            const std::string& id) const {
  std::pair<bool, ReferenceLine> res = std::make_pair<bool, ReferenceLine>(false, ReferenceLine());
  if (reference_points_.empty()) {
    std::cerr << "[ReferenceLine::generateParallelReferenceLine]: raw reference_points_ empty!";
    return res;
  }

  std::vector<math::Vec3d> pts;
  for (const auto& pt : reference_points_) {
    gpal::pnc::SLPoint sl;
    sl.set_s(pt.local_s());
    sl.set_l(offset);
    pts.emplace_back();
    sl2xy(sl, &pts.back());
  }
  if (pts.size() > 0) {
    ReferenceLine ref(pts);
    ref.set_line_type(line_type_);
    ref.set_smooth_type(ReferenceLine::SmoothType::RAW);
    ref.set_shape_type(shape_type_);
    ref.set_global_start_s(global_start_s_);
    ref.set_id(id_ + "_" + id);
    ref.setIsCurrentReferenceLine(false);
    ref.setIsMemorizedParkRoute(is_memorized_park_route_);
    ref.setIsMemorizedEndRoute(is_memorized_end_route_);
    ref.setIsTerminalEnforced(is_terminal_enforced_);
    ref.setIsParallelVirtual(true);

    std::vector<SpeedLimit> speed_limits;
    for (const auto& pt : speed_limits_) {
      SLPoint start_sl, end_sl;
      ref.xy2sl(math::Vec3d(pt.start_point.x(), pt.start_point.y(), pt.start_point.z()), &start_sl);
      ref.xy2sl(math::Vec3d(pt.end_point.x(), pt.end_point.y(), pt.end_point.z()), &end_sl);
      speed_limits.emplace_back(start_sl.s(), end_sl.s(), pt.max_speed_limit, pt.min_speed_limit, pt.start_point,
                                pt.end_point);
    }
    if (!speed_limits.empty()) {
      speed_limits.front().start_s = 0;
      speed_limits.back().end_s = ref.length();
    }
    ref.setSpeedLimits(std::move(speed_limits));

    std::vector<SegmentDirection> directions;
    for (const auto& pt : directions_) {
      SLPoint start_sl, end_sl;
      ref.xy2sl(math::Vec3d(pt.start_point.x(), pt.start_point.y(), pt.start_point.z()), &start_sl);
      ref.xy2sl(math::Vec3d(pt.end_point.x(), pt.end_point.y(), pt.end_point.z()), &end_sl);
      directions.emplace_back(start_sl.s(), end_sl.s(), pt.direction, pt.start_point, pt.end_point);
    }
    if (!directions.empty()) {
      directions.front().start_s = 0;
      directions.back().end_s = ref.length();
    }
    ref.setDirections(std::move(directions));

    std::vector<std::tuple<float, float, bool, bool>> navigation_lane_change_ranges;
    for (const auto& range : navigation_lane_change_ranges_) {
      float start_s = std::fmin(ref.length(), std::fmax(0.0, std::get<0>(range)));
      float end_s = std::fmin(ref.length(), std::fmax(0.0, std::get<1>(range)));
      navigation_lane_change_ranges.emplace_back(std::move(start_s), std::move(end_s), std::get<2>(range),
                                                 std::get<3>(range));
    }
    ref.setNavigationLaneChangeRanges(std::move(navigation_lane_change_ranges));

    std::vector<std::pair<float, float>> lane_follow_ranges;
    for (const auto& range : lane_follow_ranges_) {
      float start_s = std::fmin(ref.length(), std::fmax(0.0, range.first));
      float end_s = std::fmin(ref.length(), std::fmax(0.0, range.second));
      lane_follow_ranges.emplace_back(std::move(start_s), std::move(end_s));
    }
    ref.setlaneFollowRanges(std::move(lane_follow_ranges));

    res.first = true;
    res.second = std::move(ref);
  } else {
    std::cerr << "[ReferenceLine::generateParallelReferenceLine]: generate reference_points_ empty!";
  }

  return res;
}

/**
 * @brief 获取指定s值范围内的汇入汇出范围
 * @details 根据给定的起始s值和结束s值，返回参考线上该范围内的所有汇入汇出范围
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::vector<MergeForkRange> 返回符合条件的汇入汇出范围列表
 *
 * @par 关键变量说明:
 * - start_s (float): 起始s值，范围[0, length_]
 * - end_s (float): 结束s值，范围[0, length_]
 * - res (std::vector<MergeForkRange>): 存储符合条件的汇入汇出范围列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的汇入汇出范围列表
 * - 如果汇入汇出范围的s值小于起始s值，则跳过该范围
 * - 如果汇入汇出范围的s值大于结束s值，则停止遍历
 * - 将符合条件的汇入汇出范围添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历汇入汇出范围列表;
 * if (汇入汇出范围的s值 < 起始s值?) then (yes)
 *   :跳过该范围;
 * else if (汇入汇出范围的s值 > 结束s值?) then (yes)
 *   :停止遍历;
 * else (no)
 *   :将汇入汇出范围添加到结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的汇入汇出范围，适用于需要根据汇入汇出范围进行决策的场景
 *
 * @warning 需确保输入的起始s值和结束s值在参考线范围内，否则可能无法正确获取汇入汇出范围
 */
std::vector<MergeForkRange> ReferenceLine::getMergeForkRangesFromSRange(const float& start_s, const float& end_s) {
  std::vector<MergeForkRange> res;
  for (const auto& range : merge_fork_ranges_) {
    if (range.s() < start_s) {
      continue;
    } else if (range.s() > end_s) {
      break;
    }
    res.emplace_back(range);
  }
  return res;
}

/**
 * @brief 获取指定s值范围内的汇入汇出范围
 * @details 根据给定的s值、前向距离阈值和后向距离阈值，返回参考线上该范围内的所有汇入汇出范围
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_dis 前向距离阈值，用于确定s值范围的上限
 * @param[in] backward_dis 后向距离阈值，用于确定s值范围的下限
 * @return std::vector<MergeForkRange> 返回符合条件的汇入汇出范围列表
 *
 * @par 关键变量说明:
 * - s (float): 参考线上的s值，范围[0, length_]
 * - forward_dis (float): 前向距离阈值，用于确定s值范围的上限
 * - backward_dis (float): 后向距离阈值，用于确定s值范围的下限
 *
 * @par 处理逻辑:
 * - 调用getMergeForkRangesFromSRange(s - backward_dis, s + forward_dis)函数，
 *   将s - backward_dis和s + forward_dis作为参数传入
 * - 返回指定s值范围内的汇入汇出范围列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用getMergeForkRangesFromSRange函数;
 * :传入s - backward_dis和s + forward_dis;
 * :返回汇入汇出范围列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的汇入汇出范围，适用于需要根据汇入汇出范围进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取汇入汇出范围
 */
std::vector<MergeForkRange> ReferenceLine::getMergeForkRangesFromSRange(const float& s, const float& forward_dis,
                                                                        const float& backward_dis) {
  return getMergeForkRangesFromSRange(s - backward_dis, s + forward_dis);
}

/**
 * @brief 获取指定s值范围内的汇入范围
 * @details 根据给定的起始s值和结束s值，返回参考线上该范围内的所有汇入范围
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::vector<MergeForkRange> 返回符合条件的汇入范围列表
 *
 * @par 关键变量说明:
 * - start_s (float): 起始s值，范围[0, length_]
 * - end_s (float): 结束s值，范围[0, length_]
 * - res (std::vector<MergeForkRange>): 存储符合条件的汇入范围列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的汇入汇出范围列表
 * - 如果汇入汇出范围的类型为汇入类型，则继续判断
 * - 如果汇入汇出范围的s值小于起始s值，则跳过该范围
 * - 如果汇入汇出范围的s值大于结束s值，则停止遍历
 * - 将符合条件的汇入范围添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历汇入汇出范围列表;
 * if (汇入汇出范围的类型 == 汇入类型?) then (yes)
 *   if (汇入汇出范围的s值 < 起始s值?) then (yes)
 *     :跳过该范围;
 *   else if (汇入汇出范围的s值 > 结束s值?) then (yes)
 *     :停止遍历;
 *   else (no)
 *     :将汇入范围添加到结果列表;
 *   endif
 * else (no)
 *   :继续遍历;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的汇入范围，适用于需要根据汇入范围进行决策的场景
 *
 * @warning 需确保输入的起始s值和结束s值在参考线范围内，否则可能无法正确获取汇入范围
 */
std::vector<MergeForkRange> ReferenceLine::getMergeRangesFromSRange(const float& start_s, const float& end_s) {
  std::vector<MergeForkRange> res;
  for (const auto& range : merge_fork_ranges_) {
    if (range.type() == MergeForkRange::MergeForkType::MERGE) {
      if (range.s() < start_s) {
        continue;
      } else if (range.s() > end_s) {
        break;
      }
      res.emplace_back(range);
    }
  }
  return res;
}

/**
 * @brief 获取指定s值范围内的汇入范围
 * @details 根据给定的s值、前向距离阈值和后向距离阈值，返回参考线上该范围内的所有汇入范围
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_dis 前向距离阈值，用于确定s值范围的上限
 * @param[in] backward_dis 后向距离阈值，用于确定s值范围的下限
 * @return std::vector<MergeForkRange> 返回符合条件的汇入范围列表
 *
 * @par 关键变量说明:
 * - s (float): 参考线上的s值，范围[0, length_]
 * - forward_dis (float): 前向距离阈值，用于确定s值范围的上限
 * - backward_dis (float): 后向距离阈值，用于确定s值范围的下限
 *
 * @par 处理逻辑:
 * - 调用getMergeRangesFromSRange(s - backward_dis, s + forward_dis)函数，
 *   将s - backward_dis和s + forward_dis作为参数传入
 * - 返回指定s值范围内的汇入范围列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用getMergeRangesFromSRange函数;
 * :传入s - backward_dis和s + forward_dis;
 * :返回汇入范围列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的汇入范围，适用于需要根据汇入范围进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取汇入范围
 */
std::vector<MergeForkRange> ReferenceLine::getMergeRangesFromSRange(const float& s, const float& forward_dis,
                                                                    const float& backward_dis) {
  return getMergeRangesFromSRange(s - backward_dis, s + forward_dis);
}

/**
 * @brief 获取指定s值范围内的汇出范围
 * @details 根据给定的起始s值和结束s值，返回参考线上该范围内的所有汇出范围
 *
 * @param[in] start_s 起始s值，范围[0, length_]
 * @param[in] end_s 结束s值，范围[0, length_]
 * @return std::vector<MergeForkRange> 返回符合条件的汇出范围列表
 *
 * @par 关键变量说明:
 * - start_s (float): 起始s值，范围[0, length_]
 * - end_s (float): 结束s值，范围[0, length_]
 * - res (std::vector<MergeForkRange>): 存储符合条件的汇出范围列表
 *
 * @par 处理逻辑:
 * - 遍历参考线的汇入汇出范围列表
 * - 如果汇入汇出范围的类型为汇出类型，则继续判断
 * - 如果汇入汇出范围的s值小于起始s值，则跳过该范围
 * - 如果汇入汇出范围的s值大于结束s值，则停止遍历
 * - 将符合条件的汇出范围添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历汇入汇出范围列表;
 * if (汇入汇出范围的类型 == 汇出类型?) then (yes)
 *   if (汇入汇出范围的s值 < 起始s值?) then (yes)
 *     :跳过该范围;
 *   else if (汇入汇出范围的s值 > 结束s值?) then (yes)
 *     :停止遍历;
 *   else (no)
 *     :将汇出范围添加到结果列表;
 *   endif
 * else (no)
 *   :继续遍历;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的汇出范围，适用于需要根据汇出范围进行决策的场景
 *
 * @warning 需确保输入的起始s值和结束s值在参考线范围内，否则可能无法正确获取汇出范围
 */
std::vector<MergeForkRange> ReferenceLine::getForkRangesFromSRange(const float& start_s, const float& end_s) {
  std::vector<MergeForkRange> res;
  for (const auto& range : merge_fork_ranges_) {
    if (range.type() == MergeForkRange::MergeForkType::FORK) {
      if (range.s() < start_s) {
        continue;
      } else if (range.s() > end_s) {
        break;
      }
      res.emplace_back(range);
    }
  }
  return res;
}

/**
 * @brief 获取指定s值范围内的汇出范围
 * @details 根据给定的s值、前向距离阈值和后向距离阈值，返回参考线上该范围内的所有汇出范围
 *
 * @param[in] s 参考线上的s值，范围[0, length_]
 * @param[in] forward_dis 前向距离阈值，用于确定s值范围的上限
 * @param[in] backward_dis 后向距离阈值，用于确定s值范围的下限
 * @return std::vector<MergeForkRange> 返回符合条件的汇出范围列表
 *
 * @par 关键变量说明:
 * - s (float): 参考线上的s值，范围[0, length_]
 * - forward_dis (float): 前向距离阈值，用于确定s值范围的上限
 * - backward_dis (float): 后向距离阈值，用于确定s值范围的下限
 *
 * @par 处理逻辑:
 * - 调用getForkRangesFromSRange(s - backward_dis, s + forward_dis)函数，
 *   将s - backward_dis和s + forward_dis作为参数传入
 * - 返回指定s值范围内的汇出范围列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用getForkRangesFromSRange函数;
 * :传入s - backward_dis和s + forward_dis;
 * :返回汇出范围列表;
 * stop
 * @enduml
 *
 * @note 该函数用于获取指定s值范围内的汇出范围，适用于需要根据汇出范围进行决策的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确获取汇出范围
 */
std::vector<MergeForkRange> ReferenceLine::getForkRangesFromSRange(const float& s, const float& forward_dis,
                                                                   const float& backward_dis) {
  return getForkRangesFromSRange(s - backward_dis, s + forward_dis);
}

std::vector<KeyPoint> ReferenceLine::getKeyPointsFromSRange(const float& start_s, const float& end_s) const {
  std::vector<KeyPoint> res;
  for (const auto& pt : key_points_) {
    if (pt.s < start_s) {
      continue;
    } else if (pt.s > end_s) {
      break;
    }
    res.emplace_back(pt);
  }
  return res;
}

std::vector<KeyPoint> ReferenceLine::getKeyPointsFromSRange(const float& s, const float& forward_dis,
                                                            const float& backward_dis) const {
  return getKeyPointsFromSRange(s - backward_dis, s + forward_dis);
}

std::vector<KeyPoint> ReferenceLine::getSpecifiedKeyPoints(
    const proto::road_cognition::KeyPoint::PointType& type) const {
  std::vector<KeyPoint> res;
  for (int i = 0; i < key_points_.size(); ++i) {
    if (key_points_.at(i).type == type) {
      res.emplace_back(key_points_.at(i));
    }
  }

  return res;
}

/**
 * @brief 计算参考线信息: heading, kappa, dkappa
 * @details 根据输入的参考点集，计算每个点的航向角、曲率和曲率变化率
 *
 * @param[in] xyz_points 参考点集
 * @return bool 返回计算是否成功
 *
 * @par 处理逻辑:
 * - 检查参考点集是否为空，如果为空则返回失败
 * - 计算每个点的dx和dy，用于后续的航向角和曲率计算
 * - 计算每个点的航向角
 * - 计算累积的s值，用于后续的曲率变化率计算
 * - 计算每个点的曲率
 * - 计算每个点的曲率变化率
 *
 * @note 该函数用于初始化参考线的关键信息，适用于需要根据参考线进行路径规划的场景
 *
 * @warning 需确保输入的参考点集至少包含两个点，否则无法正确计算
 */
bool ReferenceLine::computeInitInfo(std::vector<ReferencePoint>& xyz_points) {
  if (xyz_points.size() < 2) {
    return false;
  }
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = xyz_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xyz_points[i + 1].x() - xyz_points[i].x());
      y_delta = (xyz_points[i + 1].y() - xyz_points[i].y());
    } else if (i == points_size - 1) {
      x_delta = (xyz_points[i].x() - xyz_points[i - 1].x());
      y_delta = (xyz_points[i].y() - xyz_points[i - 1].y());
    } else {
      x_delta = 0.5 * (xyz_points[i + 1].x() - xyz_points[i - 1].x());
      y_delta = 0.5 * (xyz_points[i + 1].y() - xyz_points[i - 1].y());
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    xyz_points[i].setHeading(std::atan2(dys[i], dxs[i]));
  }

  // Get linear interpolated s for dkappa calculation
  double distance = 0.0;
  std::vector<double> accumulated_s;
  accumulated_s.push_back(distance);
  double fx = xyz_points[0].x();
  double fy = xyz_points[0].y();
  double nx = 0.0;
  double ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = xyz_points[i].x();
    ny = xyz_points[i].y();
    double end_segment_s = std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s.push_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (xyz_points[i + 1].x() - xyz_points[i].x()) / (accumulated_s[i + 1] - accumulated_s[i]);
      yds = (xyz_points[i + 1].y() - xyz_points[i].y()) / (accumulated_s[i + 1] - accumulated_s[i]);
    } else if (i == points_size - 1) {
      xds = (xyz_points[i].x() - xyz_points[i - 1].x()) / (accumulated_s[i] - accumulated_s[i - 1]);
      yds = (xyz_points[i].y() - xyz_points[i - 1].y()) / (accumulated_s[i] - accumulated_s[i - 1]);
    } else {
      xds = (xyz_points[i + 1].x() - xyz_points[i - 1].x()) / (accumulated_s[i + 1] - accumulated_s[i - 1]);
      yds = (xyz_points[i + 1].y() - xyz_points[i - 1].y()) / (accumulated_s[i + 1] - accumulated_s[i - 1]);
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  // Get finite difference approximated second derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
             (accumulated_s[i + 1] - accumulated_s[i]);
      ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
             (accumulated_s[i + 1] - accumulated_s[i]);
    } else if (i == points_size - 1) {
      xdds = (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
             (accumulated_s[i] - accumulated_s[i - 1]);
      ydds = (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
             (accumulated_s[i] - accumulated_s[i - 1]);
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i - 1]) /
             (accumulated_s[i + 1] - accumulated_s[i - 1]);
      ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i - 1]) /
             (accumulated_s[i + 1] - accumulated_s[i - 1]);
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa = (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    xyz_points[i].setKappa(kappa);
  }

  // Dkappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa = (xyz_points[i + 1].kappa() - xyz_points[i].kappa()) / (accumulated_s[i + 1] - accumulated_s[i]);
    } else if (i == points_size - 1) {
      dkappa = (xyz_points[i].kappa() - xyz_points[i - 1].kappa()) / (accumulated_s[i] - accumulated_s[i - 1]);
    } else {
      dkappa = (xyz_points[i + 1].kappa() - xyz_points[i - 1].kappa()) / (accumulated_s[i + 1] - accumulated_s[i - 1]);
    }
    xyz_points[i].setDkappa(dkappa);
  }
  return true;
}

/**
 * @brief 计算参考线的曲率和曲率变化率
 * @details 根据参考线的采样点，计算每个点的曲率（kappa）和曲率变化率（dkappa）
 *
 * @par 处理逻辑:
 * - 检查参考线的采样点数量是否足够，如果少于2个点则直接返回
 * - 计算每个点的x和y方向的一阶导数（xds, yds）
 * - 计算每个点的x和y方向的二阶导数（xdds, ydds）
 * - 根据一阶和二阶导数计算每个点的曲率（kappa）
 * - 根据曲率的变化计算每个点的曲率变化率（dkappa）
 *
 * @par 关键变量说明:
 * - y_over_s_first_derivatives (std::vector<double>): 存储y方向的一阶导数
 * - x_over_s_first_derivatives (std::vector<double>): 存储x方向的一阶导数
 * - y_over_s_second_derivatives (std::vector<double>): 存储y方向的二阶导数
 * - x_over_s_second_derivatives (std::vector<double>): 存储x方向的二阶导数
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考点数量是否足够;
 * if (参考点数量 < 2?) then (yes)
 *   :直接返回;
 * else (no)
 *   :计算x和y方向的一阶导数;
 *   :计算x和y方向的二阶导数;
 *   :计算每个点的曲率;
 *   :计算每个点的曲率变化率;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于初始化参考线的曲率和曲率变化率信息，适用于需要根据参考线进行路径规划的场景
 *
 * @warning 需确保参考线的采样点数量足够，否则无法正确计算曲率和曲率变化率
 */
void ReferenceLine::computeKappaAndDkappa() {
  if (reference_points_.size() < 2) {
    return;
  }

  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;
  std::size_t points_size = reference_points_.size();

  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (reference_points_[i + 1].x() - reference_points_[i].x()) / (accumulated_s_[i + 1] - accumulated_s_[i]);
      yds = (reference_points_[i + 1].y() - reference_points_[i].y()) / (accumulated_s_[i + 1] - accumulated_s_[i]);
    } else if (i == points_size - 1) {
      xds = (reference_points_[i].x() - reference_points_[i - 1].x()) / (accumulated_s_[i] - accumulated_s_[i - 1]);
      yds = (reference_points_[i].y() - reference_points_[i - 1].y()) / (accumulated_s_[i] - accumulated_s_[i - 1]);
    } else {
      xds = (reference_points_[i + 1].x() - reference_points_[i - 1].x()) /
            (accumulated_s_[i + 1] - accumulated_s_[i - 1]);
      yds = (reference_points_[i + 1].y() - reference_points_[i - 1].y()) /
            (accumulated_s_[i + 1] - accumulated_s_[i - 1]);
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
             (accumulated_s_[i + 1] - accumulated_s_[i]);
      ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
             (accumulated_s_[i + 1] - accumulated_s_[i]);
    } else if (i == points_size - 1) {
      xdds = (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
             (accumulated_s_[i] - accumulated_s_[i - 1]);
      ydds = (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
             (accumulated_s_[i] - accumulated_s_[i - 1]);
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i - 1]) /
             (accumulated_s_[i + 1] - accumulated_s_[i - 1]);
      ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i - 1]) /
             (accumulated_s_[i + 1] - accumulated_s_[i - 1]);
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  // Kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa = (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    reference_points_[i].setKappa(kappa);
  }

  // Dkappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa = (reference_points_[i + 1].kappa() - reference_points_[i].kappa()) /
               (accumulated_s_[i + 1] - accumulated_s_[i]);
    } else if (i == points_size - 1) {
      dkappa = (reference_points_[i].kappa() - reference_points_[i - 1].kappa()) /
               (accumulated_s_[i] - accumulated_s_[i - 1]);
    } else {
      dkappa = (reference_points_[i + 1].kappa() - reference_points_[i - 1].kappa()) /
               (accumulated_s_[i + 1] - accumulated_s_[i - 1]);
    }
    reference_points_[i].setDkappa(dkappa);
  }
}


/**
 * @brief 获取多边形与参考线的重叠范围
 * @details 根据给定的多边形，计算其与参考线的重叠范围，并返回重叠的起始s值和结束s值
 *
 * @param[in] polygon 输入的多边形
 * @return std::tuple<bool, double, double> 返回一个包含是否重叠、起始s值和结束s值的tuple对象
 *
 * @par 关键变量说明:
 * - polygon (math::Polygon2d): 输入的多边形
 * - res (std::tuple<bool, double, double>): 存储返回结果，包含是否重叠、起始s值和结束s值
 * - in_range (bool): 是否重叠的标志
 * - start_s (double): 重叠范围的起始s值
 * - end_s (double): 重叠范围的结束s值
 * - has_range (bool): 是否找到重叠范围的标志
 *
 * @par 处理逻辑:
 * - 初始化返回结果，默认不重叠，起始s值为参考线长度，结束s值为0
 * - 检查参考线的线段是否为空，如果为空则直接返回默认结果
 * - 遍历参考线的每个线段，检查是否与多边形重叠
 * - 如果找到重叠，则计算重叠的起始s值和结束s值，并更新返回结果
 * - 如果已经找到重叠范围且后续线段不再重叠，则停止遍历
 * - 返回最终的重叠范围结果
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化返回结果;
 * if (参考线的线段为空?) then (yes)
 *   :直接返回默认结果;
 * else (no)
 *   :遍历参考线的每个线段;
 *   if (线段与多边形重叠?) then (yes)
 *     :计算重叠的起始s值和结束s值;
 *     :更新返回结果;
 *   else if (已经找到重叠范围且后续线段不再重叠?) then (yes)
 *     :停止遍历;
 *   endif
 * endif
 * :返回最终的重叠范围结果;
 * stop
 * @enduml
 *
 * @note 该函数用于计算多边形与参考线的重叠范围，适用于需要根据重叠范围进行决策的场景
 *
 * @warning 需确保输入的多边形和参考线有效，否则可能无法正确计算重叠范围
 */
std::tuple<bool, double, double> ReferenceLine::getOverlapRange(const math::Polygon2d& polygon) const {
  std::tuple<bool, double, double> res(false, length(), 0.0);
  auto& [in_range, start_s, end_s] = res;
  bool has_range = false;
  if (!line_segments_.empty()) {
    for (int i = 0; i < line_segments_.size(); i++) {
      if (polygon.HasOverlap(line_segments_[i])) {
        has_range = true;
        math::Vec3d first, second;
        polygon.GetOverlap(line_segments_[i], &first, &second);
        double overlap_start_s = accumulated_s_[i] + line_segments_[i].ProjectOntoUnit(first);
        double overlap_end_s = accumulated_s_[i] + line_segments_[i].ProjectOntoUnit(second);
        start_s = std::min(start_s, overlap_start_s);
        end_s = std::max(end_s, overlap_end_s);
      } else if (has_range) {
        break;
      }
    }
    in_range = start_s <= end_s;
  }
  return res;
}

/**
 * @brief 将SL坐标转换为XY坐标
 * @details 根据给定的SL坐标点，计算其在参考线上的XY坐标
 *
 * @param[in] sl_point 输入的SL坐标点
 * @param[out] xyz_point 输出的XY坐标点
 * @return bool 返回转换是否成功
 *
 * @par 关键变量说明:
 * - sl_point (SLPoint): 输入的SL坐标点，包含s值和l值
 * - xyz_point (math::Vec3d*): 输出的XY坐标点，包含x、y、z值
 * - matched_point (ReferencePoint): 与s值匹配的参考点
 * - angle (math::Angle16): 匹配点的航向角
 *
 * @par 处理逻辑:
 * - 检查输出指针是否为空，如果为空则返回失败
 * - 检查参考线的点集是否足够，如果少于2个点则返回失败
 * - 根据s值获取匹配的参考点
 * - 根据匹配点的航向角和l值计算XY坐标
 * - 设置输出点的x、y、z值
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输出指针是否为空;
 * if (输出指针为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :检查参考线的点集是否足够;
 *   if (参考线的点集 < 2?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :根据s值获取匹配的参考点;
 *     :根据匹配点的航向角和l值计算XY坐标;
 *     :设置输出点的x、y、z值;
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于将SL坐标转换为XY坐标，适用于需要根据参考线进行坐标转换的场景
 *
 * @warning 需确保参考线的点集足够，否则无法正确进行坐标转换
 */
bool ReferenceLine::sl2xy(const SLPoint& sl_point, math::Vec3d* const xyz_point) const {
  CHECK_NOTNULL(xyz_point);
  if (reference_points_.size() < 2) {
    ERT_PLOG_I << "[ReferenceLine::sl2xy]: The reference line has too few points.";
    return false;
  }

  const auto matched_point = getReferencePoint(sl_point.s());
  const auto angle = math::Angle16::from_rad(matched_point.heading());
  xyz_point->set_x(matched_point.x() - math::sin(angle) * sl_point.l());
  xyz_point->set_y(matched_point.y() + math::cos(angle) * sl_point.l());
  xyz_point->set_z(matched_point.z());
  return true;
}

/**
 * @brief 将SL坐标转换为XY坐标，并生成路径点
 * @details 根据给定的SL坐标（s, l, dl, ddl），计算其在参考线上的XY坐标，并生成路径点
 *
 * @param[in] s 参考线上的s值
 * @param[in] l 参考线上的l值（横向偏移）
 * @param[in] dl l值的一阶导数（横向速度）
 * @param[in] ddl l值的二阶导数（横向加速度）
 * @param[out] path_point 输出的路径点，包含x、y、z、航向角、曲率等信息
 * @return bool 返回转换是否成功
 *
 * @par 关键变量说明:
 * - s (double): 参考线上的s值
 * - l (double): 参考线上的l值（横向偏移）
 * - dl (double): l值的一阶导数（横向速度）
 * - ddl (double): l值的二阶导数（横向加速度）
 * - path_point (proto::PathPoint*): 输出的路径点，包含x、y、z、航向角、曲率等信息
 * - ref_point (ReferencePoint): 与s值匹配的参考点
 * - rtheta (double): 参考点的航向角
 * - rkappa (double): 参考点的曲率
 * - rdkappa (double): 参考点的曲率变化率
 *
 * @par 处理逻辑:
 * - 根据s值获取匹配的参考点
 * - 根据参考点的航向角和l值计算XY坐标
 * - 根据参考点的曲率和l值计算路径点的航向角
 * - 根据参考点的曲率变化率和l值计算路径点的曲率
 * - 设置路径点的x、y、z、航向角、曲率等信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :根据s值获取匹配的参考点;
 * :根据参考点的航向角和l值计算XY坐标;
 * :根据参考点的曲率和l值计算路径点的航向角;
 * :根据参考点的曲率变化率和l值计算路径点的曲率;
 * :设置路径点的x、y、z、航向角、曲率等信息;
 * :返回成功;
 * stop
 * @enduml
 *
 * @note 该函数用于将SL坐标转换为XY坐标，并生成路径点，适用于需要根据参考线进行路径点生成的场景
 *
 * @warning 需确保输入的s值在参考线范围内，否则可能无法正确生成路径点
 */
bool ReferenceLine::sl2xy_path_point(const double s, const double l, const double dl, const double ddl,
                                     proto::PathPoint* path_point) const {
  const auto ref_point = getReferencePoint(s);
  double rx = ref_point.x();
  double ry = ref_point.y();
  double rz = ref_point.z();
  double rslope = ref_point.slope();
  double rtheta = ref_point.heading();
  double rkappa = ref_point.kappa();
  double rdkappa = ref_point.dkappa();

  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  path_point->set_x(rx - sin_theta_r * l);
  path_point->set_y(ry + cos_theta_r * l);
  path_point->set_z(rz);
  path_point->set_slope(rslope);

  const double one_minus_kappa_r_d = 1 - rkappa * l;
  if (std::abs(one_minus_kappa_r_d) < 1e-6) {
    ERT_PLOG_I << "[ReferenceLine::sl2xy_path_point]: one_minus_kappa_r_d = 0";
    return false;
  }

  const double tan_delta_theta = dl / one_minus_kappa_r_d;
  const double delta_theta = std::atan2(dl, one_minus_kappa_r_d);
  const double cos_delta_theta = std::cos(delta_theta);

  path_point->set_theta(math::NormalizeAngle(delta_theta + rtheta));
  const double kappa_r_d_prime = rdkappa * l + rkappa * dl;

  double kappa =
      (((ddl + kappa_r_d_prime * tan_delta_theta) * cos_delta_theta * cos_delta_theta) / (one_minus_kappa_r_d) +
       rkappa) *
      cos_delta_theta / (one_minus_kappa_r_d);
  path_point->set_kappa(kappa);
  return true;
}

/**
 * @brief 将XY坐标转换为SL坐标
 * @details 根据给定的XY坐标点，计算其在参考线上的SL坐标
 *
 * @param[in] xyz_point 输入的XY坐标点
 * @param[out] sl_point 输出的SL坐标点
 * @return bool 返回转换是否成功
 *
 * @par 关键变量说明:
 * - xyz_point (math::Vec3d): 输入的XY坐标点，包含x、y、z值
 * - sl_point (SLPoint*): 输出的SL坐标点，包含s值、l值和投影点索引
 * - s (double): 参考线上的s值
 * - l (double): 参考线上的l值（横向偏移）
 * - min_idx (int): 最近线段的索引
 *
 * @par 处理逻辑:
 * - 检查输出指针是否为空，如果为空则返回失败
 * - 调用getProjection函数，计算XY坐标点在参考线上的投影
 * - 如果投影失败，则返回失败
 * - 设置SL坐标点的s值、l值和投影点索引
 * - 设置SL坐标点的投影点信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输出指针是否为空;
 * if (输出指针为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :调用getProjection函数计算投影;
 *   if (投影失败?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :设置SL坐标点的s值、l值和投影点索引;
 *     :设置SL坐标点的投影点信息;
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于将XY坐标转换为SL坐标，适用于需要根据参考线进行坐标转换的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确进行坐标转换
 */
bool ReferenceLine::xy2sl(const math::Vec3d& xyz_point, SLPoint* const sl_point) const {
  CHECK_NOTNULL(sl_point);
  double s = 0.0;
  double l = 0.0;
  int min_idx = -1;
  if (!getProjection(xyz_point, s, l, min_idx)) {
    ERT_PLOG_I << "[ReferenceLine::xy2sl]: Cannot get nearest point from path.";
    return false;
  }
  sl_point->set_s(s);
  sl_point->set_l(l);
  sl_point->set_proj_idx(min_idx);
  proto::LiteTrajectoryPoint proj_pt;
  const auto& seg = line_segments_[min_idx];
  proj_pt.set_x(seg.center().x());
  proj_pt.set_y(seg.center().y());
  proj_pt.set_z(0.0);
  proj_pt.set_theta(seg.heading());
  *sl_point->mutable_proj_pt() = proj_pt;
  return true;
}

/**
 * @brief 将XY坐标转换为SL坐标（带航向角）
 * @details 根据给定的XY坐标点和航向角，计算其在参考线上的SL坐标
 *
 * @param[in] xyz_point 输入的XY坐标点
 * @param[in] heading 输入的航向角
 * @param[out] sl_point 输出的SL坐标点
 * @return bool 返回转换是否成功
 *
 * @par 关键变量说明:
 * - xyz_point (math::Vec3d): 输入的XY坐标点，包含x、y、z值
 * - heading (float): 输入的航向角
 * - sl_point (SLPoint*): 输出的SL坐标点，包含s值、l值和投影点索引
 * - s (double): 参考线上的s值
 * - l (double): 参考线上的l值（横向偏移）
 * - min_idx (int): 最近线段的索引
 *
 * @par 处理逻辑:
 * - 检查输出指针是否为空，如果为空则返回失败
 * - 调用getProjection函数，计算XY坐标点在参考线上的投影
 * - 如果投影失败，则返回失败
 * - 设置SL坐标点的s值、l值和投影点索引
 * - 设置SL坐标点的投影点信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输出指针是否为空;
 * if (输出指针为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :调用getProjection函数计算投影;
 *   if (投影失败?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :设置SL坐标点的s值、l值和投影点索引;
 *     :设置SL坐标点的投影点信息;
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于将XY坐标转换为SL坐标，适用于需要根据参考线进行坐标转换的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确进行坐标转换
 */
bool ReferenceLine::xy2sl(const math::Vec3d& xyz_point, const float& heading, SLPoint* const sl_point) const {
  CHECK_NOTNULL(sl_point);
  double s = 0.0;
  double l = 0.0;
  int min_idx = -1;
  if (!getProjection(xyz_point, heading, s, l, min_idx)) {
    ERT_PLOG_I << "[ReferenceLine::xy2sl]: Cannot get nearest point from path.";
    return false;
  }
  sl_point->set_s(s);
  sl_point->set_l(l);
  sl_point->set_proj_idx(min_idx);
  proto::LiteTrajectoryPoint proj_pt;
  const auto& seg = line_segments_[min_idx];
  proj_pt.set_x(seg.center().x());
  proj_pt.set_y(seg.center().y());
  proj_pt.set_z(0.0);
  proj_pt.set_theta(seg.heading());
  *sl_point->mutable_proj_pt() = proj_pt;
  return true;
}

/**
 * @brief 将XY坐标转换为SL坐标（带参考s值和搜索范围）
 * @details 根据给定的XY坐标点、参考s值和搜索范围，计算其在参考线上的SL坐标
 *
 * @param[in] xyz_point 输入的XY坐标点
 * @param[in] ref_s 参考s值，用于确定搜索范围的起点
 * @param[in] search_range_idx 搜索范围索引，包含前向和后向搜索范围
 * @param[out] sl_point 输出的SL坐标点
 * @return bool 返回转换是否成功
 *
 * @par 关键变量说明:
 * - xyz_point (math::Vec3d): 输入的XY坐标点，包含x、y、z值
 * - ref_s (double): 参考s值，用于确定搜索范围的起点
 * - search_range_idx (std::pair<size_t, size_t>): 搜索范围索引，包含前向和后向搜索范围
 * - sl_point (SLPoint*): 输出的SL坐标点，包含s值、l值和投影点索引
 * - s (double): 参考线上的s值
 * - l (double): 参考线上的l值（横向偏移）
 * - min_idx (int): 最近线段的索引
 *
 * @par 处理逻辑:
 * - 检查输出指针是否为空，如果为空则返回失败
 * - 调用getProjection函数，计算XY坐标点在参考线上的投影
 * - 如果投影失败，则返回失败
 * - 设置SL坐标点的s值、l值和投影点索引
 * - 设置SL坐标点的投影点信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输出指针是否为空;
 * if (输出指针为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :调用getProjection函数计算投影;
 *   if (投影失败?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :设置SL坐标点的s值、l值和投影点索引;
 *     :设置SL坐标点的投影点信息;
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于将XY坐标转换为SL坐标，适用于需要根据参考线进行坐标转换的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确进行坐标转换
 */
bool ReferenceLine::xy2sl(const math::Vec3d& xyz_point, const double ref_s,
                          const std::pair<size_t, size_t>& search_range_idx, SLPoint* const sl_point) const {
  CHECK_NOTNULL(sl_point);
  double s = 0.0;
  double l = 0.0;
  int min_idx = -1;
  if (!getProjection(xyz_point, ref_s, search_range_idx, s, l, min_idx)) {
    ERT_PLOG_I << "[ReferenceLine::xy2sl]: Cannot get nearest point from path.";
    return false;
  }
  // ERT_PLOG_I << "corners_final_s : " << s << " corners_final_l : " << l ;
  sl_point->set_s(s);
  sl_point->set_l(l);
  sl_point->set_proj_idx(min_idx);
  proto::LiteTrajectoryPoint proj_pt;
  const auto& seg = line_segments_[min_idx];
  proj_pt.set_x(seg.center().x());
  proj_pt.set_y(seg.center().y());
  proj_pt.set_z(0.0);
  proj_pt.set_theta(seg.heading());
  *sl_point->mutable_proj_pt() = proj_pt;
  return true;
}

/**
 * @brief 获取多边形的SL边界
 * @details 根据给定的多边形顶点，计算其在参考线上的SL边界
 *
 * @param[in] corners 输入的多边形顶点
 * @param[out] sl_boundary 输出的SL边界
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - corners (std::vector<math::Vec2d>): 输入的多边形顶点
 * - sl_boundary (gpal::pnc::SLBoundary*): 输出的SL边界，包含起始s值、结束s值、起始l值和结束l值
 * - corners_3d (std::vector<math::Vec3d>): 将二维顶点转换为三维顶点
 *
 * @par 处理逻辑:
 * - 将输入的二维顶点转换为三维顶点
 * - 调用getSLBoundary(const std::vector<math::Vec3d>& corners, gpal::pnc::SLBoundary* const sl_boundary)函数进行计算
 *
 * @par 流程图:
 * @startuml
 * start
 * :将二维顶点转换为三维顶点;
 * :调用getSLBoundary函数计算SL边界;
 * :返回计算结果;
 * stop
 * @enduml
 *
 * @note 该函数用于计算多边形在参考线上的SL边界，适用于需要根据多边形进行边界计算的场景
 *
 * @warning 需确保输入的多边形顶点有效，否则可能无法正确计算SL边界
 */
bool ReferenceLine::getSLBoundary(const std::vector<math::Vec2d>& corners,
                                  gpal::pnc::SLBoundary* const sl_boundary) const {
  std::vector<math::Vec3d> corners_3d;
  for (const auto& pt : corners) {
    corners_3d.emplace_back(math::Vec3d(pt, 0.0));
  }
  return getSLBoundary(corners_3d, sl_boundary);
}

/**
 * @brief 获取多边形的SL边界
 * @details 根据给定的多边形顶点，计算其在参考线上的SL边界
 *
 * @param[in] corners 输入的多边形顶点
 * @param[out] sl_boundary 输出的SL边界
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - corners (std::vector<math::Vec3d>): 输入的多边形顶点
 * - sl_boundary (gpal::pnc::SLBoundary*): 输出的SL边界，包含起始s值、结束s值、起始l值和结束l值
 * - start_s (double): 重叠范围的起始s值
 * - end_s (double): 重叠范围的结束s值
 * - start_l (double): 重叠范围的起始l值
 * - end_l (double): 重叠范围的结束l值
 * - sl_corners (std::vector<SLPoint>): 存储多边形顶点在参考线上的投影点
 *
 * @par 处理逻辑:
 * - 初始化起始s值、结束s值、起始l值和结束l值
 * - 遍历多边形的每个顶点，计算其在参考线上的投影点
 * - 如果投影失败，则返回失败
 * - 将投影点添加到sl_corners中
 * - 遍历多边形的每条边，计算其中点在参考线上的投影点
 * - 如果投影点位于多边形外部，则将其添加到边界点列表中
 * - 根据边界点列表，计算最终的起始s值、结束s值、起始l值和结束l值
 * - 设置SL边界的起始s值、结束s值、起始l值和结束l值
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化起始s值、结束s值、起始l值和结束l值;
 * :遍历多边形的每个顶点;
 * if (投影失败?) then (yes)
 *   :返回失败;
 * else (no)
 *   :将投影点添加到sl_corners中;
 * endif
 * :遍历多边形的每条边;
 * :计算中点在参考线上的投影点;
 * if (投影点位于多边形外部?) then (yes)
 *   :将其添加到边界点列表中;
 * endif
 * :根据边界点列表计算最终的起始s值、结束s值、起始l值和结束l值;
 * :设置SL边界的起始s值、结束s值、起始l值和结束l值;
 * :返回成功;
 * stop
 * @enduml
 *
 * @note 该函数用于计算多边形在参考线上的SL边界，适用于需要根据多边形进行边界计算的场景
 *
 * @warning 需确保输入的多边形顶点有效，否则可能无法正确计算SL边界
 */
bool ReferenceLine::getSLBoundary(const std::vector<math::Vec3d>& corners,
                                  gpal::pnc::SLBoundary* const sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  // The order must be counter-clockwise
  std::vector<SLPoint> sl_corners;
  for (const auto& point : corners) {
    SLPoint sl_point;
    if (!xy2sl(point, &sl_point)) {
      ERT_PLOG_I << "[ReferenceLine::getSLBoundary]: Failed to get projection for point: " << point.DebugString()
                 << " on reference line.";
      return false;
    }
    sl_corners.push_back(std::move(sl_point));
  }
  const auto corners_size = corners.size();
  for (size_t i = 0; i < corners_size; ++i) {
    auto index0 = i;
    auto index1 = (i + 1) % corners_size;
    const auto& p0 = corners[index0];
    const auto& p1 = corners[index1];

    const auto p_mid = (p0 + p1) * 0.5;
    SLPoint sl_point_mid;
    if (!xy2sl(p_mid, &sl_point_mid)) {
      ERT_PLOG_I << "[ReferenceLine::getSLBoundary]: Failed to get projection for point: " << p_mid.DebugString()
                 << " on reference line.";
      return false;
    }

    math::Vec2d v0(sl_corners[index1].s() - sl_corners[index0].s(), sl_corners[index1].l() - sl_corners[index0].l());

    math::Vec2d v1(sl_point_mid.s() - sl_corners[index0].s(), sl_point_mid.l() - sl_corners[index0].l());

    *sl_boundary->add_boundary_point() = sl_corners[index0];

    // sl_point is outside of polygon; add to the vertex list
    if (v0.CrossProd(v1) < 0.0) {
      *sl_boundary->add_boundary_point() = sl_point_mid;
    }
  }

  for (const auto& sl_point : sl_boundary->boundary_point()) {
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }

  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}

/**
 * @brief 获取矩形框的SL边界
 * @details 根据给定的矩形框，计算其在参考线上的SL边界
 *
 * @param[in] box 输入的矩形框
 * @param[out] sl_boundary 输出的SL边界
 * @param[in] ref_s 参考s值，用于确定搜索范围的起点
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - box (math::Box2d): 输入的矩形框
 * - sl_boundary (gpal::pnc::SLBoundary*): 输出的SL边界，包含起始s值、结束s值、起始l值和结束l值
 * - ref_s (double): 参考s值，用于确定搜索范围的起点
 * - corners (std::vector<math::Vec2d>): 矩形框的所有角点
 * - sl_point_front (SLPoint): 矩形框前部角点在参考线上的投影点
 * - diff_s (double): 参考线上相邻点之间的s值差值
 * - factor (double): 搜索范围的缩放因子
 * - box_length (double): 矩形框的长度
 * - forward_search_range (double): 前向搜索范围
 * - back_search_range (double): 后向搜索范围
 * - search_range_idx (std::pair<size_t, size_t>): 搜索范围索引，包含前向和后向搜索范围
 * - start_s (double): 重叠范围的起始s值
 * - end_s (double): 重叠范围的结束s值
 * - start_l (double): 重叠范围的起始l值
 * - end_l (double): 重叠范围的结束l值
 * - sl_corners (std::vector<SLPoint>): 存储矩形框角点在参考线上的投影点
 *
 * @par 处理逻辑:
 * - 获取矩形框的所有角点
 * - 如果角点为空，则返回失败
 * - 计算参考线上相邻点之间的s值差值
 * - 根据矩形框的长度和s值差值，计算搜索范围的缩放因子
 * - 根据缩放因子和矩形框的长度，计算前向和后向搜索范围
 * - 根据参考s值和搜索范围，计算矩形框前部角点在参考线上的投影点
 * - 将前部角点的投影点添加到SL边界中
 * - 遍历矩形框的其他角点，计算其在参考线上的投影点
 * - 如果矩形框长度大于6.0，则计算每条边的中点在参考线上的投影点
 * - 根据投影点列表，计算最终的起始s值、结束s值、起始l值和结束l值
 * - 设置SL边界的起始s值、结束s值、起始l值和结束l值
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取矩形框的所有角点;
 * if (角点为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :计算参考线上相邻点之间的s值差值;
 *   :计算搜索范围的缩放因子;
 *   :计算前向和后向搜索范围;
 *   :计算矩形框前部角点在参考线上的投影点;
 *   :将前部角点的投影点添加到SL边界中;
 *   :遍历矩形框的其他角点;
 *   if (矩形框长度 > 6.0?) then (yes)
 *     :计算每条边的中点在参考线上的投影点;
 *   endif
 *   :根据投影点列表计算最终的起始s值、结束s值、起始l值和结束l值;
 *   :设置SL边界的起始s值、结束s值、起始l值和结束l值;
 *   :返回成功;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算矩形框在参考线上的SL边界，适用于需要根据矩形框进行边界计算的场景
 *
 * @warning 需确保输入的矩形框有效，否则可能无法正确计算SL边界
 */
bool ReferenceLine::getSLBoundary(const math::Box2d& box, gpal::pnc::SLBoundary* const sl_boundary,
                                  double ref_s) const {
  const auto& corners = box.GetAllCorners();
  if (corners.empty()) {
    return false;
  }
  SLPoint sl_point_front;

  double diff_s = 0.01;
  if (accumulated_s_.size() > 2) {
    diff_s = std::max(0.01, (accumulated_s_[1] - accumulated_s_[0]));
  }
  double factor = std::max(1.0, 1.0 / diff_s);

  const double box_length = box.length();
  const double forward_search_range = box_length * factor;
  const double back_search_range = box_length * 2.0 * factor;
  const std::pair<size_t, size_t> search_range_idx =
      std::make_pair(static_cast<size_t>(back_search_range), static_cast<size_t>(forward_search_range));
  if (ref_s > 5000.0) {
    if (!xy2sl(math::Vec3d(corners[0], 0.0), &sl_point_front)) {
      return false;
    }
  } else {
    if (!xy2sl(math::Vec3d(corners[0], 0.0), ref_s, search_range_idx, &sl_point_front)) {
      return false;
    }
  }

  *sl_boundary->add_boundary_point() = sl_point_front;

  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  // set search range
  ref_s = sl_point_front.s();
  // ERT_PLOG_I << "corners_ref_s : " << ref_s ;

  // The order must be counter-clockwise
  std::vector<SLPoint> sl_corners;
  for (size_t i = 1; i < corners.size(); ++i) {
    SLPoint sl_point;
    if (!xy2sl(math::Vec3d(corners[i], 0.0), ref_s, search_range_idx, &sl_point)) {
      ERT_PLOG_I << "[ReferenceLine::getSLBoundary]: Failed to get projection for point: " << corners[i].DebugString()
                 << " on reference line.";
      return false;
    }
    // ERT_PLOG_I << "corners_search_range_idx : " << search_range_idx;
    *sl_boundary->add_boundary_point() = sl_point;
    sl_corners.emplace_back(std::move(sl_point));
  }
  if (box_length > 6.0) {
    const auto corners_size = corners.size();
    for (size_t i = 0; i < corners_size; ++i) {
      auto index0 = i;
      auto index1 = (i + 1) % corners_size;
      const auto& p0 = corners[index0];
      const auto& p1 = corners[index1];

      const auto p_mid = (p0 + p1) * 0.5;
      SLPoint sl_point_mid;
      if (!xy2sl(math::Vec3d(p_mid, 0.0), ref_s, search_range_idx, &sl_point_mid)) {
        ERT_PLOG_I << "[ReferenceLine::getSLBoundary]: Failed to get projection for point: " << p_mid.DebugString()
                   << " on reference line.";
        return false;
      }

      math::Vec2d v0(sl_corners[index1].s() - sl_corners[index0].s(), sl_corners[index1].l() - sl_corners[index0].l());

      math::Vec2d v1(sl_point_mid.s() - sl_corners[index0].s(), sl_point_mid.l() - sl_corners[index0].l());

      // sl_point is outside of polygon; add to the vertex list
      if (v0.CrossProd(v1) < 0.0) {
        *sl_boundary->add_boundary_point() = sl_point_mid;
      }
    }
  }

  for (const auto& sl_point : sl_boundary->boundary_point()) {
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }

  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);

  return true;
}

/**
 * @brief 计算点在参考线上的投影
 * @details 根据给定的点，计算其在参考线上的投影，并返回投影点的s值和横向偏移
 *
 * @param[in] point 输入的点
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - accumulate_s (double): 输出的投影点的s值
 * - lateral (double): 输出的投影点的横向偏移
 * - distance (double): 点到参考线的最小距离
 * - min_index (int): 最近线段的索引
 *
 * @par 处理逻辑:
 * - 初始化最小距离和最近线段索引
 * - 调用getProjection(const math::Vec3d& point, double& accumulate_s, double& lateral, double& min_distance, int&
 * min_index)函数进行计算
 * - 返回计算结果
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化最小距离和最近线段索引;
 * :调用getProjection函数计算投影;
 * :返回计算结果;
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线上的投影，适用于需要根据参考线进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getProjection(const math::Vec3d& point, double& accumulate_s, double& lateral) const {
  double distance = 0.0;
  int min_index = -1;
  return getProjection(point, accumulate_s, lateral, distance, min_index);
}

/**
 * @brief 计算点在参考线上的投影（带航向角）
 * @details 根据给定的点和航向角，计算其在参考线上的投影，并返回投影点的s值和横向偏移
 *
 * @param[in] point 输入的点
 * @param[in] heading 输入的航向角
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - heading (float): 输入的航向角
 * - accumulate_s (double): 输出的投影点的s值
 * - lateral (double): 输出的投影点的横向偏移
 * - distance (double): 点到参考线的最小距离
 * - min_index (int): 最近线段的索引
 *
 * @par 处理逻辑:
 * - 初始化最小距离和最近线段索引
 * - 调用getProjection(const math::Vec3d& point, const float& heading, double& accumulate_s, double& lateral, double&
 * min_distance, int& min_index)函数进行计算
 * - 返回计算结果
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化最小距离和最近线段索引;
 * :调用getProjection函数计算投影;
 * :返回计算结果;
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线上的投影（带航向角），适用于需要根据参考线和航向角进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getProjection(const math::Vec3d& point, const float& heading, double& accumulate_s,
                                  double& lateral) const {
  double distance = 0.0;
  int min_index = -1;
  return getProjection(point, heading, accumulate_s, lateral, distance, min_index);
}

/**
 * @brief 计算点在参考线上的投影
 * @details 根据给定的点，计算其在参考线上的投影，并返回投影点的s值、横向偏移和最近线段索引
 *
 * @param[in] point 输入的点
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @param[out] min_index 输出的最近线段的索引
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - accumulate_s (double): 输出的投影点的s值
 * - lateral (double): 输出的投影点的横向偏移
 * - min_index (int&): 输出的最近线段的索引
 * - distance (double): 点到参考线的最小距离
 *
 * @par 处理逻辑:
 * - 初始化最小距离
 * - 调用getProjection(const math::Vec3d& point, double& accumulate_s, double& lateral, double& min_distance, int&
 * min_index)函数进行计算
 * - 返回计算结果
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化最小距离;
 * :调用getProjection函数计算投影;
 * :返回计算结果;
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线上的投影，适用于需要根据参考线进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getProjection(const math::Vec3d& point, double& accumulate_s, double& lateral,
                                  int& min_index) const {
  double distance = 0.0;
  return getProjection(point, accumulate_s, lateral, distance, min_index);
}

/**
 * @brief 计算点在参考线上的投影（带航向角）
 * @details 根据给定的点和航向角，计算其在参考线上的投影，并返回投影点的s值、横向偏移和最近线段索引
 *
 * @param[in] point 输入的点
 * @param[in] heading 输入的航向角
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @param[out] min_index 输出的最近线段的索引
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - heading (float): 输入的航向角
 * - accumulate_s (double): 输出的投影点的s值
 * - lateral (double): 输出的投影点的横向偏移
 * - min_index (int&): 输出的最近线段的索引
 * - distance (double): 点到参考线的最小距离
 *
 * @par 处理逻辑:
 * - 初始化最小距离
 * - 调用getProjection(const math::Vec3d& point, const float& heading, double& accumulate_s, double& lateral, double&
 * min_distance, int& min_index)函数进行计算
 * - 返回计算结果
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化最小距离;
 * :调用getProjection函数计算投影;
 * :返回计算结果;
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线上的投影（带航向角），适用于需要根据参考线和航向角进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getProjection(const math::Vec3d& point, const float& heading, double& accumulate_s, double& lateral,
                                  int& min_index) const {
  double distance = 0.0;
  return getProjection(point, heading, accumulate_s, lateral, distance, min_index);
}

/**
 * @brief 计算点在参考线上的投影
 * @details 根据给定的点，计算其在参考线上的投影，并返回投影点的s值、横向偏移、最小距离和最近线段索引
 *
 * @param[in] point 输入的点
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @param[out] min_distance 输出的点到参考线的最小距离
 * @param[out] min_index 输出的最近线段的索引
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - accumulate_s (double&): 输出的投影点的s值
 * - lateral (double&): 输出的投影点的横向偏移
 * - min_distance (double&): 输出的点到参考线的最小距离
 * - min_index (int&): 输出的最近线段的索引
 * - line_segments_ (std::vector<math::LineSegment2d>): 参考线的线段集
 * - num_line_segments_ (int): 参考线的线段数量
 *
 * @par 处理逻辑:
 * - 检查参考线的线段集是否为空，如果为空则返回失败
 * - 初始化最小距离为无穷大
 * - 遍历参考线的每个线段，计算点到线段的最小距离
 * - 如果找到更小的距离，则更新最小距离和最近线段索引
 * - 如果最近线段索引无效，则返回失败
 * - 计算点到最近线段的投影点，并设置s值、横向偏移和最小距离
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考线的线段集是否为空;
 * if (线段集为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :初始化最小距离为无穷大;
 *   :遍历参考线的每个线段;
 *   :计算点到线段的最小距离;
 *   if (找到更小的距离?) then (yes)
 *     :更新最小距离和最近线段索引;
 *   endif
 *   if (最近线段索引无效?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :计算点到最近线段的投影点;
 *     :设置s值、横向偏移和最小距离;
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线上的投影，适用于需要根据参考线进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getProjection(const math::Vec3d& point, double& accumulate_s, double& lateral, double& min_distance,
                                  int& min_index) const {
  if (line_segments_.empty()) {
    return false;
  }

  min_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_line_segments_; ++i) {
    const double distance = line_segments_[i].DistanceSquareTo(point);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  if (min_index < 0) {
    return false;
  }
  min_distance = std::sqrt(min_distance);
  calcProjection(point, min_distance, min_index, accumulate_s, lateral);
  // ERT_PLOG_I << "start_index = " << start_index
  //           << "  end_index = " << end_index
  //           << "  num_line_segments_ = " << num_line_segments_
  //           << "  min_index = " << min_index
  //           << "  min_distance = " << *min_distance
  //           << "  nearest_seg.length() = " << nearest_seg.length()
  //           << "  accumulate_s = " << *accumulate_s
  //           << "  lateral = " << *lateral
  //           << "  prod = " << prod
  //           << "  proj = " << proj
  //           ;
  return true;
}

/**
 * @brief 计算点在参考线上的投影（带航向角）
 * @details 根据给定的点和航向角，计算其在参考线上的投影，并返回投影点的s值、横向偏移、最小距离和最近线段索引
 *
 * @param[in] point 输入的点
 * @param[in] heading 输入的航向角
 * @param[in] heading_tolerance 输入的航向角容差
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @param[out] min_distance 输出的点到参考线的最小距离
 * @param[out] min_index 输出的最近线段的索引
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - heading (float): 输入的航向角
 * - heading_tolerance (float): 输入的航向角容差
 * - accumulate_s (double&): 输出的投影点的s值
 * - lateral (double&): 输出的投影点的横向偏移
 * - min_distance (double&): 输出的点到参考线的最小距离
 * - min_index (int&): 输出的最近线段的索引
 * - line_segments_ (std::vector<math::LineSegment2d>): 参考线的线段集
 * - num_line_segments_ (int): 参考线的线段数量
 *
 * @par 处理逻辑:
 * - 检查参考线的线段集是否为空，如果为空则返回失败
 * - 初始化最小距离为无穷大
 * - 遍历参考线的每个线段，计算点到线段的最小距离（考虑航向角）
 * - 如果找到更小的距离，则更新最小距离和最近线段索引
 * - 如果最近线段索引无效，则返回失败
 * - 计算点到最近线段的投影点，并设置s值、横向偏移和最小距离
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考线的线段集是否为空;
 * if (线段集为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :初始化最小距离为无穷大;
 *   :遍历参考线的每个线段;
 *   :计算点到线段的最小距离（考虑航向角）;
 *   if (找到更小的距离?) then (yes)
 *     :更新最小距离和最近线段索引;
 *   endif
 *   if (最近线段索引无效?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :计算点到最近线段的投影点;
 *     :设置s值、横向偏移和最小距离;
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线上的投影（带航向角），适用于需要根据参考线和航向角进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getProjection(const math::Vec3d& point, const float& heading, const float& heading_tolerance,
                                  double& accumulate_s, double& lateral, double& min_distance, int& min_index) const {
  if (line_segments_.empty()) {
    return false;
  }

  min_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_line_segments_; ++i) {
    const double distance = line_segments_[i].DistanceSquareTo(point, heading, heading_tolerance, 100.0);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  if (min_index < 0) {
    return false;
  }
  min_distance = std::sqrt(min_distance);
  calcProjection(point, min_distance, min_index, accumulate_s, lateral);
  // ERT_PLOG_I << "start_index = " << start_index
  //           << "  end_index = " << end_index
  //           << "  num_line_segments_ = " << num_line_segments_
  //           << "  min_index = " << min_index
  //           << "  min_distance = " << *min_distance
  //           << "  nearest_seg.length() = " << nearest_seg.length()
  //           << "  accumulate_s = " << *accumulate_s
  //           << "  lateral = " << *lateral
  //           << "  prod = " << prod
  //           << "  proj = " << proj
  //           ;
  return true;
}

/**
 * @brief 计算点在参考线上的投影（带航向角）
 * @details 根据给定的点和航向角，计算其在参考线上的投影，并返回投影点的s值、横向偏移、最小距离和最近线段索引
 *
 * @param[in] point 输入的点
 * @param[in] heading 输入的航向角
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @param[out] min_distance 输出的点到参考线的最小距离
 * @param[out] min_index 输出的最近线段的索引
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - heading (float): 输入的航向角
 * - accumulate_s (double&): 输出的投影点的s值
 * - lateral (double&): 输出的投影点的横向偏移
 * - min_distance (double&): 输出的点到参考线的最小距离
 * - min_index (int&): 输出的最近线段的索引
 * - line_segments_ (std::vector<math::LineSegment2d>): 参考线的线段集
 * - num_line_segments_ (int): 参考线的线段数量
 *
 * @par 处理逻辑:
 * - 检查参考线的线段集是否为空，如果为空则返回失败
 * - 初始化最小距离为无穷大
 * - 遍历参考线的每个线段，计算点到线段的最小距离（考虑航向角）
 * - 如果找到更小的距离，则更新最小距离和最近线段索引
 * - 如果最近线段索引无效，则返回失败
 * - 计算点到最近线段的投影点，并设置s值、横向偏移和最小距离
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考线的线段集是否为空;
 * if (线段集为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :初始化最小距离为无穷大;
 *   :遍历参考线的每个线段;
 *   :计算点到线段的最小距离（考虑航向角）;
 *   if (找到更小的距离?) then (yes)
 *     :更新最小距离和最近线段索引;
 *   endif
 *   if (最近线段索引无效?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :计算点到最近线段的投影点;
 *     :设置s值、横向偏移和最小距离;
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线上的投影（带航向角），适用于需要根据参考线和航向角进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getProjection(const math::Vec3d& point, const float& heading, double& accumulate_s, double& lateral,
                                  double& min_distance, int& min_index) const {
  if (line_segments_.empty()) {
    return false;
  }

  min_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_line_segments_; ++i) {
    const double distance = line_segments_[i].DistanceSquareTo(point, heading, 90.0, 100.0);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  if (min_index < 0) {
    return false;
  }
  min_distance = std::sqrt(min_distance);
  calcProjection(point, min_distance, min_index, accumulate_s, lateral);
  // ERT_PLOG_I << "start_index = " << start_index
  //           << "  end_index = " << end_index
  //           << "  num_line_segments_ = " << num_line_segments_
  //           << "  min_index = " << min_index
  //           << "  min_distance = " << *min_distance
  //           << "  nearest_seg.length() = " << nearest_seg.length()
  //           << "  accumulate_s = " << *accumulate_s
  //           << "  lateral = " << *lateral
  //           << "  prod = " << prod
  //           << "  proj = " << proj
  //           ;
  return true;
}

/**
 * @brief 计算点在参考线上的投影（带参考s值和搜索范围）
 * @details 根据给定的点、参考s值和搜索范围，计算其在参考线上的投影，并返回投影点的s值和横向偏移
 *
 * @param[in] point 输入的点
 * @param[in] ref_s 参考s值，用于确定搜索范围的起点
 * @param[in] search_range_idx 搜索范围索引，包含前向和后向搜索范围
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - ref_s (double): 参考s值，用于确定搜索范围的起点
 * - search_range_idx (std::pair<size_t, size_t>): 搜索范围索引，包含前向和后向搜索范围
 * - accumulate_s (double&): 输出的投影点的s值
 * - lateral (double&): 输出的投影点的横向偏移
 * - distance (double): 点到参考线的最小距离
 * - min_index (int): 最近线段的索引
 *
 * @par 处理逻辑:
 * - 初始化最小距离和最近线段索引
 * - 调用getProjection(const math::Vec3d& point, const double ref_s, const std::pair<size_t, size_t>& search_range_idx,
 * double& accumulate_s, double& lateral, double& min_distance, int& min_index)函数进行计算
 * - 返回计算结果
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化最小距离和最近线段索引;
 * :调用getProjection函数计算投影;
 * :返回计算结果;
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线上的投影（带参考s值和搜索范围），适用于需要根据参考线和搜索范围进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getProjection(const math::Vec3d& point, const double ref_s,
                                  const std::pair<size_t, size_t>& search_range_idx, double& accumulate_s,
                                  double& lateral) const {
  double distance = 0.0;
  int min_index = -1;
  return getProjection(point, ref_s, search_range_idx, accumulate_s, lateral, distance, min_index);
}

/**
 * @brief 计算点在参考线上的投影（带参考s值和搜索范围）
 * @details 根据给定的点、参考s值和搜索范围，计算其在参考线上的投影，并返回投影点的s值、横向偏移和最近线段索引
 *
 * @param[in] point 输入的点
 * @param[in] ref_s 参考s值，用于确定搜索范围的起点
 * @param[in] search_range_idx 搜索范围索引，包含前向和后向搜索范围
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @param[out] min_index 输出的最近线段的索引
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - ref_s (double): 参考s值，用于确定搜索范围的起点
 * - search_range_idx (std::pair<size_t, size_t>): 搜索范围索引，包含前向和后向搜索范围
 * - accumulate_s (double&): 输出的投影点的s值
 * - lateral (double&): 输出的投影点的横向偏移
 * - min_index (int&): 输出的最近线段的索引
 * - distance (double): 点到参考线的最小距离
 * - start_index (size_t): 搜索范围的起始索引
 * - end_index (size_t): 搜索范围的结束索引
 * - ref_idx (size_t): 参考s值对应的最近线段索引
 *
 * @par 处理逻辑:
 * - 检查参考线的线段集是否为空，如果为空则返回失败
 * - 初始化搜索范围的起始索引和结束索引
 * - 如果参考s值小于5000.0，则根据参考s值确定搜索范围的起始索引和结束索引
 * - 确保搜索范围的起始索引小于结束索引
 * - 初始化最小距离为无穷大
 * - 遍历搜索范围内的每个线段，计算点到线段的最小距离
 * - 如果找到更小的距离，则更新最小距离和最近线段索引
 * - 如果最近线段索引无效，则返回失败
 * - 计算点到最近线段的投影点，并设置s值、横向偏移和最小距离
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考线的线段集是否为空;
 * if (线段集为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :初始化搜索范围的起始索引和结束索引;
 *   if (参考s值 < 5000.0?) then (yes)
 *     :根据参考s值确定搜索范围的起始索引和结束索引;
 *   endif
 *   :确保搜索范围的起始索引小于结束索引;
 *   :初始化最小距离为无穷大;
 *   :遍历搜索范围内的每个线段;
 *   :计算点到线段的最小距离;
 *   if (找到更小的距离?) then (yes)
 *     :更新最小距离和最近线段索引;
 *   endif
 *   if (最近线段索引无效?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :计算点到最近线段的投影点;
 *     :设置s值、横向偏移和最小距离;
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线上的投影（带参考s值和搜索范围），适用于需要根据参考线和搜索范围进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getProjection(const math::Vec3d& point, const double ref_s,
                                  const std::pair<size_t, size_t>& search_range_idx, double& accumulate_s,
                                  double& lateral, int& min_index) const {
  double distance = 0.0;
  return getProjection(point, ref_s, search_range_idx, accumulate_s, lateral, distance, min_index);
}

/**
 * @brief 计算点在参考线上的投影（带参考s值和搜索范围）
 * @details 根据给定的点、参考s值和搜索范围，计算其在参考线上的投影，并返回投影点的s值、横向偏移、最小距离和最近线段索引
 *
 * @param[in] point 输入的点
 * @param[in] ref_s 参考s值，用于确定搜索范围的起点
 * @param[in] search_range_idx 搜索范围索引，包含前向和后向搜索范围
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @param[out] min_distance 输出的点到参考线的最小距离
 * @param[out] min_index 输出的最近线段的索引
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - ref_s (double): 参考s值，用于确定搜索范围的起点
 * - search_range_idx (std::pair<size_t, size_t>): 搜索范围索引，包含前向和后向搜索范围
 * - accumulate_s (double&): 输出的投影点的s值
 * - lateral (double&): 输出的投影点的横向偏移
 * - min_distance (double&): 输出的点到参考线的最小距离
 * - min_index (int&): 输出的最近线段的索引
 * - start_index (size_t): 搜索范围的起始索引
 * - end_index (size_t): 搜索范围的结束索引
 * - ref_idx (size_t): 参考s值对应的最近线段索引
 *
 * @par 处理逻辑:
 * - 检查参考线的线段集是否为空，如果为空则返回失败
 * - 初始化搜索范围的起始索引和结束索引
 * - 如果参考s值小于5000.0，则根据参考s值确定搜索范围的起始索引和结束索引
 * - 确保搜索范围的起始索引小于结束索引
 * - 初始化最小距离为无穷大
 * - 遍历搜索范围内的每个线段，计算点到线段的最小距离
 * - 如果找到更小的距离，则更新最小距离和最近线段索引
 * - 如果最近线段索引无效，则返回失败
 * - 计算点到最近线段的投影点，并设置s值、横向偏移和最小距离
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考线的线段集是否为空;
 * if (线段集为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :初始化搜索范围的起始索引和结束索引;
 *   if (参考s值 < 5000.0?) then (yes)
 *     :根据参考s值确定搜索范围的起始索引和结束索引;
 *   endif
 *   :确保搜索范围的起始索引小于结束索引;
 *   :初始化最小距离为无穷大;
 *   :遍历搜索范围内的每个线段;
 *   :计算点到线段的最小距离;
 *   if (找到更小的距离?) then (yes)
 *     :更新最小距离和最近线段索引;
 *   endif
 *   if (最近线段索引无效?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :计算点到最近线段的投影点;
 *     :设置s值、横向偏移和最小距离;
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线上的投影（带参考s值和搜索范围），适用于需要根据参考线和搜索范围进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getProjection(const math::Vec3d& point, const double ref_s,
                                  const std::pair<size_t, size_t>& search_range_idx, double& accumulate_s,
                                  double& lateral, double& min_distance, int& min_index) const {
  if (line_segments_.empty()) {
    return false;
  }

  size_t start_index = 0;
  size_t end_index = num_line_segments_ - 1;
  if (ref_s < 5000.0) {
    size_t ref_idx = getNearestReferenceIndex(ref_s);
    if (ref_idx > search_range_idx.first) {
      start_index = std::max((ref_idx - search_range_idx.first), start_index);
    }
    // ERT_PLOG_I<<"corners_ref_s = "<<ref_s<<" ref_idx = "<<ref_idx<<" start_index = "<<start_index;
    end_index = std::min((ref_idx + search_range_idx.second), end_index);
    // ERT_PLOG_I<<"aeb_ref_idx = "<<ref_idx<<" search_range_idx.second = "<<search_range_idx.second<<" end_index =
    // "<<end_index<<" start_index = "<<start_index
    // ;
  }
  if (start_index >= end_index) {
    start_index = end_index - 1;
  }

  min_distance = std::numeric_limits<double>::infinity();
  // ERT_PLOG_I<<"corners_min_distance_1 = "<<min_distance;
  // ERT_PLOG_I<<"corners_start_index = "<<start_index<<" end_index = "<<end_index <<" num_line_segments_ =
  // "<<num_line_segments_;
  for (int i = start_index; i <= std::min<int>(end_index, num_line_segments_ - 1); ++i) {
    const double distance = line_segments_[i].DistanceSquareTo(point);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
    // ERT_PLOG_I<<"corners_min_distance_2 = "<<min_distance<<" min_index = "<<min_index<<" distance"<<distance;
  }
  if (min_index < 0) {
    return false;
  }
  min_distance = std::sqrt(min_distance);
  calcProjection(point, min_distance, min_index, accumulate_s, lateral);
  return true;
}

/**
 * @brief 计算点到最近线段的投影点
 * @details 根据给定的点、最小距离和最近线段索引，计算其在参考线上的投影点，并设置s值和横向偏移
 *
 * @param[in] point 输入的点
 * @param[in] min_distance 点到参考线的最小距离
 * @param[in] min_index 最近线段的索引
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - min_distance (double): 点到参考线的最小距离
 * - min_index (int): 最近线段的索引
 * - accumulate_s (double&): 输出的投影点的s值
 * - lateral (double&): 输出的投影点的横向偏移
 * - nearest_seg (math::LineSegment2d): 最近线段
 * - prod (double): 点到最近线段的单位向量的点积
 * - proj (double): 点到最近线段的投影长度
 *
 * @par 处理逻辑:
 * - 获取最近线段
 * - 计算点到最近线段的单位向量的点积和投影长度
 * - 根据最近线段的位置（起点、终点或中间）计算s值和横向偏移
 * - 如果投影点在最近线段的起点之前，则横向偏移为点到最近线段的点积
 * - 如果投影点在最近线段的终点之后，则横向偏移为点到最近线段的点积
 * - 如果投影点在最近线段的中间，则横向偏移为最小距离的符号乘以最小距离
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取最近线段;
 * :计算点到最近线段的单位向量的点积和投影长度;
 * if (最近线段是起点?) then (yes)
 *   :计算s值为投影长度和线段长度的最小值;
 *   if (投影点在起点之前?) then (yes)
 *     :横向偏移为点到最近线段的点积;
 *   else (no)
 *     :横向偏移为最小距离的符号乘以最小距离;
 *   endif
 * else if (最近线段是终点?) then (yes)
 *   :计算s值为累计s值加上投影长度;
 *   if (投影点在终点之后?) then (yes)
 *     :横向偏移为点到最近线段的点积;
 *   else (no)
 *     :横向偏移为最小距离的符号乘以最小距离;
 *   endif
 * else (no)
 *   :计算s值为累计s值加上投影长度和线段长度的最小值;
 *   :横向偏移为最小距离的符号乘以最小距离;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算点到最近线段的投影点，适用于需要根据参考线进行投影计算的场景
 *
 * @warning 需确保最近线段索引有效，否则可能无法正确计算投影点
 */
void ReferenceLine::calcProjection(const math::Vec3d& point, const double min_distance, const int min_index,
                                   double& accumulate_s, double& lateral) const {
  const auto& nearest_seg = line_segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    // ERT_PLOG_I<<"corners_1 = ";
    accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      lateral = prod;
    } else {
      lateral = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else if (min_index == num_line_segments_ - 1) {
    // ERT_PLOG_I<<"corners_2 = ";
    accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      lateral = prod;
    } else {
      lateral = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else {
    // ERT_PLOG_I<<"corners_3 = ";
    accumulate_s = accumulated_s_[min_index] + std::max(0.0, std::min(proj, nearest_seg.length()));
    lateral = (prod > 0.0 ? 1 : -1) * min_distance;
  }
  // ERT_PLOG_I<<"corners_lateral = "<<lateral;
}

/**
 * @brief 计算点在参考线前部的投影
 * @details 根据给定的点，计算其在参考线前部的投影，并返回投影点的s值和横向偏移
 *
 * @param[in] point 输入的点
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - accumulate_s (double*): 输出的投影点的s值
 * - lateral (double*): 输出的投影点的横向偏移
 * - nearest_seg (math::LineSegment2d): 参考线前部的最近线段
 * - min_distance (double): 点到参考线的最小距离
 * - prod (double): 点到最近线段的单位向量的点积
 * - proj (double): 点到最近线段的投影长度
 *
 * @par 处理逻辑:
 * - 检查参考线的线段集是否为空，如果为空则返回失败
 * - 检查输出指针是否为空，如果为空则返回失败
 * - 获取参考线前部的最近线段
 * - 计算点到最近线段的最小距离
 * - 计算点到最近线段的单位向量的点积和投影长度
 * - 设置投影点的s值为投影长度和线段长度的最小值
 * - 如果投影点在最近线段的起点之前，则横向偏移为点到最近线段的点积
 * - 如果投影点在最近线段的终点之后，则横向偏移为最小距离的符号乘以最小距离
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考线的线段集是否为空;
 * if (线段集为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :检查输出指针是否为空;
 *   if (输出指针为空?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :获取参考线前部的最近线段;
 *     :计算点到最近线段的最小距离;
 *     :计算点到最近线段的单位向量的点积和投影长度;
 *     :设置投影点的s值为投影长度和线段长度的最小值;
 *     if (投影点在最近线段的起点之前?) then (yes)
 *       :横向偏移为点到最近线段的点积;
 *     else (no)
 *       :横向偏移为最小距离的符号乘以最小距离;
 *     endif
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线前部的投影，适用于需要根据参考线前部进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getFrontProjection(const math::Vec3d& point, double* accumulate_s, double* lateral) const {
  if (line_segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr) {
    return false;
  }
  const auto& nearest_seg = line_segments_.front();
  const double min_distance = nearest_seg.DistanceTo(point);
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  *accumulate_s = std::min(proj, nearest_seg.length());
  if (proj < 0) {
    *lateral = prod;
  } else {
    *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
  }
  return true;
}

/**
 * @brief 计算点在参考线后部的投影
 * @details 根据给定的点，计算其在参考线后部的投影，并返回投影点的s值和横向偏移
 *
 * @param[in] point 输入的点
 * @param[out] accumulate_s 输出的投影点的s值
 * @param[out] lateral 输出的投影点的横向偏移
 * @return bool 返回计算是否成功
 *
 * @par 关键变量说明:
 * - point (math::Vec3d): 输入的点，包含x、y、z值
 * - accumulate_s (double*): 输出的投影点的s值
 * - lateral (double*): 输出的投影点的横向偏移
 * - nearest_seg (math::LineSegment2d): 参考线后部的最近线段
 * - min_distance (double): 点到参考线的最小距离
 * - prod (double): 点到最近线段的单位向量的点积
 * - proj (double): 点到最近线段的投影长度
 *
 * @par 处理逻辑:
 * - 检查参考线的线段集是否为空，如果为空则返回失败
 * - 检查输出指针是否为空，如果为空则返回失败
 * - 获取参考线后部的最近线段
 * - 计算点到最近线段的最小距离
 * - 计算点到最近线段的单位向量的点积和投影长度
 * - 设置投影点的s值为累计s值加上投影长度
 * - 如果投影点在最近线段的终点之后，则横向偏移为点到最近线段的点积
 * - 如果投影点在最近线段的终点之前，则横向偏移为最小距离的符号乘以最小距离
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查参考线的线段集是否为空;
 * if (线段集为空?) then (yes)
 *   :返回失败;
 * else (no)
 *   :检查输出指针是否为空;
 *   if (输出指针为空?) then (yes)
 *     :返回失败;
 *   else (no)
 *     :获取参考线后部的最近线段;
 *     :计算点到最近线段的最小距离;
 *     :计算点到最近线段的单位向量的点积和投影长度;
 *     :设置投影点的s值为累计s值加上投影长度;
 *     if (投影点在最近线段的终点之后?) then (yes)
 *       :横向偏移为点到最近线段的点积;
 *     else (no)
 *       :横向偏移为最小距离的符号乘以最小距离;
 *     endif
 *     :返回成功;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算点在参考线后部的投影，适用于需要根据参考线后部进行投影计算的场景
 *
 * @warning 需确保参考线的线段集不为空，否则无法正确计算投影
 */
bool ReferenceLine::getRearProjection(const math::Vec3d& point, double* accumulate_s, double* lateral) const {
  if (line_segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr) {
    return false;
  }
  const auto& nearest_seg = line_segments_.back();
  const double min_distance = nearest_seg.DistanceTo(point);
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  *accumulate_s = accumulated_s_.back() + std::max(0.0, proj);
  if (proj > 0) {
    *lateral = prod;
  } else {
    *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
  }
  return true;
}

/**
 * @brief 在两个参考点之间进行插值
 * @details 根据给定的两个参考点和对应的s值，计算在指定s值处的插值点
 *
 * @param[in] p0 第一个参考点
 * @param[in] s0 第一个参考点对应的s值
 * @param[in] p1 第二个参考点
 * @param[in] s1 第二个参考点对应的s值
 * @param[in] s 插值点的s值
 * @return ReferencePoint 返回插值后的参考点
 *
 * @par 关键变量说明:
 * - p0 (ReferencePoint): 第一个参考点，包含位置、航向角、曲率等信息
 * - s0 (double): 第一个参考点对应的s值
 * - p1 (ReferencePoint): 第二个参考点，包含位置、航向角、曲率等信息
 * - s1 (double): 第二个参考点对应的s值
 * - s (double): 插值点的s值
 * - x (double): 插值点的x坐标
 * - y (double): 插值点的y坐标
 * - z (double): 插值点的z坐标
 * - slope (float): 插值点的坡度
 * - heading (double): 插值点的航向角
 * - kappa (double): 插值点的曲率
 * - dkappa (double): 插值点的曲率变化率
 * - lw (double): 插值点的左侧边界
 * - rw (double): 插值点的右侧边界
 * - rlw (double): 插值点的道路左侧边界
 * - rrw (double): 插值点的道路右侧边界
 * - offset (double): 插值点的偏移量
 *
 * @par 处理逻辑:
 * - 检查s0和s1是否相等，如果相等则直接返回p0
 * - 检查s值是否在s0和s1之间，如果不在则返回错误
 * - 使用线性插值计算插值点的x、y、z坐标
 * - 使用线性插值计算插值点的坡度
 * - 使用球面线性插值计算插值点的航向角
 * - 使用线性插值计算插值点的曲率、曲率变化率、左右边界、道路左右边界和偏移量
 * - 返回插值后的参考点
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查s0和s1是否相等;
 * if (s0 == s1?) then (yes)
 *   :返回p0;
 * else (no)
 *   :检查s值是否在s0和s1之间;
 *   if (s < s0 - 1.0e-6 或 s > s1 + 1.0e-6?) then (yes)
 *     :返回错误;
 *   else (no)
 *     :使用线性插值计算插值点的x、y、z坐标;
 *     :使用线性插值计算插值点的坡度;
 *     :使用球面线性插值计算插值点的航向角;
 *     :使用线性插值计算插值点的曲率、曲率变化率、左右边界、道路左右边界和偏移量;
 *     :返回插值后的参考点;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于在两个参考点之间进行插值，适用于需要根据参考线进行插值计算的场景
 *
 * @warning 需确保s值在s0和s1之间，否则可能无法正确计算插值点
 */
ReferencePoint ReferenceLine::Interpolate(const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
                                          const double s1, const double s) {
  if (std::fabs(s0 - s1) < math::kMathEpsilon) {
    return p0;
  }
  CHECK_LE(s0 - 1.0e-6, s);
  CHECK_LE(s, s1 + 1.0e-6);

  const double x = math::lerp(p0.x(), s0, p1.x(), s1, s);
  const double y = math::lerp(p0.y(), s0, p1.y(), s1, s);
  const double z = math::lerp(p0.z(), s0, p1.z(), s1, s);
  const float slope = math::lerp(p0.slope(), s0, p1.slope(), s1, s);
  const double heading = math::slerp(p0.heading(), s0, p1.heading(), s1, s);
  const double kappa = math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);
  const double lw = math::lerp(p0.left_bound(), s0, p1.left_bound(), s1, s);
  const double rw = math::lerp(p0.right_bound(), s0, p1.right_bound(), s1, s);
  const double rlw = math::lerp(p0.road_left_bound(), s0, p1.road_left_bound(), s1, s);
  const double rrw = math::lerp(p0.road_right_bound(), s0, p1.road_right_bound(), s1, s);
  const double offset = math::lerp(p0.offset(), s0, p1.offset(), s1, s);

  return ReferencePoint(Vec3d(x, y, z), slope, heading, kappa, dkappa, lw, rw, rlw, rrw, offset, s);
}

/**
 * @brief 在参考线上找到距离给定点最近的点
 * @details 根据给定的两个参考点和对应的s值，使用Brent搜索算法在参考线上找到距离给定点最近的点
 *
 * @param[in] p0 第一个参考点
 * @param[in] s0 第一个参考点对应的s值
 * @param[in] p1 第二个参考点
 * @param[in] s1 第二个参考点对应的s值
 * @param[in] x 给定点的x坐标
 * @param[in] y 给定点的y坐标
 * @return double 返回最近点的s值
 *
 * @par 关键变量说明:
 * - p0 (ReferencePoint): 第一个参考点，包含位置、航向角、曲率等信息
 * - s0 (double): 第一个参考点对应的s值
 * - p1 (ReferencePoint): 第二个参考点，包含位置、航向角、曲率等信息
 * - s1 (double): 第二个参考点对应的s值
 * - x (double): 给定点的x坐标
 * - y (double): 给定点的y坐标
 * - func_dist_square (std::function<double(double)>): 计算给定s值处参考点与给定点距离平方的函数
 *
 * @par 处理逻辑:
 * - 定义一个lambda函数func_dist_square，用于计算给定s值处参考点与给定点的距离平方
 * - 使用Brent搜索算法在s0和s1之间找到使func_dist_square最小的s值
 * - 返回找到的s值
 *
 * @par 流程图:
 * @startuml
 * start
 * :定义lambda函数func_dist_square;
 * :使用Brent搜索算法在s0和s1之间找到使func_dist_square最小的s值;
 * :返回找到的s值;
 * stop
 * @enduml
 *
 * @note 该函数用于在参考线上找到距离给定点最近的点，适用于需要根据参考线进行最近点计算的场景;
 * Brent搜索算法是一种基于黄金分割搜索和线性插值的算法，用于在给定的区间内找到单峰函数的最小值. 8是最大迭代次数.
 * @warning 需确保s0和s1有效，否则可能无法正确找到最近点
 */
double ReferenceLine::FindMinDistancePoint(const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
                                           const double s1, const double x, const double y) {
  auto func_dist_square = [&p0, &p1, &s0, &s1, &x, &y](const double s) {
    auto p = Interpolate(p0, s0, p1, s1, s);
    double dx = p.x() - x;
    double dy = p.y() - y;
    return dx * dx + dy * dy;
  };
  return ::boost::math::tools::brent_find_minima(func_dist_square, s0, s1, 8).first;
}

void ReferenceLine::setDestinationData(const proto::HpaRoutingInfo::DestinationInfo& destination_info) {
  auto& dest_data = destination_data_;
  constexpr int slot_corners_size = 4;
  bool base_condition = destination_info.has_navi_task_stage() && destination_info.has_dest_point();
  bool stage_condition =
      (destination_info.navi_task_stage() == proto::HpaRoutingInfo_DestinationInfo_NaviTaskStage_kNaviParkingInCruise
       && destination_info.parking_slot_corners_size() == slot_corners_size)
      || (destination_info.navi_task_stage()
          == proto::HpaRoutingInfo_DestinationInfo_NaviTaskStage_kNaviParkingOutCruise);
  dest_data.is_valid = base_condition && stage_condition;
  dest_data.navi_task_stage = destination_info.navi_task_stage();
  const auto dest_point = destination_info.dest_point();

  double offset = 0.0;
  double local_s = 0.0;
  if (!getProjection(math::Vec3d(dest_point.x(), dest_point.y(), dest_point.z()), local_s, offset)) {
    ERT_PLOG_W << "Failed to get destination projection on reference line.";
    dest_data.is_valid = false;
    return;
  }
  dest_data.destination_point = ReferencePoint(math::Vec3d(dest_point.x(), dest_point.y(), dest_point.z()), 0.0,
                                               dest_point.heading(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, offset, local_s);
  dest_data.floor = destination_info.floor();
  for (const auto& slot_corner : destination_info.parking_slot_corners()) {
    dest_data.constraint_polygon_corners.emplace_back(math::Vec3d(slot_corner.x(), slot_corner.y(), slot_corner.z()));
  }
}

}  // namespace gpal::pnc::planning
