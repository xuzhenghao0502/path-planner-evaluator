/**
 * @file reference_line.h
 * @brief 参考线模块实现文件
 * @details
 * 该文件实现了参考线的核心功能，包括参考线的初始化、更新、查询等操作。参考线用于描述车辆行驶的路径，包含路径的几何信息、速度限制、停止线等。
 */

#pragma once
#include <list>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "adas_data/adas_road_marking.h"
#include "center_lines/road_structure.h"
#include "gpal-interface/navigation/navigation_result.pb.h"
#include "math/box2d.h"
#include "math/line_segment2d.h"
#include "math/math_utils.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
#include "memorized_route/memorized_route.h"
#include "navigation_data/local_router.h"
#include "point/path_pt.h"
#include "point/trajectory_pt.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/sl_boundary.pb.h"
#include "reference_point.h"
#include "vehicle_model/driving_tube.h"

namespace gpal::pnc::planning {

/**
 * @brief 主路径历史数据结构
 * @details 该结构体用于存储主路径的历史信息，包括路径ID和路径点集合。
 *          主要用于记录和操作历史路径数据。
 */
struct MainPathHistory {
  std::string id = "";              ///< 路径ID，用于唯一标识该路径
  std::vector<math::Vec3d> values;  ///< 路径点集合，存储路径的x、y、z坐标
  planning::MapPoint adc_loc;       ///< 自车定位信息，记录路径生成时的自车位置
                                    /**
                                     * @brief 默认构造函数
                                     * @details 初始化一个空的MainPathHistory对象
                                     */
  MainPathHistory() {
    id = "";
    values.clear();
  }
  /**
   * @brief 清空路径数据
   * @details 清空路径ID和路径点集合，重置为初始状态
   */
  void clear() {
    id = "";
    values.clear();
  }
};

/**
 * @brief 参考线类定义
 */
class ReferenceLine {
 public:
  /**
   * @brief 默认构造函数
   * @details 创建一个空的ReferenceLine对象
   */
  ReferenceLine() = default;
  explicit ReferenceLine(const std::vector<math::Vec3d>& xyz_points);
  explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);
  explicit ReferenceLine(const SlicedRoute* route);
  explicit ReferenceLine(const PerceptionLane* lane);
  explicit ReferenceLine(const MainPathHistory* path_history);
  explicit ReferenceLine(const adas::LaneLineMarking* lane_line_marking);
  explicit ReferenceLine(const LocalRoute* local_route);
  explicit ReferenceLine(const proto::LocalRouteReferenceLine& local_route_ref,
                         const std::tuple<Eigen::Matrix4d, Eigen::Matrix4d, math::Vec3d>& tf,
                         const proto::LocalRouteRoadInfo& road_info, const std::string& frame_id,
                         const MapPoint& adc_loc);
  explicit ReferenceLine(const proto::road_cognition::ReferenceLine& env_road_cognition_ref, const MapPoint& adc_loc);

  enum class SmoothType {  ///< 参考线平滑类型
    RAW = 0,               ///< 未平滑
    OCP = 1                ///< 基于OCP平滑
  };
  enum class LineType {             ///< 参考线来源类型
    UNKNOWN = 0,                    ///< 未知
    MEMORIZED_ROUTE = 1,            ///< 来源于离线记忆路线
    PERCEPTION_LANE = 2,            ///< 来源于感知车道线
    PERCEPTION_ROAD_STRUCTURE = 3,  ///< 来源于感知道路标线
    LOCAL_ROUTE = 4,                ///< 来源于local_route_ref
    ENV_ROAD_COGNITION = 5          ///< 来源于env_road_cognition_ref
  };
  enum class ShapeType {  ///< 参考线形状类型
    REGULAR = 0,          ///< 常规
    RIGHT_TURN = 1,       ///< 右转
    LEFT_TURN = 2,        ///< 左转
    U_TURN = 3            ///< 掉头
  };

  struct DestinationData {                                ///< 目的地信息结构体
    bool is_valid = false;                                ///< 目的地信息是否有效
    ReferencePoint destination_point;                     ///< 目的地参考点
    std::vector<math::Vec3d> constraint_polygon_corners;  ///< 约束多边形角点集合
    int floor = 0;                                        ///< 目的地所在楼层
    proto::HpaRoutingInfo::DestinationInfo::NaviTaskStage navi_task_stage =
        proto::HpaRoutingInfo_DestinationInfo_NaviTaskStage_kNaviUnknown;  ///< 导航任务阶段
  };

  // 基础属性.
  /**
   * @brief 获取参考线ID
   * @return 当前参考线ID
   */
  const std::string& id() const { return id_; }
  /**
   * @brief 设置参考线ID
   * @param id 要设置的参考线ID
   */
  void set_id(const std::string& id) { id_ = id; }
  /**
   * @brief 获取参考点的数量
   * @return 当前参考点的数量
   */
  int num_points() const { return num_points_; }
  /**
   * @brief 获取参考线段的数量
   * @return 当前参考线段的数量
   */
  int num_line_segments() const { return num_line_segments_; }
  /**
   * @brief 获取参考线长度
   * @return 当前参考线长度，单位：米
   */
  double length() const { return length_; }
  bool offset(const double s, double* l_offset) const;
  /**
   * @brief 获取累积s值
   * @return 当前累积s值
   */
  const std::vector<double>& accumulated_s() const { return accumulated_s_; }
  /**
   * @brief 获取全局系下参考线起点的s值
   * @return 当前全局系下参考线起点的s值，单位：米
   */
  double global_start_s() const { return global_start_s_; }
  /**
   * @brief 设置全局系下参考线起点的s值
   * @param s 要设置的全局系下参考线起点的s值，单位：米
   */
  void set_global_start_s(double s) { global_start_s_ = s; }
  /**
   * @brief 设置平滑类型
   * @param smooth_type 要设置的平滑类型
   */
  void set_smooth_type(SmoothType smooth_type) { smooth_type_ = smooth_type; }
  /**
   * @brief 获取平滑类型
   * @return 当前平滑类型
   */
  SmoothType smooth_type() const { return smooth_type_; }
  /**
   * @brief 设置参考线类型
   * @param line_type 要设置的参考线类型
   */
  void set_line_type(LineType line_type) { line_type_ = line_type; }
  /**
   * @brief 获取参考线类型
   * @return 当前参考线类型
   */
  LineType line_type() const { return line_type_; }
  /**
   * @brief 设置参考线形状类型
   * @param shape_type 要设置的参考线形状类型
   */
  void set_shape_type(ShapeType shape_type) { shape_type_ = shape_type; }
  /**
   * @brief 获取参考线形状类型
   * @return 当前参考线形状类型
   */
  ShapeType shape_type() const { return shape_type_; }
  /**
   * @brief 获取自车定位信息的可修改指针
   * @return 自车定位信息的指针
   */
  MapPoint* mutableAdcLocalization() { return &adc_loc_; }
  /**
   * @brief 获取自车定位信息
   * @return 当前自车定位信息
   */
  const MapPoint& adcLocalization() const { return adc_loc_; }
  /**
   * @brief 获取自车在参考线上位置匹配的s值
   * @return 当前自车在参考线上位置匹配的s值，单位：米
   */
  const float& adcS() const { return adc_s_; }
  /**
   * @brief 设置自车在参考线上位置匹配的s值
   * @param adc_s 要设置的自车在参考线上位置匹配的s值，单位：米
   */
  void setAdcS(const float& adc_s) { adc_s_ = adc_s; }
  /**
   * @brief 获取自车在参考线上位置匹配的l值
   * @return 当前自车在参考线上位置匹配的l值，单位：米
   */
  const float& adcL() const { return adc_l_; }
  /**
   * @brief 设置自车在参考线上位置匹配的l值
   * @param adc_l 要设置的自车在参考线上位置匹配的l值，单位：米
   */
  void setAdcL(const float& adc_l) { adc_l_ = adc_l; }
  /**
   * @brief 判断边界是否被车道修改
   * @return 当前边界是否被车道修改
   */
  bool isBoundModifiedByLane() const { return is_bound_modified_by_lane_; }

  // reference points
  /**
   * @brief 获取参考点集合
   * @return 当前参考点集合
   */
  const std::vector<ReferencePoint>& reference_points() const { return reference_points_; }
  ReferencePoint getReferencePoint(const double x, const double y) const;
  ReferencePoint getReferencePoint(const double s) const;
  std::vector<ReferencePoint> getReferencePoints(double start_s, double end_s) const;
  std::vector<ReferencePoint> getInterpolatedRefPoints(double start_s, double end_s, double point_interval) const;
  // for memorized_routegetReferencePoints
  void getMemorizedRouteReferencePoints(const math::Vec3d& origin, const MapPoint& loc);
  void modifyMemorizedReferenceLineBoundByPerceptionLane(const PerceptionLane& lane, float theta_deviation_thresold,
                                                         float lateral_deviation_thresold);
  // nearest reference point
  size_t getNearestReferenceIndex(const double s) const;
  ReferencePoint getNearestReferencePoint(const math::Vec3d& xyz) const;
  ReferencePoint getNearestReferencePoint(const double s) const;
  int getUpperIndexFromS(double s) const;

  // frenet points
  FrenetFramePoint getFrenetPoint(const planning::PathPt& path_point) const;
  std::pair<std::array<double, 3>, std::array<double, 3>> toFrenetFrame(const planning::TrajectoryPt& traj_point) const;

  // 导航分 navigation_scores_
  /**
   * @brief 设置导航分数集合
   * @param navigation_scores 导航分数集合，每个元素为一个三元组，包含分数、起始s值和结束s值
   */
  void setNavigationScores(const std::vector<std::tuple<float, float, float>>& navigation_scores) {
    navigation_scores_ = navigation_scores;
  }
  /**
   * @brief 获取导航分数集合
   * @return 导航分数集合，每个元素为一个三元组，包含分数、起始s值和结束s值
   */
  const std::vector<std::tuple<float, float, float>>& getNavigationScores() const { return navigation_scores_; }
  std::vector<std::tuple<float, float, float>> getNavigationScoresFromSRange(const double s,
                                                                             const double forward_distance_thrd,
                                                                             const double backward_dist_thrd) const;
  std::vector<std::tuple<float, float, float>> getNavigationScoresFromSRange(const float& start_s,
                                                                             const float& end_s) const;
  float getNavigationScoreFromS(const double s) const;

  // navigation lane change ranges from memorized route for POC
  /**
   * @brief 获取基于记忆路线的导航变道范围集合
   * @return 基于记忆路线的导航变道范围集合，每个元素为一个四元组，包含起始s值、结束s值、是否左变道、是否来自路线文件
   */
  std::vector<std::tuple<float, float, bool, bool>> getNavigationLaneChangeRanges() const {
    return navigation_lane_change_ranges_;
  }
  /**
   * @brief 设置基于记忆路线的导航变道范围集合
   * @param navigation_lane_change_ranges
   * 导航变道范围集合，每个元素为一个四元组，包含起始s值、结束s值、是否左变道、是否来自路线文件
   */
  void setNavigationLaneChangeRanges(
      const std::vector<std::tuple<float, float, bool, bool>>& navigation_lane_change_ranges) {
    navigation_lane_change_ranges_ = navigation_lane_change_ranges;
  }

  // 基于视觉车道线+里程计的巡线区域
  /**
   * @brief 获取车道线巡线范围
   * @return 车道线巡线范围集合
   */
  std::vector<std::pair<float, float>> getLaneFollowRanges() const { return lane_follow_ranges_; }
  /**
   * @brief 设置车道线巡线范围
   * @param lane_follow_ranges 车道线巡线范围集合
   */
  void setlaneFollowRanges(const std::vector<std::pair<float, float>>& lane_follow_ranges) {
    lane_follow_ranges_ = lane_follow_ranges;
  }
  bool isInLaneFollowRanges() const;

  // 曲率, 不区分正负值
  double getMaxCurvatureInRange(double start_s, double end_s) const;
  // 曲率, 区分正负值, std::pair<max, min>
  std::pair<double, double> getMaxAndMinCurvatureInRange(double start_s, double end_s) const;

  // bound along reference line
  bool getLaneBound(const double s, double* const lane_left_bound, double* const lane_right_bound) const;
  std::pair<float, float> getAverageLaneBoundFromRange(const float &start_s, const float &end_s,
                                                       const float &interval) const;
  bool getRoadBound(const double s, double* const road_left_bound, double* const road_right_bound) const;
  std::pair<float, float> getAverageRoadBoundFromRange(const float &start_s, const float &end_s,
                                                       const float &interval) const;
  std::tuple<float, float, float> getDrivingBound(const gpal::pnc::SLBoundary& sl_boundary) const;
  std::pair<double, double> getDrivingTubeBound(const double s) const;

  // 位置关系计算.
  bool isOnLane(const SLPoint& sl_point) const;
  bool isOnLane(const math::Vec3d& vec3d_point) const;
  /**
   * @brief 判断给定点是否在车道内（模板函数）
   * @tparam XYZPoint 点类型，需包含x(), y(), z()方法
   * @param xyz 输入点
   * @return 如果点在车道内返回true，否则返回false
   */
  template <class XYZPoint>
  bool isOnLane(const XYZPoint& xyz) const {
    return isOnLane(math::Vec3d(xyz.x(), xyz.y(), xyz.z()));
  }
  bool isOnLane(const gpal::pnc::SLBoundary& sl_boundary) const;
  bool isOnLane(const gpal::pnc::SLBoundary& sl_boundary, std::pair<double, double>& lane_range,
                const double buffer = 0) const;
  bool isOnRoad(const SLPoint& sl_point) const;
  bool isOnRoad(const math::Vec3d& vec3d_point) const;
  bool isOnRoad(const gpal::pnc::SLBoundary& sl_boundary) const;
  bool hasOverlap(const math::Box2d& box) const;
  bool isRelavent(const gpal::pnc::SLBoundary& sl_boundary, std::pair<double, double>& lane_range,
                  std::pair<double, double>& adc_range, const double buffer = 0) const;

  // speed_limits_
  /**
   * @brief 设置速度限制集合
   * @param speed_limits 速度限制集合，包含多个速度限制信息
   */
  void setSpeedLimits(const std::vector<SpeedLimit>& speed_limits) { speed_limits_ = speed_limits; };
  /**
   * @brief 获取速度限制集合
   * @return 速度限制集合，包含多个速度限制信息，每个速度限制信息包含起始s值、结束s值和速度限制值
   */
  const std::vector<SpeedLimit>& getSpeedLimits() const { return speed_limits_; };
  double GetSpeedLimitFromS(const double s) const;
  void AddSpeedLimit(SpeedLimit speed_limit);
  void AddSpeedLimit(double s, double speed_limit, math::Vec3d point);
  void AddSpeedLimit(double start_s, double end_s, double speed_limit, math::Vec3d start_point, math::Vec3d end_point);
  // for memorized_route
  std::vector<SpeedLimit> getMemorizedRouteSpeedLimits(const math::Vec3d& origin) const;

  // stop_lines_
  /**
   * @brief 设置停止线集合
   * @param stop_lines 停止线集合，包含多个停止线信息
   */
  void setStopLines(const std::vector<StopLine>& stop_lines) { stop_lines_ = stop_lines; };
  /**
   * @brief 获取停止线集合
   * @return 停止线集合，包含多个停止线信息，每个停止线信息包含起始s值、结束s值和停止线类型
   */
  const std::vector<StopLine>& getStopLines() const { return stop_lines_; };
  std::vector<StopLine> getStopLinesFromSRange(const double& s, const double forward_distance_thrd = 100.0,
                                               const double backward_dist_thrd = 2.0) const;
  // for memorized_route
  std::vector<StopLine> getMemorizedRouteStopLines(const math::Vec3d& origin) const;
  std::vector<StopLine> getMemorizedRouteStopLines(const math::Vec3d& origin, const double s,
                                                   const double forward_distance_thrd = 100.0,
                                                   const double backward_dist_thrd = 2.0) const;

  // gates_
  /**
   * @brief 设置闸机集合
   * @param gates 闸机集合，包含多个闸机信息
   */
  void setGates(const std::vector<Gate>& gates) { gates_ = gates; };
  /**
   * @brief 获取闸机集合
   * @return 闸机集合，包含多个闸机信息，每个闸机信息包含起始s值、结束s值和闸机类型
   */
  const std::vector<Gate>& getGates() const { return gates_; };
  std::vector<Gate> getGatesFromSRange(const double s, const double forward_distance_thrd,
                                       const double backward_dist_thrd) const;
  std::vector<Gate> getGatesFromSRange(const float& start_s, const float& end_s) const;

  // directions_
  /**
   * @brief 设置方向集合
   * @param directions 方向集合，包含多个方向信息
   */
  void setDirections(const std::vector<SegmentDirection>& directions) { directions_ = directions; };
  /**
   * @brief 获取方向集合
   * @return 方向集合，包含多个方向信息，每个方向信息包含起始s值、结束s值和方向类型
   */
  const std::vector<SegmentDirection>& getDirections() const { return directions_; };
  std::vector<SegmentDirection> getDirectionsFromSRange(const double s, const double forward_distance_thrd,
                                                        const double backward_dist_thrd) const;
  std::vector<SegmentDirection> getDirectionsFromSRange(const float& start_s, const float& end_s) const;
  SegmentDirection getDirectionFromS(const double s) const;
  std::vector<std::pair<float, float>> getSpecifiedDirectionRanges(const DrivingDirection& direction) const;
  // for memorized_route
  std::vector<SegmentDirection> getMemorizedRouteSegmentsDirection(const math::Vec3d& origin) const;

  void setLaneDirection(const proto::perception::CenterLine::Direction &lane_direction) {
    lane_direction_ = lane_direction;
  }
  const proto::perception::CenterLine::Direction &getLaneDirection() const { return lane_direction_; }

  // source infos
  /**
   * @brief 设置来源信息集合
   * @param source_infos 来源信息集合，包含多个来源信息，每个来源信息包含来源类型、起始s值和结束s值
   */
  void setSourceInfos(const std::vector<std::tuple<LineSourceType, float, float>>& source_infos) {
    source_infos_ = source_infos;
  }
  /**
   * @brief 获取来源信息集合
   * @return 来源信息集合，包含多个来源信息，每个来源信息包含来源类型、起始s值和结束s值
   */
  const std::vector<std::tuple<LineSourceType, float, float>>& getSourceInfos() const { return source_infos_; }
  std::vector<std::tuple<LineSourceType, float, float>> getSourceInfosFromSRange(const float& start_s,
                                                                                 const float& end_s) const;
  std::vector<std::tuple<LineSourceType, float, float>> getSourceInfosFromSRange(const float& s,
                                                                                 const float& forward_distance_thrd,
                                                                                 const float& backward_dist_thrd) const;
  std::tuple<LineSourceType, float, float> getSourceInfoFromS(const float& s) const;
  std::vector<std::tuple<LineSourceType, float, float>> getSpecifiedSourceInfos(const LineSourceType& type) const;

  // navigation_lane_change_info_
  /**
   * @brief 获取基于LocalRoute的导航变道信息
   * @return 基于LocalRoute的导航变道信息，包含距离变道点的距离、变道方向和变道次数
   */
  const std::tuple<float, int, int>& getNavigationLaneChangeInfo() const { return navigation_lane_change_info_; }
  /**
   * @brief 设置基于LocalRoute的导航变道信息
   * @param navigation_lane_change_info 基于LocalRoute的导航变道信息，包含距离变道点的距离、变道方向和变道次数
   */
  void setNavigationLaneChangeInfo(const std::tuple<float, int, int>& navigation_lane_change_info) {
    navigation_lane_change_info_ = navigation_lane_change_info;
  }

  // boundary type
  /**
   * @brief 设置边界类型集合
   * @param boundary_types 边界类型集合，包含多个边界类型信息
   */
  void setBoundaryTypes(const std::vector<SegmentBoundaryType>& boundary_types) { boundary_types_ = boundary_types; };
  /**
   * @brief 获取边界类型集合
   * @return 边界类型集合，包含多个边界类型信息，每个边界类型信息包含起始s值、结束s值和边界类型
   */
  const std::vector<SegmentBoundaryType>& getBoundaryTypes() const { return boundary_types_; };
  std::vector<SegmentBoundaryType> getBoundaryTypesFromSRange(const double s, const double forward_distance_thrd,
                                                              const double backward_dist_thrd) const;
  std::vector<SegmentBoundaryType> getBoundaryTypesFromSRange(const float& start_s, const float& end_s) const;
  SegmentBoundaryType getBoundaryTypeFromS(const double s) const;

  // 基于env_road_cognition的LineAttributeRange(来源类型source_type, 车道类型line_type)
  void setLineAttributeRanges(const std::vector<LineAttributeRange>& line_attribute_ranges) {
    line_attribute_ranges_ = line_attribute_ranges;
  }
  const std::vector<LineAttributeRange>& getLineAttributeRanges() const { return line_attribute_ranges_; }
  std::vector<LineAttributeRange> getLineAttributeRangesFromSRange(const float& start_s, const float& end_s) const;
  std::vector<LineAttributeRange> getLineAttributeRangesFromSRange(const float& s, const float& forward_distance_thrd,
                                                                   const float& backward_dist_thrd) const;
  LineAttributeRange getLineAttributeRangeFromS(const float& s) const;
  std::vector<LineAttributeRange> getSpecifiedLineAttributeRanges(
      const proto::road_cognition::LineAttributeRange::LineSourceType& source_type) const;
  std::vector<LineAttributeRange> getSpecifiedLineAttributeRanges(
      const proto::perception::CenterLine::TypeRange::Type& line_type) const;

  // 基于env_road_cognition的LaneBoundaries及RoadBoundaries
  void setLeftLaneBoundaries(const std::vector<BoundaryRange>& left_lane_boundaries) {
    left_lane_boundaries_ = left_lane_boundaries;
  };
  const std::vector<BoundaryRange>& getLeftLaneBoundaries() const { return left_lane_boundaries_; };
  void setRightLaneBoundaries(const std::vector<BoundaryRange>& right_lane_boundaries) {
    right_lane_boundaries_ = right_lane_boundaries;
  };
  const std::vector<BoundaryRange>& getRightLaneBoundaries() const { return right_lane_boundaries_; };
  void setLeftRoadBoundaries(const std::vector<BoundaryRange>& left_road_boundaries) {
    left_road_boundaries_ = left_road_boundaries;
  };
  const std::vector<BoundaryRange>& getLeftRoadBoundaries() const { return left_road_boundaries_; };
  void setRightRoadBoundaries(const std::vector<BoundaryRange>& right_road_boundaries) {
    right_road_boundaries_ = right_road_boundaries;
  };
  const std::vector<BoundaryRange>& getRightRoadBoundaries() const { return right_road_boundaries_; };

  std::vector<BoundaryRange> getBoundaryRangesFromSRange(const std::vector<BoundaryRange>& boundaries, const float& s,
                                                         const float& forward_distance_thrd,
                                                         const float& backward_dist_thrd) const;
  std::vector<BoundaryRange> getBoundaryRangesFromSRange(const std::vector<BoundaryRange>& boundaries,
                                                         const float& start_s, const float& end_s) const;
  BoundaryRange getBoundaryRangeFromS(const std::vector<BoundaryRange>& boundaries, const float& s) const;

  // 基于env_road_cognition的RiskBoundariesIds
  void setRelatedRiskBoundariesIds(const std::vector<std::string>& related_risk_boundaries_ids) {
    related_risk_boundaries_ids_ = related_risk_boundaries_ids;
  };
  const std::vector<std::string>& getRelatedRiskBoundariesIds() const { return related_risk_boundaries_ids_; };

  void setRelatedGodBoundaries(const std::vector<GodBoundary>& related_god_boundaries) {
    related_god_boundaries_ = related_god_boundaries;
  };
  const std::vector<GodBoundary>& getRelatedGodBoundaries() const { return related_god_boundaries_; };
  
  // areas_
  /**
   * @brief 设置区域集合
   * @param areas 区域集合，包含多个区域信息
   */
  void setAreas(const std::vector<PerceptionArea>& areas) { areas_ = areas; };
  /**
   * @brief 获取区域集合
   * @return 区域集合，包含多个区域信息，每个区域信息包含起始s值、结束s值和区域类型
   */
  const std::vector<PerceptionArea>& getAreas() const { return areas_; };

  // RoadInfo
  /**
   * @brief 获取道路信息
   * @return 当前道路信息，包含道路类型、车道数量等信息
   */
  const PerceptionRoadInfo& getRoadInfo() const { return road_info_; }
  /**
   * @brief 设置道路信息
   * @param road_info 要设置的道路信息，包含道路类型、车道数量等信息
   */
  void setRoadInfo(const PerceptionRoadInfo& road_info) { road_info_ = road_info; }

  // 基于env_road_cognition的道路信息
  void setRoadRanges(const std::vector<RoadRange>& road_ranges) { road_ranges_ = road_ranges; }
  const std::vector<RoadRange>& getRoadRanges() const { return road_ranges_; }

  // 各参考线之前的关系.
  /**
   * @brief 设置左参考线集合
   * @param left_reference_lines 左参考线集合，包含多个左参考线信息, id, elements(bound_type, range, points, segment_id)
   */
  void setLeftReferenceLines(const std::unordered_map<std::string, NeighborLocalRoute>& left_reference_lines) {
    left_reference_lines_ = left_reference_lines;
  }
  /**
   * @brief 获取左参考线集合
   * @return 左参考线集合，包含多个左参考线信息
   */
  const std::unordered_map<std::string, NeighborLocalRoute>& getLeftReferenceLines() const {
    return left_reference_lines_;
  }
  /**
   * @brief 设置右参考线集合
   * @param right_reference_lines 右参考线集合，包含多个右参考线信息, id, elements(bound_type, range, points,
   * segment_id)
   */
  void setRightReferenceLines(const std::unordered_map<std::string, NeighborLocalRoute>& right_reference_lines) {
    right_reference_lines_ = right_reference_lines;
  }
  /**
   * @brief 获取右参考线集合
   * @return 右参考线集合，包含多个右参考线信息
   */
  const std::unordered_map<std::string, NeighborLocalRoute>& getRightReferenceLines() const {
    return right_reference_lines_;
  }

  /**
   * @brief 基于env_road_cognition设置左参考线集合
   * @param left_ref_lines_ 左参考线集合，包含多个左参考线信息
   */
  void setLeftRefLines(const std::unordered_map<std::string, NeighborReferenceLine>& left_reference_lines) {
    left_ref_lines_ = left_reference_lines;
  }
  /**
   * @brief 基于env_road_cognition获取左参考线集合
   * @return 左参考线集合，包含多个左参考线信息
   */
  const std::unordered_map<std::string, NeighborReferenceLine>& getLeftRefLines() const { return left_ref_lines_; }
  /**
   * @brief 基于env_road_cognition设置右参考线集合
   * @param right_ref_lines_ 右参考线集合，包含多个右参考线信息
   */
  void setRightRefLines(const std::unordered_map<std::string, NeighborReferenceLine>& right_reference_lines) {
    right_ref_lines_ = right_reference_lines;
  }
  /**
   * @brief 基于env_road_cognition获取右参考线集合
   * @return 右参考线集合，包含多个右参考线信息
   */
  const std::unordered_map<std::string, NeighborReferenceLine>& getRightRefLines() const { return right_ref_lines_; }

  std::vector<std::string> getNeighborReferenceLineIdsFromS(const float &s, const bool &is_left_neighbor) const;

  /**
   * @brief 设置当前参考线是否是自车最近的参考线
   * @param is_current_reference_line 是否是自车最近的参考线
   */
  void setIsCurrentReferenceLine(const bool& is_current_reference_line) {
    is_current_reference_line_ = is_current_reference_line;
  }
  /**
   * @brief 获取当前参考线是否是自车最近的参考线
   * @return 当前参考线是否是自车最近的参考线
   */
  const bool& isCurrentReferenceLine() const { return is_current_reference_line_; }
  std::pair<bool, ReferenceLine> generateParallelReferenceLine(const float& offset, const std::string& id) const;
  /**
   * @brief 获取是否是平行虚拟参考线
   * @return 当前是否是平行虚拟参考线
   */
  const bool& isParallelVirtual() const { return is_parallel_virtual_; }
  /**
   * @brief 设置是否是平行虚拟参考线
   * @param is_parallel_virtual 是否是平行虚拟参考线
   */
  void setIsParallelVirtual(const bool& is_parallel_virtual) { is_parallel_virtual_ = is_parallel_virtual; }

  // merge and fork info
  /**
   * @brief 获取合并/分叉范围集合
   * @return 合并/分叉范围集合，包含多个合并/分叉范围信息
   */
  const std::vector<MergeForkRange>& getMergeForkRanges() const { return merge_fork_ranges_; }
  /**
   * @brief 设置合并/分叉范围集合
   * @param merge_fork_ranges 合并/分叉范围集合，包含多个合并/分叉范围信息
   */
  void setMergeForkRanges(const std::vector<MergeForkRange>& merge_fork_ranges) {
    merge_fork_ranges_ = merge_fork_ranges;
  }
  std::vector<MergeForkRange> getMergeForkRangesFromSRange(const float& start_s, const float& end_s);
  std::vector<MergeForkRange> getMergeForkRangesFromSRange(const float& s, const float& forward_dis,
                                                           const float& backward_dis);
  std::vector<MergeForkRange> getMergeRangesFromSRange(const float& start_s, const float& end_s);
  std::vector<MergeForkRange> getMergeRangesFromSRange(const float& s, const float& forward_dis,
                                                       const float& backward_dis);
  std::vector<MergeForkRange> getForkRangesFromSRange(const float& start_s, const float& end_s);
  std::vector<MergeForkRange> getForkRangesFromSRange(const float& s, const float& forward_dis,
                                                      const float& backward_dis);

  // 基于env_road_cognition的KeyPoints
  const std::vector<KeyPoint>& getKeyPoints() const { return key_points_; }
  void setKeyPoints(const std::vector<KeyPoint>& key_points) { key_points_ = key_points; }
  const std::vector<LaneMarkingCrossAttributeRange>& getLaneMarkingCrossAttributeRanges() const {
    return lane_marking_cross_attributes_;
  }
  void setLaneMarkingCrossAttributeRanges(
      const std::vector<LaneMarkingCrossAttributeRange>& lane_marking_cross_attribute_ranges) {
    lane_marking_cross_attributes_ = lane_marking_cross_attribute_ranges;
  }
  std::vector<KeyPoint> getKeyPointsFromSRange(const float& start_s, const float& end_s) const;
  std::vector<KeyPoint> getKeyPointsFromSRange(const float& s, const float& forward_dis,
                                               const float& backward_dis) const;
  std::vector<KeyPoint> getSpecifiedKeyPoints(const proto::road_cognition::KeyPoint::PointType& type) const;

  // 相比上帧参考线，是否发生变更，目前横向规划使用中
  /**
   * @brief 获取相比上帧参考线是否发生变更
   * @return 如果相比上帧参考线发生变更返回true，否则返回false
   */
  bool has_change_reference() const { return has_change_reference_; }
  /**
   * @brief 设置相比上帧参考线是否发生变更
   * @param has_change_reference 相比上帧参考线是否发生变更
   */
  void set_has_change_reference(bool has_change_reference) { has_change_reference_ = has_change_reference; }

  // 计算参考线信息: heading, kappa, dkappa
  bool computeInitInfo(std::vector<ReferencePoint>& xyz_points);
  // kappa, dkappa
  void computeKappaAndDkappa();

  std::tuple<bool, double, double> getOverlapRange(const math::Polygon2d& polygon) const;

  // 工具函数.
  // sl <-> xy
  bool sl2xy(const SLPoint& sl_point, math::Vec3d* const xyz_point) const;
  bool sl2xy_path_point(const double s, const double l, const double dl, const double ddl,
                        proto::PathPoint* path_point) const;
  bool xy2sl(const math::Vec3d& xyz_point, const double ref_s, const std::pair<size_t, size_t>& search_range_idx,
             SLPoint* const sl_point) const;
  bool xy2sl(const math::Vec3d& xyz_point, SLPoint* const sl_point) const;
  bool xy2sl(const math::Vec3d& xyz_point, const float& heading, SLPoint* const sl_point) const;
  /**
   * @brief 将任意类型的点坐标转换为SL坐标（无航向角）
   * @tparam XYZPoint 点类型，需包含x(), y(), z()方法
   * @param xyz 输入点坐标
   * @param sl_point 输出的SL坐标
   * @return 如果转换成功返回true，否则返回false
   */
  template <class XYZPoint>
  bool xy2sl(const XYZPoint& xyz, SLPoint* const sl_point) const {
    return xy2sl(math::Vec3d(xyz.x(), xyz.y(), xyz.z()), sl_point);
  }
  /**
   * @brief 将任意类型的点坐标转换为SL坐标（带航向角）
   * @tparam XYZPoint 点类型，需包含x(), y(), z()方法
   * @param xyz 输入点坐标
   * @param heading 航向角
   * @param sl_point 输出的SL坐标
   * @return 如果转换成功返回true，否则返回false
   */
  template <class XYZPoint>
  bool xy2sl(const XYZPoint& xyz, const float& heading, SLPoint* const sl_point) const {
    return xy2sl(math::Vec3d(xyz.x(), xyz.y(), xyz.z()), heading, sl_point);
  }

  // 投影函数.
  bool getSLBoundary(const std::vector<math::Vec2d>& points, gpal::pnc::SLBoundary* const sl_boundary) const;
  bool getSLBoundary(const std::vector<math::Vec3d>& points, gpal::pnc::SLBoundary* const sl_boundary) const;
  bool getSLBoundary(const math::Box2d& box, gpal::pnc::SLBoundary* const sl_boundary, double ref_s = 9999.0) const;
  void calcProjection(const math::Vec3d& point, const double min_distance, const int min_index, double& accumulate_s,
                      double& lateral) const;
  bool getProjection(const math::Vec3d& point, const double ref_s, const std::pair<size_t, size_t>& search_range_idx,
                     double& accumulate_s, double& lateral) const;
  bool getProjection(const math::Vec3d& point, const double ref_s, const std::pair<size_t, size_t>& search_range_idx,
                     double& accumulate_s, double& lateral, int& min_index) const;
  bool getProjection(const math::Vec3d& point, const double ref_s, const std::pair<size_t, size_t>& search_range_idx,
                     double& accumulate_s, double& lateral, double& min_distance, int& min_index) const;
  bool getProjection(const math::Vec3d& point, double& accumulate_s, double& lateral) const;
  bool getProjection(const math::Vec3d& point, const float& heading, double& accumulate_s, double& lateral) const;
  bool getProjection(const math::Vec3d& point, double& accumulate_s, double& lateral, int& min_index) const;
  bool getProjection(const math::Vec3d& point, const float& heading, double& accumulate_s, double& lateral,
                     int& min_index) const;
  bool getProjection(const math::Vec3d& point, const float& heading, const float& heading_tolerance,
                     double& accumulate_s, double& lateral, double& min_distance, int& min_index) const;
  bool getProjection(const math::Vec3d& point, double& accumulate_s, double& lateral, double& min_distance,
                     int& min_index) const;
  bool getProjection(const math::Vec3d& point, const float& heading, double& accumulate_s, double& lateral,
                     double& min_distance, int& min_index) const;
  bool getFrontProjection(const math::Vec3d& point, double* accumulate_s, double* lateral) const;
  bool getRearProjection(const math::Vec3d& point, double* accumulate_s, double* lateral) const;

  // 平滑需求
  /**
   * @brief 设置是否是记忆泊车路线参考线
   * @param is_memorized_park_route 是否是记忆泊车路线参考线
   */
  void setIsMemorizedParkRoute(const bool& is_memorized_park_route) {
    is_memorized_park_route_ = is_memorized_park_route;
  }
  /**
   * @brief 获取是否是记忆泊车路线参考线
   * @return 当前是否是记忆泊车路线参考线
   */
  bool isMemorizedParkRoute() const { return is_memorized_park_route_; }
  /**
   * @brief 设置是否是记忆路线分段末段参考线
   * @param is_memorized_end_route 是否是记忆路线分段末段参考线
   */
  void setIsMemorizedEndRoute(const bool& is_memorized_end_route) { is_memorized_end_route_ = is_memorized_end_route; }
  /**
   * @brief 获取是否是记忆路线分段末段参考线
   * @return 当前是否是记忆路线分段末段参考线
   */
  bool isMemorizedEndRoute() const { return is_memorized_end_route_; }
  /**
   * @brief 设置是否需要终点强约束
   * @param is_terminal_enforced 是否需要终点强约束
   */
  void setIsTerminalEnforced(const bool& is_terminal_enforced) { is_terminal_enforced_ = is_terminal_enforced; }
  /**
   * @brief 获取是否需要终点强约束
   * @return 当前是否需要终点强约束
   */
  bool isTerminalEnforced() const { return is_terminal_enforced_; }

  /**
   * @brief 设置目的地信息
   * @param destination_info 目的地信息
   */
  void setDestinationData(const proto::HpaRoutingInfo::DestinationInfo& destination_info);
  /**
   * @brief 获取目的地信息
   * @return 目的地信息
   */
  DestinationData getDestinationData() const { return destination_data_; }

 private:
  static ReferencePoint Interpolate(const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
                                    const double s1, const double s);
  // 在给定的区间内找到单峰函数的最小值.
  static double FindMinDistancePoint(const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
                                     const double s1, const double x, const double y);

 private:
  std::string id_ = "";                                                 ///< 参考线ID
  int num_points_ = 0;                                                  ///< 参考点的数量
  int num_line_segments_ = 0;                                           ///< 参考线段的数量
  std::vector<math::LineSegment2d> line_segments_;                      ///< 参考线段集合
  double length_ = 0.0;                                                 ///< 参考线长度，单位：米
  std::vector<double> accumulated_s_;                                   ///< 累积s值
  MapPoint adc_loc_;                                                    ///< 自车定位信息
  float adc_s_ = 0.0;                                                   ///< 自车在参考线上位置匹配的s值
  float adc_l_ = 0.0;                                                   ///< 自车在参考线上位置匹配的l值
  double global_start_s_ = 0;                                           ///< 全局系下参考线起点的s值
  SmoothType smooth_type_ = SmoothType::RAW;                            ///< 平滑类型
  LineType line_type_ = LineType::UNKNOWN;                              ///< 参考线类型
  ShapeType shape_type_ = ShapeType::REGULAR;                           ///< 参考线形状类型
  std::vector<ReferencePoint> reference_points_;                        ///< 参考点集合
  std::vector<SpeedLimit> speed_limits_;                                ///< 速度限制集合
  std::vector<StopLine> stop_lines_;                                    ///< 停止线集合
  std::vector<Gate> gates_;                                             ///< 闸机集合
  std::vector<PerceptionArea> areas_;                                   ///< 区域集合
  PerceptionRoadInfo road_info_;                                        ///< 道路信息
  std::vector<RoadRange> road_ranges_;                                  ///<  基于env_road_cognition的道路信息
  std::vector<SegmentDirection> directions_;                            ///< 行驶方向集合
  proto::perception::CenterLine::Direction lane_direction_ =
      proto::perception::CenterLine::kDirectionSame;                    ///< 车道方向(同向/反向/双向, 等)
  std::vector<std::tuple<LineSourceType, float, float>> source_infos_;  ///< 来源信息集合
  std::vector<LineAttributeRange>
      line_attribute_ranges_;  ///< 基于env_road_cognition的线属性数组(来源类型source_type, 车道类型line_type)
  std::tuple<float, int, int> navigation_lane_change_info_ = {-1.0, 0,
                                                              0};  ///< 导航变道信息, 依次为换道距离, 换道方向, 换道次数
  std::vector<SegmentBoundaryType> boundary_types_;                ///< 边界类型集合
  std::vector<BoundaryRange> left_lane_boundaries_;                ///< 基于env_road_cognition的左车道边界类型集合
  std::vector<BoundaryRange> right_lane_boundaries_;               ///< 基于env_road_cognition的右车道边界类型集合
  std::vector<BoundaryRange> left_road_boundaries_;                ///< 基于env_road_cognition的左道路边界类型集合
  std::vector<BoundaryRange> right_road_boundaries_;               ///< 基于env_road_cognition的右道路边界类型集合
  std::vector<std::string> related_risk_boundaries_ids_;           ///< 基于env_road_cognition的风险边界id集合
  std::vector<GodBoundary> related_god_boundaries_;                ///< 基于env_road_cognition的相关GOD（通用障碍物检测）边界类型集合
  std::vector<std::tuple<float, float, bool, bool>>
      navigation_lane_change_ranges_;  ///< 基于记忆路线的导航变道区域信息, start_s, end_s, is_left_lane_change,
                                       ///< is_from_route_file
  std::vector<std::pair<float, float>> lane_follow_ranges_;                    ///< 基于记忆路线的车道线巡线范围
  std::unordered_map<std::string, NeighborLocalRoute> left_reference_lines_;   ///< 左邻参考线集合
  std::unordered_map<std::string, NeighborLocalRoute> right_reference_lines_;  ///< 右邻参考线集合
  std::unordered_map<std::string, NeighborReferenceLine> left_ref_lines_;   ///< 基于env_road_cognition的左邻参考线集合
  std::unordered_map<std::string, NeighborReferenceLine> right_ref_lines_;  ///< 基于env_road_cognition的右邻参考线集合
  std::vector<MergeForkRange> merge_fork_ranges_;                           ///< 合并/分叉范围集合
  std::vector<KeyPoint> key_points_;                                        ///< 基于env_road_cognition的关键点数组
  std::vector<std::tuple<float, float, float>> navigation_scores_;          // 导航分集合, score, start_s, end_s
  std::vector<LaneMarkingCrossAttributeRange> lane_marking_cross_attributes_;  ///< 基于env_road_cognition的车道线跨线属性数组
  bool is_current_reference_line_ = false;                                  ///< 当前参考线是否是自车最近的参考线
  bool is_memorized_park_route_ = false;                                    ///< 是否是记忆泊车路线参考线
  bool is_memorized_end_route_ = false;                                     ///< 是否是记忆路线分段末段参考线
  bool is_terminal_enforced_ = false;                                       ///< 是否需要终点强约束
  bool is_parallel_virtual_ = false;                                        ///< 是否是平行虚拟参考线
  bool is_bound_modified_by_lane_ = false;                                  ///< 边界是否被车道修改

  bool has_change_reference_ = false;  ///< 是否变更参考线

  DestinationData destination_data_;  ///< 目的地信息
};

}  // namespace gpal::pnc::planning
