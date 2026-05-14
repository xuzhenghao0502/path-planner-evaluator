/**
 * @file lane_marking.h
 * @brief 车道标线实现文件
 * @details 该文件实现了LaneMarking类，用于表示车道标线信息。
 *          主要功能包括：初始化车道标线、设置车道标线类型、获取车道标线类型、
 *          清空车道标线数据等。
 */

#pragma once

#include <string>
#include <vector>

#include "common_typedefs.h"
#include "gpal-interface/perception/env_road_instance.pb.h"
#include "gpal-interface/planning/trajectory.pb.h"
#include "math/aabox2d.h"
#include "math/line_segment2d.h"
#include "math/vec2d.h"
#include "proto/common/pnc_point.pb.h"

namespace gpal::pnc::planning::road_instance {

using Vec2d = pnc::planning::math::Vec2d;
using Vec3d = pnc::planning::math::Vec3d;
using SLPoint = pnc::SLPoint;
using AABox2d = pnc::planning::math::AABox2d;
using LineSegment2d = pnc::planning::math::LineSegment2d;
using BoundInfo = gpal::pnc::planning::road_instance::PerceptionCenterLine::BoundInfo;

class LaneMarking;
using LaneMarkingPtr = std::shared_ptr<LaneMarking>;

class LaneMarking {
 public:
  struct RelatedReferenceLineInfo {
    std::string ref_line_id = "";  ///< 关联的参考线ID
    int8_t marking_lateral_position = 0;  ///< 车道线相对参考线的横向位置，0：未知，1：左车道线，-1：右车道线
    float start_s_in_line = 0.0;     ///< 关联区域在参考线的起始s值
    float end_s_in_line = 0.0;       ///< 关联区域在参考线的结束s值
    float start_s_in_marking = 0.0;  ///< 关联区域在本车道线的起始s值
    float end_s_in_marking = 0.0;    ///< 关联区域在本车道线的结束s值
  };

  LaneMarking() = default;
  ~LaneMarking() = default;
  void clear();

  // 基础数据查询、更新接口
  const std::string& getId() const { return id_; }
  std::string& getMutableId() { return id_; }

  const proto::perception::LaneMarking::LineType& getLineType() const { return type_; }
  proto::perception::LaneMarking::LineType& getMutableLineType() { return type_; }

  const proto::perception::LaneMarking::LineShape& getLineShape() const { return shape_; }
  proto::perception::LaneMarking::LineShape& getMutableLineShape() { return shape_; }

  const proto::perception::LaneMarking::LineColor& getLineColor() const { return color_; }
  proto::perception::LaneMarking::LineColor& getMutableLineColor() { return color_; }

  const std::vector<Vec3d>& getPoints() const { return pts_; }
  std::vector<Vec3d>& getMutablePoints() { return pts_; }

  const std::vector<RelatedReferenceLineInfo>& getRelatedReferenceLineInfos() const { return reference_line_infos_; }
  std::vector<RelatedReferenceLineInfo>& getMutableRelatedReferenceLineInfos() { return reference_line_infos_; }

  const std::vector<KeyPoint>& getKeyPoints() const { return key_points_; }
  std::vector<KeyPoint>* mutableKeyPoints() { return &key_points_; }

  double setDelayTime(double t) { return time_delay_ = t; }

  // 用于KdTree搜索相关接口，需要init之后使用
  bool init();
  bool isInited() { return is_inited_; }
  double DistanceSquareTo(const Vec2d& point) const;
  double DistanceSquareTo(const Vec2d& point, double heading) const;
  /**
   * @brief 计算目标点到车道线的距离平方（带符号）
   *        符号表示线上最近点相对目标点的左右位置，
   *        - 正值：线上最近点在目标点左侧
   *        - 负值： 线上最近点在目标点右侧
   *
   * @param point 目标点
   * @param heading 目标点的朝向角
   * @param enable_heading_search 是否启用朝向搜索，
   *                              若不启用，则仅用于计算线上最近点相对目标点的左右位置
   * @param is_point_as_base 计算带符号的距离时是否以point+heading作为基准向量；若为false, 则以车道线本身为基准向量
   * @param proj_s point相对车道线的s坐标，米
   * @return double 有符号距离平方
   */
  double DistanceSignedSquareTo(const Vec2d& point, double heading, bool enable_heading_search, bool is_point_as_base,
                                double& proj_s) const;

  const AABox2d& getAABox() const { return aabox_; }
  AABox2d& getMutableAABox() { return aabox_; }
  const std::vector<LineSegment2d>& getLineSegmentArray() const { return line_segments_; }
  std::vector<LineSegment2d>& getMutableLineSegmentArray() { return line_segments_; }
  const std::vector<float>& getLengthArray() const { return lens_; }
  std::vector<float>& getMutableLengthArray() { return lens_; }

  //  坐标转换相关，需要init之后使用
  template<class XYZPoint>
  bool xy2sl(const XYZPoint& xyz, double& s, double& l) const {
    return xy2sl(Vec3d(xyz.x(), xyz.y(), xyz.z()), s, l);
  }
  bool xy2sl(const Vec3d& xyz_point, double& s, double& l) const;

  // 投影相关，需要init之后使用
  bool getProjection(const Vec3d& point, float& accumulate_s, float& lateral, float& min_distance,
                     int& min_index) const;
  void calcProjection(const Vec3d& point, const float& min_distance, const int& min_index, float& accumulate_s,
                      float& lateral) const;
  float getLength() const;

  bool calcInterSection(const LineSegment2d& line_seg, BoundInfo& bound_info, double& s_in_marking,
                        double& dist_to_line_seg_start) const;

 private:
  int findNearestLineSegmentIndex(const Vec2d& point) const;
  bool calcTwoLineSegmentInterSection(const LineSegment2d& seg1, const LineSegment2d& seg2, double& intersection_x,
                                      double& intersection_y, double& s_in_seg1, double& s_in_seg2) const;

 private:
  std::string id_ = "";  ///< 车道线跟踪ID, 时空唯一
  proto::perception::LaneMarking::LineType type_ = proto::perception::LaneMarking::kTypeUnknown;  ///< 车道线类型
  proto::perception::LaneMarking::LineShape shape_ =
      proto::perception::LaneMarking::kShapeUnknown;  ///< 车道线线型;
                                                      ///< 枚举值小于50(不含)用于地面标线,
                                                      ///< 大于50(不含)用于道路边界，对应
                                                      ///< type 为 kTypeRoadSide
  proto::perception::LaneMarking::LineColor color_ = proto::perception::LaneMarking::kColorUnknown;  ///< 车道线颜色
  std::vector<Vec3d> pts_;  ///< 形点, 当前仅 x,y 维度有效, unit: m, 按车辆行驶方向单向排序;
  std::vector<RelatedReferenceLineInfo> reference_line_infos_;  ///< 关联的参考线信息集合
  // 用于KdTree搜索
  bool is_inited_ = false;                    ///< 初始化状态
  AABox2d aabox_;                             ///< AABB box
  std::vector<LineSegment2d> line_segments_;  ///< 线段数组
  std::vector<float> lens_;                   ///< S长度数组
  std::vector<KeyPoint> key_points_;          ///< 关键点数组

  double time_delay_ = 0.0;  ///< 时延
};

}  // namespace gpal::pnc::planning::road_instance
