/**
 * @copyright Copyright(C): Geometrical-Pal, 2024-2026, All Rights Reserved
 *
 * @file    geometry_utils.h
 * @brief   Geometry Utility Library: Computes geometric relationships between lines, circles, and points
 *          (e.g., intersections, tangents, translations).
 * @author  WeiTang(weitang@geometricalpal.com)
 * @version 2.0
 * @date    2025-05-14
 */
#pragma once
#include "basic_algorithm_lib/basic_algorithm_lib.h"
#include "config_manager/config_manager.h"
#include "math/box2d.h"
#include "math/circle.h"
#include "math/line_segment2d.h"
#include "openspace_path_planner/utils/openspace_common.h"
#include "point/path_pt.h"

namespace gpal::pnc::planning {

/**
 * @brief 计算直线和圆弧的交点
 * @param line 直线段
 * @param circle 圆
 * @param[out] cross_pt 交点结果（可能返回0、1或2个点）
 * @param check_on_segment 考虑是否需要交点在线段内部
 * @return 返回交点的个数
 */

uint8_t calCrossPointOfLineAndCircle(const math::LineSegment2d& line, const math::Circle& circle,
                                     std::vector<PathPt>& cross_pt, bool check_on_segment = false);

/**
 * @brief 计算与直线相切于指定点的圆
 * @param line 直线段
 * @param tangent_point 直线上的切点
 * @param radius 圆的半径
 * @param[out] circles 输出的圆（最多两个解）
 * @return 圆的个数
 */
uint8_t calCirclesTangentToLinePoint(const math::LineSegment2d& line, const math::Vec2d& tangent_point,
                                     const double radius, std::vector<math::Circle>& circles);

/**
 * @brief 计算与直线相切且通过指定点的圆（0/1/2个解）
 * @param line 直线段（取其所在直线）
 * @param point 圆必须经过的点
 * @param radius 圆的半径
 * @param[out] tangent_circles 输出的相切圆集合
 * @return 实际找到的圆的数量
 */
uint8_t calCirclesTangentToLineThroughPoint(const math::LineSegment2d& line, const math::Vec2d& point, double radius,
                                            std::vector<math::Circle>& tangent_circles);

/**
 * @brief 平移直线段
 * @param line 原始直线段
 * @param distance 平移距离
 * @param direction 平移方向向量（不需要单位化）
 * @return 平移后的直线段
 */
math::LineSegment2d translateLine(const math::LineSegment2d& line, double distance, const math::Vec2d& direction);

/**
 * @brief 判断点是否在直线上
 * @param line 原始直线段
 * @param point 判断点
 * @param tolerance 距离容差
 * @return bool
 */
bool isPointOnLine(const math::LineSegment2d& line, const math::Vec2d& point, double tolerance = 1e-6);

/**
 * @brief   过一点计算直线的垂线
 * @param   line        目标直线
 * @param   point       点
 * @param   extension_length 垂直段长度
 * @return  math::LineSegment2d
 */
math::LineSegment2d calPerpendicularLineThroughPoint(const math::LineSegment2d& line, const math::Vec2d& point,
                                                     double extension_length = 1.0);
/**
 * @brief 通过点并沿其方向生成直线段
 * @param point 点
 * @param extension_length 线段长度
 * @return 表示直线的线段
 */
math::LineSegment2d calLineThroughPointInDirection(const PathPt& point, double extension_length = 1.0);

/**
 * @brief 计算从参考点沿方向纵向和横向移动后的新坐标
 * @param point 参考点
 * @param forward_dist 前方距离（正数）
 * @param left_dis 左侧距离（正数）
 * @return 新的坐标点
 */
PathPt calOffsetPoint(const PathPt& point, double forward_dis, double left_dis = 0.0);

/**
 * @brief 计算两个圆的交点
 * @param circle1 第一个圆
 * @param circle2 第二个圆
 * @param[out] cross_pts 交点集合
 * @return 交点数量 (0: 不相交, 1: 相切, 2: 相交)
 */
uint8_t calCrossPointOfTwoCircles(const math::Circle& circle1, const math::Circle& circle2,
                                  std::vector<PathPt>& cross_pts);

/**
 * @brief 对圆上的点进行排序
 * @param circle  目标圆
 * @param base_point 基准点
 * @param[out] points 待排序的点集合
 * @param clockwise 是否顺时针排序
 * @return void
 */
void sortPointsOnCircle(const math::Circle& circle, const PathPt& base_point, std::vector<PathPt>& points,
                        bool clockwise);

/**
 * @brief 生成圆上两点之间的轨迹点
 * @param circle 圆对象
 * @param start_pt 起点（必须在圆上）
 * @param end_pt 终点（必须在圆上）
 * @param clockwise 是否顺时针方向生成
 * @param step_length 点距（弧长）
 * @return 生成的轨迹点序列（包含起点和终点）
 */
std::vector<PathPt> generatePathOnCircle(const math::Circle& circle, const PathPt& start_pt, const PathPt& end_pt,
                                         bool clockwise, double step_length = 0.1);

}  // namespace gpal::pnc::planning