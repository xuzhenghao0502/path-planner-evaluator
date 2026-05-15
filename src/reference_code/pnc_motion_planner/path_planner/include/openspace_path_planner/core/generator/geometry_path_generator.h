/**
 * @copyright Copyright(C): Geometrical-Pal, 2024-2026, All Rights Reserved
 *
 * @file    geometry_path_generator.h
 * @brief   Geometry-based path generation for open-space parking scenarios.
 *          This class provides algorithms to generate fine-tuning paths
 *          considering vehicle geometry, slot boundaries and collision avoidance.
 * @author  WeiTang(weitang@geometricalpal.com)
 * @version 2.0
 * @date    2025-05-14
 */

#pragma once
#include <openspace_path_planner/utils/geometry_utils.h>

#include "config_manager/config_manager.h"
#include "local_view/local_view.h"
#include "openspace_path_planner/utils/openspace_common.h"
#include "openspace_path_planner/utils/openspace_tools.h"
#include "openspace_path_planner/utils/slot.h"
namespace gpal::pnc::planning {
class GeometryPathGenerator {
 public:
  GeometryPathGenerator() {};
  ~GeometryPathGenerator() {};
  void init();
  /**
   * @brief Computes multiple fine-tuning path candidates inside a parallel parking slot
   * @param slot The target parking slot geometry
   * @param start_pt Initial vehicle pose (position + heading)
   * @param radius Minimum turning radius constraint
   * @param max_fine_tuning_num Maximum number of path variations to generate
   * @param[out] fine_tuning_paths Generated path candidates (each as a sequence of PathPt)
   * @return true if at least one valid path was generated
   */
  bool calFineTuningPathInsideParallelSlot(const Slot& slot, const PathPt& start_pt, const double radius,
                                           const int max_fine_tuning_num,
                                           std::vector<std::vector<PathPt>>& fine_tuning_paths);
  /**
   * @brief Generates a single alignment path considering boundary constraints
   * @param start_pt Initial vehicle pose
   * @param block_boundarys Obstacle boundaries as line segments
   * @param radius Minimum turning radius
   * @param ref_direction Preferred moving direction vector
   * @param safe_distance safe distance from block_boundarys
   * @param[out] single_path Resulting collision-free path
   * @return true if a valid path was generated
   */
  bool calSingleFineTuningAlignPath(const PathPt& start_pt, const std::vector<math::LineSegment2d>& block_boundarys,
                                    const double radius, const math::Vec2d& ref_direction, const double safe_distance,
                                    std::vector<PathPt>& single_path);
  /**
   * @brief Sets vehicle geometry parameters (for gtest)
   * @param veh_ego_2_rear Distance from ego point to rear edge
   * @param veh_ego_2_front Distance from ego point to front edge
   * @param veh_width  Vehicle width
   */
  void setVehParam(double veh_ego_2_rear, double veh_ego_2_front, double veh_width);

 private:
  ConfigManager* config_manager_ = nullptr;
  double veh_ego_2_rear_ = 0.0;
  double veh_ego_2_front_ = 0.0;
  double veh_width_ = 0.0;
};

}  // namespace gpal::pnc::planning
