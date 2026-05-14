#pragma once

/**
 * @file
 * @brief Defines the Freespace class.
 */

#include <array>

#include "basic_algorithm_lib/basic_algorithm_lib.h"
// #include "vehicle_config.pb.h"
#include <math/box2d.h>
#include <math/circle.h>

#include "config/freespace_config.pb.h"
namespace gpal::pnc::planning {
/**
 * @class Freespace
 *
 * @brief Implements a class of Freespace Data.
 */

using namespace std;

// constexpr int kDefaultMapLenth = 800;  // 二维地图长度
// constexpr int kDefaultMapWidth = 400;  // 二维地图宽度
// constexpr int kMapVehOriX = 200;       // 地图中车辆原点x坐标
// constexpr int kMapVehOriY = 200;       // 地图中车辆原点y坐标
constexpr int kInfFactor = 800;  // 间隙地图膨胀系数

struct CircleData {
  float x;
  float y;
  float radius;
  float radius2veh;  // 该圆的圆心到车体坐标系原点的圆的半径
  float theta;       // 该圆圆心到车体坐标系原点连线的角度，x轴正向逆时针旋转为正
  CircleData() {
    x = 0;
    y = 0;
    radius = 0;
    theta = 0;
    radius2veh = 0;
  }
};

struct VehCollisonData_ {
  CircleData OutCircle;
  CircleData RecCircle[2];
  CircleData SquareCircle[4];
};

struct ClearancePoint {
  int x;
  int y;
  ClearancePoint() {
    x = 0;
    y = 0;
  }
};

class GridMap {
 public:
  GridMap() = default;
  void init(GridData grid_data = GridData());

  void setOccupy(const float x, const float y);
  void reset();
  void update();

  void setResolution(const double resolution);
  float resolution() const { return grid_data_.resolution(); }

  const GridData& grid_data() const { return grid_data_; }
  GridData* mutable_grid_data() { return &grid_data_; }

  float left_boundary() const { return left_boundary_; }
  float right_boundary() const { return right_boundary_; }

  float top_boundary() const { return top_boundary_; }
  float bottom_boundary() const { return bottom_boundary_; }

  std::pair<int16_t, int16_t> getIndexPair(const double x, const double y) const;
  math::Vec2d getVec2d(const int16_t w, const int16_t l) const;

  bool checkLateralBoundary(const double x, const double y) const;
  bool checkLongiBoundary(const double x, const double y) const;

  float distance(const int16_t w, const int16_t l) const;
  float distance(const std::pair<int16_t, int16_t>& index_pair) const;

  std::pair<int16_t, int16_t> min_index(const int16_t w, const int16_t l) const;
  std::pair<int16_t, int16_t> min_index(const std::pair<int16_t, int16_t>& index_pair) const;

  float max_distance() const;

 protected:
  struct GridCell {
    uint16_t distance;
    std::pair<int16_t, int16_t> min_index;
    GridCell(const uint16_t _distance,
             const std::pair<int16_t, int16_t> _min_index = std::pair<int16_t, int16_t>(-1, -1))
        : distance(_distance), min_index(_min_index) {}
  };

 protected:
  std::vector<std::vector<GridCell>> grid_cells_;
  GridData grid_data_;

  float max_distance_ = 0.0;

  float grid_distance_ = 0.0;
  float grid_resolution_2_ = 0.0;

  float left_boundary_ = 0.0;
  float right_boundary_ = 0.0;
  float top_boundary_ = 0.0;
  float bottom_boundary_ = 0.0;
};

class Freespace : public StampedBase {
 protected:
 public:
  friend class PerceptionFreespaceAdapter;
  Freespace();
  virtual ~Freespace();

  void Clear();

  FreespaceConfig* mutable_config() { return &config_; }
  const FreespaceConfig& config() const { return config_; }
  GridMap* mutable_grid_map() { return &grid_map_; }
  const GridMap& grid_map() const { return grid_map_; }

  const std::pair<double, double> dynamic_minx_point() const { return dynamic_minx_point_; }
  const std::pair<double, double> dynamic_miny_point() const { return dynamic_miny_point_; }

  // init
  void Init();
  void init(const FreespaceConfig& config);
  void Init4CollisionChecking();

  void initGridMap();
  void resetGridMap();
  void updateGridMap();

  bool isOutOfMap(const float& x, const float& y) const;
  float Resolution() const { return grid_map_.resolution(); }

  // get nearest dis from clearance map
  bool isOccupied(const math::Box2d& box, math::Vec2d* key_vec, const double round_corner_width = 0.0) const;
  double getMinFreespaceDisFromBox(const math::Box2d& box, math::Vec2d* key_vec) const;
  float GetDisFromClearanceMap(const float& x, const float& y, ClearancePoint& p) const;
  float GetDisFromClearanceMap(const float& x, const float& y, ClearancePoint& p,
                               const std::pair<double, double>& check_tb_range) const;
  float GetDisFromClearanceMap(const float& x, const float& y, ClearancePoint& p,
                               const std::pair<double, double>& check_tb_range,
                               const std::pair<double, double>& check_lr_range) const;

  void testGeneCheckCircles();

  std::pair<double, double> getLateralBoundary(const math::Box2d& box, const std::pair<double, double>& search_range,
                                               const double round_corner_width = 0.0) const;
  std::pair<double, double> getLateralBoundary(const math::Vec2d& rpt, const double heading,
                                               const std::pair<double, double>& search_range,
                                               const double buffer = 0.0) const;
  // park freespace visualization
  void freespaceVisualizationForParking(ClearancePoint& pt, math::Vec2d* key_vec) const;

 protected:
  std::pair<double, double> getOffset(const math::Vec2d& rpt, const math::Vec2d& unit_direction,
                                      const math::Vec2d& pt) const;
  std::pair<double, double> getBoxCenterOffset(const math::Box2d& box, const math::Vec2d& pt) const;
  bool isInBox(const math::Box2d& box, const double longi, const double lateral, const double grid_radius,
               const double round_corner_width = 0.0) const;
  double getLeftOffset(const math::Box2d& box, const double longi, const double lateral, const double grid_radius,
                       const double round_corner_width = 0.0) const;
  double getRightOffset(const math::Box2d& box, const double longi, const double lateral, const double grid_radius,
                        const double round_corner_width = 0.0) const;

 protected:
  bool init_ = false;
  FreespaceConfig config_;
  GridMap grid_map_;
  std::pair<double, double> dynamic_minx_point_;
  std::pair<double, double> dynamic_miny_point_;

  VehCollisonData_ VehCollisonData;

  bool LRBoundaryCheck(const double x, const double y, const double l, const double r) const;
  bool TBBoundaryCheck(const double x, const double y, const double t, const double b) const;
  ClearancePoint CodTransf_Veh2Clr(const double x, const double y) const;

  void geneCheckCircles(math::Box2d box, std::vector<math::Circle>* check_circles) const;
  void modifyBoxIfNeed(math::Box2d* box) const;

 protected:
  static constexpr double grid_radius_coefs_ = 0.707106781;
};

}  // namespace gpal::pnc::planning
