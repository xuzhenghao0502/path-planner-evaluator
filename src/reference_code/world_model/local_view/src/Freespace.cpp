#include "local_view/Freespace.h"

#include <queue>

#include "config_manager/config_manager.h"
namespace gpal::pnc::planning {

void GridMap::init(GridData grid_data) {
  grid_data_ = std::move(grid_data);
  max_distance_ = grid_data_.resolution() * kInfFactor;
  grid_distance_ = grid_data_.resolution() / 3;  // 用于计算间隙地图中某点的最近障碍物真实距离的转换参数
  grid_resolution_2_ = grid_data_.resolution() * grid_data_.resolution();  // 栅格地图分辨率的平方
  left_boundary_ = (grid_data_.origin_y() - 1) * grid_data_.resolution();  // 查询间隙地图时的边界
  right_boundary_ = -(grid_data_.map_width() - grid_data_.origin_y() - 1) * grid_data_.resolution();
  top_boundary_ = (grid_data_.map_length() - grid_data_.origin_x()) * grid_data_.resolution();
  if (grid_data_.has_force_bottom_boundary()) {
    bottom_boundary_ = -grid_data_.force_bottom_boundary();  // 限制从下侧搜索出路径
  } else {
    bottom_boundary_ = -grid_data_.origin_x() * grid_data_.resolution();
  }
  if (grid_data_.map_length() > 0 && grid_data_.map_width() > 0) {
    if (grid_data_.map_width() != grid_cells_.size() || grid_data_.map_length() != grid_cells_.front().size()) {
      grid_cells_ = std::vector<std::vector<GridCell>>(
          grid_data_.map_width(), std::vector<GridCell>(grid_data_.map_length(), GridCell(kInfFactor)));
    } else {
      for (int i = 0; i < grid_cells_.size(); i++) {
        for (int j = 0; j < grid_cells_[i].size(); j++) {
          grid_cells_[i][j] = GridCell(kInfFactor);
        }
      }
    }
  } else {
    grid_cells_.clear();
  }
}

void GridMap::setOccupy(const float x, const float y) {
  auto index_pair = getIndexPair(x, y);
  // for (const auto& grid_cell : grid_cells_) {
  //   ERT_PLOG_I << "grid_cell size: " << grid_cell.size() ;
  // }
  if (index_pair.first >= 0 && index_pair.first < grid_cells_.size()) {
    if (index_pair.second >= 0 && index_pair.second < grid_cells_[index_pair.first].size()) {
      grid_cells_[index_pair.first][index_pair.second].distance = 0;
      grid_cells_[index_pair.first][index_pair.second].min_index = index_pair;
      // ERT_PLOG_I << " grid.min_idex = " << index_pair.first<< ","
      //           << index_pair.second ;
    }
  }
}

void GridMap::reset() { grid_cells_.clear(); }

void GridMap::update() {
  if (grid_cells_.empty() || grid_cells_.front().empty()) {
    return;
  }
  constexpr std::array<std::pair<int, int>, 4> fps{std::pair<int, int>(0, -1), std::pair<int, int>(-1, 1),
                                                   std::pair<int, int>(-1, 0), std::pair<int, int>(-1, -1)};
  constexpr std::array<std::pair<int, int>, 4> bps{std::pair<int, int>(0, 1), std::pair<int, int>(1, -1),
                                                   std::pair<int, int>(1, 0), std::pair<int, int>(1, 1)};
  constexpr std::array<int, 4> dist_buffer{3, 4, 3, 4};
  for (int i = 1; i < grid_cells_.size(); i++) {
    for (int j = 1; j < grid_cells_[i].size() - 1; j++) {
      auto& curr_cell = grid_cells_[i][j];
      if (curr_cell.distance) {
        for (int k = 0; k < 4; k++) {
          auto& cell = grid_cells_[i + fps[k].first][j + fps[k].second];
          int cell_distance = cell.distance + dist_buffer[k];
          if (cell_distance < curr_cell.distance) {
            curr_cell.distance = cell_distance;
            curr_cell.min_index = cell.min_index;
          }
        }
      }
    }
  }

  // 倒角距离变化:3)使用后向模板b对前向遍历的结果矩阵f1从右到左、从下到上的进行移动遍历。操作与上一步相同，得到矩阵f2。同上，不计算矩阵的最后1行，第1列和最后1列。
  // 后向遍历阶段:
  for (int i = grid_cells_.size() - 3; i > 0; i--) {
    for (int j = grid_cells_[i].size() - 2; j > 1; j--) {
      auto& curr_cell = grid_cells_[i][j];
      if (curr_cell.distance) {
        for (int k = 0; k < 4; k++) {
          auto& cell = grid_cells_[i + bps[k].first][j + bps[k].second];
          int cell_distance = cell.distance + dist_buffer[k];
          if (cell_distance < curr_cell.distance) {
            curr_cell.distance = cell_distance;
            curr_cell.min_index = cell.min_index;
          }
        }
      }
    }
  }
}

std::pair<int16_t, int16_t> GridMap::getIndexPair(const double x, const double y) const {
  return std::pair<int16_t, int16_t>(static_cast<int>(round(-y / grid_data_.resolution())) + grid_data_.origin_y(),
                                     static_cast<int>(round(x / grid_data_.resolution())) + grid_data_.origin_x());
}

math::Vec2d GridMap::getVec2d(const int16_t w, const int16_t l) const {
  double y = -(w - grid_data_.origin_y()) * grid_data_.resolution();
  double x = (l - grid_data_.origin_x()) * grid_data_.resolution();
  return math::Vec2d(x, y);
}

float GridMap::distance(const int16_t w, const int16_t l) const {
  if (w >= 0 && w < grid_cells_.size()) {
    if (l >= 0 && l < grid_cells_[w].size()) {
      return grid_distance_ * grid_cells_[w][l].distance;
    }
  }
  return max_distance();
}

float GridMap::distance(const std::pair<int16_t, int16_t>& index_pair) const {
  return distance(index_pair.first, index_pair.second);
}

std::pair<int16_t, int16_t> GridMap::min_index(const int16_t w, const int16_t l) const {
  if (w >= 0 && w < grid_cells_.size()) {
    if (l >= 0 && l < grid_cells_[w].size()) {
      return grid_cells_[w][l].min_index;
    }
  }
  return std::pair<int16_t, int16_t>(-1, -1);
}

std::pair<int16_t, int16_t> GridMap::min_index(const std::pair<int16_t, int16_t>& index_pair) const {
  return min_index(index_pair.first, index_pair.second);
}

float GridMap::max_distance() const { return max_distance_; }

void GridMap::setResolution(const double resolution) {
  grid_data_.set_resolution(resolution);
  max_distance_ = grid_data_.resolution() * kInfFactor;
  grid_distance_ = grid_data_.resolution() / 3;  // 用于计算间隙地图中某点的最近障碍物真实距离的转换参数
  grid_resolution_2_ = grid_data_.resolution() * grid_data_.resolution();  // 栅格地图分辨率的平方
  left_boundary_ = (grid_data_.origin_y() - 1) * grid_data_.resolution();  // 查询间隙地图时的边界
  right_boundary_ = -(grid_data_.map_width() - grid_data_.origin_y() - 1) * grid_data_.resolution();
  top_boundary_ = (grid_data_.map_length() - grid_data_.origin_x()) * grid_data_.resolution();
}

Freespace::Freespace() { Clear(); }

Freespace::~Freespace() {}

void Freespace::Clear() { StampedBase::reset(); }

void Freespace::Init() {
  ConfigManager* config_manager = Singleton<ConfigManager>::get_instance();
  init(config_manager->getConfig<FreespaceConfig>("FreespaceConfig"));
  // config_manager->registConfigCallback<FreespaceConfig>("FreespaceConfig",
  //                                                       {[=](const FreespaceConfig& config) { init(config); }});
  init_ = true;
}

void Freespace::init(const FreespaceConfig& config) {
  config_ = config;
  initGridMap();
  Init4CollisionChecking();
}

void Freespace::Init4CollisionChecking()  // 计算七圆碰撞校验模型中的参数.
{
  ConfigManager* config_manager = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager);
  auto& vehicle_param = config_manager->vehicle_config().vehicle_param();
  const auto vehicleLength = vehicle_param.length();
  const auto vehicleWidth = vehicle_param.width();
  const auto rearOverhang = vehicle_param.rear_overhang();
  // 外包络圆.
  VehCollisonData.OutCircle.radius = sqrt(vehicleLength * vehicleLength + vehicleWidth * vehicleWidth) / 2;
  VehCollisonData.OutCircle.x = vehicleLength / 2 - rearOverhang;
  VehCollisonData.OutCircle.y = 0.0;
  VehCollisonData.OutCircle.theta = atan2(VehCollisonData.OutCircle.y, VehCollisonData.OutCircle.x);
  VehCollisonData.OutCircle.radius2veh = Distance(VehCollisonData.OutCircle.y, VehCollisonData.OutCircle.x);

  // 以w/4为半径的四个正方形小圆.
  // 左下圆.
  VehCollisonData.SquareCircle[0].radius = sqrt(vehicleWidth * vehicleWidth / 2) / 2;
  VehCollisonData.SquareCircle[0].x = vehicleWidth / 4 - rearOverhang;
  VehCollisonData.SquareCircle[0].y = vehicleWidth / 4;
  VehCollisonData.SquareCircle[0].theta = atan2(VehCollisonData.SquareCircle[0].y, VehCollisonData.SquareCircle[0].x);
  VehCollisonData.SquareCircle[0].radius2veh =
      Distance(VehCollisonData.SquareCircle[0].y, VehCollisonData.SquareCircle[0].x);
  // 右下圆.
  VehCollisonData.SquareCircle[1].radius = VehCollisonData.SquareCircle[0].radius;
  VehCollisonData.SquareCircle[1].x = VehCollisonData.SquareCircle[0].x;
  VehCollisonData.SquareCircle[1].y = -VehCollisonData.SquareCircle[0].y;
  VehCollisonData.SquareCircle[1].theta = atan2(VehCollisonData.SquareCircle[1].y, VehCollisonData.SquareCircle[1].x);
  VehCollisonData.SquareCircle[1].radius2veh =
      Distance(VehCollisonData.SquareCircle[1].y, VehCollisonData.SquareCircle[1].x);
  // 左上圆.
  VehCollisonData.SquareCircle[2].radius = VehCollisonData.SquareCircle[0].radius;
  VehCollisonData.SquareCircle[2].x = vehicleLength - vehicleWidth / 4 - rearOverhang;
  VehCollisonData.SquareCircle[2].y = VehCollisonData.SquareCircle[0].y;
  VehCollisonData.SquareCircle[2].theta = atan2(VehCollisonData.SquareCircle[2].y, VehCollisonData.SquareCircle[2].x);
  VehCollisonData.SquareCircle[2].radius2veh =
      Distance(VehCollisonData.SquareCircle[2].y, VehCollisonData.SquareCircle[2].x);
  // 右上圆.
  VehCollisonData.SquareCircle[3].radius = VehCollisonData.SquareCircle[0].radius;
  VehCollisonData.SquareCircle[3].x = VehCollisonData.SquareCircle[2].x;
  VehCollisonData.SquareCircle[3].y = VehCollisonData.SquareCircle[1].y;
  VehCollisonData.SquareCircle[3].theta = atan2(VehCollisonData.SquareCircle[3].y, VehCollisonData.SquareCircle[3].x);
  VehCollisonData.SquareCircle[3].radius2veh =
      Distance(VehCollisonData.SquareCircle[3].y, VehCollisonData.SquareCircle[3].x);

  // 两个矩形圆
  float w = (vehicleLength - vehicleWidth) / 2;
  float l = vehicleWidth;
  VehCollisonData.RecCircle[0].radius = sqrt(l * l + w * w) / 2;
  VehCollisonData.RecCircle[0].x = l / 2 + w / 2 - rearOverhang;
  VehCollisonData.RecCircle[0].y = 0;
  VehCollisonData.RecCircle[0].theta = atan2(VehCollisonData.RecCircle[0].y, VehCollisonData.RecCircle[0].x);
  VehCollisonData.RecCircle[0].radius2veh = Distance(VehCollisonData.RecCircle[0].y, VehCollisonData.RecCircle[0].x);

  VehCollisonData.RecCircle[1].radius = VehCollisonData.RecCircle[0].radius;
  VehCollisonData.RecCircle[1].x = l / 2 + w * 3 / 2 - rearOverhang;
  VehCollisonData.RecCircle[1].y = 0;
  VehCollisonData.RecCircle[1].theta = atan2(VehCollisonData.RecCircle[1].y, VehCollisonData.RecCircle[1].x);
  VehCollisonData.RecCircle[1].radius2veh = Distance(VehCollisonData.RecCircle[1].y, VehCollisonData.RecCircle[1].x);
}

void Freespace::initGridMap() { grid_map_.init(config_.grid_data()); }

void Freespace::resetGridMap() {
  if (!init_) {
    Init();
  }
  grid_map_.reset();
}

void Freespace::updateGridMap() {
  if (!init_) {
    Init();
  }
  grid_map_.update();
}

// 判断查询的位置是否在间隙地图边界内，1代表在边界内
bool Freespace::LRBoundaryCheck(const double x, const double y, const double l, const double r) const {
  return r <= y && y <= l;
}

// 判断查询的位置是否在间隙地图边界内，1代表在边界内
bool Freespace::TBBoundaryCheck(const double x, const double y, const double t, const double b) const {
  return b <= x && x <= t;
}

ClearancePoint Freespace::CodTransf_Veh2Clr(const double x,
                                            const double y) const  // 将车体坐标系坐标转换为间隙地图中的栅格坐标
{
  ClearancePoint point;
  std::tie(point.x, point.y) = grid_map_.getIndexPair(x, y);
  return point;
}

bool Freespace::isOutOfMap(const float& x, const float& y) const {
  return !TBBoundaryCheck(x, y, grid_map_.top_boundary(), grid_map_.bottom_boundary()) ||
         !LRBoundaryCheck(x, y, grid_map_.left_boundary(), grid_map_.right_boundary());
}
// 查询某点间隙地图中对应的最近障碍物真实距离
float Freespace::GetDisFromClearanceMap(const float& x, const float& y, ClearancePoint& p) const {
  return GetDisFromClearanceMap(x, y, p,
                                std::pair<double, double>(grid_map_.top_boundary(), grid_map_.bottom_boundary()),
                                std::pair<double, double>(grid_map_.left_boundary(), grid_map_.right_boundary()));
}

float Freespace::GetDisFromClearanceMap(const float& x, const float& y, ClearancePoint& p,
                                        const std::pair<double, double>& check_tb_range) const {
  return GetDisFromClearanceMap(x, y, p, check_tb_range,
                                std::pair<double, double>(grid_map_.left_boundary(), grid_map_.right_boundary()));
}

float Freespace::GetDisFromClearanceMap(const float& x, const float& y, ClearancePoint& p,
                                        const std::pair<double, double>& check_tb_range,
                                        const std::pair<double, double>& check_lr_range) const {
  if (!TBBoundaryCheck(x, y, check_tb_range.first, check_tb_range.second)) {
    return grid_map_.max_distance();
  } else if (!LRBoundaryCheck(x, y, check_lr_range.first, check_lr_range.second)) {
    return grid_map_.max_distance();
  }

  p = CodTransf_Veh2Clr(x, y);

  // 遍历结束后除以3作为最终结果.
  return grid_map_.distance(p.x, p.y);
}

// check all clearance points in the box are occupied or not
bool Freespace::isOccupied(const math::Box2d& box, math::Vec2d* key_vec, const double round_corner_width) const {
  ClearancePoint pt;

  std::vector<math::Circle> check_circles;
  geneCheckCircles(box, &check_circles);
  const double grid_radius = grid_map_.resolution() * grid_radius_coefs_;
  for (auto& cir : check_circles) {
    double df = GetDisFromClearanceMap(cir.center().x(), cir.center().y(), pt) - cir.radius();
    if (df < 0) {
      auto key_pt = grid_map_.min_index(pt.x, pt.y);
      auto vec = grid_map_.getVec2d(key_pt.first, key_pt.second);
      auto [longi, lateral] = getBoxCenterOffset(box, vec);
      if (isInBox(box, longi, lateral, grid_radius, round_corner_width)) {
        *key_vec = vec;
        return true;
      }
    }
  }
  return false;
}

// get min dis and min dis point to input box
double Freespace::getMinFreespaceDisFromBox(const math::Box2d& box, math::Vec2d* key_vec) const {
  double final_min_dis = kPostiveInfinity;
  ClearancePoint pt;

  std::vector<math::Circle> check_circles;
  geneCheckCircles(box, &check_circles);
  const double grid_radius = grid_map_.resolution() * grid_radius_coefs_;
  for (auto& cir : check_circles) {
    double df = GetDisFromClearanceMap(cir.center().x(), cir.center().y(), pt) - cir.radius();

    auto key_pt = grid_map_.min_index(pt.x, pt.y);
    auto vec = grid_map_.getVec2d(key_pt.first, key_pt.second);

    if (df < 0.0) {
      auto [longi, lateral] = getBoxCenterOffset(box, vec);
      if (isInBox(box, longi, lateral, grid_radius, 0.0)) {
        *key_vec = vec;
        return 0.0;
      }
    }

    auto min_dis = box.DistanceTo(vec);

    if (min_dis < final_min_dis) {
      *key_vec = vec;
      final_min_dis = min_dis;
    }
  }

  return final_min_dis;
}

std::pair<double, double> Freespace::getLateralBoundary(const math::Box2d& box,
                                                        const std::pair<double, double>& search_range,
                                                        const double round_corner_width) const {
  std::pair<double, double> res{-search_range.second, search_range.first};
  const double max_check_range = std::max(search_range.first, search_range.second);
  auto& [lower, upper] = res;
  std::vector<math::Circle> check_circles;
  geneCheckCircles(box, &check_circles);
  std::queue<std::tuple<math::Circle, double, double>> process_queue;
  for (auto& cir : check_circles) {
    process_queue.emplace(cir, -box.half_width(), box.half_width());
  }
  math::Vec2d lateral_unit_direction(-box.sin_heading(), box.cos_heading());
  const double grid_radius = grid_map_.resolution() * grid_radius_coefs_;
  while (!process_queue.empty()) {
    auto [cir, check_lower, check_upper] = process_queue.front();
    process_queue.pop();
    double cir_offset = getBoxCenterOffset(box, cir.center()).second;
    if (cir_offset > upper || cir_offset < lower) {
      continue;
    }
    ClearancePoint pt;
    double df = GetDisFromClearanceMap(cir.center().x(), cir.center().y(), pt) - cir.radius();
    if (df < max_check_range) {
      auto key_pt = grid_map_.min_index(pt.x, pt.y);
      auto key_vec = grid_map_.getVec2d(key_pt.first, key_pt.second);
      auto [longi, lateral] = getBoxCenterOffset(box, key_vec);
      if (std::abs(longi) <= box.half_length() + grid_map_.resolution()) {
        if (lateral > 0) {
          upper = std::min(getLeftOffset(box, longi, lateral, grid_radius, round_corner_width), upper);
        } else {
          lower = std::max(getRightOffset(box, longi, lateral, grid_radius, round_corner_width), lower);
        }
      } else {
        double check_distance = std::min(cir.radius(), (key_vec - cir.center()).Length());
        check_circles.clear();
        if (check_lower > -max_check_range && cir_offset - check_distance < check_lower) {
          check_lower = cir_offset - check_distance;
          check_circles.emplace_back(cir.center() - check_distance * lateral_unit_direction, cir.radius());
        }
        if (check_upper < max_check_range && cir_offset + check_distance > check_upper) {
          check_upper = cir_offset + check_distance;
          check_circles.emplace_back(cir.center() + check_distance * lateral_unit_direction, cir.radius());
        }
        for (auto& check_cir : check_circles) {
          process_queue.emplace(check_cir, check_lower, check_upper);
        }
      }
    }
  }
  return res;
}

std::pair<double, double> Freespace::getLateralBoundary(const math::Vec2d& rpt, const double heading,
                                                        const std::pair<double, double>& search_range,
                                                        const double buffer) const {
  std::pair<double, double> res{search_range.second, search_range.first};
  const double max_check_range = std::max(std::fabs(search_range.first), std::fabs(search_range.second));
  auto& [lower, upper] = res;
  const double config_space_width = buffer + grid_map_.resolution() * grid_radius_coefs_;
  std::queue<std::tuple<math::Circle, double, double>> process_queue;
  process_queue.emplace(math::Circle(rpt, config_space_width), -config_space_width, config_space_width);
  math::Vec2d unit_direction(cos(heading), sin(heading));
  math::Vec2d lateral_unit_direction(-unit_direction.y(), unit_direction.x());
  while (!process_queue.empty()) {
    auto [cir, check_lower, check_upper] = process_queue.front();
    process_queue.pop();
    if (isOutOfMap(cir.center().x(), cir.center().y())) {
      continue;
    }
    double cir_offset = getOffset(rpt, unit_direction, cir.center()).second;
    if (cir_offset > upper || cir_offset < lower) {
      continue;
    }
    ClearancePoint pt;
    double df = GetDisFromClearanceMap(cir.center().x(), cir.center().y(), pt) - cir.radius();
    if (df < max_check_range) {
      auto key_pt = grid_map_.min_index(pt.x, pt.y);
      auto key_vec = grid_map_.getVec2d(key_pt.first, key_pt.second);
      double distance = (cir.center() - key_vec).Length();
      if (distance < cir.radius()) {
        auto [longi, lateral] = getOffset(rpt, unit_direction, key_vec);
        if (lateral > 0) {
          upper = std::min(lateral - std::sqrt(cir.radius() * cir.radius() - longi * longi), upper);
        } else {
          lower = std::max(lateral + std::sqrt(cir.radius() * cir.radius() - longi * longi), lower);
        }
      } else {
        std::vector<math::Circle> check_circles;
        if (check_lower > -max_check_range && cir_offset - distance < check_lower) {
          check_lower = cir_offset - distance;
          check_circles.emplace_back(cir.center() - distance * lateral_unit_direction, cir.radius());
        }
        if (check_upper < max_check_range && cir_offset + distance > check_upper) {
          check_upper = cir_offset + distance;
          check_circles.emplace_back(cir.center() + distance * lateral_unit_direction, cir.radius());
        }
        for (auto& check_cir : check_circles) {
          process_queue.emplace(check_cir, check_lower, check_upper);
        }
      }
    }
  }
  return res;
}

std::pair<double, double> Freespace::getBoxCenterOffset(const math::Box2d& box, const math::Vec2d& pt) const {
  std::pair<double, double> res;
  auto& [longi, offset] = res;
  const double dx = pt.x() - box.center_x();
  const double dy = pt.y() - box.center_y();
  longi = dx * box.cos_heading() + dy * box.sin_heading();
  offset = -dx * box.sin_heading() + dy * box.cos_heading();
  return res;
}

std::pair<double, double> Freespace::getOffset(const math::Vec2d& rpt, const math::Vec2d& unit_direction,
                                               const math::Vec2d& pt) const {
  std::pair<double, double> res;
  auto& [longi, lateral] = res;
  auto dpt = pt - rpt;
  longi = unit_direction.InnerProd(dpt);
  lateral = unit_direction.CrossProd(dpt);
  return res;
}

bool Freespace::isInBox(const math::Box2d& box, const double longi, const double lateral, const double grid_radius,
                        const double round_corner_width) const {
  const double longi_abs = std::abs(longi);
  const double lateral_abs = std::abs(lateral);
  const double config_half_length = box.half_length() + grid_radius;
  const double config_corner_width = round_corner_width + grid_radius;
  const double box_half_length = box.half_length() - round_corner_width;
  double lateral_check_length = box.half_width() + grid_radius;
  if (longi_abs <= config_half_length) {
    double corner_offset = longi_abs - box_half_length;
    if (1e-6 < corner_offset && corner_offset < config_corner_width - 1e-6) {
      lateral_check_length -=
          config_corner_width - std::sqrt(config_corner_width * config_corner_width - corner_offset * corner_offset);
    }
    return lateral_abs <= lateral_check_length;
  }
  return false;
}

double Freespace::getLeftOffset(const math::Box2d& box, const double longi, const double lateral,
                                const double grid_radius, const double round_corner_width) const {
  const double config_half_width = box.half_width() + grid_radius;
  const double config_corner_width = round_corner_width + grid_radius;
  const double box_half_length = box.half_length() - round_corner_width;
  double offset = lateral - config_half_width;
  double corner_offset = std::abs(longi) - box_half_length;
  if (1e-6 < corner_offset && corner_offset < config_corner_width - 1e-6) {
    offset +=
        config_corner_width - std::sqrt(config_corner_width * config_corner_width - corner_offset * corner_offset);
  }
  return offset;
}

double Freespace::getRightOffset(const math::Box2d& box, const double longi, const double lateral,
                                 const double grid_width, const double round_corner_width) const {
  const double config_half_width = box.half_width() + grid_width;
  const double config_corner_width = round_corner_width + grid_width;
  const double box_half_length = box.half_length() - round_corner_width;
  double offset = lateral + config_half_width;
  double corner_offset = std::abs(longi) - box_half_length;
  if (1e-6 < corner_offset && corner_offset < config_corner_width - 1e-6) {
    offset -=
        config_corner_width - std::sqrt(config_corner_width * config_corner_width - corner_offset * corner_offset);
  }
  return offset;
}

void Freespace::freespaceVisualizationForParking(ClearancePoint& pt, math::Vec2d* key_vec) const {
  auto key_pt = grid_map_.min_index(pt.x, pt.y);
  auto vec = grid_map_.getVec2d(key_pt.first, key_pt.second);
  *key_vec = vec;
  return;
}

void Freespace::geneCheckCircles(math::Box2d box, std::vector<math::Circle>* check_circles) const {
  // 1 rotate the box to make sure the box length is larger than the width
  modifyBoxIfNeed(&box);
  // 2
  const double half_w = box.half_width();
  const double half_l = box.half_length();
  const double check_radius = 0.554 * box.width();
  const double step = 2 * std::sqrt(check_radius * check_radius - half_w * half_w);
  double delta_offset = box.length() - 2 * half_w;
  int n_step = std::max<int>(1, std::ceil(delta_offset / step) + 1);
  double offset_0 = -half_l + half_w;
  math::Vec2d unit_vec(box.cos_heading(), box.sin_heading());
  for (int i = 0; i < n_step; i++) {
    double ratio = static_cast<double>(i) / (n_step - 1);
    double offset = offset_0 + ratio * delta_offset;
    math::Circle circle;
    circle.set_center(box.center() + offset * unit_vec);
    circle.set_radius(check_radius);
    check_circles->push_back(circle);
  }
  // 3 add corner circles
  // base circle
  double a1 = std::sqrt(check_radius * check_radius - half_w * half_w);
  double a2 = (half_w - a1) * 0.5;
  double center_x = half_l - a2;
  double center_y = half_w - a2;
  double radius = 1.414 * a2;

  math::Circle base_circle;
  base_circle.set_radius(radius);

  Eigen::Matrix<double, 2, 3> tf;
  tf << box.cos_heading(), -box.sin_heading(), box.center().x(), box.sin_heading(), box.cos_heading(), box.center().y();

  std::vector<Eigen::Vector3d> center_vecs(4);
  center_vecs[0] << center_x, center_y, 1;
  center_vecs[1] << center_x, -center_y, 1;
  center_vecs[2] << -center_x, -center_y, 1;
  center_vecs[3] << -center_x, center_y, 1;

  for (auto& center_vec : center_vecs) {
    auto new_center_vec = tf * center_vec;
    base_circle.set_center(math::Vec2d(new_center_vec[0], new_center_vec[1]));
    check_circles->push_back(base_circle);
  }
}

void Freespace::modifyBoxIfNeed(math::Box2d* box) const {
  if (box->length() < box->width()) {
    *box = math::Box2d(box->center(), box->heading() + M_PI / 2.0, box->width(), box->length());
  }
}

void Freespace::testGeneCheckCircles() {
  {
    math::Box2d box(math::Vec2d(3, 2), 1.2, 5, 2);
    std::vector<math::Circle> check_circles;
    geneCheckCircles(box, &check_circles);
    for (auto& cir : check_circles) {
      ERT_PLOG_I << cir.center().x() << " " << cir.center().y() << " " << cir.radius();
    }
  }
  {
    math::Box2d box(math::Vec2d(-3, 2), 0, 2, 3);
    std::vector<math::Circle> check_circles;
    geneCheckCircles(box, &check_circles);
    for (auto& cir : check_circles) {
      ERT_PLOG_I << cir.center().x() << " " << cir.center().y() << " " << cir.radius();
    }
  }
}

}  // namespace gpal::pnc::planning
