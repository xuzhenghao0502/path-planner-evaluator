#include "memorized_route/memorized_route.h"

#include "config_manager/config_manager.h"

namespace gpal::pnc::planning {

SlicedRoute::SlicedRoute(const std::vector<RoutePoint>& pts) : pts_(pts) {
  if (pts_.size() > 0) {
    global_start_s_ = pts_.front().s();
    global_end_s_ = pts_.back().s();
    length_ = global_end_s_ - global_start_s_;
  }
}
SlicedRoute::SlicedRoute(
    const std::vector<RoutePoint>& pts, const uint64_t& index, const std::string& id,
    const std::shared_ptr<std::vector<RoutePoint>> stop_line_points,
    const std::shared_ptr<std::vector<SpeedLimit>> speed_limits,
    const std::shared_ptr<std::vector<SegmentDirection>> segments_direction,
    const std::shared_ptr<std::vector<std::tuple<bool, float, float, bool, bool>>> navigation_lane_change_ranges,
    const std::shared_ptr<std::vector<std::tuple<bool, float, float>>> lane_follow_ranges)
    : pts_(pts), index_(index), id_(id) {
  if (pts_.size() > 0) {
    global_start_s_ = pts_.front().s();
    global_end_s_ = pts_.back().s();
    length_ = global_end_s_ - global_start_s_;

    stop_line_points_.clear();
    for (auto& pt : *(stop_line_points)) {
      if (global_start_s_ < pt.s() && pt.s() < global_end_s_ + 1e-2f) {
        stop_line_points_.emplace_back(pt);
      }
    }

    speed_limits_.clear();
    SpeedLimit sp;
    for (const auto& pt : *(speed_limits)) {
      if (pt.start_s < global_end_s_ && pt.end_s > global_start_s_) {
        sp.start_s = std::fmax(std::fmin(pt.start_s, global_end_s_), global_start_s_);
        sp.end_s = std::fmax(std::fmin(pt.end_s, global_end_s_), global_start_s_);
        sp.max_speed_limit = pt.max_speed_limit;
        sp.min_speed_limit = pt.min_speed_limit;
        sp.start_point = std::fabs(sp.start_s - global_start_s_) > 1e-2f
                             ? pt.start_point
                             : math::Vec3d(pts_.front().x(), pts_.front().y(), pts_.front().z());
        sp.end_point = std::fabs(sp.end_s - global_end_s_) > 1e-2f
                           ? pt.end_point
                           : math::Vec3d(pts_.back().x(), pts_.back().y(), pts_.back().z());
        speed_limits_.emplace_back(sp);
      }
    }

    segments_direction_.clear();
    SegmentDirection sd;
    for (const auto& pt : *(segments_direction)) {
      if (pt.start_s < global_end_s_ && pt.end_s > global_start_s_) {
        sd.start_s = std::fmax(std::fmin(pt.start_s, global_end_s_), global_start_s_);
        sd.end_s = std::fmax(std::fmin(pt.end_s, global_end_s_), global_start_s_);
        sd.direction = pt.direction;
        sd.start_point = std::fabs(sd.start_s - global_start_s_) > 1e-2f
                             ? pt.start_point
                             : math::Vec3d(pts_.front().x(), pts_.front().y(), pts_.front().z());
        sd.end_point = std::fabs(sd.end_s - global_end_s_) > 1e-2f
                           ? pt.end_point
                           : math::Vec3d(pts_.back().x(), pts_.back().y(), pts_.back().z());
        segments_direction_.emplace_back(sd);
      }
    }

    navigation_lane_change_ranges_.clear();
    ConfigManager* config_manager = Singleton<ConfigManager>::get_instance();
    auto memorized_route_config = config_manager->getConfig<MemorizedRouteConfig>("MemorizedRouteConfig");
    if (navigation_lane_change_ranges->size() > 0) {
      ERT_PLOG_I << "[SlicedRoute::SlicedRoute]: get navigation lane change ranges from raw route file !!!";
      for (const auto& range : *navigation_lane_change_ranges) {
        if (std::get<0>(range) && std::get<1>(range) < global_end_s_ && std::get<2>(range) > global_start_s_) {
          float start_s = std::fmax(std::fmin(std::get<1>(range), global_end_s_), global_start_s_);
          float end_s = std::fmax(std::fmin(std::get<2>(range), global_end_s_), global_start_s_);
          navigation_lane_change_ranges_.emplace_back(std::move(start_s), std::move(end_s), std::get<3>(range),
                                                      std::get<4>(range));
        }
      }
    } else if (memorized_route_config.is_routes_config_enable()) {
      ERT_PLOG_I << "[SlicedRoute::SlicedRoute]: get navigation lane change ranges from planning config file !!!";
      auto route_name = MemorizedRouteConfig::INVALID_ROUTE;
      for (const auto& route_config : memorized_route_config.routes_config()) {
        if (route_config.route_name() == memorized_route_config.default_route_name()) {
          route_name = route_config.route_name();
          for (const auto& range : route_config.special_ranges()) {
            if (range.type() == MemorizedRouteConfig::RouteConfig::SpecialRange::NAVIGATION_LANE_CHANGE &&
                range.start_s() < global_end_s_ && range.end_s() > global_start_s_) {
              float start_s = std::fmax(std::fmin(range.start_s(), global_end_s_), global_start_s_);
              float end_s = std::fmax(std::fmin(range.end_s(), global_end_s_), global_start_s_);
              bool is_left_direction = range.is_left_direction();
              navigation_lane_change_ranges_.emplace_back(std::move(start_s), std::move(end_s),
                                                          std::move(is_left_direction), false);
            }
          }
          break;
        }
      }
      // ERT_PLOG_I << "[SlicedRoute::SlicedRoute]: route_name = " << static_cast<int>(route_name)
      //           << "  navigation_lane_change_ranges_.size() = " << navigation_lane_change_ranges_.size() ;
      // for(int i = 0; i < navigation_lane_change_ranges_.size(); ++i) {
      //   ERT_PLOG_I << "[SlicedRoute::SlicedRoute]: i = " << i
      //             << " start_s = " << std::get<0>(navigation_lane_change_ranges_.at(i))
      //             << " end_s = " << std::get<1>(navigation_lane_change_ranges_.at(i))
      //             << " is_left = " << std::get<2>(navigation_lane_change_ranges_.at(i))
      //             << " is_from_route_file = " << std::get<3>(navigation_lane_change_ranges_.at(i))
      //             ;
      // }
    }

    lane_follow_ranges_.clear();
    for (const auto& range : *lane_follow_ranges) {
      if (std::get<0>(range) && std::get<1>(range) < global_end_s_ && std::get<2>(range) > global_start_s_) {
        float start_s = std::fmax(std::fmin(std::get<1>(range), global_end_s_), global_start_s_);
        float end_s = std::fmax(std::fmin(std::get<2>(range), global_end_s_), global_start_s_);
        lane_follow_ranges_.emplace_back(start_s, end_s);
      }
    }
  }

  // ERT_PLOG_I << "[SlicedRoute::SlicedRoute]: global_start_s_ = " << global_start_s_ << "  global_end_s_ = " <<
  // global_end_s_ << "  id = " << id_ ; ERT_PLOG_I << "[SlicedRoute::SlicedRoute]: speed_limits_.size() = " <<
  // speed_limits_.size() ; for(int i = 0; i < speed_limits_.size(); ++i) {
  //   ERT_PLOG_I << "i = " << i
  //             << " start_s = " << speed_limits_.at(i).start_s
  //             << " end_s = " << speed_limits_.at(i).end_s
  //             << " max_speed_limit = " << speed_limits_.at(i).max_speed_limit * MS_KMH
  //             << " start_x = " << speed_limits_.at(i).start_point.x()
  //             << " start_y = " << speed_limits_.at(i).start_point.y()
  //             << " end_x = " << speed_limits_.at(i).end_point.x()
  //             << " end_y = " << speed_limits_.at(i).end_point.y()
  //             ;
  // }
  // for(int i = 0; i < segments_direction_.size(); ++i) {
  //   ERT_PLOG_I << "i = " << i
  //             << " start_s = " << segments_direction_.at(i).start_s
  //             << " end_s = " << segments_direction_.at(i).end_s
  //             << " direction = " << static_cast<int>(segments_direction_.at(i).direction)
  //             << " start_x = " << segments_direction_.at(i).start_point.x()
  //             << " start_y = " << segments_direction_.at(i).start_point.y()
  //             << " end_x = " << segments_direction_.at(i).end_point.x()
  //             << " end_y = " << segments_direction_.at(i).end_point.y()
  //             ;
  // }
  // ERT_PLOG_I << "[SlicedRoute::SlicedRoute]: stop_line_points_.size() = " << stop_line_points_.size() ;
  // for(int i = 0; i < stop_line_points_.size(); ++i) {
  //   ERT_PLOG_I << "i = " << i
  //             << " x = " << stop_line_points_.at(i).x()
  //             << " y = " << stop_line_points_.at(i).y()
  //             << " s = " << stop_line_points_.at(i).s()
  //             << " direction = " << static_cast<int>(stop_line_points_.at(i).drivingDirection())
  //             ;
  // }
  // ERT_PLOG_I << "[SlicedRoute::SlicedRoute]: navigation_lane_change_ranges_.size() = " <<
  // navigation_lane_change_ranges_.size() ; for(const auto& range : navigation_lane_change_ranges_) {
  //   ERT_PLOG_I << "  [SlicedRoute::SlicedRoute]: start_s = " << std::get<0>(range)
  //             << "  end_s = " << std::get<1>(range)
  //             << "  is_left = " << std::get<2>(range)
  //             << "  is_from_route_file = " << std::get<3>(range)
  //             ;
  // }
  // ERT_PLOG_I << "[SlicedRoute::SlicedRoute]: lane_follow_ranges_.size() = " << lane_follow_ranges_.size() ;
  // for(const auto& range : lane_follow_ranges_) {
  //   ERT_PLOG_I << "start_s = " << range.first
  //             << "  end_s = " << range.second
  //             ;
  // }
}

MemorizedRoute::MemorizedRoute() {
  route_ = std::make_shared<std::vector<RoutePoint>>();
  sliced_routes_ = std::make_shared<std::vector<SlicedRoute>>();
  stop_line_points_ = std::make_shared<std::vector<RoutePoint>>();
  speed_limits_ = std::make_shared<std::vector<SpeedLimit>>();
  segments_direction_ = std::make_shared<std::vector<SegmentDirection>>();
  navigation_lane_change_ranges_ = std::make_shared<std::vector<std::tuple<bool, float, float, bool, bool>>>();
  lane_follow_ranges_ = std::make_shared<std::vector<std::tuple<bool, float, float>>>();
}

void MemorizedRoute::reset() {
  adc_global_s_ = -1.0;
  last_adc_global_s_ = 0.0;
}

void MemorizedRoute::slice() {
  sliced_routes_->clear();

  if (route_ != nullptr && route_->size() > 0) {
    size_t start_index = 0;
    size_t end_index = 0;
    size_t swap_index = 0;
    float start_s = 0.0;
    std::vector<std::pair<size_t, size_t>> index_pair;
    auto itr_begin = route_->begin();
    while (itr_begin != route_->end()) {
      bool exit_flag = false;

      // slice process
      start_index = std::distance(route_->begin(), itr_begin);
      start_s = itr_begin->s();
      float end_s_thresold = start_s + look_backward_dis_ + look_forward_dis_;
      auto itr_end = std::upper_bound(itr_begin, route_->end(), end_s_thresold,
                                      [](const float& s, const RoutePoint& pt) { return pt.s() >= s; });
      if (itr_end == itr_begin) {
        end_index = start_index;
        ERT_PLOG_I << "[MemorizedRoute::slice()]: failed! start_index = end_index = " << start_index
                   << ", route_->size() = " << route_->size();
        exit_flag = true;
      } else if (itr_end == route_->end()) {
        end_index = route_->size() - 1;
        exit_flag = true;
      } else {
        end_index = std::distance(route_->begin(), itr_end);
      }

      // swap process
      float start_s_thresold = start_s + (look_forward_dis_ - swap_dis_);
      auto itr_swap = std::upper_bound(itr_begin, route_->end(), start_s_thresold,
                                       [](const float& s, const RoutePoint& pt) { return pt.s() >= s; });
      if (itr_swap == itr_begin) {
        swap_index = start_index;
        ERT_PLOG_I << "[MemorizedRoute::slice()]: failed! swap_index = start_index = " << start_index
                   << ", route_->size() = " << route_->size();
        exit_flag = true;
      } else if (itr_swap == route_->end()) {
        swap_index = route_->size();
        end_index = route_->size() - 1;
        ERT_PLOG_I
            << "[MemorizedRoute::slice()]: no need to swap, remain route is too short! swap_index = route_->end()";
        exit_flag = true;
      } else {
        swap_index = std::distance(route_->begin(), itr_swap);
        itr_begin = itr_swap;
      }

      // output
      index_pair.emplace_back(std::make_pair(std::min(start_index, end_index), std::max(start_index, end_index)));
      if (exit_flag) {
        break;
      }
    }

    for (int i = 0; i < index_pair.size(); ++i) {
      auto start_index = index_pair.at(i).first;
      auto end_index = index_pair.at(i).second;
      std::string id = id_ + "_sliced_" + std::to_string(i);
      std::vector<RoutePoint> data;
      data.assign(route_->begin() + start_index, route_->begin() + end_index + 1);
      sliced_routes_->emplace_back(SlicedRoute(data, i, id, stop_line_points_, speed_limits_, segments_direction_,
                                               navigation_lane_change_ranges_, lane_follow_ranges_));
      sliced_routes_->back().setIsParkSlicedRoute(is_park_route_);
    }
  }

  if (!sliced_routes_->empty()) {
    sliced_routes_->back().setIsEndSlicedRoute(true);
    // if(sliced_routes_->back().isParkSlicedRoute()) {
    //   if(sliced_routes_->back().pts().size() >= 2) {
    //     std::vector<RoutePoint> expand_pts;
    //     for(int i = sliced_routes_->back().pts().size() - 2; i >= 0; i--) {
    //       if(sliced_routes_->back().pts().back().s() - sliced_routes_->back().pts().at(i).s() > 0.1) {
    //         float theta = std::atan2(sliced_routes_->back().pts().back().y() -
    //         sliced_routes_->back().pts().at(i).y(),
    //                                  sliced_routes_->back().pts().back().x() -
    //                                  sliced_routes_->back().pts().at(i).x());
    //         for(float length = 1.0; length <= 10.0; length += 1.0) {
    //           RoutePoint pt;
    //           pt.set_x(sliced_routes_->back().pts().back().x() + length * std::cos(theta));
    //           pt.set_y(sliced_routes_->back().pts().back().y() + length * std::sin(theta));
    //           pt.set_z(sliced_routes_->back().pts().back().z());
    //           pt.setS(sliced_routes_->back().pts().back().s() + length);
    //           pt.setYaw(sliced_routes_->back().pts().back().yaw());
    //           expand_pts.emplace_back(std::move(pt));
    //         }
    //         break;
    //       }
    //     }
    //     if(expand_pts.size() > 0) {
    //       sliced_routes_->back().mutablePts()->insert(sliced_routes_->back().mutablePts()->end(), expand_pts.begin(),
    //       expand_pts.end());
    //     }
    //   }
    // }
  }
}

bool MemorizedRoute::isInLaneFollowRanges() const {
  for (const auto& range : *lane_follow_ranges_) {
    if (std::get<0>(range) && adc_global_s_ >= std::get<1>(range) && adc_global_s_ <= std::get<2>(range)) {
      return true;
    }
  }
  return false;
}

}  // namespace gpal::pnc::planning