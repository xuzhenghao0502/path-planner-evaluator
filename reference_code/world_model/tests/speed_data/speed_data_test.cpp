#include <base/singleton.h>
#include <fmt/chrono.h>
#include <gtest/gtest.h>
#include <any>

#include "config_manager/config_manager.h"

#define private public
#include "speed/speed_data.h"


namespace gpal::pnc::planning {

TEST(SpeedDataTest, TestSpeedData) {
  std::vector<SpeedPoint> speed_points;
  for (int i = 0; i < 10; i++) {
    SpeedPoint speed_point;
    speed_point.set_s(i);
    speed_point.set_v(10.0);
    speed_point.set_a(0.0);
    speed_point.set_t(i * 0.1);
    speed_point.set_da(0.0);
    speed_points.push_back(speed_point);
  }

  SpeedData speed_data(speed_points);

  speed_data.AppendSpeedPoint(11.0, 11.0, 10.0, 0.0, 0.0);

  SpeedPoint tmp_speed_pt;
  speed_data.EvaluateByTime(5.0, &tmp_speed_pt);
  speed_data.EvaluateByTime(-0.1, &tmp_speed_pt);
  speed_data.EvaluateByTime(-12.0, &tmp_speed_pt);
  speed_data.EvaluateByTime(0.0, &tmp_speed_pt);
  speed_data.EvaluateByTime(11.0, &tmp_speed_pt);

  speed_data.DebugString();

  speed_data.TotalTime();
}

TEST(SpeedDataTest, Empty) {
  std::vector<SpeedPoint> speed_points;

  SpeedData speed_data(speed_points);

  SpeedPoint tmp_speed_pt;
  speed_data.EvaluateByTime(5.0, &tmp_speed_pt);

  speed_data.DebugString();

  speed_data.TotalTime();

  speed_data.AppendSpeedPoint(11.0, 11.0, 10.0, 0.0, 0.0);
}

}  // namespace gpal::pnc::planning