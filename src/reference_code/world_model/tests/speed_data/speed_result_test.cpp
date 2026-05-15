#include <base/singleton.h>
#include <fmt/chrono.h>
#include <gtest/gtest.h>

#include <any>

#include "config_manager/config_manager.h"

#define private public
#include "speed/speed_result.h"


namespace gpal::pnc::planning {

TEST(SpeedResultTest, ClearTest) {
  SpeedResult speed_result;

  speed_result.clear();
}

}  // namespace gpal::pnc::planning