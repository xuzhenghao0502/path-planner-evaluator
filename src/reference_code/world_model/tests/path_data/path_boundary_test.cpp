#include <gtest/gtest.h>

#define private public
#define protected public
#include "path/path_boundary.h"

#undef protected

namespace gpal::pnc::planning {

class PathBoundaryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 添加边界数据
    for (int i = 0; i < 50; i++) {
      path_boundary_.mutable_decision_boundary()->emplace_back(i * 2.0, -2.5, 2.5);
      path_boundary_.mutable_soft_boundary()->emplace_back(i * 2.0, -2.5, 2.5);
      path_boundary_.mutable_barrier_boundary()->emplace_back(i * 2.0, -2.5, 2.5);
    }

    // 添加障碍物信息
    PathBoundary::ObstacleInfo obs_info;
    obs_info.ob_id_left = "obs1_1";
    obs_info.ob_id_right = "obs1_2";
    path_boundary_.mutable_key_obstacles()->emplace(5.0, obs_info);
    obs_info.ob_id_left = "obs2_1";
    obs_info.ob_id_right = "obs2_2";
    path_boundary_.mutable_key_obstacles()->emplace(10.0, obs_info);
  }

  PathBoundary path_boundary_;
};

// 测试构造函数和基本属性
TEST_F(PathBoundaryTest, ConstructorAndBasicProperties) {
  EXPECT_DOUBLE_EQ(0.0, path_boundary_.start_s());
  EXPECT_DOUBLE_EQ(2.0, path_boundary_.delta_s());
  EXPECT_DOUBLE_EQ(198.0, path_boundary_.length());
  EXPECT_EQ(100, path_boundary_.size());
}

// 测试边界数据访问
TEST_F(PathBoundaryTest, BoundaryDataAccess) {
  auto& decision_boundary = path_boundary_.decision_boundary();
  EXPECT_EQ(50, decision_boundary.size());
  EXPECT_DOUBLE_EQ(-2.5, std::get<1>(decision_boundary[0]));
  EXPECT_DOUBLE_EQ(2.5, std::get<2>(decision_boundary[0]));
}

// 测试障碍物管理1
TEST_F(PathBoundaryTest, ObstacleManagement) {
  auto key_obs = path_boundary_.key_obstacles();
  EXPECT_EQ(2, key_obs.size());

  // 测试查找功能
  auto obs = path_boundary_.findKeyObstacle(9.9);
  EXPECT_TRUE(obs.has_value());
  EXPECT_EQ("obs2_1", (*obs)->second.ob_id_left);

  auto non_exist_obs = path_boundary_.findKeyObstacle(20.0);
  EXPECT_FALSE(non_exist_obs.has_value());
  non_exist_obs = path_boundary_.findKeyObstacle(4.0);
  EXPECT_FALSE(non_exist_obs.has_value());

  PathBoundary path_boundary_temp;
  non_exist_obs = path_boundary_temp.findKeyObstacle(20.0);
  EXPECT_FALSE(non_exist_obs.has_value());
}

// 测试障碍物管理2
TEST_F(PathBoundaryTest, findConstKeyObstacle) {
  auto key_obs = path_boundary_.key_obstacles();
  EXPECT_EQ(2, key_obs.size());

  // 测试查找功能
  auto obs = path_boundary_.findConstKeyObstacle(9.9);
  EXPECT_TRUE(obs.has_value());
  EXPECT_EQ("obs2_1", (*obs)->second.ob_id_left);

  auto non_exist_obs = path_boundary_.findConstKeyObstacle(20.0);
  EXPECT_FALSE(non_exist_obs.has_value());
  non_exist_obs = path_boundary_.findConstKeyObstacle(4.0);
  EXPECT_FALSE(non_exist_obs.has_value());

  PathBoundary path_boundary_temp;
  non_exist_obs = path_boundary_temp.findConstKeyObstacle(20.0);
  EXPECT_FALSE(non_exist_obs.has_value());
}

// 测试裁剪功能
TEST_F(PathBoundaryTest, TrimFunctionality) {
  path_boundary_.trim(10.0);

  EXPECT_DOUBLE_EQ(108.0, path_boundary_.length());
  EXPECT_EQ(55, path_boundary_.size());

  // 验证障碍物是否被正确裁剪
  auto obs = path_boundary_.findKeyObstacle(5.0);
  EXPECT_TRUE(obs.has_value());

  // !barrier_boundary_.empty()验证
  PathBoundary path_boundary_temp;
  path_boundary_temp.trim(10.0);

  auto path_boundary_trim = path_boundary_;
  path_boundary_trim.trim(1.0);
}

// 测试裁剪功能2
TEST_F(PathBoundaryTest, TrimBoundFunctionality) {
  PathBoundary path_boundary_temp;
  path_boundary_temp.size_ = 4;
  path_boundary_temp.start_s_ = 0.0;
  path_boundary_temp.delta_s_ = 1.0;
  std::vector<std::tuple<double, double, double>> boundary;
  boundary.emplace_back(0.0, 0.0, 0.0);
  boundary.emplace_back(1.0, 0.0, 0.0);
  boundary.emplace_back(2.0, 0.0, 0.0);
  boundary.emplace_back(3.0, 0.0, 0.0);
  boundary.emplace_back(4.0, 0.0, 0.0);
  boundary.emplace_back(5.0, 0.0, 0.0);
  boundary.emplace_back(6.0, 0.0, 0.0);

  path_boundary_temp.trimBoundary(boundary);
}

// 测试标签和阻塞信息
TEST_F(PathBoundaryTest, LabelAndBlockingInfo) {
  path_boundary_.set_label("test_label");
  EXPECT_EQ("test_label", path_boundary_.label());

  path_boundary_.set_blocking_obstacle_id("blocking_obs");
  EXPECT_EQ("blocking_obs", path_boundary_.blocking_obstacle_id());

  std::pair<bool, double> freespace_info;
  freespace_info.first = true;
  freespace_info.second = 10.0;
  *path_boundary_.mutable_blocking_freespace_info() = freespace_info;
  EXPECT_TRUE(path_boundary_.blocking_freespace_info().first);

  path_boundary_.set_blocking_freespace_info(11);
  EXPECT_TRUE(path_boundary_.blocking_freespace_info().first);
  EXPECT_DOUBLE_EQ(10, path_boundary_.blocking_freespace_info().second);

  *path_boundary_.mutable_blocking_freespace_info() = std::pair<bool, double>{false, 0.0};
  path_boundary_.set_blocking_freespace_info(9);
  EXPECT_TRUE(path_boundary_.blocking_freespace_info().first);
  EXPECT_DOUBLE_EQ(9, path_boundary_.blocking_freespace_info().second);
}

// 测试重置功能
TEST_F(PathBoundaryTest, ResetFunctionality) {
  path_boundary_.reset();
  EXPECT_DOUBLE_EQ(0.0, path_boundary_.start_s());
  EXPECT_DOUBLE_EQ(0.0, path_boundary_.delta_s());  // 默认值
  EXPECT_DOUBLE_EQ(0.0, path_boundary_.length());
  EXPECT_TRUE(path_boundary_.decision_boundary().empty());
}

}  // namespace gpal::pnc::planning