#include <gtest/gtest.h>
#include <filesystem>
#include <fmt/chrono.h>
#include "path_planner/path_planner_base.h"
#include "base/singleton.h"
#include "config_manager/config_manager.h"
#include "local_view/local_view.h"

namespace gpal::pnc::planning {
// ====================================================================================
// Tester 派生类，用于访问 protected 方法
// ====================================================================================
class PathPlannerBaseTester : public PathPlannerBase {
 public:
  // 将所有需要测试的 protected 方法公开
  bool reset() override {
    // 为测试提供一个简单的实现
    return true;
  }
  void callRunRefinePath(const std::shared_ptr<PathData> prev_path_data, const TrajectoryPt& start_pt,
                         PathData* path_data) {
    this->runRefinePath(prev_path_data, start_pt, path_data);
  }

  PathData::BlockFSInfo callCollisionCheck(const Freespace& freespace, const std::vector<PathPt>& path,
                                           const double& collision_check_buffer, const double& corner_width,
                                           const bool& enable_curve_decide_process,
                                           const double& curve_look_ahead_distance, const double& curve_kappa_thresold,
                                           const double& side_box_length, const double& side_box_width) {
    // 为该函数设置其依赖的 vehicle_config_
    auto config_manager = Singleton<ConfigManager>::get_instance();
    this->vehicle_config_ = std::make_shared<VehicleConfig>(config_manager->vehicle_config());
    return this->collisionCheck(freespace, path, collision_check_buffer, corner_width, enable_curve_decide_process,
                                curve_look_ahead_distance, curve_kappa_thresold, side_box_length, side_box_width);
  }

  void callDecideInCurve(const std::vector<PathPt>& path, const double& look_ahead, const double& kappa_th,
                         bool& is_left, bool& is_right) {
    this->decideInCurve(path, look_ahead, kappa_th, is_left, is_right);
  }

  math::Box2d callGenerateSideCheckBox(const math::Box2d& box, const bool& is_right, const double& length,
                                       const double& width) {
    return this->generateSideCheckBox(box, is_right, length, width);
  }

  PathData::BlockPointDirection callGetBlockFsPointDirection(const PathPt& check_point,
                                                             const math::Vec2d& collision_point) {
    return this->getBlockFsPointDirection(check_point, collision_point);
  }

  // 公开 loadKappa 以便单独测试
  void callLoadKappa(DiscretizedPath* path) { this->loadKappa(path); }
};

// ====================================================================================
// GTest Fixture: 用于共享设置和辅助函数
// ====================================================================================
class PathPlannerBaseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    planner_tester_ = std::make_unique<PathPlannerBaseTester>();
    planner_tester_->init();
  }

  std::unique_ptr<PathPlannerBaseTester> planner_tester_;
};

// ====================================================================================
// 测试用例
// ====================================================================================

TEST_F(PathPlannerBaseTest, InitAndReset) {
  // init() 在 SetUp 中已调用，这里验证不为空即可
  ASSERT_NE(planner_tester_, nullptr);
  // **【代码修正】**: 现在可以安全地调用 reset()
  EXPECT_TRUE(planner_tester_->reset());
}

TEST_F(PathPlannerBaseTest, RunRefinePathCoverage) {
  PathData path_data;
  TrajectoryPt start_pt;

  // 场景1: prev_path_data 为空
  PathPlannerBase::runRefinePath(nullptr, start_pt, &path_data);

  // 场景2: prev_path 为空
  auto prev_path_data = std::make_shared<PathData>();
  PathPlannerBase::runRefinePath(prev_path_data, start_pt, &path_data);

  // 场景3: 航向角误差过大
  prev_path_data->mutableDiscretizedPath()->emplace_back(0, 0, 0, 0, M_PI, 0, 0, 0, 0);
  PathPlannerBase::runRefinePath(prev_path_data, start_pt, &path_data);
  EXPECT_TRUE(path_data.discretized_path().empty());

  // 场景4: 正常refine & 循环中s超出范围
  prev_path_data->mutableDiscretizedPath()->clear();
  for (int i = 0; i < 50; ++i) {  // 创建一条较短的路径
    prev_path_data->mutableDiscretizedPath()->emplace_back(i, 0, 0, 0, 0, 0, i, 0, 0);
  }
  PathPlannerBase::runRefinePath(prev_path_data, start_pt, &path_data);
  EXPECT_FALSE(path_data.discretized_path().empty());
}

TEST_F(PathPlannerBaseTest, CollisionCheckCoverage) {
  SCOPED_TRACE("Testing CollisionCheck with robust obstacles");
  Freespace freespace;
  FreespaceConfig fs_config;
  fs_config.mutable_grid_data()->set_resolution(0.1);
  fs_config.mutable_grid_data()->set_map_length(2000);
  fs_config.mutable_grid_data()->set_map_width(400);
  fs_config.mutable_grid_data()->set_origin_x(200);
  fs_config.mutable_grid_data()->set_origin_y(200);
  freespace.init(fs_config);

  // 在 x=10 处创建一堵障碍物墙
  for (double y = -1.0; y <= 1.0; y += 0.1) {
    freespace.mutable_grid_map()->setOccupy(10.0, y);
  }
  freespace.mutable_grid_map()->update();

  // 场景1: 空路径
  planner_tester_->callCollisionCheck(freespace, {}, 0.1, 0.1, false, 10, 0.1, 1, 1);

  // 场景2: 正常碰撞
  std::vector<PathPt> path;
  // **【代码修正】**: 路径从障碍物前开始，然后穿过障碍物
  for (int i = 0; i <= 20; ++i) {
    PathPt pt;
    pt.set_x(0.0 + i);
    pt.set_y(0.0);
    pt.set_theta(0.2);  // 左转航向
    pt.set_kappa(0.2);  // 左转曲率
    pt.set_s(0.0 + i);
    path.push_back(pt);
  }
  auto result = planner_tester_->callCollisionCheck(freespace, path, 0.5, 0.1, false, 10, 0.1, 1, 1);
  EXPECT_TRUE(result.is_valid) << "Normal collision was not detected.";
}

TEST_F(PathPlannerBaseTest, CollisionCheckCurveCoverageTest) {
  SCOPED_TRACE("Testing CollisionCheck with curve logic");
  Freespace freespace;
  FreespaceConfig fs_config;
  fs_config.mutable_grid_data()->set_resolution(0.1);
  fs_config.mutable_grid_data()->set_map_length(2000);
  fs_config.mutable_grid_data()->set_map_width(400);
  fs_config.mutable_grid_data()->set_origin_x(200);
  fs_config.mutable_grid_data()->set_origin_y(200);
  freespace.init(fs_config);

  // 在路径左侧放置障碍物墙
  for (double x = 15.0; x <= 25.0; x += 0.5) {
    for (double y = -3.0; y <= 3.0; y += 0.1) {
      freespace.mutable_grid_map()->setOccupy(x, y);
    }
  }
  freespace.mutable_grid_map()->update();

  std::vector<PathPt> curve_path;
  // **【代码修正】**: 确保路径有多个点
  for (int i = 0; i <= 40; ++i) {
    PathPt pt;
    pt.set_x(0.0 + i);
    pt.set_y(0.0);
    pt.set_theta(0.0 + i * 0.1);  // 左转航向
    pt.set_kappa(0.2);  // 左转曲率
    pt.set_s(0.0 + i);
    curve_path.push_back(pt);
  }

  auto result = planner_tester_->callCollisionCheck(freespace, curve_path, 0.5, 0.1, true, 20, 0.01, 5, 2);
  EXPECT_TRUE(result.is_valid) << "Side collision in curve was not detected.";
}

TEST_F(PathPlannerBaseTest, DecideInCurveAndGenerateSideBoxCoverage) {
  bool is_left = false, is_right = false;
  const double look_ahead = 15.0;
  const double kappa_th = 0.1;

  // 场景1: 路径点少于2
  planner_tester_->callDecideInCurve({PathPt()}, look_ahead, kappa_th, is_left, is_right);
  EXPECT_FALSE(is_left || is_right);

  // 场景2: 在前瞻点检测到右转
  is_left = false;
  is_right = false;
  std::vector<PathPt> path_right_turn;
  path_right_turn.emplace_back();
  path_right_turn.emplace_back();
  path_right_turn[0].set_kappa(0.0);
  path_right_turn[1].set_s(20.0);      // s > look_ahead
  path_right_turn[1].set_kappa(-0.2);  // kappa < -kappa_th
  planner_tester_->callDecideInCurve(path_right_turn, look_ahead, kappa_th, is_left, is_right);
  EXPECT_FALSE(is_left);
  EXPECT_TRUE(is_right);

  // 场景3: 在起点检测到左转
  is_left = false;
  is_right = false;
  std::vector<PathPt> path_left_turn_start;
  path_left_turn_start.emplace_back();
  path_left_turn_start.emplace_back();
  path_left_turn_start[0].set_kappa(0.2);  // kappa > kappa_th
  path_left_turn_start[1].set_s(20.0);
  path_left_turn_start[1].set_kappa(0.0);  // 前瞻点不转
  planner_tester_->callDecideInCurve(path_left_turn_start, look_ahead, kappa_th, is_left, is_right);
  EXPECT_TRUE(is_left);
  EXPECT_FALSE(is_right);

  // 场景4: 无转向
  is_left = false;
  is_right = false;
  std::vector<PathPt> path_no_turn;
  path_no_turn.emplace_back();
  path_no_turn.emplace_back();
  path_no_turn[0].set_kappa(0.0);
  path_no_turn[1].set_s(20.0);
  path_no_turn[1].set_kappa(0.0);
  planner_tester_->callDecideInCurve(path_no_turn, look_ahead, kappa_th, is_left, is_right);
  EXPECT_FALSE(is_left || is_right);

  // 覆盖 generateSideCheckBox
  math::Box2d box({0, 0}, 0, 4, 2);
  planner_tester_->callGenerateSideCheckBox(box, true, 1, 1);
  planner_tester_->callGenerateSideCheckBox(box, false, 1, 1);
}

TEST_F(PathPlannerBaseTest, GetBlockFsPointDirectionCoverage) {
  PathPt check_point;
  // 左侧碰撞
  auto dir = planner_tester_->callGetBlockFsPointDirection(check_point, {1.0, 1.0});
  EXPECT_EQ(dir, PathData::BlockPointDirection::LEFT);
  // 右侧碰撞
  dir = planner_tester_->callGetBlockFsPointDirection(check_point, {1.0, -1.0});
  EXPECT_EQ(dir, PathData::BlockPointDirection::RIGHT);
}

TEST_F(PathPlannerBaseTest, LoadKappaCoverage) {
  // 覆盖 size < 3 的情况
  DiscretizedPath short_path;
  short_path.emplace_back();
  short_path.emplace_back();
  planner_tester_->callLoadKappa(&short_path);

  // 覆盖 size >= 3 的情况
  DiscretizedPath long_path;
  long_path.emplace_back(0, 0, 0);
  long_path.emplace_back(1, 1, 0);
  long_path.emplace_back(2, 0, 0);
  planner_tester_->callLoadKappa(&long_path);
  EXPECT_NE(long_path.at(1).kappa(), 0.0);
}

}  // namespace gpal::pnc::planning