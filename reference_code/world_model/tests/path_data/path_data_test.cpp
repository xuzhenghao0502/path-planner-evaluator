#include <base/singleton.h>
#include <fmt/chrono.h>
#include <gtest/gtest.h>

#include "base/log.h"

#include "config_manager/config_manager.h"

#define private public
#include "path/path_data.h"


namespace gpal::pnc::planning {

class PathDataTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ERT_PLOG_I << "\n[SetUp] Initializing test environment...";
    int argc = 0;
    char path[] = "policy_planner";
    char* argv[]{path};
    // if (!Singleton<ConfigManager>::get_instance()->init(argc, argv)) {
    //   ERT_PLOG_I << "[SetUp] Failed to init config!" ;
    // }

    // 创建测试用的参考线
    std::vector<math::Vec3d> ref_points;
    for (int i = 0; i < 50; i++) {
      ref_points.emplace_back(math::Vec3d(i, 0.0, 0.0));
    }
    reference_line_ = std::make_unique<ReferenceLine>(ref_points);
    // ERT_PLOG_I << "[SetUp] Created reference line with " << ref_points.size() << " points" ;

    // 创建测试用的离散路径
    std::vector<PathPt> path_points;
    for (int i = 0; i < 50; i++) {
      path_points.emplace_back(PathPt(i, 0.0, 0.0));
    }
    discretized_path_ = std::make_unique<DiscretizedPath>(path_points);
    // ERT_PLOG_I << "[SetUp] Created discretized path with " << path_points.size() << " points" ;

    // 创建测试用的Frenet路径
    std::vector<FrenetFramePoint> frenet_points;
    for (int i = 0; i < 50; i++) {
      FrenetFramePoint point;
      point.set_s(i);
      point.set_l(0.0);
      frenet_points.push_back(point);
    }
    frenet_path_ = std::make_unique<FrenetFramePath>(frenet_points);
    // ERT_PLOG_I << "[SetUp] Created frenet path with " << frenet_points.size() << " points" ;
  }

  std::unique_ptr<ReferenceLine> reference_line_;
  std::unique_ptr<DiscretizedPath> discretized_path_;
  std::unique_ptr<FrenetFramePath> frenet_path_;
  PathData path_data_;
};

TEST_F(PathDataTest, ClearTest) {
  ERT_PLOG_I << "\n[TEST] Running ClearTest...";

  // 设置一些数据
  path_data_.setReferenceLine(reference_line_.get());
  path_data_.setDiscretizedPath(*discretized_path_);
  path_data_.setFrenetPath(*frenet_path_);
  path_data_.setPathLabel("test_label");
  ERT_PLOG_I << "[TEST] Set reference line, discretized path, frenet path and label";

  // 验证数据已设置
  EXPECT_NE(path_data_.reference_line_, nullptr);
  EXPECT_FALSE(path_data_.discretized_path_.empty());
  EXPECT_FALSE(path_data_.frenet_path_.empty());
  EXPECT_FALSE(path_data_.path_label_.empty());
  ERT_PLOG_I << "[TEST] Verified data is set correctly";

  // 执行clear
  path_data_.clear();
  ERT_PLOG_I << "[TEST] Called clear()";

  // 验证所有数据已清除
  EXPECT_EQ(path_data_.reference_line_, nullptr);
  EXPECT_TRUE(path_data_.discretized_path_.empty());
  EXPECT_TRUE(path_data_.frenet_path_.empty());
  EXPECT_TRUE(path_data_.path_label_.empty());
  EXPECT_TRUE(path_data_.bounds_.empty());
  EXPECT_TRUE(path_data_.blocking_obstacle_id_.empty());
  EXPECT_TRUE(path_data_.block_fs_info_.empty());
  EXPECT_EQ(path_data_.planner_status_, PathData::StatusType::INVALID);
  EXPECT_TRUE(path_data_.planner_debug_status_.empty());
  EXPECT_TRUE(path_data_.debug_info_.empty());
  ERT_PLOG_I << "[TEST] Verified all data is cleared";
}

TEST_F(PathDataTest, SetReferenceLineTest) {
  ERT_PLOG_I << "\n[TEST] Running SetReferenceLineTest...";

  // 测试设置有效参考线
  ERT_PLOG_I << "[TEST] Setting valid reference line...";
  EXPECT_TRUE(path_data_.setReferenceLine(reference_line_.get()));
  EXPECT_EQ(path_data_.reference_line_, reference_line_.get());
  ERT_PLOG_I << "[TEST] Successfully set reference line";

  // 测试设置空参考线
  ERT_PLOG_I << "[TEST] Setting null reference line...";
  EXPECT_TRUE(path_data_.setReferenceLine(nullptr));
  EXPECT_EQ(path_data_.reference_line_, nullptr);
  ERT_PLOG_I << "[TEST] Successfully set null reference line";
}

TEST_F(PathDataTest, SetFrenetPathTest) {
  ERT_PLOG_I << "\n[TEST] Running SetFrenetPathTest...";

  // 测试无参考线情况
  ERT_PLOG_I << "[TEST] Testing without reference line...";
  PathData empty_path_data;
  EXPECT_FALSE(empty_path_data.setFrenetPath(*frenet_path_));
  ERT_PLOG_I << "[TEST] Correctly failed to set frenet path without reference line";

  ERT_PLOG_I << "[TEST] Testing without reference_points...";
  auto temp_reference_line = std::make_unique<ReferenceLine>();
  empty_path_data.setReferenceLine(temp_reference_line.get());
  EXPECT_FALSE(empty_path_data.setFrenetPath(*frenet_path_));
  ERT_PLOG_I << "[TEST] Correctly failed to set frenet path without reference_points";

  // 测试有效情况
  ERT_PLOG_I << "[TEST] Setting reference line...";
  path_data_.setReferenceLine(reference_line_.get());
  ERT_PLOG_I << "[TEST] Setting valid frenet path...";
  EXPECT_TRUE(path_data_.setFrenetPath(*frenet_path_));
  EXPECT_EQ(path_data_.frenet_path_.size(), frenet_path_->size());
  EXPECT_EQ(path_data_.discretized_path_.size(), frenet_path_->size());
  ERT_PLOG_I << "[TEST] Successfully set frenet path with " << frenet_path_->size() << " points";

  // 测试参考线异常情况
  ERT_PLOG_I << "[TEST] Testing with reference line error...";
  std::vector<math::Vec3d> ref_points{{1, 0.0, 0.0}};
  auto error_reference_line = std::make_unique<ReferenceLine>(ref_points);
  path_data_.setReferenceLine(error_reference_line.get());
  ERT_PLOG_I << "[TEST] Setting error reference line...";
  EXPECT_FALSE(path_data_.setFrenetPath(*frenet_path_));
  ERT_PLOG_I << "[TEST] Correctly failed to set frenet path with error reference line";
}

TEST_F(PathDataTest, SetDiscretizedPathTest) {
  ERT_PLOG_I << "\n[TEST] Running SetDiscretizedPathTest...";

  ERT_PLOG_I << "[TEST] Setting discretized path...";
  path_data_.setDiscretizedPath(*discretized_path_);
  EXPECT_EQ(path_data_.discretized_path_.size(), discretized_path_->size());
  ERT_PLOG_I << "[TEST] Successfully set discretized path with " << discretized_path_->size() << " points";
}

TEST_F(PathDataTest, SetLocalPathTest) {
  ERT_PLOG_I << "\n[TEST] Running SetLocalPathTest...";

  ERT_PLOG_I << "[TEST] Setting local path...";
  path_data_.setLocalPath(*discretized_path_);
  EXPECT_EQ(path_data_.local_path_.size(), discretized_path_->size());
  ERT_PLOG_I << "[TEST] Successfully set local path with " << discretized_path_->size() << " points";
}

TEST_F(PathDataTest, SetBoundsTest) {
  ERT_PLOG_I << "\n[TEST] Running SetBoundsTest...";

  ERT_PLOG_I << "[TEST] Setting bounds...";
  std::vector<LineSegments> bounds;
  bounds.emplace_back(LineSegments());
  path_data_.setBounds(bounds);
  EXPECT_EQ(path_data_.bounds_.size(), 1);
  ERT_PLOG_I << "[TEST] Successfully set bounds with size " << path_data_.bounds_.size();
}

TEST_F(PathDataTest, SetPathLabelTest) {
  ERT_PLOG_I << "\n[TEST] Running SetPathLabelTest...";

  ERT_PLOG_I << "[TEST] Setting path label...";
  path_data_.setPathLabel("test_label");
  EXPECT_EQ(path_data_.path_label(), "test_label");
  ERT_PLOG_I << "[TEST] Successfully set path label to: " << path_data_.path_label_;

  ERT_PLOG_I << "[TEST] Setting empty path label...";
  path_data_.setPathLabel("");
  EXPECT_TRUE(path_data_.path_label().empty());
  ERT_PLOG_I << "[TEST] Successfully cleared path label";
}

TEST_F(PathDataTest, SetBlockingObstacleIdTest) {
  ERT_PLOG_I << "\n[TEST] Running SetBlockingObstacleIdTest...";

  ERT_PLOG_I << "[TEST] Setting blocking obstacle ID...";
  path_data_.setBlockingObstacleId("obs_123");
  EXPECT_EQ(path_data_.blocking_obstacle_id(), "obs_123");
  ERT_PLOG_I << "[TEST] Successfully set blocking obstacle ID to: " << path_data_.blocking_obstacle_id_;

  ERT_PLOG_I << "[TEST] Setting empty blocking obstacle ID...";
  path_data_.setBlockingObstacleId("");
  EXPECT_TRUE(path_data_.blocking_obstacle_id().empty());
  ERT_PLOG_I << "[TEST] Successfully cleared blocking obstacle ID";
}

TEST_F(PathDataTest, SetPlannerStatusTest) {
  ERT_PLOG_I << "\n[TEST] Running SetPlannerStatusTest...";

  ERT_PLOG_I << "[TEST] Setting planner status to RUNNING...";
  path_data_.setPlannerStatus(PathData::StatusType::RUNNING);
  EXPECT_EQ(path_data_.getPlannerStatus(), PathData::StatusType::RUNNING);
  ERT_PLOG_I << "[TEST] Successfully set planner status to RUNNING";

  ERT_PLOG_I << "[TEST] Setting planner status to FAILED...";
  path_data_.setPlannerStatus(PathData::StatusType::FAILED);
  EXPECT_EQ(path_data_.getPlannerStatus(), PathData::StatusType::FAILED);
  ERT_PLOG_I << "[TEST] Successfully set planner status to FAILED";
}

TEST_F(PathDataTest, SetRemainDisInfoTest) {
  ERT_PLOG_I << "\n[TEST] Running SetRemainDisInfoTest...";

  ERT_PLOG_I << "[TEST] Setting valid remain distance info...";
  std::pair<bool, double> info(true, 10.0);
  path_data_.setRemainDisInfo(info);
  EXPECT_EQ(path_data_.getRemainDisInfo(), info);
  ERT_PLOG_I << "[TEST] Successfully set remain distance info: valid=" << info.first << ", distance=" << info.second;

  ERT_PLOG_I << "[TEST] Setting invalid remain distance info...";
  std::pair<bool, double> invalid_info(false, -1.0);
  path_data_.setRemainDisInfo(invalid_info);
  EXPECT_EQ(path_data_.getRemainDisInfo(), invalid_info);
  ERT_PLOG_I << "[TEST] Successfully set remain distance info: valid=" << invalid_info.first
             << ", distance=" << invalid_info.second;
}

TEST_F(PathDataTest, SL2XYTest) {
  ERT_PLOG_I << "\n[TEST] Running SL2XYTest...";

  // 测试参考线异常情况
  ERT_PLOG_I << "[TEST] Testing with reference line error...";
  std::vector<math::Vec3d> ref_points{{1, 0.0, 0.0}};
  auto error_reference_line = std::make_unique<ReferenceLine>(ref_points);
  path_data_.setReferenceLine(error_reference_line.get());
  DiscretizedPath empty_path;
  EXPECT_FALSE(path_data_.sl2xy(*frenet_path_, &empty_path));
  ERT_PLOG_I << "[TEST] Correctly failed to convert SL to XY with reference line error";

  // 测试有效转换
  ERT_PLOG_I << "[TEST] Setting reference line...";
  path_data_.setReferenceLine(reference_line_.get());
  DiscretizedPath result_path;
  ERT_PLOG_I << "[TEST] Converting valid SL to XY...";
  EXPECT_TRUE(path_data_.sl2xy(*frenet_path_, &result_path));
  EXPECT_EQ(result_path.size(), frenet_path_->size());
  ERT_PLOG_I << "[TEST] Successfully converted SL to XY with " << result_path.size() << " points";

  // 测试无效Frenet点
  ERT_PLOG_I << "[TEST] Testing with invalid SL points...";
  FrenetFramePath invalid_path;
  EXPECT_TRUE(path_data_.sl2xy(invalid_path, &result_path));
  EXPECT_TRUE(result_path.empty());
  ERT_PLOG_I << "[TEST] Correctly failed to convert invalid SL points";
}

TEST_F(PathDataTest, XY2SLTest) {
  ERT_PLOG_I << "\n[TEST] Running XY2SLTest...";

  // 测试有效转换
  ERT_PLOG_I << "[TEST] Setting reference line...";
  path_data_.setReferenceLine(reference_line_.get());
  FrenetFramePath result_path;
  ERT_PLOG_I << "[TEST] Converting valid XY to SL...";
  EXPECT_TRUE(path_data_.xy2sl(*discretized_path_, &result_path));
  EXPECT_EQ(result_path.size(), discretized_path_->size());
  ERT_PLOG_I << "[TEST] Successfully converted XY to SL with " << result_path.size() << " points";

  // 测试参考线异常转换1
  ERT_PLOG_I << "[TEST] Setting error reference line1...";
  auto temp_reference_line = std::make_unique<ReferenceLine>();
  temp_reference_line->line_segments_ = reference_line_->line_segments_;
  temp_reference_line->num_line_segments_ = reference_line_->num_line_segments_;
  path_data_.setReferenceLine(temp_reference_line.get());
  std::vector<PathPt> path_points;
  path_points.emplace_back(PathPt(1, 0.0, 0.0));
  auto temp_discretized_path = std::make_unique<DiscretizedPath>(path_points);
  ERT_PLOG_I << "[TEST] Converting valid XY to SL...";
  EXPECT_TRUE(path_data_.xy2sl(*temp_discretized_path, &result_path));
  ERT_PLOG_I << "[TEST] Successfully converted XY to SL with error reference line1" << result_path.size() << " points";

  // 测试参考线异常转换2
  ERT_PLOG_I << "[TEST] Setting error reference line2...";
  temp_reference_line = std::make_unique<ReferenceLine>();
  PathData temp_path_data;
  temp_path_data.setReferenceLine(temp_reference_line.get());
  ERT_PLOG_I << "[TEST] Converting valid XY to SL...";
  EXPECT_FALSE(temp_path_data.xy2sl(*discretized_path_, &result_path));
  ERT_PLOG_I << "[TEST] Successfully converted XY to SL with error reference line2" << result_path.size() << " points";
}

TEST_F(PathDataTest, GetterMethodsTest) {
  ERT_PLOG_I << "\n[TEST] Running GetterMethodsTest...";

  // 设置测试数据
  ERT_PLOG_I << "[TEST] Setting up test data...";
  path_data_.setReferenceLine(reference_line_.get());
  path_data_.setDiscretizedPath(*discretized_path_);
  path_data_.setLocalPath(*discretized_path_);
  path_data_.setFrenetPath(*frenet_path_);
  path_data_.setPathLabel("test_label");
  path_data_.setBlockingObstacleId("obs_123");
  path_data_.setPlannerStatus(PathData::StatusType::RUNNING);

  // 测试所有getter方法
  ERT_PLOG_I << "[TEST] Testing getter methods...";
  EXPECT_EQ(path_data_.reference_line_, reference_line_.get());
  ERT_PLOG_I << "[TEST] Verified reference_line getter";

  EXPECT_EQ(path_data_.discretized_path().size(), discretized_path_->size());
  ERT_PLOG_I << "[TEST] Verified discretized_path getter with size " << path_data_.discretized_path().size();

  EXPECT_EQ(path_data_.local_path().size(), discretized_path_->size());
  ERT_PLOG_I << "[TEST] Verified local_path getter with size " << path_data_.local_path().size();

  EXPECT_EQ(path_data_.frenet_path().size(), frenet_path_->size());
  ERT_PLOG_I << "[TEST] Verified frenet_path getter with size " << path_data_.frenet_path().size();

  EXPECT_EQ(path_data_.path_label(), "test_label");
  ERT_PLOG_I << "[TEST] Verified path_label getter: " << path_data_.path_label();

  EXPECT_EQ(path_data_.blocking_obstacle_id(), "obs_123");
  ERT_PLOG_I << "[TEST] Verified blocking_obstacle_id getter: " << path_data_.blocking_obstacle_id();

  EXPECT_EQ(path_data_.getPlannerStatus(), PathData::StatusType::RUNNING);
  ERT_PLOG_I << "[TEST] Verified planner status getter: RUNNING";

  // 测试mutable方法
  ERT_PLOG_I << "[TEST] Testing mutable methods...";
  EXPECT_NE(path_data_.mutableDiscretizedPath(), nullptr);
  ERT_PLOG_I << "[TEST] Verified mutableDiscretizedPath";

  EXPECT_NE(path_data_.mutableLocalPath(), nullptr);
  ERT_PLOG_I << "[TEST] Verified mutableLocalPath";

  EXPECT_NE(path_data_.mutableDebugInfo(), nullptr);
  ERT_PLOG_I << "[TEST] Verified mutableDebugInfo";

  EXPECT_NE(path_data_.mutableLocalPathDebugInfo(), nullptr);
  ERT_PLOG_I << "[TEST] Verified mutableLocalPathDebugInfo";

  EXPECT_NE(path_data_.mutablePlannerDebugStatus(), nullptr);
  ERT_PLOG_I << "[TEST] Verified mutablePlannerDebugStatus";
}

TEST_F(PathDataTest, ParkFullDiscretizedPathTest) {
  ERT_PLOG_I << "\n[TEST] Running ParkFullDiscretizedPathTest...";

  // 测试初始状态
  ERT_PLOG_I << "[TEST] Checking initial state...";
  EXPECT_TRUE(path_data_.park_full_discretized_path().empty());
  ERT_PLOG_I << "[TEST] Verified initial park_full_discretized_path is empty";

  // 设置测试数据
  ERT_PLOG_I << "[TEST] Setting up test data...";
  std::vector<std::vector<PathPt>> park_paths;
  std::vector<PathPt> path_points;
  for (int i = 0; i < 10; i++) {
    path_points.emplace_back(PathPt(i, 0.0, 0.0));
  }
  park_paths.push_back(path_points);

  *path_data_.mutableParkFullDiscretizedPath() = park_paths;
  ERT_PLOG_I << "[TEST] Set park_full_discretized_path with " << park_paths.size() << " paths";

  // 验证数据
  EXPECT_EQ(path_data_.park_full_discretized_path().size(), 1);
  EXPECT_EQ(path_data_.park_full_discretized_path()[0].size(), 10);
  ERT_PLOG_I << "[TEST] Verified park_full_discretized_path contains 1 path with "
             << path_data_.park_full_discretized_path()[0].size() << " points";
}

TEST_F(PathDataTest, BlockFSInfoTest) {
  ERT_PLOG_I << "\n[TEST] Running BlockFSInfoTest...";

  // 测试初始状态
  ERT_PLOG_I << "[TEST] Checking initial state...";
  EXPECT_TRUE(path_data_.blockFSInfo().empty());
  ERT_PLOG_I << "[TEST] Verified initial blockFSInfo is empty";

  // 添加测试数据
  ERT_PLOG_I << "[TEST] Adding test data...";
  PathData::BlockFSInfo info;
  info.s = 10.0;
  info.is_valid = true;
  path_data_.block_fs_info_.push_back(info);
  ERT_PLOG_I << "[TEST] Added blockFSInfo with s=" << info.s << ", is_valid=" << info.is_valid;

  // 验证数据
  EXPECT_EQ(path_data_.blockFSInfo().size(), 1);
  EXPECT_EQ(path_data_.blockFSInfo()[0].s, 10.0);
  EXPECT_TRUE(path_data_.blockFSInfo()[0].is_valid);
  ERT_PLOG_I << "[TEST] Verified blockFSInfo contains correct data";

  // 测试mutable方法
  ERT_PLOG_I << "[TEST] Testing mutableBlockFSInfo...";
  path_data_.mutableBlockFSInfo()->clear();
  EXPECT_TRUE(path_data_.blockFSInfo().empty());
  ERT_PLOG_I << "[TEST] Verified mutableBlockFSInfo works correctly";
}

TEST_F(PathDataTest, BoundsVec3dTest) {
  ERT_PLOG_I << "\n[TEST] Running BoundsVec3dTest...";

  // 测试初始状态
  ERT_PLOG_I << "[TEST] Checking initial state...";
  EXPECT_TRUE(path_data_.boundsVec3d().barrier_bound_right.empty());
  ERT_PLOG_I << "[TEST] Verified initial boundsVec3d is empty";

  // 添加测试数据
  ERT_PLOG_I << "[TEST] Adding test data...";
  path_data_.bounds_vec3d_.barrier_bound_right.emplace_back(1.0, 2.0, 3.0);
  ERT_PLOG_I << "[TEST] Added point (1.0, 2.0, 3.0) to barrier_bound_right";

  // 验证数据
  EXPECT_EQ(path_data_.boundsVec3d().barrier_bound_right.size(), 1);
  EXPECT_EQ(path_data_.boundsVec3d().barrier_bound_right[0].x(), 1.0);
  ERT_PLOG_I << "[TEST] Verified boundsVec3d contains correct data";

  // 测试mutable方法
  ERT_PLOG_I << "[TEST] Testing mutableBoundsVec3d...";
  path_data_.mutableBoundsVec3d()->clear();
  EXPECT_TRUE(path_data_.boundsVec3d().barrier_bound_right.empty());
  ERT_PLOG_I << "[TEST] Verified mutableBoundsVec3d works correctly";
}

TEST_F(PathDataTest, BoundsVec3dWithIdTest) {
  ERT_PLOG_I << "\n[TEST] Running BoundsVec3dWithIdTest...";

  // 测试初始状态
  ERT_PLOG_I << "[TEST] Checking initial state...";
  EXPECT_TRUE(path_data_.boundsVec3dWithId().empty());
  ERT_PLOG_I << "[TEST] Verified initial boundsVec3dWithId is empty";

  // 添加测试数据
  ERT_PLOG_I << "[TEST] Adding test data...";
  PathData::BoundsVec3dWithId bounds;
  bounds.id = "test_id";
  bounds.barrier_bound_right.emplace_back(1.0, 2.0, 3.0);
  path_data_.bounds_vec3d_with_id_.push_back(bounds);
  ERT_PLOG_I << "[TEST] Added bounds with id=" << bounds.id << " and 1 point";

  // 验证数据
  EXPECT_EQ(path_data_.boundsVec3dWithId().size(), 1);
  EXPECT_EQ(path_data_.boundsVec3dWithId()[0].id, "test_id");
  ERT_PLOG_I << "[TEST] Verified boundsVec3dWithId contains correct data";

  // 测试mutable方法
  ERT_PLOG_I << "[TEST] Testing mutableBoundsVec3dWithId...";
  path_data_.mutableBoundsVec3dWithId()->clear();
  EXPECT_TRUE(path_data_.boundsVec3dWithId().empty());
  ERT_PLOG_I << "[TEST] Verified mutableBoundsVec3dWithId works correctly";
}

TEST_F(PathDataTest, OcpPathInfoTest) {
  ERT_PLOG_I << "\n[TEST] Running OcpPathInfoTest...";

  // 测试初始状态
  ERT_PLOG_I << "[TEST] Checking initial state...";
  EXPECT_TRUE(path_data_.ocpPathInfo().empty());
  ERT_PLOG_I << "[TEST] Verified initial ocpPathInfo is empty";

  // 添加测试数据
  ERT_PLOG_I << "[TEST] Adding test data...";
  PathData::OcpPathInfo info;
  info.ind = 1;
  info.lateral_dis = 0.5;
  path_data_.ocp_path_info_.push_back(info);
  ERT_PLOG_I << "[TEST] Added ocpPathInfo with ind=" << info.ind << ", lateral_dis=" << info.lateral_dis;

  // 验证数据
  EXPECT_EQ(path_data_.ocpPathInfo().size(), 1);
  EXPECT_EQ(path_data_.ocpPathInfo()[0].ind, 1);
  ERT_PLOG_I << "[TEST] Verified ocpPathInfo contains correct data";

  // 测试mutable方法
  ERT_PLOG_I << "[TEST] Testing mutableOcpPathInfo...";
  path_data_.mutableOcpPathInfo()->clear();
  EXPECT_TRUE(path_data_.ocpPathInfo().empty());
  ERT_PLOG_I << "[TEST] Verified mutableOcpPathInfo works correctly";
}

TEST_F(PathDataTest, ParkDebugStatusTest) {
  ERT_PLOG_I << "\n[TEST] Running ParkDebugStatusTest...";

  // 测试初始状态
  ERT_PLOG_I << "[TEST] Checking initial state...";
  EXPECT_TRUE(path_data_.parkDebugStatus().empty());
  ERT_PLOG_I << "[TEST] Verified initial parkDebugStatus is empty";

  // 添加测试数据
  ERT_PLOG_I << "[TEST] Adding test data...";
  path_data_.park_debug_status_.insert(PathData::ParkDebugStatusType::PARKIN);
  ERT_PLOG_I << "[TEST] Added PARKIN to parkDebugStatus";

  // 验证数据
  EXPECT_EQ(path_data_.parkDebugStatus().size(), 1);
  EXPECT_TRUE(path_data_.parkDebugStatus().count(PathData::ParkDebugStatusType::PARKIN));
  ERT_PLOG_I << "[TEST] Verified parkDebugStatus contains PARKIN";

  // 测试mutable方法
  ERT_PLOG_I << "[TEST] Testing mutableParkDebugStatus...";
  path_data_.mutableParkDebugStatus()->clear();
  EXPECT_TRUE(path_data_.parkDebugStatus().empty());
  ERT_PLOG_I << "[TEST] Verified mutableParkDebugStatus works correctly";
}

TEST_F(PathDataTest, PlannerDebugStatusTest) {
  ERT_PLOG_I << "\n[TEST] Running PlannerDebugStatusTest...";

  // 测试初始状态
  ERT_PLOG_I << "[TEST] Checking initial state...";
  EXPECT_TRUE(path_data_.plannerDebugStatus().empty());
  ERT_PLOG_I << "[TEST] Verified initial plannerDebugStatus is empty";

  // 添加测试数据
  ERT_PLOG_I << "[TEST] Adding test data...";
  path_data_.planner_debug_status_.push_back(PathData::DebugStatusType::REGULAR);
  ERT_PLOG_I << "[TEST] Added REGULAR to plannerDebugStatus";

  // 验证数据
  EXPECT_EQ(path_data_.plannerDebugStatus().size(), 1);
  EXPECT_EQ(path_data_.plannerDebugStatus()[0], PathData::DebugStatusType::REGULAR);
  ERT_PLOG_I << "[TEST] Verified plannerDebugStatus contains REGULAR";

  // 测试mutable方法
  ERT_PLOG_I << "[TEST] Testing mutablePlannerDebugStatus...";
  path_data_.mutablePlannerDebugStatus()->clear();
  EXPECT_TRUE(path_data_.plannerDebugStatus().empty());
  ERT_PLOG_I << "[TEST] Verified mutablePlannerDebugStatus works correctly";
}

TEST_F(PathDataTest, DebugInfoTest) {
  ERT_PLOG_I << "\n[TEST] Running DebugInfoTest...";

  // 测试初始状态
  ERT_PLOG_I << "[TEST] Checking initial state...";
  EXPECT_TRUE(path_data_.debugInfo().empty());
  ERT_PLOG_I << "[TEST] Verified initial debugInfo is empty";

  // 设置调试信息
  ERT_PLOG_I << "[TEST] Setting debug info...";
  *path_data_.mutableDebugInfo() = "test_debug_info";

  // 验证数据
  EXPECT_EQ(path_data_.debugInfo(), "test_debug_info");
  ERT_PLOG_I << "[TEST] Verified debugInfo contains: " << path_data_.debugInfo();

  // 测试本地路径调试信息
  ERT_PLOG_I << "[TEST] Setting local path debug info...";
  *path_data_.mutableLocalPathDebugInfo() = "local_debug_info";
  EXPECT_EQ(path_data_.localPathDebugInfo(), "local_debug_info");
  ERT_PLOG_I << "[TEST] Verified localPathDebugInfo contains: " << path_data_.localPathDebugInfo();

  // 测试泊车调试信息
  ERT_PLOG_I << "[TEST] Setting park debug info...";
  path_data_.park_debug_info_.insert("park_debug");
  EXPECT_EQ(path_data_.parkDebugInfo().size(), 1);
  ERT_PLOG_I << "[TEST] Verified parkDebugInfo contains 1 item";
}

}  // namespace gpal::pnc::planning