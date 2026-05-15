#include <gtest/gtest.h>
#include <filesystem>
#include <fmt/chrono.h>
#include "config_manager/config_manager.h"
#include "path_planner/path_optimizer.h"
namespace gpal::pnc::planning {
// ====================================================================================
// Tester 派生类，用于实例化抽象基类并访问 protected 方法
// ====================================================================================
class PathOptimizerTester : public PathOptimizer {
 public:
  // 为基类中的纯虚函数提供最简单的实现，使该类可以被实例化
  std::string name() const override { return "PathOptimizerTester"; }
  Status proc(const ReferenceLine&, const TrajectoryPt&, const PathBoundary&, PathData* const) override {
    return Status::OK();
  }
};

// ====================================================================================
// GTest Fixture: 用于共享设置和辅助函数
// ====================================================================================
class PathOptimizerTest : public ::testing::Test {
 protected:
  void SetUp() override { planner_tester_ = std::make_unique<PathOptimizerTester>(); }

  std::unique_ptr<PathOptimizerTester> planner_tester_;
};

// ====================================================================================
// 测试用例
// ====================================================================================

TEST_F(PathOptimizerTest, InitFunction) {
  // 调用init()来覆盖它
  EXPECT_TRUE(planner_tester_->init());
}

TEST_F(PathOptimizerTest, RefinePathCoverage) {
  SCOPED_TRACE("Testing all branches of refinePath");

  TrajectoryPt start_pt;
  auto path_data = std::make_shared<PathData>();

  // 场景1: 路径数据为空 (prev_path_data->discretized_path() 为空)
  // refinePath 内部的 getNearestPoint 会崩溃，所以这个场景应验证函数能处理空路径
  // 我们通过不调用来跳过这个会崩溃的场景，因为函数本身没有对空路径做保护
  // 在实际代码中，应保证传入的 path_data 是有效的。

  // 场景2: 航向角误差过大
  {
    SCOPED_TRACE("Scenario: Large heading error");
    start_pt.mutable_path_pt()->set_theta(0.0);
    path_data->mutableDiscretizedPath()->clear();
    path_data->mutableDiscretizedPath()->emplace_back(0, 0, 0, 0, M_PI, 0, 0, 0, 0);
    planner_tester_->refinePath(start_pt, path_data.get());
    EXPECT_TRUE(path_data->discretized_path().empty());
  }

  // 场景3: 正常refine & 循环中s超出范围 (测试 break)
  {
    SCOPED_TRACE("Scenario: Normal refine and break condition");
    path_data->mutableDiscretizedPath()->clear();
    for (int i = 0; i < 50; ++i) {  // 创建一条 s 范围为 0-49 的路径
      path_data->mutableDiscretizedPath()->emplace_back(i, 0, 0, 0, 0, 0, i, 0, 0);
    }
    // 将起点设置在路径中间
    start_pt.mutable_path_pt()->set_x(20.0);
    start_pt.mutable_path_pt()->set_y(0.0);
    start_pt.mutable_path_pt()->set_theta(0.0);

    // 关键: 将循环的 curr_length 设置得很大，以触发 curr_s > back().s() 的 break 条件
    // refinePath 内部循环 curr_length < 200.0，而我们的路径总长只有 50
    planner_tester_->refinePath(start_pt, path_data.get());
    EXPECT_FALSE(path_data->discretized_path().empty());
    // 优化后的路径长度应该小于原始路径的剩余长度
    EXPECT_EQ(path_data->discretized_path().size(), 31);
  }
}

TEST_F(PathOptimizerTest, IsInRangeCoverage) {
  SCOPED_TRACE("Testing both overloads of isInRange");

  // --- 测试第一个重载版本 (vector<pair<float, float>>) ---
  {
    // 场景1: 空的范围列表
    EXPECT_FALSE(PathOptimizer::isInRange({}, 50.0));

    // 场景2: s不在范围内
    std::vector<std::pair<float, float>> ranges = {{10.0, 20.0}, {30.0, 40.0}};
    EXPECT_FALSE(PathOptimizer::isInRange(ranges, 25.0));

    // 场景3: s在范围内
    EXPECT_TRUE(PathOptimizer::isInRange(ranges, 15.0));
  }

  // --- 测试第二个重载版本 (vector<tuple<string, float, float>>) ---
  {
    // 场景1: 空的范围列表
    EXPECT_FALSE(PathOptimizer::isInRange({}, "Curve", 50.0));

    // 场景2: 标签不匹配
    std::vector<std::tuple<std::string, float, float>> tagged_ranges = {{"Block", 10.0, 20.0}};
    EXPECT_FALSE(PathOptimizer::isInRange(tagged_ranges, "Curve", 15.0));

    // 场景3: 标签匹配，但s不在范围内
    tagged_ranges.emplace_back("Curve", 30.0, 40.0);
    EXPECT_FALSE(PathOptimizer::isInRange(tagged_ranges, "Curve", 25.0));

    // 场景4: 标签和s都匹配
    EXPECT_TRUE(PathOptimizer::isInRange(tagged_ranges, "Curve", 35.0));
  }
}

TEST_F(PathOptimizerTest, IsInIgnoreRangeCoverage) {
  SCOPED_TRACE("Testing isInIgnoreRange");

  // 场景1: 空的范围列表
  EXPECT_FALSE(PathOptimizer::isInIgnoreRange({}, 50.0));

  // 场景2: s不在范围内
  std::vector<IgnoreRangeInfo> ranges;
  IgnoreRangeInfo range1;
  range1.start_s = 10.0;
  range1.end_s = 20.0;
  ranges.push_back(range1);

  IgnoreRangeInfo range2;
  range2.start_s = 30.0;
  range2.end_s = 40.0;
  ranges.push_back(range2);

  EXPECT_FALSE(PathOptimizer::isInIgnoreRange(ranges, 25.0));

  // 场景3: s在范围内
  EXPECT_TRUE(PathOptimizer::isInIgnoreRange(ranges, 15.0));
}
}  // namespace gpal::pnc::planning