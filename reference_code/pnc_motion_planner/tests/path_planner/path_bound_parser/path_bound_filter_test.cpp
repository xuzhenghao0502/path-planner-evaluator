#include <gtest/gtest.h>
#include <filesystem>
#include <fmt/chrono.h>
#include <algorithm>
#include "config_manager/config_manager.h"
#include "path_bound_parser/path_bound_filter.h"
#include "decision_data/decision_define.h"

namespace gpal::pnc::planning {
// ====================================================================================
// Tester 派生类，用于访问 protected 方法
// ====================================================================================
class PathBoundFilterTester : public PathBoundFilter {
 public:
  // 使用基类的构造函数
  using PathBoundFilter::PathBoundFilter;

  // 将受保护的 updateFilterBoundary 方法公开
  void callUpdateFilterBoundary(const double& filter_in, const double& filter_out,
                                std::tuple<double, double, double>& current_bound) {
    this->updateFilterBoundary(filter_in, filter_out, current_bound);
  }
};

// ====================================================================================
// GTest Fixture: 用于共享设置和辅助函数
// ====================================================================================
class PathBoundFilterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 创建一个默认的配置供所有测试使用
    PathBoundFilterConfig config;
    config.set_filter_in(0.8);
    config.set_filter_out(0.2);
    config.set_confidence_range(20.0);
    filter_ = std::make_unique<PathBoundFilterTester>(config);
  }

  std::unique_ptr<PathBoundFilterTester> filter_;
};

// ====================================================================================
// 测试用例
// ====================================================================================

TEST_F(PathBoundFilterTest, ResetFunction) {
  // 验证reset功能
  std::vector<std::tuple<double, double, double>> bound = {{0.0, -1.0, 1.0}};
  filter_->update(bound);
  ASSERT_FALSE(filter_->bound().empty());

  filter_->reset();
  EXPECT_TRUE(filter_->bound().empty());
}

TEST_F(PathBoundFilterTest, UpdateAndFilterBoundaryCoverage) {
  SCOPED_TRACE("Testing update and updateFilterBoundary");

  // 场景1: 第一次更新，bound_ 为空，has_prev_bound 为 false
  std::vector<std::tuple<double, double, double>> initial_bound = {{10.0, -2.0, 2.0}};
  filter_->update(initial_bound);
  ASSERT_EQ(filter_->bound().size(), 1);
  EXPECT_DOUBLE_EQ(std::get<1>(filter_->bound().at(0)), -2.0);
  EXPECT_DOUBLE_EQ(std::get<2>(filter_->bound().at(0)), 2.0);

  // 场景2: 第二次更新，边界收缩 (lower变大, upper变小)
  // 触发 if (lower > prev_lower) 和 if (upper < prev_upper)
  std::vector<std::tuple<double, double, double>> contracting_bound = {{10.0, -1.0, 1.0}};
  filter_->update(contracting_bound);
  ASSERT_EQ(filter_->bound().size(), 1);
  // 预期: lower = 0.8 * (-1.0) + 0.2 * (-2.0) = -0.8 - 0.4 = -1.2
  EXPECT_NEAR(std::get<1>(filter_->bound().at(0)), -1.2, 1e-9);
  // 预期: upper = 0.8 * (1.0) + 0.2 * (2.0) = 0.8 + 0.4 = 1.2
  EXPECT_NEAR(std::get<2>(filter_->bound().at(0)), 1.2, 1e-9);

  // 场景3: 第三次更新，边界扩张 (lower变小, upper变大)
  // 触发 else 分支
  std::vector<std::tuple<double, double, double>> expanding_bound = {{10.0, -3.0, 3.0}};
  filter_->update(expanding_bound);
  ASSERT_EQ(filter_->bound().size(), 1);
  // 预期: lower = 0.2 * (-3.0) + 0.8 * (-1.2) = -0.6 - 0.96 = -1.56
  EXPECT_NEAR(std::get<1>(filter_->bound().at(0)), -1.56, 1e-9);
  // 预期: upper = 0.2 * (3.0) + 0.8 * (1.2) = 0.6 + 0.96 = 1.56
  EXPECT_NEAR(std::get<2>(filter_->bound().at(0)), 1.56, 1e-9);
}

TEST_F(PathBoundFilterTest, UpdateWithSpeedCoverage) {
  SCOPED_TRACE("Testing updateWithSpeed (2 arguments)");

  // 场景1: 第一次更新，建立初始状态
  std::vector<std::tuple<double, double, double>> initial_bound = {{10.0, -2.0, 2.0}};
  filter_->updateWithSpeed(initial_bound);
  ASSERT_EQ(filter_->bound().size(), 1);
  EXPECT_DOUBLE_EQ(std::get<1>(filter_->bound().at(0)), -2.0);
  EXPECT_DOUBLE_EQ(std::get<2>(filter_->bound().at(0)), 2.0);

  // 场景2: 第二次更新，边界扩张，触发 filter out 逻辑
  std::vector<std::tuple<double, double, double>> expanding_bound = {{10.0, -3.0, 3.0}};
  filter_->setFilterOutSpeed(0.5);  // 设置一个已知的过滤速度
  filter_->updateWithSpeed(expanding_bound);
  ASSERT_EQ(filter_->bound().size(), 1);
  // 预期: lower = max(-3.0, -2.0 - 0.5) = -2.5
  EXPECT_DOUBLE_EQ(std::get<1>(filter_->bound().at(0)), -2.5);
  // 预期: upper = min(3.0, 2.0 + 0.5) = 2.5
  EXPECT_DOUBLE_EQ(std::get<2>(filter_->bound().at(0)), 2.5);
}

TEST_F(PathBoundFilterTest, UpdateWithSpeedAndTypeCoverage) {
  SCOPED_TRACE("Testing updateWithSpeed (3 arguments)");

  // 场景1: type="soft" 且在置信区间内
  {
    std::vector<std::tuple<double, double, double>> bound = {{10.0, -3.0, 3.0}};
    filter_->update(bound);  // 先设置一个初始值
    filter_->setFilterOutSpeed(0.5);

    // 调用，s=15, cur_s=0, confidence_range=20.0, s < cur_s + confidence_range
    filter_->updateWithSpeed(bound, 0.0, "soft");
    // 预期: tmp_filter_out_speed = 0.1
    // lower = max(-3.0, -3.0 - 0.1) = -3.0
    EXPECT_DOUBLE_EQ(std::get<1>(filter_->bound().at(0)), -3.0);
  }

  // 场景2: type="barrier"
  {
    std::vector<std::tuple<double, double, double>> bound = {{10.0, -3.0, 3.0}};
    filter_->update(bound);
    filter_->setFilterOutSpeed(0.5);

    // 调用，type="barrier"，不进入特殊逻辑
    filter_->updateWithSpeed(bound, 0.0, "barrier");
    // 预期: tmp_filter_out_speed = 0.5
    // lower = max(-3.0, -3.0 - 0.5) = -3.0
    EXPECT_DOUBLE_EQ(std::get<1>(filter_->bound().at(0)), -3.0);
  }
}
}  // namespace gpal::pnc::planning