/**
 * @file discretized_path_test.cpp
 * @brief DiscretizedPath类的单元测试
 * @details 该文件包含对DiscretizedPath类的全面测试，验证路径管理、评估和查询功能的正确性
 */

#include <gtest/gtest.h>

#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "base/log.h"
#define protected public
#include "path/discretized_path.h"
#undef protected

#include "point/path_pt.h"

namespace gpal::pnc::planning {

using math::Vec3d;

class DiscretizedPathTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ERT_PLOG_I << "\n[SetUp] Initializing test environment...";
    // 创建测试路径点 (s从0开始，每1米一个点)
    for (int i = 0; i < 10; ++i) {
      test_points_.emplace_back(static_cast<double>(i),  // x
                                0.0,                     // y
                                0.0);                    // z
      test_points_.back().set_s(i);
    }
    path_ = DiscretizedPath(test_points_);
  }

  std::vector<PathPt> test_points_;
  DiscretizedPath path_;
};

// 测试默认构造函数
TEST_F(DiscretizedPathTest, DefaultConstructor) {
  ERT_PLOG_I << "\n[TEST] Testing DefaultConstructor...";
  DiscretizedPath empty_path;
  ERT_PLOG_I << "Created empty path, size: " << empty_path.size();
  EXPECT_TRUE(empty_path.empty());
  EXPECT_DOUBLE_EQ(empty_path.length(), 0.0);
  ERT_PLOG_I << "DefaultConstructor test passed.";
}

// 测试参数化构造函数
TEST_F(DiscretizedPathTest, ParameterizedConstructor) {
  ERT_PLOG_I << "\n[TEST] Testing ParameterizedConstructor...";
  ERT_PLOG_I << "Path size: " << path_.size() << ", expected: " << test_points_.size();
  EXPECT_FALSE(path_.empty());
  EXPECT_EQ(path_.size(), test_points_.size());
  EXPECT_DOUBLE_EQ(path_.length(), 9.0);
  ERT_PLOG_I << "Path length: " << path_.length() << "m, expected: 9.0m";
  ERT_PLOG_I << "ParameterizedConstructor test passed.";
}

// 测试length()函数
TEST_F(DiscretizedPathTest, Length) {
  ERT_PLOG_I << "\n[TEST] Testing Length...";
  ERT_PLOG_I << "Testing normal path length...";
  EXPECT_DOUBLE_EQ(path_.length(), 9.0);
  ERT_PLOG_I << "Normal path length: " << path_.length() << "m, correct.";

  // 测试空路径
  ERT_PLOG_I << "Testing empty path length...";
  DiscretizedPath empty_path;
  EXPECT_DOUBLE_EQ(empty_path.length(), 0.0);
  ERT_PLOG_I << "Empty path length: " << empty_path.length() << "m, correct.";
}

// 测试evaluate()函数
TEST_F(DiscretizedPathTest, Evaluate) {
  ERT_PLOG_I << "\n[TEST] Testing Evaluate...";

  // 测试边界点
  ERT_PLOG_I << "Testing front point evaluation...";
  auto front_pt = path_.evaluate(0.0);
  EXPECT_DOUBLE_EQ(front_pt.s(), 0.0);
  EXPECT_DOUBLE_EQ(front_pt.x(), 0.0);
  EXPECT_DOUBLE_EQ(front_pt.y(), 0.0);
  ERT_PLOG_I << "Front point at s=0.0: (" << front_pt.x() << ", " << front_pt.y() << "), correct.";

  ERT_PLOG_I << "Testing back point evaluation...";
  auto back_pt = path_.evaluate(9.0);
  EXPECT_DOUBLE_EQ(back_pt.s(), 9.0);
  EXPECT_DOUBLE_EQ(back_pt.x(), 9.0);
  EXPECT_DOUBLE_EQ(back_pt.y(), 0);
  ERT_PLOG_I << "Back point at s=9.0: (" << back_pt.x() << ", " << back_pt.y() << "), correct.";

  // 测试中间插值点
  ERT_PLOG_I << "Testing midpoint evaluation...";
  auto mid_pt = path_.evaluate(4.5);
  EXPECT_DOUBLE_EQ(mid_pt.s(), 4.5);
  EXPECT_DOUBLE_EQ(mid_pt.x(), 4.5);
  EXPECT_DOUBLE_EQ(mid_pt.y(), 0);
  ERT_PLOG_I << "Mid point at s=4.5: (" << mid_pt.x() << ", " << mid_pt.y() << "), correct.";

  // 测试超过区间
  ERT_PLOG_I << "Testing outpoint evaluation...";
  auto out_pt = path_.evaluate(10.0);
  EXPECT_DOUBLE_EQ(out_pt.s(), 9.0);
  EXPECT_DOUBLE_EQ(out_pt.x(), 9.0);
  EXPECT_DOUBLE_EQ(out_pt.y(), 0);
  ERT_PLOG_I << "Mid point at s=4.5: (" << out_pt.x() << ", " << out_pt.y() << "), correct.";
}

// 测试evaluateReverse()函数
TEST_F(DiscretizedPathTest, EvaluateReverse) {
  ERT_PLOG_I << "\n[TEST] Testing EvaluateReverse...";

  // 测试边界点
  ERT_PLOG_I << "Testing front point evaluation (reverse)...";
  auto front_pt = path_.evaluateReverse(0.0);
  EXPECT_DOUBLE_EQ(front_pt.s(), 0.0);
  ERT_PLOG_I << "Front point at s=0.0 evaluated correctly in reverse.";

  ERT_PLOG_I << "Testing back point evaluation (reverse)...";
  auto back_pt = path_.evaluateReverse(9.0);
  EXPECT_DOUBLE_EQ(back_pt.s(), 9.0);
  ERT_PLOG_I << "Back point at s=9.0 evaluated correctly in reverse.";

  // 测试中间插值点
  ERT_PLOG_I << "Testing midpoint evaluation (reverse)...";
  auto mid_pt = path_.evaluateReverse(4.5);
  EXPECT_DOUBLE_EQ(mid_pt.s(), 4.5);
  EXPECT_DOUBLE_EQ(mid_pt.x(), 4.5);
  EXPECT_DOUBLE_EQ(mid_pt.y(), 0.0);
  ERT_PLOG_I << "Mid point at s=4.5: (" << mid_pt.x() << ", " << mid_pt.y() << "), correct in reverse.";

  // 测试超过区间
  ERT_PLOG_I << "Testing outpoint evaluation (reverse)...";
  auto out_pt = path_.evaluateReverse(-1);
  EXPECT_DOUBLE_EQ(out_pt.s(), 0);
  EXPECT_DOUBLE_EQ(out_pt.x(), 0);
  EXPECT_DOUBLE_EQ(out_pt.y(), 0.0);
  ERT_PLOG_I << "Mid point at s=4.5: (" << mid_pt.x() << ", " << mid_pt.y() << "), correct in reverse.";
}

// 测试getNearestPoint()函数 - 无区间版本
TEST_F(DiscretizedPathTest, GetNearestPoint) {
  ERT_PLOG_I << "\n[TEST] Testing GetNearestPoint...";
  double min_dist = 0.0;

  // 测试路径上的点
  ERT_PLOG_I << "Testing point exactly on path...";
  Vec3d on_path_pt(4.5, 0.0, 0.0);  // 对应s=4.5
  ERT_PLOG_I << "Searching nearest point to (" << on_path_pt.x() << ", " << on_path_pt.y() << ")";
  auto nearest = path_.getNearestPoint(on_path_pt, min_dist);
  EXPECT_DOUBLE_EQ(nearest.s(), 4.5);
  EXPECT_NEAR(min_dist, 0.0, 1e-6);
  ERT_PLOG_I << "Found nearest point at s=" << nearest.s() << ", distance=" << min_dist << ", correct.";

  // 测试路径附近的点
  ERT_PLOG_I << "Testing point near path...";
  Vec3d near_path_pt(4.0, 0.2, 0.0);  // 最近点应该是s=4.0
  ERT_PLOG_I << "Searching nearest point to (" << near_path_pt.x() << ", " << near_path_pt.y() << ")";
  nearest = path_.getNearestPoint(near_path_pt, min_dist);
  EXPECT_DOUBLE_EQ(nearest.s(), 4.0);
  EXPECT_NEAR(min_dist, 0.2, 1e-6);
  ERT_PLOG_I << "Found nearest point at s=" << nearest.s() << ", distance=" << min_dist << " (expected: " << 0.2
             << "), correct.";
}

// 测试getNearestPoint()函数 - 带区间版本
TEST_F(DiscretizedPathTest, GetNearestPointWithRange) {
  ERT_PLOG_I << "\n[TEST] Testing GetNearestPointWithRange...";

  // getNewtonStep测试
  ERT_PLOG_I << "Testing getNewtonStep()...";
  std::vector<PathPt> path;
  for (int i = 0; i < 10; ++i) {
    double s = i * 1.0;
    PathPt pt(i * 1.0, 0, 0);
    pt.set_theta(0.0);
    pt.set_kappa(-1);
    pt.set_s(s);
    path.emplace_back(pt);
  }
  DiscretizedPath dp(path);
  math::Vec3d vehicle_pt(5.0, -1, 0);
  double min_dist;
  PathPt nearest = dp.getNearestPoint(vehicle_pt, min_dist);

  // 区间错误
  ERT_PLOG_I << "Testing invalid range...";
  vehicle_pt = math::Vec3d(3.0, 0.0, 0.0);
  nearest = path_.getNearestPoint(vehicle_pt, 4.0, 0.0, min_dist);
  EXPECT_DOUBLE_EQ(nearest.s(), 3.0);
  EXPECT_NEAR(min_dist, 0.0, 1e-6);
  ERT_PLOG_I << "Found nearest point at s=" << nearest.s() << ", distance=" << min_dist << ", correct.";

  // ui < s_range.first branch测试
  ERT_PLOG_I << "Testing ui < s_range.first...";
  path.clear();
  for (int i = 0; i < 10; ++i) {
    double s = i * 1.0;
    PathPt pt(i * 1.0, 0, 0);
    pt.set_theta(0.0);
    pt.set_kappa(-1);
    pt.set_s(s);
    path.emplace_back(pt);
  }
  dp = DiscretizedPath(path);
  vehicle_pt = math::Vec3d(10.0, -1, 0);
  nearest = dp.getNearestPoint(vehicle_pt, min_dist);

  // ui > s_range.second 条件测试
  ERT_PLOG_I << "Testing ui > s_range.second...";
  path.clear();
  for (int i = 0; i < 10; ++i) {
    double s = i * 1.0;
    PathPt pt(i * 1.0, 0, 0);
    pt.set_theta(0.0);
    pt.set_kappa(1);
    pt.set_s(s);
    path.emplace_back(pt);
  }
  dp = DiscretizedPath(path);
  vehicle_pt = math::Vec3d(2.0, -1, 0);
  nearest = dp.getNearestPoint(vehicle_pt, 3.0, 6.0, min_dist);

  // newton_range.first > newton_range.second 条件测试
  ERT_PLOG_I << "Testing newton_range.first > newton_range.second...";
  path.clear();
  for (int i = 0; i < 10; ++i) {
    double s = i * 1.0;
    PathPt pt(i * 1.0, 0, 0);
    pt.set_theta(0.0);
    pt.set_kappa(0);
    pt.set_s(9.0 - s);
    path.emplace_back(pt);
  }
  dp = DiscretizedPath(path);
  vehicle_pt = math::Vec3d(5.0, 0, 0);
  nearest = dp.getNearestPoint(vehicle_pt, 3.0, 6.0, min_dist);

  ERT_PLOG_I << "GetNearestPointWithRange test passed.";
}

// 测试getPathPts()函数
TEST_F(DiscretizedPathTest, GetPathPts) {
  ERT_PLOG_I << "\n[TEST] Testing GetPathPts...";
  DiscretizedPath sub_path;

  // 测试正常区间
  ERT_PLOG_I << "Testing normal range [2.0, 5.0]...";
  path_.getPathPts(2.0, 5.0, &sub_path);
  EXPECT_EQ(sub_path.size(), 4);  // s=2,3,4,5
  EXPECT_DOUBLE_EQ(sub_path.front().s(), 2.0);
  EXPECT_DOUBLE_EQ(sub_path.back().s(), 5.0);
  ERT_PLOG_I << "Got " << sub_path.size() << " points from s=" << sub_path.front().s()
             << " to s=" << sub_path.back().s() << ", correct.";

  // 测试边界情况
  ERT_PLOG_I << "Testing full range [0.0, 9.0]...";
  path_.getPathPts(0.0, 9.0, &sub_path);
  EXPECT_EQ(sub_path.size(), 10);  // 全部点
  ERT_PLOG_I << "Got " << sub_path.size() << " points (full path), correct.";

  // 测试空区间
  ERT_PLOG_I << "Testing invalid range [5.0, 2.0]...";
  path_.getPathPts(5.0, 2.0, &sub_path);
  EXPECT_TRUE(sub_path.empty());
  ERT_PLOG_I << "Got " << sub_path.size() << " points from invalid range, correct.";

  // 测试超出范围的区间
  ERT_PLOG_I << "Testing out-of-range [10.0, 12.0]...";
  path_.getPathPts(10.0, 12.0, &sub_path);
  EXPECT_TRUE(sub_path.empty());
  ERT_PLOG_I << "Got " << sub_path.size() << " points from out-of-range, correct.";

  // 测试部分超出范围
  ERT_PLOG_I << "Testing partially out-of-range [8.0, 11.0]...";
  path_.getPathPts(8.0, 11.0, &sub_path);
  EXPECT_EQ(sub_path.size(), 2);  // s=8,9
  ERT_PLOG_I << "Got " << sub_path.size() << " points from partially out-of-range, correct.";

  ERT_PLOG_I << "GetPathPts test passed.";
}

// 测试queryLowerBound()函数
TEST_F(DiscretizedPathTest, QueryLowerBound) {
  ERT_PLOG_I << "\n[TEST] Testing QueryLowerBound...";

  // 测试精确匹配
  ERT_PLOG_I << "Testing exact match (s=3.0)...";
  auto it = path_.queryLowerBound(3.0);
  EXPECT_DOUBLE_EQ(it->s(), 3.0);
  ERT_PLOG_I << "Found point at s=" << it->s() << ", correct.";

  // 测试非精确匹配
  ERT_PLOG_I << "Testing non-exact match (s=3.5)...";
  it = path_.queryLowerBound(3.7);
  EXPECT_DOUBLE_EQ(it->s(), 4.0);  // lower_bound返回第一个不小于的
  ERT_PLOG_I << "Found point at s=" << it->s() << " for s=3.5 query, correct.";

  // 测试边界情况
  ERT_PLOG_I << "Testing below-range query (s=-1.0)...";
  it = path_.queryLowerBound(-1.0);
  EXPECT_DOUBLE_EQ(it->s(), 0.0);  // 返回第一个点
  ERT_PLOG_I << "Found point at s=" << it->s() << " for below-range query, correct.";

  ERT_PLOG_I << "Testing above-range query (s=10.0)...";
  it = path_.queryLowerBound(10.0);
  EXPECT_EQ(it, path_.end());  // 超出范围返回end()
  ERT_PLOG_I << "Returned end() iterator for above-range query, correct.";

  ERT_PLOG_I << "QueryLowerBound test passed.";
}

// 测试queryUpperBound()函数
TEST_F(DiscretizedPathTest, QueryUpperBound) {
  ERT_PLOG_I << "\n[TEST] Testing QueryUpperBound...";

  // 测试精确匹配
  ERT_PLOG_I << "Testing exact match (s=3.0)...";
  auto it = path_.queryUpperBound(3.0);
  EXPECT_DOUBLE_EQ(it->s(), 4.0);  // upper_bound返回第一个大于的
  ERT_PLOG_I << "Found point at s=" << it->s() << " for s=3.0 query, correct.";

  // 测试非精确匹配
  ERT_PLOG_I << "Testing non-exact match (s=3.5)...";
  it = path_.queryUpperBound(3.5);
  EXPECT_DOUBLE_EQ(it->s(), 4.0);
  ERT_PLOG_I << "Found point at s=" << it->s() << " for s=3.5 query, correct.";

  // 测试边界情况
  ERT_PLOG_I << "Testing below-range query (s=-1.0)...";
  it = path_.queryUpperBound(-1.0);
  EXPECT_DOUBLE_EQ(it->s(), 0.0);  // 返回第一个点
  ERT_PLOG_I << "Found point at s=" << it->s() << " for below-range query, correct.";

  ERT_PLOG_I << "Testing above-range query (s=9.0)...";
  it = path_.queryUpperBound(9.0);
  EXPECT_EQ(it, path_.end());  // 超出范围返回end()
  ERT_PLOG_I << "Returned end() iterator for above-range query, correct.";

  ERT_PLOG_I << "QueryUpperBound test passed.";
}

}  // namespace gpal::pnc::planning
