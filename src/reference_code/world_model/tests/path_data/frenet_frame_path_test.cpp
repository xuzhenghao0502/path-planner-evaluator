#include "path/frenet_frame_path.h"

#include <gtest/gtest.h>

#include <vector>

namespace gpal::pnc::planning {

class FrenetFramePathTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 创建测试用的Frenet路径点
    for (double s = 0.0; s <= 10.0; s += 1.0) {
      gpal::pnc::FrenetFramePoint point;
      point.set_s(s);
      point.set_l(s * 0.5);  // l坐标随s线性增长
      point.set_dl(0.5);     // 固定横向速度
      point.set_ddl(0.0);    // 无横向加速度
      test_points_.push_back(point);
    }
  }

  std::vector<FrenetFramePoint> test_points_;
};

TEST_F(FrenetFramePathTest, ConstructorTest) {
  // 测试移动构造
  auto points_copy = test_points_;
  FrenetFramePath path(std::move(points_copy));
  EXPECT_EQ(path.size(), test_points_.size());
  EXPECT_TRUE(points_copy.empty());  // 确保使用了移动语义

  // 测试默认构造
  FrenetFramePath empty_path;
  EXPECT_TRUE(empty_path.empty());
}

TEST_F(FrenetFramePathTest, LengthTest) {
  FrenetFramePath path(test_points_);
  EXPECT_DOUBLE_EQ(path.length(), 10.0);  // 0到10的总长度

  // 测试空路径
  FrenetFramePath empty_path;
  EXPECT_DOUBLE_EQ(empty_path.length(), 0.0);
}

TEST_F(FrenetFramePathTest, EvaluateBySTest) {
  FrenetFramePath path(test_points_);

  // 测试精确匹配点
  auto point_at_5 = path.evaluateByS(5.0);
  EXPECT_DOUBLE_EQ(point_at_5.s(), 5.0);
  EXPECT_DOUBLE_EQ(point_at_5.l(), 2.5);

  // 测试插值点
  auto point_at_3_5 = path.evaluateByS(3.5);
  EXPECT_DOUBLE_EQ(point_at_3_5.s(), 3.5);
  EXPECT_DOUBLE_EQ(point_at_3_5.l(), 1.75);  // 3.5 * 0.5

  // 测试边界条件
  auto first_point = path.evaluateByS(-1.0);
  EXPECT_DOUBLE_EQ(first_point.s(), 0.0);

  auto last_point = path.evaluateByS(11.0);
  EXPECT_DOUBLE_EQ(last_point.s(), 10.0);
}

TEST_F(FrenetFramePathTest, GetNearestPointTest) {
  FrenetFramePath path(test_points_);

  // 创建测试SL边界
  SLBoundary sl_boundary;
  sl_boundary.set_start_s(3.0);
  sl_boundary.set_end_s(7.0);
  sl_boundary.set_start_l(1.0);
  sl_boundary.set_end_l(2.0);

  // 测试获取最近点
  auto nearest_point = path.getNearestPoint(sl_boundary);
  EXPECT_GE(nearest_point.s(), 3.0);
  EXPECT_LE(nearest_point.s(), 7.0);

  // 测试it->l() > sl.end_l()条件
  SLBoundary out_of_range;
  out_of_range.set_start_s(3.0);
  out_of_range.set_end_s(7.0);
  out_of_range.set_start_l(5.0);  // 超出路径上的l值范围
  out_of_range.set_end_l(6.0);
  auto out_point = path.getNearestPoint(out_of_range);

  // 测试无匹配点时返回区间内最近点
  out_of_range.set_start_s(3.0);
  out_of_range.set_end_s(7.0);
  out_of_range.set_start_l(5.0);
  out_of_range.set_end_l(6.0);
  auto test_points_temp = test_points_;
  test_points_temp[4].set_l(0.5);
  FrenetFramePath path_temp(test_points_temp);
  out_point = path_temp.getNearestPoint(out_of_range);

  // 测试无匹配点时返回区间内最近点
  out_of_range.set_start_s(12.0);
  out_of_range.set_end_s(13.0);
  out_of_range.set_start_l(1.0);
  out_of_range.set_end_l(2.0);
  out_point = path.getNearestPoint(out_of_range);
  EXPECT_GE(out_point.s(), 10.0);
}

TEST_F(FrenetFramePathTest, SetAndGetPathTest) {
  FrenetFramePath path;

  // 测试设置路径
  path.setFrenetFramePath(test_points_);
  EXPECT_EQ(path.size(), test_points_.size());

  // 测试获取路径
  const auto& points = path.getFrenetFramePath();
  EXPECT_EQ(points.size(), test_points_.size());

  // 测试清空路径
  path.clearFrenetFramePath();
  EXPECT_TRUE(path.empty());
}

TEST_F(FrenetFramePathTest, BoundaryConditionsTest) {
  // 测试单点路径
  std::vector<FrenetFramePoint> single_point = {test_points_[0]};
  FrenetFramePath single_path(single_point);

  EXPECT_DOUBLE_EQ(single_path.length(), 0.0);
  auto eval_point = single_path.evaluateByS(0.0);
  EXPECT_DOUBLE_EQ(eval_point.s(), 0.0);

  // 测试两点路径
  std::vector<FrenetFramePoint> two_points = {test_points_[0], test_points_[1]};
  FrenetFramePath two_path(two_points);

  EXPECT_DOUBLE_EQ(two_path.length(), 1.0);
  auto mid_point = two_path.evaluateByS(0.5);
  EXPECT_DOUBLE_EQ(mid_point.s(), 0.5);
  EXPECT_DOUBLE_EQ(mid_point.l(), 0.25);
}

}  // namespace gpal::pnc::planning