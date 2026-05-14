#include <gtest/gtest.h>
#include <any>
#include "config_manager/config_manager.h"
#include "config/speed_planner/speed_preprocessor.pb.h"

#define private public
#include "speed_preprocessor/lateral_path.h"


namespace gpal::pnc::planning{

class LateralPathProcessorTest : public ::testing::Test {
 protected:
  LateralPathProcessor processor;
  DiscretizedPath path;
};

TEST_F(LateralPathProcessorTest, EmptyPath) {
  PathGroup result = processor.getDiscretizedPathGroup(path, 10.0, 0.5, 1.0);
  PathGroup result_2 = processor.getLocalPathGroup(path, 10.0, 0.5, 1.0);
  EXPECT_TRUE(result.origin_path_.empty());
  EXPECT_TRUE(result_2.origin_path_.empty());
}

TEST_F(LateralPathProcessorTest, NormalPath) {
  std::vector<PathPt> path_points;
  for (int i = 0; i < 50; i++) {
    PathPt pt;
    pt.set_x(i * 1.0);
    pt.set_y(0.0);
    pt.set_theta(0.0);
    pt.set_kappa(0.001);
    pt.set_s(i * 1.0);
    path_points.push_back(pt);
  }
  DiscretizedPath path = DiscretizedPath(path_points);

  PathGroup result = processor.getDiscretizedPathGroup(path, 2.0, 0.5, 0.5);
  PathGroup result_2 = processor.getLocalPathGroup(path, 2.0, 0.5, 0.5);
  EXPECT_FALSE(result.origin_path_.empty());
  EXPECT_FALSE(result_2.origin_path_.empty());
}

TEST_F(LateralPathProcessorTest, ReversePath) {
  std::vector<PathPt> path_points;
  for (int i = 0; i < 50; i++) {
    PathPt pt;
    pt.set_x(i * 1.0);
    pt.set_y(0.0);
    pt.set_theta(0.0);
    pt.set_kappa(0.001);
    pt.set_s(i * 1.0);
    pt.set_direction(PathPt::Direction::BACKWARD);
    path_points.push_back(pt);
  }
  DiscretizedPath path = DiscretizedPath(path_points);
  PathGroup result = processor.getDiscretizedPathGroup(path, 2.0, 0.5, 0.5);
  PathGroup result_2 = processor.getLocalPathGroup(path, 2.0, 0.5, 0.5);
  EXPECT_FALSE(result.origin_path_.empty());
  EXPECT_FALSE(result_2.origin_path_.empty());
}


TEST_F(LateralPathProcessorTest, CurvePath) {
  std::vector<PathPt> path_points;
  for (int i = 0; i < 50; i++) {
    PathPt pt;
    pt.set_x(50*sin(i*2*ANG2RAD));
    pt.set_y(50*cos(i*2*ANG2RAD));
    pt.set_theta(i*2*ANG2RAD);
    pt.set_kappa(0.02);
    pt.set_s(50*i*2*ANG2RAD);
    path_points.push_back(pt);
  }
  DiscretizedPath path = DiscretizedPath(path_points);

  PathGroup result = processor.getDiscretizedPathGroup(path, 2.0, 0.5, 0.5);
  PathGroup result_2 = processor.getLocalPathGroup(path, 2.0, 0.5, 0.5);
  EXPECT_FALSE(result.origin_path_.empty());
  EXPECT_FALSE(result_2.origin_path_.empty());
}

TEST_F(LateralPathProcessorTest, OnePointPath) {
  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(1.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(1.0);
  path_points.push_back(pt);

  DiscretizedPath path = DiscretizedPath(path_points);
  PathGroup path_group;
  processor.getPathGroup(path, 2.0, 0.5, 0.5, path_group);
}


TEST_F(LateralPathProcessorTest, OnePointPathInvalid) {
  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(1.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.push_back(pt);

  DiscretizedPath path = DiscretizedPath(path_points);
  PathGroup path_group;
  processor.getPathGroup(path, 2.0, 0.5, 0.5, path_group);
}

TEST_F(LateralPathProcessorTest, OnePointPathInvalid2) {
  std::vector<PathPt> path_points;
  PathPt pt;
  pt.set_x(1.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(0.0);
  path_points.push_back(pt);
  pt.set_x(1.0);
  pt.set_y(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.001);
  pt.set_s(-1.0);
  path_points.push_back(pt);
  DiscretizedPath path = DiscretizedPath(path_points);
  PathGroup path_group;
  processor.getPathGroup(path, 2.0, 0.5, 0.5, path_group);
}

TEST_F(LateralPathProcessorTest, BigCurvePath) {
  std::vector<PathPt> path_points;
  for (int i = 0; i < 50; i++) {
    PathPt pt;
    pt.set_x(50*sin(i*2*ANG2RAD));
    pt.set_y(50*cos(i*2*ANG2RAD));
    pt.set_theta(i*70*ANG2RAD);
    pt.set_kappa(0.02);
    pt.set_s(50*i*2*ANG2RAD);
    path_points.push_back(pt);
  }
  DiscretizedPath path = DiscretizedPath(path_points);
  PathGroup path_group;
  processor.getPathGroup(path, 50.0, 0.5, 0.5, path_group);

}

}