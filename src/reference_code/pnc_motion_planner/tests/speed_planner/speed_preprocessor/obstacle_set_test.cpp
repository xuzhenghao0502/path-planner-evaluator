#include <gtest/gtest.h>
#include <any>
#include "config_manager/config_manager.h"
#include "config/speed_planner/speed_preprocessor.pb.h"

#define private public
#include "speed_preprocessor/obstacle_set.h"


namespace gpal::pnc::planning {
class MockObstacle : public Obstacle {
 public:
};

class MockLocalView : public LocalView {
 public:
  void addObstacle(string id) {
    Obstacle obs(id, math::Polygon2d(math::Box2d(math::Vec2d(20.0, 0), 0.0, 5.0, 2.0)));
    this->getMutableIndexedObstaclesPtr()->add(obs);
  }

  void addObstacle(string id, proto::PerceptionObstacle_ObstacleType type) {
    proto::PerceptionObstacle perception_obstacle;
    perception_obstacle.add_sub_obstacles();
    perception_obstacle.set_obstacle_type(type);
    Obstacle obs(id, 0, perception_obstacle);
    this->getMutableIndexedObstaclesPtr()->add(obs);
  }
};

class MockDecisionResult : public DecisionResult {
 public:
  void addDecision(string id, LongitudinalOdTag tag) {
    std::vector<Decision::RawSinglePrediction> raw_prediction_in;
    Decision::DecisionObject decision_obj(id, 5.0, 2.0, 20.0, 0.0, 0.0, 5.0, false, raw_prediction_in);
    decision_obj.long_od_tag = tag;
    this->getMutableOdDecisions()->emplace(id, decision_obj);
  }
  void addCipvCandidates(string id) { this->cipo_candidates_.emplace_back(id, 0.0); }
};

TEST(FilterObstaclesByDecisionTagTest, NullDecisionMap) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;

  decision_result.od_decision_ = nullptr;
  processor.filterObstaclesByDecisionTag(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(FilterObstaclesByDecisionTagTest, EmptyDecisionMap) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;

  // Test case 1: Empty decision map
  local_view.addObstacle("1001");
  processor.filterObstaclesByDecisionTag(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(FilterObstaclesByDecisionTagTest, ObstacleWithInvalidTag) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  // Test case 2: Obstacle with INVALID tag
  local_view.addObstacle("1001");
  decision_result.addDecision("1001", LongitudinalOdTag::INVALID);
  processor.filterObstaclesByDecisionTag(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(FilterObstaclesByDecisionTagTest, ObstacleWithIgnoreTag) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  // Test case 3: Obstacle with IGNORE tag
  local_view.addObstacle("1001");
  decision_result.addDecision("1001", LongitudinalOdTag::IGNORE);
  processor.filterObstaclesByDecisionTag(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(FilterObstaclesByDecisionTagTest, ObstacleWithFollowTag) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  // Test case 4: Obstacle with Follow tag
  local_view.addObstacle("1001");
  decision_result.addDecision("1001", LongitudinalOdTag::FOLLOW);
  processor.filterObstaclesByDecisionTag(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 1);
}

TEST(filterInterestingObstaclesForParkingTest, NullDecisionMap) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;

  decision_result.od_decision_ = nullptr;
  processor.filterInterestingObstaclesForParking(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(filterInterestingObstaclesForParkingTest, EmptyDecisionMap) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;

  // Test case 1: Empty decision map
  local_view.addObstacle("1001");
  processor.filterInterestingObstaclesForParking(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(filterInterestingObstaclesForParkingTest, InvalidObstacle) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;

  local_view.addObstacle("1001", proto::PerceptionObstacle_ObstacleType_kTypeInvalid);
  decision_result.addDecision("1002", LongitudinalOdTag::INVALID);
  processor.filterInterestingObstaclesForParking(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(filterInterestingObstaclesForParkingTest, ObstacleWithInvalidTag) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  // Test case 2: Obstacle with INVALID tag
  local_view.addObstacle("1001", proto::PerceptionObstacle_ObstacleType_kTypeInvalid);
  decision_result.addDecision("1001", LongitudinalOdTag::INVALID);
  processor.filterInterestingObstaclesForParking(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(filterInterestingObstaclesForParkingTest, ObstacleWithIgnoreTag) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  // Test case 3: Obstacle with IGNORE tag
  local_view.addObstacle("1001", proto::PerceptionObstacle_ObstacleType_kTypeCar);
  decision_result.addDecision("1001", LongitudinalOdTag::IGNORE);
  processor.filterInterestingObstaclesForParking(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 1);
}

TEST(filterInterestingObstaclesForParkingTest, ObstacleWithFollowTagWithType) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  // Test case 5: Obstacle with Follow tag with type
  local_view.addObstacle("1001", proto::PerceptionObstacle_ObstacleType_kTypeCar);
  decision_result.addDecision("1001", LongitudinalOdTag::FOLLOW);
  processor.filterInterestingObstaclesForParking(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 1);
}

TEST(filterInterestingObstaclesForParkingTest, ObstacleWithemptyDecision) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  // Test case 5: Obstacle with Follow tag with type
  local_view.addObstacle("1001", proto::PerceptionObstacle_ObstacleType_kTypeCar);
  decision_result.addDecision("1002", LongitudinalOdTag::FOLLOW);
  processor.filterInterestingObstaclesForParking(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 1);
}

TEST(filterInterestingObstaclesForAccTest, Null) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  decision_result.od_decision_ = nullptr;
  processor.filterInterestingObstaclesForAcc(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(filterInterestingObstaclesForAccTest, Empty) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  processor.filterInterestingObstaclesForAcc(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(filterInterestingObstaclesForAccTest, EmptyCipv) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  local_view.addObstacle("1001");
  processor.filterInterestingObstaclesForAcc(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}
TEST(filterInterestingObstaclesForAccTest, EmptyDecisionObstacle) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  local_view.addObstacle("1001");
  decision_result.addCipvCandidates("1001");
  processor.filterInterestingObstaclesForAcc(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(filterInterestingObstaclesForAccTest, EmptyObstacle) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  decision_result.addDecision("1001", LongitudinalOdTag::FOLLOW);
  decision_result.addCipvCandidates("1001");
  processor.filterInterestingObstaclesForAcc(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 0);
}

TEST(filterInterestingObstaclesForAccTest, Normal) {
  ObstacleSetProcessor processor;
  MockLocalView local_view;
  MockDecisionResult decision_result;
  ObstacleSet obstacle_set;
  decision_result.addDecision("1001", LongitudinalOdTag::FOLLOW);
  decision_result.addCipvCandidates("1001");
  local_view.addObstacle("1001");
  processor.filterInterestingObstaclesForAcc(local_view, decision_result, &obstacle_set);
  EXPECT_EQ(obstacle_set.size(), 1);
}

TEST(IsBlackListObjectTest, BlackListObject) {
  ObstacleSetProcessor processor;
  EXPECT_TRUE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypeInvalid));
  EXPECT_TRUE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypeParkingLockUp));
  EXPECT_TRUE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypeParkingLockDown));
  EXPECT_TRUE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypeWheelStopper));
  EXPECT_TRUE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypeSprinkler));
  EXPECT_TRUE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypeFallenCone));
  EXPECT_TRUE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypePedestrianNonStanding));
  EXPECT_TRUE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypeUnderDrivable));
  EXPECT_TRUE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypeOverDrivable));
  EXPECT_TRUE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypeUnknown));
  EXPECT_FALSE(processor.IsBlackListObject(proto::PerceptionObstacle_ObstacleType_kTypeCar));
}
}