/**
 * @file obstacle_set.h
 * @brief 障碍物集合处理器
 * @details 本类负责多场景下的障碍物过滤与预处理
 */

#include "decision_data/decision_result.h"
#include "local_view/local_view.h"
#include "speed/st_graph.h"
#include "speed_common/speed_common.h"
#include "speed/speed_planner_obstacle.h"

namespace gpal::pnc::planning {

class ObstacleSetProcessor {
 public:
  /// @brief 默认构造函数
  ObstacleSetProcessor() {};
  /// @brief 析构函数
  ~ObstacleSetProcessor() = default;
  void filterObstaclesByDecisionTag(const DecisionResult& decision_result, ObstacleSet* obstacle_set);
  void filterInterestingObstaclesForParking(const DecisionResult& decision_result, ObstacleSet* obstacle_set);
  void filterInterestingObstaclesForAcc(const DecisionResult& decision_result, ObstacleSet* obstacle_set);

 private:
  // void setObstacleDefaultPrediction(std::shared_ptr<Obstacle> obstacle);
  bool IsBlackListObject(const Decision::ObjectType& obj_type);
};

}  // namespace gpal::pnc::planning