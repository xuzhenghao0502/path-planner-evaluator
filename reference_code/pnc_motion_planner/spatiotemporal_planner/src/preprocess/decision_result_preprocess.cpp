#include "preprocess/decision_result_preprocess.h"

namespace gpal::pnc::planning {

bool DecisionResultPreprocess::init() {
  return true;
}

void DecisionResultPreprocess::reset() {
  // 重置逻辑
}

std::string DecisionResultPreprocess::id() const {
  return "DecisionResultPreprocess";
}

bool DecisionResultPreprocess::run(SpatiotemporalPlannerDataManager& data_manager) {
  auto& input_data = data_manager.mutableInputData();

  if (!decisionObjectPreprocess(data_manager)) {
    return false;
  }
  if (!decisionTrajectoryPreprocess(data_manager)) {
    return false;
  }
  if (!decisionBoundaryPreprocess(data_manager)) {
    return false;
  }

  return true;
}

bool DecisionResultPreprocess::decisionObjectPreprocess(SpatiotemporalPlannerDataManager& data_manager) {
  const auto& input_data = data_manager.inputData();
  auto& decision_info = data_manager.mutableObjectsInfo();
  const auto& decision_result = input_data.decision_result;

  if (!decision_result || !decision_result->getOdDecisions()) {
    STLOG(W, "[DecisionObjectsParser] decision_result or od_decisions is null");
    return false;
  }
  const size_t& traj_point_count = data_manager.gridsInfo().time_grid_info.grid.size();

  auto& target_objects = decision_info.target_decision_objects;
  auto& static_objects = decision_info.static_objects;
  target_objects.clear();
  static_objects.clear();

  for (const auto& [id, object] : *decision_result->getOdDecisions()) {
    if (isValidDecisionObject(object)) {
      auto new_object = std::make_unique<Decision::DecisionObject>(object);
      Decision::RawSinglePrediction raw_prediction;
      if (object.is_static || object.raw_predictions.empty()) {
        bool is_small_static_obstacle = object.type == Decision::ObjectType::PEDESTRIAN
                                        || object.type == Decision::ObjectType::VRU
                                        || object.type == Decision::ObjectType::VIRTUAL_OBJECT_POLYLINE;
        if (input_data.stage_state == StageState::HpaDrivingStage) {
          is_small_static_obstacle = true;
        }
        if (is_small_static_obstacle
            && (object.lat_od_tag == LateralOdTag::LEFT_BYPASS || object.lat_od_tag == LateralOdTag::RIGHT_BYPASS)) {
          static_objects.emplace_back(std::make_shared<Decision::DecisionObject>(object));
          STLOG(D, "[DecisionObjectsParser] target obstacle is static");
          continue;
        }
        setObstacleDefaultPrediction(traj_point_count, object, raw_prediction.traj);
        new_object->raw_predictions.emplace_back(std::move(raw_prediction));
      } else {
        setObstaclePrediction(traj_point_count, object, raw_prediction.traj);
        new_object->raw_predictions.emplace_back(std::move(raw_prediction));
      }
      target_objects.emplace_back(std::move(new_object));
      STLOG(D, "[DecisionObjectsParser] target_objects size: ", target_objects.size(), ", id: ", id,
            ", long_od_tag: ", static_cast<int>(object.long_od_tag),
            ", lat_od_tag: ", static_cast<int>(object.lat_od_tag));
    }
  }
  STLOG(D, "[DecisionObjectsParser] target_objects size: ", target_objects.size());
  return true;
}

bool DecisionResultPreprocess::isValidDecisionObject(const Decision::DecisionObject& object) {
    static const std::unordered_set<LongitudinalOdTag> validLongTags = {
        LongitudinalOdTag::FOLLOW,
        LongitudinalOdTag::YIELD,
        LongitudinalOdTag::RISKY,
        LongitudinalOdTag::OVERTAKE
    };

    static const std::unordered_set<LateralOdTag> validLatTags = {
        LateralOdTag::LEFT_BYPASS,
        LateralOdTag::RIGHT_BYPASS,
        LateralOdTag::DYNAMIC_LEFT_BYPASS,
        LateralOdTag::DYNAMIC_RIGHT_BYPASS
    };

    // 检查纵向标签（单个标签或序列中任一标签有效）
    bool longitudinalValid = validLongTags.count(object.long_od_tag) > 0
                             || std::any_of(object.long_od_tags_seq.begin(), object.long_od_tags_seq.end(),
                                            [&](auto&& tag) { return validLongTags.count(tag) > 0; });

    // 检查横向标签（单个标签或序列中任一标签有效）
    bool lateralValid = validLatTags.count(object.lat_od_tag) > 0
                        || std::any_of(object.lat_od_tags_seq.begin(), object.lat_od_tags_seq.end(),
                                       [&](auto&& tag) { return validLatTags.count(tag) > 0; });

    return longitudinalValid || lateralValid;
}

void DecisionResultPreprocess::setObstacleDefaultPrediction(const size_t& traj_point_count,
                                                            const Decision::DecisionObject& object,
                                                            std::vector<proto::TrajectoryPoint>& predicted_trajectory) {
  predicted_trajectory.clear();
  proto::TrajectoryPoint point;
  point.mutable_path_point()->set_x(object.cur_box.center().x());
  point.mutable_path_point()->set_y(object.cur_box.center().y());
  point.mutable_path_point()->set_theta(object.cur_box.heading());
  point.set_v(0.0);
  point.set_a(0.0);
  for (int i = 0; i < traj_point_count; i++) {
    point.set_relative_time(0.1 * i);
    predicted_trajectory.push_back(point);
  }
}

void DecisionResultPreprocess::setObstaclePrediction(const size_t& traj_point_count,
                                                     const Decision::DecisionObject& object,
                                                     std::vector<proto::TrajectoryPoint>& predicted_trajectory) {
  predicted_trajectory.clear();
  auto& raw_prediction = object.raw_predictions.front();
  for (const auto& traj_point : raw_prediction.traj) {
    if (traj_point.relative_time() > -kEpsilon) {
      predicted_trajectory.push_back(traj_point);
    }
  }
  if (predicted_trajectory.empty()) {
    setObstacleDefaultPrediction(traj_point_count, object, predicted_trajectory);
  } else {
    if (predicted_trajectory.size() < traj_point_count) {
      for (int i = predicted_trajectory.size(); i < traj_point_count; i++) {
        predicted_trajectory.push_back(predicted_trajectory.back());
      }
    }
  }
}

bool DecisionResultPreprocess::decisionTrajectoryPreprocess(SpatiotemporalPlannerDataManager& data_manager) {
  const auto& input_data = data_manager.inputData();
  const auto& decision_result = input_data.decision_result;

  return true;  // 假设预处理成功
}

bool DecisionResultPreprocess::decisionBoundaryPreprocess(SpatiotemporalPlannerDataManager& data_manager) {
  const auto& input_data = data_manager.inputData();
  const auto& decision_result = input_data.decision_result;

  return true;  // 假设预处理成功
}

REGIST_MODULE(DecisionResultPreprocess);

}  // namespace gpal::pnc::planning
