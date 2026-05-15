#include "decision_objects_parser/decision_objects_parser.h"

namespace gpal::pnc::planning {

std::string DecisionObjectsParser::id() const {
  return "DecisionObjectsParser";
}

void DecisionObjectsParser::reset() {}

bool DecisionObjectsParser::init() {
  return true;
}

// 根据障碍物类型获取对应的buffer配置
BaseObjectParserConfig::ObstacleBufferInfo DecisionObjectsParser::getObstacleBufferInfo(
    const BaseObjectParserConfig::ObjectType& object_type) const {
  // 遍历配置中的障碍物类型buffer信息
  for (const auto& type_buffer_info : decision_object_parser_config_.obstacle_type_buffer_info()) {
    if (type_buffer_info.obs_type() == object_type) {
      return type_buffer_info.obs_info();
    }
  }

  // 如果没找到对应类型的配置，返回默认配置
  return decision_object_parser_config_.obstacle_buffer_info();
}

// 计算纵向buffer
double DecisionObjectsParser::calculateLongitudinalBuffer(const BaseObjectParserConfig::ObstacleBufferInfo& buffer_info, double v_ego,
                                                          double object_speed, bool is_follower) const {
  // 速度松弛参数
  constexpr double speed_relaxation = 0.3;
  const double speed_diff = v_ego - object_speed;
  double res_buffer = 0.0;
  if (is_follower) {
    res_buffer = std::clamp(speed_diff * speed_relaxation, buffer_info.longitudinal_safe_distance_lower(),
                            buffer_info.longitudinal_safe_distance_upper());
  } else {
    res_buffer = std::clamp(object_speed * speed_relaxation, buffer_info.longitudinal_safe_distance_lower(),
                            buffer_info.longitudinal_safe_distance_upper());
  }
  return res_buffer;
}

// 计算横向buffer
double DecisionObjectsParser::calculateLateralBuffer(const BaseObjectParserConfig::ObstacleBufferInfo& buffer_info, double ego_v,
                                                     double object_speed) const {
  const double coff_a = buffer_info.coff_a();
  const double coff_b = buffer_info.coff_b();
  const double coff_c_min = buffer_info.coff_c_min();

  // 根据相对速度和自车速度计算横向buffer
  const double speed_diff = ego_v - object_speed;
  return coff_a * speed_diff * speed_diff + coff_b * ego_v + coff_c_min;
}

BaseObjectParserConfig::ObjectType DecisionObjectsParser::convertToLateralObjectType(
    const Decision::ObjectType& object_type) const {
  switch (object_type) {
    case Decision::ObjectType::VEHICLE:
      return BaseObjectParserConfig::VEHICLE;
    case Decision::ObjectType::HEAVY_VEHICLE:
      return BaseObjectParserConfig::HEAVY_VEHICLE;
    case Decision::ObjectType::PEDESTRIAN:
      return BaseObjectParserConfig::PEDESTRIAN;
    case Decision::ObjectType::VRU:  // 非机动车
      return BaseObjectParserConfig::VRU;
    default:
      return BaseObjectParserConfig::UNKNOWN;
  }
}

Eigen::Matrix2d DecisionObjectsParser::createBaseShapeCovariance(double length, double width) const {
  // 在障碍物局部坐标系下的基础形状协方差
  // 使用 1.415 (≈ √2) 作为从矩形到椭圆的缩放因子
  double a_base = 1.415 * length / 2.0;
  double b_base = 1.415 * width / 2.0;

  // 转换为方差（使用3-sigma原则）
  double var_long = std::pow(a_base / 3.0, 2);
  double var_lat = std::pow(b_base / 3.0, 2);

  // 构造对角协方差矩阵（局部坐标系）
  Eigen::Matrix2d base_cov;
  base_cov << var_long, 0.0, 0.0, var_lat;

  return base_cov;
}

Eigen::Matrix2d DecisionObjectsParser::createBufferCovariance(double length, double width, double long_buffer,
                                                              double lateral_buffer, double ego_buffer) const {
  // 计算有效的缓冲距离（扣除已经包含在基础形状中的部分）
  double ellipse_long_buffer = std::max(0.0, long_buffer - 0.415 * length / 2.0);
  double ellipse_lat_buffer = std::max(0.0, lateral_buffer - 0.415 * width / 2.0);

  // 总缓冲距离 = 有效缓冲 + 自车缓冲
  double total_long_buffer = ellipse_long_buffer + ego_buffer;
  double total_lat_buffer = ellipse_lat_buffer + ego_buffer;

  // 转换为方差（使用3-sigma原则）
  double var_long_buffer = std::pow(total_long_buffer / 3.0, 2);
  double var_lat_buffer = std::pow(total_lat_buffer / 3.0, 2);

  // 构造缓冲协方差矩阵
  Eigen::Matrix2d buffer_cov;
  buffer_cov << var_long_buffer, 0.0, 0.0, var_lat_buffer;

  return buffer_cov;
}

Eigen::Matrix2d DecisionObjectsParser::computeStaticCovariance(double obs_x, double obs_y, double obs_heading,
                                                               double obs_length, double obs_width,
                                                               const Decision::DecisionObject* object,
                                                               int prediction_index, double& ellipse_heading) const {
  // ========== 步骤 1: 创建基础形状协方差==========
  Eigen::Matrix2d base_shape_cov = createBaseShapeCovariance(obs_length, obs_width);

  // ========== 步骤 2: 创建BUFF协方差 ==========
  // Eigen::Matrix2d buffer_cov = createBufferCovariance(obs_length, obs_width, long_buffer, lateral_buffer,
  // ego_buffer);

  // ========== 步骤 3: 创建感知不确定性协方差==========
  // TODO: 实现感知不确定性建模
  Eigen::Matrix2d perception_cov = Eigen::Matrix2d::Zero();

  // ========== 步骤 4: 合成总协方差 ==========
  // Σ_total = Σ_shape + Σ_buffer + Σ_perception
  Eigen::Matrix2d total_cov = base_shape_cov + perception_cov;

  // ========== 步骤 5: 确保协方差矩阵正定 ==========
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(total_cov);
  Eigen::Vector2d eigenvalues = solver.eigenvalues();

  if (eigenvalues.minCoeff() < 1e-6) {
    // 添加正则化项确保数值稳定性
    total_cov += 1e-6 * Eigen::Matrix2d::Identity();
  }

  // ========== 步骤 6: 设置椭圆朝向（非旋转模式）==========
  // 椭圆角度固定为障碍物朝向
  ellipse_heading = obs_heading;

  return total_cov;
}

bool DecisionObjectsParser::run(SpatiotemporalPlannerDataManager& data_manager) {
  const auto& input_data = data_manager.inputData();
  const auto& scenario_info = data_manager.scenarioInfo();
  const auto& decision_result = input_data.decision_result;
  auto& decision_objects = data_manager.mutableObjectsInfo();
  const auto& target_objects = decision_objects.target_decision_objects;
  const auto& target_ref = input_data.target_ref_line_info->ref_line();

   // 获取决策结果
  decision_objects.target_objects_info.clear();
  const auto& target_ref_line_info = input_data.target_ref_line_info;
  target_ref_line_.reset(new ReferenceLine(input_data.target_ref_line_info->ref_line()));
  decision_object_parser_config_ = *data_manager.configInfo().decision_object_parser_profile;

  const double ego_v = input_data.vehicle_info->start_point.v();
  const double time_horizon = decision_object_parser_config_.time_horizon();
  constexpr double ego_buffer = 1.08;

  for (auto& object : target_objects) {
    if (object == nullptr) {
      STLOG(W, "[DecisionObjectsParser] Encountered null object pointer, skipping.");
      continue;
    }

    // 将Decision::ObjectType转换为BaseObjectParserConfig::ObjectType
    BaseObjectParserConfig::ObjectType lateral_object_type = convertToLateralObjectType(object->type);
    // 获取障碍物类型对应的buffer配置
    BaseObjectParserConfig::ObstacleBufferInfo buffer_info = getObstacleBufferInfo(lateral_object_type);

    std::vector<ObjectInfo> objects;
    double obj_speed = object->is_static ? 0.0 : object->spd;
    double obj_acc = object->is_static ? 0.0 : object->acc;
    int cur_pred_index = object->raw_predictions.front().current_index;

    for (int i = cur_pred_index; i < object->raw_predictions.front().traj.size(); ++i) {
      auto pt = object->raw_predictions.front().traj[i];

      double obs_x = pt.path_point().x();
      double obs_y = pt.path_point().y();
      double obs_heading = pt.path_point().theta();
      double ellipse_heading = obs_heading;
      // 根据障碍物类型计算buffer
      bool is_follower = object->long_od_tags_seq.at(i) == LongitudinalOdTag::FOLLOW
                         || object->long_od_tags_seq.at(i) == LongitudinalOdTag::YIELD;
      // 定义buffer计算lambda函数
      auto buffer_calculator = [buffer_info, is_follower](const ObjectInfo& obj, double curr_v, double curr_a,
                                                          int k) -> std::pair<double, double> {
        // 纵向buffer计算
        constexpr double speed_relaxation = 0.3;
        const double speed_diff = curr_v - obj.v;
        double long_buffer = 0.0;

        if (is_follower) {
          long_buffer = std::clamp(speed_diff * speed_relaxation, buffer_info.longitudinal_safe_distance_lower(),
                                   buffer_info.longitudinal_safe_distance_upper());
        } else {
          long_buffer = std::clamp(obj.v * speed_relaxation, buffer_info.longitudinal_safe_distance_lower(),
                                   buffer_info.longitudinal_safe_distance_upper());
        }

        // 横向buffer计算
        const double coff_a = buffer_info.coff_a();
        const double coff_b = buffer_info.coff_b();
        const double coff_c_min = buffer_info.coff_c_min();
        double lat_buffer = coff_a * speed_diff * speed_diff + coff_b * curr_v + coff_c_min;

        // TODO 根据迭代步数k调整buff

        return {long_buffer, lat_buffer};
      };

      double hypot_long = std::hypot(object->cur_box.length(), object->cur_box.width());
      if (hypot_long < 1e-6) {
        hypot_long = 1e-6;
      }
      double width_buffer =
          (object->cur_box.width() / hypot_long) * (hypot_long + buffer_info.hypot_buffer()) - object->cur_box.width();
      double length_buffer = (object->cur_box.length() / hypot_long) * (hypot_long + buffer_info.hypot_buffer())
                             - object->cur_box.length();

      // 获取障碍物在参考线上的最近匹配点
      ReferencePoint ref_point = target_ref_line_->getNearestReferencePoint(math::Vec3d(obs_x, obs_y, 0.0));

      // 计算协方差矩阵
      Eigen::Matrix2d static_covariance = computeStaticCovariance(
          obs_x, obs_y, obs_heading, object->cur_box.length() + length_buffer, object->cur_box.width() + width_buffer,
          object.get(), i - cur_pred_index, ellipse_heading);

      // 使用协方差构造障碍物对象
      ObjectInfo discretized_object(obs_x, obs_y, obs_heading, ref_point.local_s(),
                                    object->cur_box.length() + length_buffer, object->cur_box.width() + width_buffer,
                                    obj_speed, obj_acc, ego_buffer, static_covariance, ellipse_heading,
                                    buffer_calculator);
      // 设置ID和其他重要属性
      discretized_object.id = object->id;
      discretized_object.safe_distance = buffer_info.longitudinal_stop_safe_distance();

      // 设置决策纵向标签
      if (object->long_od_tags_seq.size() > i) {
        discretized_object.longitudinal_od_tag = object->long_od_tags_seq.at(i);
      }

      // 设置决策横向标签
      if (object->lat_od_tags_seq.size() > i) {
        discretized_object.lateral_od_tag = object->lat_od_tags_seq.at(i);
      }

      // 设置博弈类型
      discretized_object.object_game_type = object->game_type;
      if (object->long_od_tags_seq.at(i) == LongitudinalOdTag::YIELD
              && object->game_type == Decision::ObjectGameType::OPPOSITE_GAME
          || object->long_od_tags_seq.at(i) == LongitudinalOdTag::OVERTAKE
      ) {
        if (!object->is_static
            && i * 0.1 < decision_object_parser_config_.dynamic_interaction_object_confidence_duration()) {
        } else {
          discretized_object.enable_optimized = false;
        }
      }

      if (object->long_od_tags_seq.at(i) == LongitudinalOdTag::IGNORE
          && object->lat_od_tags_seq.at(i) == LateralOdTag::IGNORE) {
        discretized_object.enable_optimized = false;
      }

      // 坐标转换
      gpal::pnc::SLPoint temp_sl;
      target_ref_line_->xy2sl(math::Vec3d(obs_x, obs_y, 0.0), &temp_sl);
      discretized_object.local_s = temp_sl.s();
      discretized_object.l = temp_sl.l();

      // 更新速度
      if (!objects.empty()) {
        discretized_object.v = (discretized_object.local_s - objects.back().local_s) / 0.1;
      }
      objects.emplace_back(discretized_object);
    }

    if( objects.size() > 1){
      objects[0].v = objects[1].v;
    }

    //检查速度计算有效性，并基于中心差分进行修正
    float reasonable_delta_v = 1.0;
    for (int i = 1; i < objects.size() - 1; ++i) {
      if(std::abs(objects[i].v - objects[i-1].v) > reasonable_delta_v){
        objects[i].v  = (objects[i+1].v + objects[i-1].v) / 2;
      }
      objects[i].acc = (objects[i].v - objects[i-1].v) / 0.1;
    }

    // 处理预测的SL边界
    const auto& pred_sl_bounds = object->pred_sl_bounds;
    // todo: 解决pred_sl_bounds 与raw_predictions.front().traj.size() 不一致问题
    if (pred_sl_bounds.empty()) {
      for (int i = 0; i < objects.size(); ++i) {
        objects[i].start_s = object->cur_sl_bound.start_s();
        objects[i].end_s = object->cur_sl_bound.end_s();
        objects[i].start_l = object->cur_sl_bound.start_l();
        objects[i].end_l = object->cur_sl_bound.end_l();
      }
    } else {
      for (int i = 0; i < objects.size(); ++i) {
        if (i < pred_sl_bounds.size()) {
          objects[i].start_s = pred_sl_bounds[i].start_s();
          objects[i].end_s = pred_sl_bounds[i].end_s();
          objects[i].start_l = pred_sl_bounds[i].start_l();
          objects[i].end_l = pred_sl_bounds[i].end_l();
        } else {
          // 使用最后一个有效的边界信息
          objects[i].start_s = pred_sl_bounds.back().start_s();
          objects[i].end_s = pred_sl_bounds.back().end_s();
          objects[i].start_l = pred_sl_bounds.back().start_l();
          objects[i].end_l = pred_sl_bounds.back().end_l();
        }
      }
    }

    decision_objects.target_objects_info.emplace_back(objects);
  }

  // if (decision_result != nullptr) {
  //   auto* mutable_decision_result = const_cast<DecisionResult*>(decision_result);
  //   auto decision_object_info = mutable_decision_result->getDecisionObjectInfo();
  //   decision_objects.target_objects_info = {decision_object_info};
  // }

  return true;
}

REGIST_MODULE(DecisionObjectsParser)
}  // namespace gpal::pnc::planning
