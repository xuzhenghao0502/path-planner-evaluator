#include "empirical_speed_planner/empirical_speed_planning.h"

namespace gpal::pnc::planning {

void EmpiricalSpeedPlan::getSpeedLimit(const LocalView& local_view, const DiscretizedPath& local_path,  vector<proto::TrajectoryPoint>& path_v_t) {

  proto::TrajectoryPoint Tp;
  path_v_t.resize(local_path.size());
  Tp.mutable_path_point()->set_x(local_path.back().x());
  Tp.mutable_path_point()->set_y(local_path.back().y());
  Tp.mutable_path_point()->set_z(local_path.back().z());
  Tp.mutable_path_point()->set_kappa(local_path.back().kappa());
  Tp.mutable_path_point()->set_s(local_path.back().s());
  Tp.set_v(fmin(map_speed_, (float)sqrt(abs(parameter_.lateral_a_limit() / Tp.path_point().kappa()))));
  Tp.set_relative_time(0);
  path_v_t.at(local_path.size() - 1).CopyFrom(Tp);
  for (int i = local_path.size() - 2; i >= 0; i--) {
    Tp.mutable_path_point()->set_x(local_path.at(i).x());
    Tp.mutable_path_point()->set_y(local_path.at(i).y());
    Tp.mutable_path_point()->set_z(local_path.at(i).z());
    Tp.mutable_path_point()->set_kappa(local_path.at(i).kappa());
    Tp.mutable_path_point()->set_s(local_path.at(i).s());
    float kappa_speed_limit = sqrt(abs(parameter_.lateral_a_limit() / Tp.path_point().kappa()));
    float acc_speed_limit =
        sqrt(abs(2 * parameter_.a_hard_upper_bound() * (path_v_t.at(i + 1).path_point().s() - Tp.path_point().s()) +
                 path_v_t.at(i + 1).v() * path_v_t.at(i + 1).v()));
    Tp.set_v(fmin(map_speed_, min(kappa_speed_limit, acc_speed_limit)));
    path_v_t.at(i).CopyFrom(Tp);
  }

  if (path_v_t.at(0).v() > local_view.getChassisPtr()->Speed()) {
    path_v_t.at(0).set_v(local_view.getChassisPtr()->Speed());
    for (size_t i = 1; i < path_v_t.size() - 1; i++) {
      double acc = parameter_.a_hard_upper_bound() * (path_v_t.at(i).v() - path_v_t.at(i - 1).v());
      if (acc < 0) {
        acc = 0;
      } else if (acc > parameter_.a_hard_upper_bound()) {
        acc = parameter_.a_hard_upper_bound();
      }
      double v_i = sqrt(path_v_t.at(i - 1).v() * path_v_t.at(i - 1).v() +
                        2 * acc * (path_v_t.at(i).path_point().s() - path_v_t.at(i - 1).path_point().s()));
      if (v_i < path_v_t.at(i).v()) {
        path_v_t.at(i).set_v(v_i);
      } else {
        break;
      }
    }
  } else if (path_v_t.at(0).v() < local_view.getChassisPtr()->Speed()) {
    path_v_t.at(0).set_v(fmin(map_speed_, local_view.getChassisPtr()->Speed()));
    for (size_t i = 1; i < path_v_t.size() - 1; i++) {
      double acc = parameter_.dacc_coefficient() * (path_v_t.at(i).v() - path_v_t.at(i - 1).v());
      if (acc > 0) {
        acc = 0;
      } else if (acc < parameter_.a_hard_lower_bound()) {
        acc = parameter_.a_hard_lower_bound();
      }
      double v_i = sqrt(path_v_t.at(i - 1).v() * path_v_t.at(i - 1).v() +
                        2 * acc * (path_v_t.at(i).path_point().s() - path_v_t.at(i - 1).path_point().s()));
      if (v_i > path_v_t.at(i).v()) {
        path_v_t.at(i).set_v(fmin(map_speed_, v_i));
      } else {
        break;
      }
    }
  }
  for (size_t i = 1; i < path_v_t.size(); i++) {
    path_v_t.at(i).set_relative_time(path_v_t.at(i - 1).relative_time() +
                                     abs(2 * (path_v_t.at(i).path_point().s() - path_v_t.at(i - 1).path_point().s()) /
                                         (path_v_t.at(i).v() + path_v_t.at(i - 1).v())));
  }
}

void EmpiricalSpeedPlan::calcEmpiricalSpeedPlanningResult(const LocalView& local_view, const DiscretizedPath& local_path, std::shared_ptr<SpeedResult> speed_result, const double& front_obj_s0,
                                                          const double& front_obj_v) {
  double t = 0.0;
  double s = init_s_;
  double v = init_v_;
  double a = init_a_;
  auto mutable_speed_data = speed_result->mutable_debug_speed_data();
  if (mutable_speed_data->size() != speed_size_) {
    mutable_speed_data->resize(speed_size_);
  }
  // if (last_frame_speed_data_->size() != speed_size_) {
  //   last_frame_speed_data_->resize(speed_size_);
  // }
  PINFO << "[IDM] front_obj_s0 = " << front_obj_s0 << ", front_obj_v = " << front_obj_v;
  mutable_speed_data->at(0).set_t(t);
  mutable_speed_data->at(0).set_s(s);
  mutable_speed_data->at(0).set_v(v);
  mutable_speed_data->at(0).set_a(a);
  // Record our result as history data
  // last_frame_speed_data_->at(0).set_t(t);
  // last_frame_speed_data_->at(0).set_s(s);
  // last_frame_speed_data_->at(0).set_v(v);
  // last_frame_speed_data_->at(0).set_a(a);

  double speed_data_dt_sq = speed_dt_ * speed_dt_;
  double speed_data_dt_cu = speed_data_dt_sq * speed_dt_;

  std::vector<proto::TrajectoryPoint> path_v_t;
  getSpeedLimit(local_view,local_path , path_v_t);
  size_t reserve_index = 0;
  double current_speed_limit;
  double k_1, k_2 = 0.3, k_3 = 10.0, k_4 = 1.5;
  double distance, delta_v, ideal_distance, delta_dist;
  double empirical_a, empirical_u;
  double min_speed_limit = 0.0 * KMH_MS;
  // SFIELD(IDM, "===== idm result =====");
  for (size_t i = 1UL; i < mutable_speed_data->size(); i++) {
    distance = front_obj_s0 + front_obj_v * t - s;
    delta_v = v - front_obj_v;
    ideal_distance = safe_distance_ + (K_0_ + gamma_ * fmax(0.0, delta_v)) * v;
    delta_dist = distance - ideal_distance;

    k_1 = (delta_v >= 0.5) ? gamma_ : 0.0;
    empirical_a = calcEmpiricalAcc(v, delta_dist, delta_v, 0.0, K_0_, k_1, k_2);
    // Consider road speed-limit
    for (; reserve_index < path_v_t.size(); ++reserve_index) {
      if (path_v_t[reserve_index].path_point().s() >= s) {
        break;
      }
    }
    if (reserve_index == 0) {
      current_speed_limit = fmax(path_v_t[0].v(), min_speed_limit);
    } else if (reserve_index == path_v_t.size()) {
      current_speed_limit = fmax(path_v_t.back().v(), min_speed_limit);
    } else {
      current_speed_limit =
          fmax(math::lerp(path_v_t[reserve_index - 1].v(), path_v_t[reserve_index - 1].path_point().s(),
                                  path_v_t[reserve_index].v(), path_v_t[reserve_index].path_point().s(), s),
               min_speed_limit);
    }
    empirical_a = fmin(empirical_a, -k_3 * (v - current_speed_limit));
    empirical_a = fmin(fmax(empirical_a, parameter_.a_hard_lower_bound()), parameter_.a_hard_upper_bound());

    empirical_u = -k_4 * (a - empirical_a);
    empirical_u = fmax(fmin(empirical_u, parameter_.j_hard_upper_bound()), parameter_.j_hard_lower_bound());

    t = static_cast<double>(i) * speed_dt_;
    s = s + v * speed_dt_ + 0.5 * a * speed_data_dt_sq + (1.0 / 6.0) * empirical_u * speed_data_dt_cu;
    v = v + a * speed_dt_ + 0.5 * empirical_u * speed_data_dt_sq;
    a = a + empirical_u * speed_dt_;

    // SFIELD(IDM, "i = {}, s = {}, v = {}, a = {}", i, s, v, a);
    // SFIELD(IDM, "delta_dis = {}, delta_v = {}, map_limit = {}", delta_dist, delta_v, current_speed_limit);
    mutable_speed_data->at(i).set_t(t);
    mutable_speed_data->at(i).set_s(s);
    mutable_speed_data->at(i).set_v(v);
    mutable_speed_data->at(i).set_a(a);
    // Record our result as history data
    // last_frame_speed_data_->at(i).set_t(t);
    // last_frame_speed_data_->at(i).set_s(s);
    // last_frame_speed_data_->at(i).set_v(v);
    // last_frame_speed_data_->at(i).set_a(a);
  }
}

double EmpiricalSpeedPlan::calcEmpiricalAcc(const double& ego_v, const double& delta_dist, const double& delta_v,
                                            const double& front_a, const double& k_0, const double& k_1,
                                            const double& k_2) {
  return (k_2 * delta_dist - delta_v + k_1 * front_a * ego_v) / (k_0 + k_1 * (delta_v + ego_v));
}

}  // namespace uto::planning