#include "center_lines/lane_marking_process.h"

#include <algorithm>
#include <limits>

#include "base/log.h"

namespace gpal::pnc::planning {
using namespace std;
LaneMarkingProcess::LaneMarkingProcess() {
  optimal_values_ = {};
  variances_.assign(200, 0.0);
}

std::vector<math::Vec3d> LaneMarkingProcess::Get() { return optimal_values_; }

void LaneMarkingProcess::Clear() { return optimal_values_.clear(); }

void LaneMarkingProcess::TrimStart() {
  double MinLaneTrailX = -11.0;
  double MaxLaneTrailY = 10.0;
  uint32_t trimmedPoints = 0;
  for (size_t index = 0; index < optimal_values_.size(); ++index) {
    double xValue = optimal_values_[index].x();
    double yValue = optimal_values_[index].y();
    if (xValue >= MinLaneTrailX && fabs(yValue) <= MaxLaneTrailY) {
      break;
    } else {
      ++trimmedPoints;
    }
  }
  // GPAL_INFO("trimmedPoints = %D", trimmedPoints);
  auto it = optimal_values_.begin();
  optimal_values_.erase(it, std::next(it, trimmedPoints));
}

void LaneMarkingProcess::Update(const std::vector<math::Vec3d> &lane_marking_buffer, const double speed,
                                const bool is_curve, const double max_x_updatable) {
  if (lane_marking_buffer.empty()) {  // 目前7可进行标定
    Clear();
    return;  // 如果trail为空的话，那么直接清除所有滤波点并返回
  }
  if (!optimal_values_.empty()) {
    int8_t size_delta = optimal_values_.size() - lane_marking_buffer.size();  // 待标定
    bool is_return = (size_delta > 0) && (size_delta < 4);
    bool is_trail_change = (size_delta >= 4);
    bool is_lane_change = (fabs(lane_marking_buffer.front().y() - optimal_values_.front().y()) > 1.5) ||
                          (fabs(lane_marking_buffer.back().y() - optimal_values_.back().y()) > 2.0);

    // GPAL_INFO("size_delta = %d, is_return = %d, is_trail_change = %d, is_lane_change = %d", size_delta, is_return,
    // is_trail_change, is_lane_change);

    if (is_lane_change || is_trail_change) {
      Clear();  // 如果trail和coordinates_都不为0，且trail和coordinates_首y元素的绝对值相差1.5，则认为是换道，那么直接清除所有滤波点，但不返回
    } else if (is_return) {
      return;
    }
  }
  // 计算三次样条斜率
  std::vector<double> slopes = CubicSplineInterpolation::CalcCubicSplineSlopes(lane_marking_buffer);
  // 设置process_error_variance_查找表
  std::array<double, 2> spd_x = {0, 20};
  std::array<double, 2> spd_y = {1, 50};
  double process_error = SearchIndex(spd_x, spd_y, speed);
  double threshold = speed * 0.4f;
  std::array<double, 2> x_table = {threshold, 50.0f};
  std::array<double, 2> y_table = {0, process_error * 0.5f};
  if (optimal_values_.empty()) {
    FillPriorBasedOnMarker(x_table, y_table, lane_marking_buffer, slopes, max_x_updatable);
  } else {
    FillPriorBasedOnOldData(x_table, y_table, lane_marking_buffer, is_curve, max_x_updatable);
  }
  // 根据是否是曲线和速度设置测量误差方差
  double measurement_error = 5.0;
  // 遍历 optimal_values_ 双端队列
  for (size_t index = 0; index < optimal_values_.size(); ++index) {
    // 将点的 x 和 y 坐标分别添加到对应的向量中
    double xValue = optimal_values_[index].x();
    double yValue = optimal_values_[index].y();
    double variance = variances_[index];
    double yValueNew = CubicSplineInterpolation::EvaluateCubicSpline(lane_marking_buffer, xValue, slopes);
    double kalmanGain;
    kalmanGain = variance / (variance + measurement_error);
    yValue += kalmanGain * (yValueNew - yValue);
    optimal_values_[index].set_x(xValue);
    optimal_values_[index].set_y(yValue);
    //  = {xValue, yValue};
    variances_[index] = (1.0f - kalmanGain) * variance;
    // GPAL_INFO("index = %d, kalmanGain = %f", index, kalmanGain);
  }
}

double LaneMarkingProcess::SearchIndex(const std::array<double, 2> &x_table, const std::array<double, 2> &y_table,
                                       const double x) {
  double x_in;
  x_in = std::clamp(x, x_table[0], x_table[1]);
  // 计算线性插值
  double y;
  if (x_in == x_table[0]) {
    y = y_table[0];
  } else if (x_in == x_table[1]) {
    y = y_table[1];
  } else {
    // 使用线性插值公式
    y = y_table[0] + ((x - x_table[0]) * (y_table[1] - y_table[0]) / (x_table[1] - x_table[0]));
  }
  return y;
}

void LaneMarkingProcess::Transform(const double veh_spd, const double yaw_rate) {
  ERT_PLOG_I << "be in left ";
  double delta_x = veh_spd * 0.1 * std::cos(yaw_rate * 0.1 * 0.5);
  double delta_y = veh_spd * 0.1 * std::sin(yaw_rate * 0.1 * 0.5);
  double delta_thet = yaw_rate * 0.1;
  ERT_PLOG_I << "delta_x" << delta_x << "delta_y" << delta_y << "delta_thet" << delta_thet;
  double sin_thet = std::sin(delta_thet);
  double cos_thet = std::cos(delta_thet);
  for (size_t index = 0; index < optimal_values_.size(); ++index) {
    // 将点的 x 和 y 坐标分别添加到对应的向量中
    double x = optimal_values_[index].x();
    double y = optimal_values_[index].y();
    double xt = x * cos_thet + y * sin_thet - delta_x;
    double yt = -x * sin_thet + y * cos_thet - delta_y;
    math::Vec3d point;
    point.set_x(xt);
    point.set_y(yt);
    point.set_z(optimal_values_[index].z());
    optimal_values_[index] = point;
  }
}

void LaneMarkingProcess::FillPriorBasedOnMarker(const std::array<double, 2> &x_table,
                                                const std::array<double, 2> &y_table,
                                                const std::vector<math::Vec3d> &lane_marking_buffer,
                                                const std::vector<double> slopes, const double max_x_updatable) {
  double offset;
  double MinLaneTrailX = lane_marking_buffer.front().x();
  double MinSeparation = 1.0;
  for (size_t index = 0; index < lane_marking_buffer.size(); ++index) {
    double xValue = MinLaneTrailX + index * MinSeparation;
    if (xValue < max_x_updatable) {
      double yValue;
      yValue = CubicSplineInterpolation::EvaluateCubicSpline(lane_marking_buffer, xValue, slopes);
      math::Vec3d point;
      point.set_x(xValue);
      point.set_y(yValue);
      point.set_z(lane_marking_buffer.at(index).z());
      optimal_values_.emplace_back(point);
      double variance = SearchIndex(x_table, y_table, xValue);
      variances_[index] = variance;  // 假设getActual返回当前元素的索引
    }
  }
}

void LaneMarkingProcess::FillPriorBasedOnOldData(const std::array<double, 2> &x_table,
                                                 const std::array<double, 2> &y_table,
                                                 const std::vector<math::Vec3d> &lane_marking_buffer,
                                                 const bool is_curve, const double max_x_updatable) {
  double deltaX = 1.0;
  uint32_t point_num = optimal_values_.size();
  uint32_t capacity = lane_marking_buffer.size();  // 后续可以做成标定量，表示点数
  uint32_t pre_index = 0;
  double xValue = 0.0;
  double yValue = 0.0;
  for (size_t index = point_num; index < capacity; ++index) {
    xValue = optimal_values_[index - 1].x();
    yValue = optimal_values_[index - 1].y();
    xValue = xValue + deltaX;
    // 仅在snail trail的末端扩展过滤器长度，因为超出的标记可能是不可见的
    if (xValue <= max_x_updatable) {
      if ((point_num) >= 3 && is_curve) {
        // 使用二阶导数计算yValue
        const math::Vec3d &k_minus_1val = optimal_values_[index - 1];
        const math::Vec3d &k_minus_2val = optimal_values_[index - 2];
        const math::Vec3d &k_minus_3val = optimal_values_[index - 3];
        yValue = 3 * k_minus_1val.y() - 3 * k_minus_2val.y() + k_minus_3val.y();
      } else {
        // 使用平均heading计算yValue
        yValue += CubicSplineInterpolation::GetAveragedHeading(optimal_values_, (optimal_values_.size())) * deltaX;
      }
      // 添加新的坐标点
      math::Vec3d point;
      point.set_x(xValue);
      point.set_y(yValue);
      point.set_z(optimal_values_[index - 1].z());
      optimal_values_.emplace_back(point);
      // 新点获得更高的方差以实现圆锥形
      // float errorVarianceY[] = {0, 0.5};
      double variance = SearchIndex(x_table, y_table, xValue);
      variances_[index] = variances_[pre_index] + std::max(10.0 * std::min(variance, 0.25), 0.3);
    }
    pre_index = index;
  }
}

}  // namespace gpal::pnc::planning