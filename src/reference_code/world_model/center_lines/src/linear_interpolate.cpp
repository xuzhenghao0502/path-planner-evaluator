#include "center_lines/linear_interpolate.h"
namespace LinearInterpolation {

double CalculateHeading(std::vector<double> &x, std::vector<double> &y, uint32_t n, uint32_t point) {
  if (n < 2) {
    // 如果点的数量少于2，返回0 作为 heading
    return 0.0;
  } else {
    double denominator = 0.0;
    double nominator = 0.0;
    // 确保我们不会访问数组的越界索引
    uint32_t twoInt = 2;
    uint32_t startIndex = std::max(twoInt, point - twoInt);
    uint32_t endIndex = std::min(n, point + twoInt);
    for (uint32_t k = startIndex - 1; k < endIndex; ++k) {
      if (k == 0 || k >= n) break;  // 防止数组越界
      double dx = x[k] - x[k - 1];
      double dy = y[k] - y[k - 1];
      if (dx != 0.0) {  // 避免除以0
        nominator += dy / dx;
      }
      denominator += 1.0;
    }
    // 如果denominator为0，返回0避免未定义的atan2行为
    if (denominator == 0) {
      return 0.0;
    }
    return std::atan2(nominator, denominator);
  }
}

std::vector<double> LinearInterpolation(std::vector<double> &x, std::vector<double> &y, const uint32_t &n,
                                        const std::vector<double> &x_sampled, const uint32_t &n_sampled) {
  if (n == 0) {
    return {};  // throw std::invalid_argument("Number of input points (n) must be greater than 0.");
  }
  // 初始化输出数组，用 0 填充
  std::vector<double> ySampled(n_sampled, 0.0);
  uint32_t startSearch = 0;
  for (uint32_t i = 0; i < n_sampled; ++i) {
    // 检查 x_sampled 是否在 x 的范围内
    double k = 0.0;
    if (x_sampled[i] <= x[0]) {
      // 线性外推
      k = CalculateHeading(x, y, n, 1);
      ySampled[i] = y[0] + std::tan(k) * (x_sampled[i] - x[0]);
    } else if (x_sampled[i] >= x[n - 1]) {
      // 线性外推
      k = CalculateHeading(x, y, n, n);
      ySampled[i] = y[n - 1] + std::tan(k) * (x_sampled[i] - x[n - 1]);
    } else {
      // 线性插值
      while (x[startSearch + 1] < x_sampled[i] && (startSearch < n - 1)) {
        ++startSearch;
      }
      float t = (x_sampled[i] - x[startSearch]) / (x[startSearch + 1] - x[startSearch]);
      ySampled[i] = (1 - t) * y[startSearch] + t * y[startSearch + 1];
    }
  }
  // 清零剩余的数组元素
  return ySampled;
}

}  // namespace LinearInterpolation
