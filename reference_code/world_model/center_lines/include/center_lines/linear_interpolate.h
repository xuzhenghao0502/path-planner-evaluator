#ifndef _COMMON_LINEARINTERPOLATION_H_
#define _COMMON_LINEARINTERPOLATION_H_
#include <cmath>
#include <memory>
#include <vector>

namespace LinearInterpolation {

double CalculateHeading(std::vector<double> &x, std::vector<double> &y, uint32_t n, uint32_t point);
std::vector<double> LinearInterpolation(std::vector<double> &x, std::vector<double> &y, const uint32_t &n,
                                        const std::vector<double> &x_sampled, const uint32_t &n_sampled);

}  // namespace LinearInterpolation

#endif  // _COMMON_LINEARINTERPOLATION_H_