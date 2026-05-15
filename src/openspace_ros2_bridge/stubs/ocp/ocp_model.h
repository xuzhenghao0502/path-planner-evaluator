#pragma once
#include <Eigen/Core>
#include <cstdint>

namespace ocp {

template <int StateDim, int ControlDim, int ConstraintDim>
class OcpModel {
 public:
  using StateVec = Eigen::Matrix<double, StateDim, 1>;
  using ControlVec = Eigen::Matrix<double, ControlDim, 1>;
  OcpModel() = default;
};

}  // namespace ocp
