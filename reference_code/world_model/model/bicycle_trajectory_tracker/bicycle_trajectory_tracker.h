#pragma once

#include "bicycle_trajectory_tracker_constraints.h"
#include "bicycle_trajectory_tracker_cost.h"
#include "bicycle_trajectory_tracker_dynamics.h"
#include "bicycle_trajectory_tracker_ipm_evaluator.h"
#include "ocp/ocp_model.h"

namespace gpal::pnc::planning {

class BicycleTrajectoryTracker : public OptimalControlProblem {
 public:
  BicycleTrajectoryTracker();
  virtual ~BicycleTrajectoryTracker() = default;
  virtual std::shared_ptr<Dynamics> createDynamics(OcpConfig::IntegratorType type) const override;
  virtual std::shared_ptr<CostFunction> createCostFunction() const override;
  virtual bool hasConstraints() const override { return true; }
  virtual std::shared_ptr<Constraints> createConstraint() const override;
  virtual std::shared_ptr<Constraints> createInitialConstraint() const override;
  virtual std::shared_ptr<Constraints> createTerminalConstraint() const override;
  virtual std::shared_ptr<Constraints> createStateOnlyEqualities() const override;
  virtual std::shared_ptr<Constraints> createInitialStateOnlyEqualities() const override;
  virtual std::shared_ptr<Constraints> createTerminalEqualities() const override;
  virtual std::shared_ptr<IPMHelper> createBaseIpmHelper() const override;
};

DECLARE_MODEL(BicycleTrajectoryTracker)
}  // namespace gpal::pnc::planning