#pragma once

#include "ocp/ocp_model.h"

#include "multi_agents_trajectory_model_dynamics.h"
#include "multi_agents_trajectory_model_cost.h"
#include "multi_agents_trajectory_model_constraints.h"
#include "multi_agents_trajectory_model_ipm_evaluator.h"

namespace gpal::pnc::planning {

class MultiAgentsTrajectoryModel : public OptimalControlProblem {
 public:
  MultiAgentsTrajectoryModel();
  virtual ~MultiAgentsTrajectoryModel() = default;
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

DECLARE_MODEL(MultiAgentsTrajectoryModel)
}  // namespace gpal::pnc::planning