#pragma once

#include "lateral_multi_gear_constraints.h"
#include "lateral_multi_gear_cost.h"
#include "lateral_multi_gear_dynamics.h"
#include "lateral_multi_gear_ipm_evaluator.h"
#include "ocp/ocp_model.h"

namespace gpal::pnc::planning {

class LateralMultiGear : public OptimalControlProblem {
 public:
  LateralMultiGear();
  virtual ~LateralMultiGear() = default;
  virtual std::shared_ptr<Dynamics> createDynamics(OcpConfig::IntegratorType type) const override;
  virtual std::shared_ptr<CostFunction> createCostFunction() const override;
  virtual std::shared_ptr<CostFunction> createTerminalCostFunction() const override;
  virtual bool hasConstraints() const override { return true; }
  virtual std::shared_ptr<Constraints> createConstraint() const override;
  virtual std::shared_ptr<Constraints> createInitialConstraint() const override;
  virtual std::shared_ptr<Constraints> createTerminalConstraint() const override;
  virtual std::shared_ptr<Constraints> createStateOnlyEqualities() const override;
  virtual std::shared_ptr<Constraints> createInitialStateOnlyEqualities() const override;
  virtual std::shared_ptr<Constraints> createTerminalEqualities() const override;
  virtual std::shared_ptr<IPMHelper> createBaseIpmHelper() const override;
};

DECLARE_MODEL(LateralMultiGear)
}  // namespace gpal::pnc::planning