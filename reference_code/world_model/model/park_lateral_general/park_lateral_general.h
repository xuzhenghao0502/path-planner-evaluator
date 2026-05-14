#pragma once

#include "ocp/ocp_model.h"
#include "park_lateral_general_constraints.h"
#include "park_lateral_general_cost.h"
#include "park_lateral_general_dynamics.h"
#include "park_lateral_general_ipm_evaluator.h"

namespace gpal::pnc::planning {

class ParkLateralGeneral : public OptimalControlProblem {
 public:
  ParkLateralGeneral();
  virtual ~ParkLateralGeneral() = default;
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

DECLARE_MODEL(ParkLateralGeneral)
}  // namespace gpal::pnc::planning