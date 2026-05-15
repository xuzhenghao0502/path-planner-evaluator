#pragma once

#include "ocp/ocp_model.h"

#include "parking_general_dynamics.h"
#include "parking_general_cost.h"
#include "parking_general_constraints.h"
#include "parking_general_ipm_evaluator.h"

namespace gpal::pnc::planning {

class ParkingGeneral : public OptimalControlProblem {
 public:
  ParkingGeneral();
  virtual ~ParkingGeneral() = default;
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

DECLARE_MODEL(ParkingGeneral)
}  // namespace gpal::pnc::planning