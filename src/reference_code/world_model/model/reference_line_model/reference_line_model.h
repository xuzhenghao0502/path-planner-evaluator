#pragma once

#include "ocp/ocp_model.h"
#include "reference_line_model_constraints.h"
#include "reference_line_model_cost.h"
#include "reference_line_model_dynamics.h"
#include "reference_line_model_ipm_evaluator.h"

namespace gpal::pnc::planning {

class ReferenceLineModel : public OptimalControlProblem {
 public:
  ReferenceLineModel();
  virtual ~ReferenceLineModel() = default;
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

DECLARE_MODEL(ReferenceLineModel)
}  // namespace gpal::pnc::planning