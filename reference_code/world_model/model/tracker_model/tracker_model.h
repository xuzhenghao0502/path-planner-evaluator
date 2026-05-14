#pragma once

#include "ocp/ocp_model.h"

#include "tracker_model_dynamics.h"
#include "tracker_model_cost.h"
#include "tracker_model_constraints.h"
#include "tracker_model_ipm_evaluator.h"

namespace gpal::pnc::planning {

class TrackerModel : public OptimalControlProblem {
 public:
  TrackerModel();
  virtual ~TrackerModel() = default;
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

DECLARE_MODEL(TrackerModel)
}  // namespace gpal::pnc::planning