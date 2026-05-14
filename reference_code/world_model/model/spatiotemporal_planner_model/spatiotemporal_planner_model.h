#pragma once

#include "ocp/ocp_model.h"

#include "spatiotemporal_planner_model_dynamics.h"
#include "spatiotemporal_planner_model_cost.h"
#include "spatiotemporal_planner_model_constraints.h"
#include "spatiotemporal_planner_model_ipm_evaluator.h"

namespace gpal::pnc::planning {

class SpatiotemporalPlannerModel : public OptimalControlProblem {
 public:
  SpatiotemporalPlannerModel();
  virtual ~SpatiotemporalPlannerModel() = default;
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

DECLARE_MODEL(SpatiotemporalPlannerModel)
}  // namespace gpal::pnc::planning