#pragma once

#include "gpal-interface/planning/task_response.pb.h"
#include "util/base_struct.h"

namespace gpal::pnc::planning {

class ScenarioOdd {
 public:
  ScenarioOdd();
  ~ScenarioOdd();

  void Clear();

  void setAccOdd(const proto::TaskResponse::AccOdd& acc_odd) { acc_odd_ = acc_odd; }
  proto::TaskResponse::AccOdd accOdd() const { return acc_odd_; }
  proto::TaskResponse::AccOdd* mutableAccOdd() { return &acc_odd_; }

  void setLccOdd(const proto::TaskResponse::LccOdd& lcc_odd) { lcc_odd_ = lcc_odd; }
  proto::TaskResponse::LccOdd lccOdd() const { return lcc_odd_; }
  proto::TaskResponse::LccOdd* mutableLccOdd() { return &lcc_odd_; }

  void setNoaOdd(const proto::TaskResponse::NoaOdd& noa_odd) { noa_odd_ = noa_odd; }
  proto::TaskResponse::NoaOdd noaOdd() const { return noa_odd_; }
  proto::TaskResponse::NoaOdd* mutableNoaOdd() { return &noa_odd_; }

  void setHdFunctionOdd(const proto::TaskResponse::HDFunctionOdd& hd_function_odd) {
    hd_function_odd_ = hd_function_odd;
  }
  proto::TaskResponse::HDFunctionOdd hdFunctionOdd() const { return hd_function_odd_; }
  proto::TaskResponse::HDFunctionOdd* mutableHdFunctionOdd() { return &hd_function_odd_; }

  void setTargetFunctionState(proto::TaskResponse::TargetFunctionState state) {
    target_function_state_ = state;
  }
  proto::TaskResponse::TargetFunctionState targetFunctionState() const { return target_function_state_; }

 protected:
  proto::TaskResponse::AccOdd acc_odd_;
  proto::TaskResponse::LccOdd lcc_odd_;
  proto::TaskResponse::NoaOdd noa_odd_;
  proto::TaskResponse::HDFunctionOdd hd_function_odd_;
  proto::TaskResponse::TargetFunctionState target_function_state_{proto::TaskResponse::kInvalidFunctionState};
};
}  // namespace gpal::pnc::planning