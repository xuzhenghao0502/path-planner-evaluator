#pragma once
#include "basic_algorithm_lib/basic_algorithm_lib.h"
#include "gpal-interface/planning/decision_result.pb.h"

namespace gpal::pnc::planning {

class DecisionResultProto : public StampedBase {
 public:
  friend class DecisionResultProtoAdapter;
  DecisionResultProto();
  ~DecisionResultProto() = default;

  void SetValidity(const bool is_valid) { is_valid_ = is_valid; }
  bool IsValid() const { return is_valid_; }

  void SetTimestamp(const double timestamp) { timestamp_ = timestamp; }
  double GetTimestamp() const { return timestamp_; }

  std::shared_ptr<proto::DecisionResult> GetMutableProtoDecisionResultPtr() { return proto_decision_result_; }
  const std::shared_ptr<proto::DecisionResult>& GetProtoDecisionResultPtr() const { return proto_decision_result_; }

 private:
  bool is_valid_ = false;
  double timestamp_ = 0.0;
  std::shared_ptr<proto::DecisionResult> proto_decision_result_ = nullptr;
};

}  // namespace gpal::pnc::planning
