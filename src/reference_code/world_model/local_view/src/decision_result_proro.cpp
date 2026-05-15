#include "local_view/decision_result_proto.h"

namespace gpal::pnc::planning {

DecisionResultProto::DecisionResultProto() { proto_decision_result_ = std::make_shared<proto::DecisionResult>(); }

}  // namespace gpal::pnc::planning