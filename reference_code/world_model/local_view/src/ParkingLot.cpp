#include "local_view/ParkingLot.h"

namespace gpal::pnc::planning {

ParkingLot::ParkingLot() { Clear(); }

ParkingLot::~ParkingLot() {}

void ParkingLot::Clear() { StampedBase::reset(); }

}  // namespace gpal::pnc::planning
