#pragma once

/**
 * @file
 * @brief Defines the ParkingLot class.
 */

#include "basic_algorithm_lib/basic_algorithm_lib.h"

namespace gpal::pnc::planning {
/**
 * @class ParkingLot
 *
 * @brief Implements a class of ParkingLot Data.
 */

using namespace std;

struct Space {
  uint8_t nValid;
  uint8_t nType;
  uint8_t nSource;
  uint8_t nQuality;
  std::string id;
  std::pair<bool, float> direction;  // 库位方向(ego系)，指向停靠完成时自车车头方向，默认false不启用
  float gfPos[4][3];

  Space() {
    nValid = 0;
    nType = 0;
    nSource = 0;
    nQuality = 0;
    id = "";
    direction = std::make_pair(false, 0.0);
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
        gfPos[i][j] = 0.0;
      }
    }
  }
  void Clear() {
    nValid = 0;
    nType = 0;
    nSource = 0;
    nQuality = 0;
    id = "";
    direction = std::make_pair(false, 0.0);
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
        gfPos[i][j] = 0.0;
      }
    }
  }
};

struct ParkingLotData {
  uint64_t nImageTimeStamp;
  uint8_t apaParklotTypeAndDir;
  Space gstSpace[10];

  ParkingLotData() {
    nImageTimeStamp = 0;
    apaParklotTypeAndDir = 0;
    for (auto& space : gstSpace) {
      space.Clear();
    }
  }
  void Clear() {
    nImageTimeStamp = 0;
    apaParklotTypeAndDir = 0;
    for (auto& space : gstSpace) {
      space.Clear();
    }
  }
};

class ParkingLot : public StampedBase {
 public:
  friend class PerceptionParkingLotAdapter;
  explicit ParkingLot();
  ~ParkingLot();

  ParkingLotData data_;

  void Clear();

 protected:
};

}  // namespace gpal::pnc::planning
