#pragma once
namespace gpal::pnc::planning {

class StampedBase {
 public:
  StampedBase() = default;
  virtual ~StampedBase() = default;
  void reset() {}
  double timestamp() const { return timestamp_; }
  void set_timestamp(double t) { timestamp_ = t; }
 private:
  double timestamp_ = 0;
};

}  // namespace gpal::pnc::planning
