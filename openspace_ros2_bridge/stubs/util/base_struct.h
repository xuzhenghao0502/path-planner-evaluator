#pragma once
#include <cstdint>
#include <chrono>
#include <string>

namespace util {

template <typename Duration = std::milli>
class TimerLogger {
 public:
  template <typename Callback>
  TimerLogger(const std::string& name, Callback cb) {
    (void)name;
    (void)cb;
  }

  TimerLogger tap(const std::string& name) {
    (void)name;
    return TimerLogger(name, [](const std::string&) {});
  }
};

}  // namespace util
