#pragma once
#include <thread>

template <size_t N = 1>
class TaskHandler {
 public:
  TaskHandler() = default;
  ~TaskHandler() {
    if (thread_.joinable()) thread_.join();
  }

  template <typename F>
  void importTask(F&& task) {
    if (thread_.joinable()) thread_.join();
    thread_ = std::thread(std::forward<F>(task));
  }

 private:
  std::thread thread_;
};
