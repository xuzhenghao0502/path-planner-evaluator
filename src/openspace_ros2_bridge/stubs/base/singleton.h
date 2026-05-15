#pragma once
#include <memory>
#include <mutex>

template <typename T>
class Singleton {
 public:
  static T* get_instance() {
    static T instance;
    return &instance;
  }

 protected:
  Singleton() = default;
  ~Singleton() = default;
  Singleton(const Singleton&) = delete;
  Singleton& operator=(const Singleton&) = delete;
};
