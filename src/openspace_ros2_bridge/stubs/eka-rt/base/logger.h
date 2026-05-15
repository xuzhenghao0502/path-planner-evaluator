#pragma once
#include <iostream>
#include <sstream>

#define ERT_LOG_I(module, ...) \
  do { std::cout << "[INFO]" << module; _ert_print_args(__VA_ARGS__); std::cout << std::endl; } while(0)
#ifdef OPENSPACE_STANDALONE
#define ERT_LOG_D(module, ...) do {} while(0)
#else
#define ERT_LOG_D(module, ...) \
  do { std::cout << "[DEBUG]" << module; _ert_print_args(__VA_ARGS__); std::cout << std::endl; } while(0)
#endif
#define ERT_LOG_W(module, ...) \
  do { std::cout << "[WARN]" << module; _ert_print_args(__VA_ARGS__); std::cout << std::endl; } while(0)
#define ERT_LOG_E(module, ...) \
  do { std::cerr << "[ERROR]" << module; _ert_print_args(__VA_ARGS__); std::cerr << std::endl; } while(0)

#define RAD2ANG (180.0 / M_PI)
#define ANG2RAD (M_PI / 180.0)

template <typename T>
inline void _ert_print_one(std::ostream& os, const T& v) { os << " " << v; }

template <typename T, typename... Args>
inline void _ert_print_args(const T& first, const Args&... rest) {
  std::cout << " " << first;
  if constexpr (sizeof...(rest) > 0) _ert_print_args(rest...);
}
inline void _ert_print_args() {}
