#pragma once

#include "math/interval_data.h"
#include "util/base_struct.h"
#include <array>
namespace gpal::pnc::planning {

struct GridsInfo {
  struct GridInfo {
    GridInfo() = default;
    GridInfo(double step, double horizon) : step(step), horizon(horizon) {
      for (double t = 0; t <= horizon + kMathEpsilon; t += step) {
        grid.push_back(t);
      }
    }
    double step = 0.0;           ///< 网格步长
    double horizon = 0.0;        ///< 网格时间范围
    std::vector<double> grid{};  ///< 网格点

    void reset() {
      step = 0.0;
      horizon = 0.0;
      grid.clear();
    }
  };

  GridInfo time_grid_info;
  GridInfo velocity_grid_info;

  /**
   * @brief 重置网格信息
   */
  void reset() {
    time_grid_info.reset();
    velocity_grid_info.reset();
    // 重置网格信息
    constexpr double time_step = 0.1;                    ///< 时间网格步长
    constexpr double time_horizon = 5.0;                 ///< 时间网格范围
    constexpr double velocity_step = 10.0 * KMH_MS;      ///< 速度网格步长
    constexpr double velocity_horizon = 130.0 * KMH_MS;  ///< 速度网格范围
    time_grid_info = GridInfo(time_step, time_horizon);
    velocity_grid_info = GridInfo(velocity_step, velocity_horizon);
  }
};

/**
 * @brief 通用约束模板基类
 * @tparam N 约束值的数量
 */
template <size_t N>
struct ConstraintBase {
  double s_ = 0;                      // 位置
  std::array<double, N> values_;  // 约束值数组

  // 默认构造函数
  ConstraintBase() : s_(0.0) { values_.fill(0.0); }

  // 单参数构造函数
  explicit ConstraintBase(double s_val) : s_(s_val) { values_.fill(0.0); }

  // 变参构造函数 - 参数个数是N
  template <typename... Args>
  ConstraintBase(double s_val, Args... args) : s_(s_val), values_{static_cast<double>(args)...} {
    static_assert(sizeof...(Args) == N, "Number of arguments must match template parameter N");
  }

  // 访问器
  double& operator[](size_t idx) { return values_[idx]; }
  const double& operator[](size_t idx) const { return values_[idx]; }
};

/**
 * @brief 边界约束
 */
struct Boundary : ConstraintBase<2> {
  // 使用 using 声明继承构造函数
  using ConstraintBase<2>::ConstraintBase;

  // 语义化访问器
  double& s() { return s_; }
  double& lower() { return values_[0]; }
  double& upper() { return values_[1]; }

  const double& s() const { return s_; }
  const double& lower() const { return values_[0]; }
  const double& upper() const { return values_[1]; }

  // 默认构造函数
  Boundary() : ConstraintBase<2>() {
    lower() = -std::numeric_limits<double>::max();
    upper() = std::numeric_limits<double>::max();
  }

  // 三参数构造函数
  Boundary(double s_val, double low, double up) : ConstraintBase<2>() {  // 先调用基类默认构造
    s() = s_val;
    lower() = low;
    upper() = up;
  }

  Boundary operator*(double t) const {
    Boundary result;
    result.s() = s() * t;
    result.lower() = lower() * t;
    result.upper() = upper() * t;
    return result;
  }

  Boundary operator+(const Boundary& other) const {
    Boundary result;
    result.s() = s() + other.s();
    result.lower() = lower() + other.lower();
    result.upper() = upper() + other.upper();
    return result;
  }

  // 约束检查
  bool isValid() const { return lower() <= upper(); }

  // 获取边界
  std::tuple<double, double, double> getBounds() const { return std::make_tuple(s(), lower(), upper()); }

  // 获取边界的可变引用,s不可变
  std::tuple<double&, double&> mutableBounds() { return std::tie(lower(), upper()); }

  // 边界裁剪
  void clip(double min_val, double max_val) {
    lower() = std::max(lower(), min_val);
    upper() = std::min(upper(), max_val);
  }
  // 只裁剪上界
  void clipUpper(double max_val) { upper() = std::min(upper(), max_val); }
  void clipUpper(double max_val, const std::string& type) {
    if (upper() > max_val) {
      upper() = max_val;
      setUpperType(type);
    }
  }

  // 只裁剪下界
  void clipLower(double min_val) { lower() = std::max(lower(), min_val); }
  void clipLower(double min_val, const std::string& type) {
    if (lower() < min_val) {
      lower() = min_val;
      setLowerType(type);
    }
  }

  // 检查是否包含某个值
  bool contains(double value) const { return value >= lower() && value <= upper(); }

  // 获取宽度
  double width() const { return upper() - lower(); }

  std::string upperType() const { return upper_type_; }
  void setUpperType(const std::string& type) { upper_type_ = type; }

  std::string lowerType() const { return lower_type_; }
  void setLowerType(const std::string& type) { lower_type_ = type; }

 private:
  std::string upper_type_ = "";
  std::string lower_type_ = "";
};

/**
 * @brief 创建插值函数
 */
template <typename T>
auto createInterpolator() {
  return [](const T& c1, const T& c2, double s) -> T {
    if (std::abs(c2.s_ - c1.s_) < 1e-10) {
      return c1;  // 避免除零
    }
    double t = (s - c1.s_) / (c2.s_ - c1.s_);
    T result = c1 * (1.0 - t) + c2 * t;
    result.s_ = s;  // 确保s值精确
    return result;
  };
}

// 1. 提供 Traits 的通用模板声明
template <typename T>
struct Traits;

// 2. 提供针对最基础类型 double 的特化实现
template <>
struct Traits<double> {
  static double indice(const double& value) { return value; }
};

/**
 * @brief 边界管理器的模板基类
 * @tparam Derived 派生类的类型
 */
template <typename T, typename Derived>
class BoundaryManagerBase {
 public:
  // 构造函数：接收输入数据并进行验证和初始化
  explicit BoundaryManagerBase(const std::vector<T>& indices) {
    if (!validate(indices)) {
      is_valid_ = false;
      // 在构造函数中抛出异常
      throw std::invalid_argument("Invalid indices: must have at least 2 points with equal spacing.");
    }
    // 调用派生类实现的特定初始化逻辑
    derived().initializeImpl(indices);
    is_valid_ = true;
  }

  // 默认析构函数
  BoundaryManagerBase() = default;
  ~BoundaryManagerBase() = default;

  // 禁用拷贝，只允许移动
  BoundaryManagerBase(const BoundaryManagerBase&) = delete;
  BoundaryManagerBase& operator=(const BoundaryManagerBase&) = delete;
  BoundaryManagerBase(BoundaryManagerBase&&) = default;
  BoundaryManagerBase& operator=(BoundaryManagerBase&&) = default;

  // 公共的 reset 方法
  void reset() {
    is_valid_ = false;
    size_ = 0;
    start_s_ = 0.0;
    delta_s_ = 0.0;
    // 调用派生类实现的特定重置逻辑
    derived().resetImpl();
  }

  // 公共的 update 方法
  void update(const std::vector<T>& indices) {
    if (validate(indices)) {
      // 调用派生类实现的特定初始化逻辑
      derived().initializeImpl(indices);
      is_valid_ = true;
    } else {
      std::cout << "Indices are not valid, update failed." << std::endl;
      is_valid_ = false;
    }
  }

  // 公共的访问器
  bool isValid() const { return is_valid_; }
  size_t size() const { return size_; }
  double start_s() const { return start_s_; }
  double delta_s() const { return delta_s_; }

 protected:
  // --- 成员变量 ---
  bool is_valid_ = false;
  double start_s_ = 0.0;
  double delta_s_ = 0.0;
  size_t size_ = 0;

 private:
  Derived& derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }

  bool validate(const std::vector<T>& indices) {
    if (indices.size() < 2) {
      return false;
    }
    const double first_delta = Traits<T>::indice(indices[1]) - Traits<T>::indice(indices[0]);
    constexpr double epsilon = 1e-6;
    // 验证第一个间隔是否大于 epsilon
    if (first_delta <= epsilon) {
      return false;
    }
    // 验证所有后续点的间隔是否与第一个间隔相同
    for (size_t i = 2; i < indices.size(); ++i) {
      const double current_delta = Traits<T>::indice(indices[i]) - Traits<T>::indice(indices[i - 1]);
      if (std::abs(current_delta - first_delta) > epsilon) {
        return false;
      }
    }
    // 验证通过后，更新公共成员变量
    start_s_ = Traits<T>::indice(indices.front());
    delta_s_ = first_delta;
    size_ = indices.size();
    return true;
  }
};
}  // namespace gpal::pnc::planning