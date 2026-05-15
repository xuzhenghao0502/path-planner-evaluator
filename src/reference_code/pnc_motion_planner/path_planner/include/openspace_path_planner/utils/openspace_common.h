#pragma once
#include <eka-rt/base/logger.h>
#include <variant>
#include "local_view/local_view.h"

namespace gpal::pnc::planning {
#define OPENSPACE_LOG(level, ...) ERT_LOG_##level("[openspace] ", __VA_ARGS__)

enum class OpenspaceBoundaryType { ROAD_SIDE = 100, SLOT = 101, AISLE = 102, GATE_WALL = 103, INVALID = 110 };
struct OpenspaceObjectType {
 private:
  // 定义私有的 variant 类型别名
  using VariantType = std::variant<Decision::ObjectType, OpenspaceBoundaryType>;

  // 成员变量，存储实际的 variant 值
  VariantType value;

 public:
  // 提供 constexpr 构造函数，以便创建静态常量
  constexpr OpenspaceObjectType(Decision::ObjectType v) : value(v) {}
  constexpr OpenspaceObjectType(OpenspaceBoundaryType v) : value(v) {}

  // 定义统一的静态常量访问点
  // --- 来自原始 Decision::ObjectType ---
  static const OpenspaceObjectType UNKNOWN;
  static const OpenspaceObjectType PEDESTRIAN;
  static const OpenspaceObjectType VRU;
  static const OpenspaceObjectType VEHICLE;
  static const OpenspaceObjectType HEAVY_VEHICLE;
  // --- 来自 OpenspaceBoundaryType ---
  static const OpenspaceObjectType ROAD_SIDE;
  static const OpenspaceObjectType SLOT;
  static const OpenspaceObjectType AISLE;
  static const OpenspaceObjectType GATE_WALL;
  static const OpenspaceObjectType INVALID;

  // 提供到 variant 的隐式转换
  operator const VariantType&() const { return value; }
  operator VariantType&() { return value; }

  // 提供比较运算符
  bool operator==(const OpenspaceObjectType& other) const { return this->value == other.value; }
  bool operator!=(const OpenspaceObjectType& other) const { return this->value != other.value; }
};
// 定义静态常量
inline const OpenspaceObjectType OpenspaceObjectType::UNKNOWN{Decision::ObjectType::UNKNOWN};
inline const OpenspaceObjectType OpenspaceObjectType::PEDESTRIAN{Decision::ObjectType::PEDESTRIAN};
inline const OpenspaceObjectType OpenspaceObjectType::VEHICLE{Decision::ObjectType::VEHICLE};
inline const OpenspaceObjectType OpenspaceObjectType::VRU{Decision::ObjectType::VRU};
inline const OpenspaceObjectType OpenspaceObjectType::HEAVY_VEHICLE{Decision::ObjectType::HEAVY_VEHICLE};

inline const OpenspaceObjectType OpenspaceObjectType::ROAD_SIDE{OpenspaceBoundaryType::ROAD_SIDE};
inline const OpenspaceObjectType OpenspaceObjectType::SLOT{OpenspaceBoundaryType::SLOT};
inline const OpenspaceObjectType OpenspaceObjectType::AISLE{OpenspaceBoundaryType::AISLE};
inline const OpenspaceObjectType OpenspaceObjectType::GATE_WALL{OpenspaceBoundaryType::GATE_WALL};
inline const OpenspaceObjectType OpenspaceObjectType::INVALID{OpenspaceBoundaryType::INVALID};

inline std::ostream& operator<<(std::ostream& os,
                                const std::variant<Decision::ObjectType, OpenspaceBoundaryType>& obj_type) {
  std::visit([&os](auto&& arg) { os << static_cast<int>(arg); }, obj_type);
  return os;
}

using ObstaclesVertice =
    std::tuple<std::string, std::variant<Decision::ObjectType, OpenspaceBoundaryType>, std::vector<math::Vec2d>>;
using ObstaclesLinesegment = std::tuple<std::string, std::variant<Decision::ObjectType, OpenspaceBoundaryType>,
                                        std::vector<math::LineSegment2d>>;

}  // namespace gpal::pnc::planning