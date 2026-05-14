/**
 * @file reference_point.cpp
 * @brief 参考点实现文件
 * @details 该文件实现了ReferencePoint类，用于表示参考线中的参考点信息。
 *          主要功能包括：初始化参考点、设置驾驶管道缓冲区、将参考点转换为路径点、
 *          横向和纵向偏移参考点、生成调试信息等。
 */

#include "reference_line/reference_point.h"

#include "util/string_util.h"
#include "util/util.h"

namespace gpal::pnc::planning {
using util::StrCat;

namespace {
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;
}  // namespace

/**
 * @brief 构造函数，用于初始化参考点
 * @details 根据给定的位置、坡度、航向角、曲率和曲率变化率，初始化参考点对象
 *
 * @param[in] pt 参考点的位置，包含x、y、z坐标
 * @param[in] slope 参考点的坡度
 * @param[in] heading 参考点的航向角
 * @param[in] kappa 参考点的曲率
 * @param[in] dkappa 参考点的曲率变化率
 *
 * @par 关键变量说明:
 * - pt (Vec3d): 参考点的位置，包含x、y、z坐标
 * - slope (float): 参考点的坡度
 * - heading (double): 参考点的航向角
 * - kappa (double): 参考点的曲率
 * - dkappa (double): 参考点的曲率变化率
 *
 * @par 处理逻辑:
 * - 调用基类Vec3d的构造函数，初始化参考点的位置
 * - 初始化参考点的坡度
 * - 初始化参考点的航向角
 * - 初始化参考点的曲率
 * - 初始化参考点的曲率变化率
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用基类Vec3d的构造函数，初始化参考点的位置;
 * :初始化参考点的坡度;
 * :初始化参考点的航向角;
 * :初始化参考点的曲率;
 * :初始化参考点的曲率变化率;
 * stop
 * @enduml
 *
 * @note 该构造函数用于初始化参考点，适用于需要根据位置、坡度、航向角、曲率和曲率变化率创建参考点的场景
 *
 * @warning 需确保输入的参数有效，否则可能无法正确初始化参考点
 */
ReferencePoint::ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa,
                               const double dkappa)
    : Vec3d(pt), slope_(slope), heading_(heading), kappa_(kappa), dkappa_(dkappa) {}

/**
 * @brief 构造函数，用于初始化参考点
 * @details 根据给定的位置、坡度、航向角、曲率、曲率变化率以及边界信息，初始化参考点对象
 *
 * @param[in] pt 参考点的位置，包含x、y、z坐标
 * @param[in] slope 参考点的坡度
 * @param[in] heading 参考点的航向角
 * @param[in] kappa 参考点的曲率
 * @param[in] dkappa 参考点的曲率变化率
 * @param[in] left_bound 参考点的左侧边界
 * @param[in] right_bound 参考点的右侧边界
 * @param[in] road_left_bound 参考点的道路左侧边界
 * @param[in] road_right_bound 参考点的道路右侧边界
 *
 * @par 关键变量说明:
 * - pt (Vec3d): 参考点的位置，包含x、y、z坐标
 * - slope (float): 参考点的坡度
 * - heading (double): 参考点的航向角
 * - kappa (double): 参考点的曲率
 * - dkappa (double): 参考点的曲率变化率
 * - left_bound (double): 参考点的左侧边界
 * - right_bound (double): 参考点的右侧边界
 * - road_left_bound (double): 参考点的道路左侧边界
 * - road_right_bound (double): 参考点的道路右侧边界
 *
 * @par 处理逻辑:
 * - 调用基类Vec3d的构造函数，初始化参考点的位置
 * - 初始化参考点的坡度
 * - 初始化参考点的航向角
 * - 初始化参考点的曲率
 * - 初始化参考点的曲率变化率
 * - 初始化参考点的左侧边界
 * - 初始化参考点的右侧边界
 * - 初始化参考点的道路左侧边界
 * - 初始化参考点的道路右侧边界
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用基类Vec3d的构造函数，初始化参考点的位置;
 * :初始化参考点的坡度;
 * :初始化参考点的航向角;
 * :初始化参考点的曲率;
 * :初始化参考点的曲率变化率;
 * :初始化参考点的左侧边界;
 * :初始化参考点的右侧边界;
 * :初始化参考点的道路左侧边界;
 * :初始化参考点的道路右侧边界;
 * stop
 * @enduml
 *
 * @note 该构造函数用于初始化参考点，适用于需要根据位置、坡度、航向角、曲率、曲率变化率以及边界信息创建参考点的场景
 *
 * @warning 需确保输入的参数有效，否则可能无法正确初始化参考点
 */
ReferencePoint::ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa,
                               const double dkappa, const double left_bound, const double right_bound,
                               const double road_left_bound, const double road_right_bound)
    : Vec3d(pt),
      slope_(slope),
      heading_(heading),
      kappa_(kappa),
      dkappa_(dkappa),
      left_bound_(left_bound),
      right_bound_(right_bound),
      road_left_bound_(road_left_bound),
      road_right_bound_(road_right_bound) {}

/**
 * @brief 构造函数，用于初始化参考点
 * @details 根据给定的位置、坡度、航向角、曲率、曲率变化率以及偏移量，初始化参考点对象
 *
 * @param[in] pt 参考点的位置，包含x、y、z坐标
 * @param[in] slope 参考点的坡度
 * @param[in] heading 参考点的航向角
 * @param[in] kappa 参考点的曲率
 * @param[in] dkappa 参考点的曲率变化率
 * @param[in] offset 参考点的偏移量
 *
 * @par 关键变量说明:
 * - pt (Vec3d): 参考点的位置，包含x、y、z坐标
 * - slope (float): 参考点的坡度
 * - heading (double): 参考点的航向角
 * - kappa (double): 参考点的曲率
 * - dkappa (double): 参考点的曲率变化率
 * - offset (double): 参考点的偏移量
 *
 * @par 处理逻辑:
 * - 调用基类Vec3d的构造函数，初始化参考点的位置
 * - 初始化参考点的坡度
 * - 初始化参考点的航向角
 * - 初始化参考点的曲率
 * - 初始化参考点的曲率变化率
 * - 初始化参考点的偏移量
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用基类Vec3d的构造函数，初始化参考点的位置;
 * :初始化参考点的坡度;
 * :初始化参考点的航向角;
 * :初始化参考点的曲率;
 * :初始化参考点的曲率变化率;
 * :初始化参考点的偏移量;
 * stop
 * @enduml
 *
 * @note 该构造函数用于初始化参考点，适用于需要根据位置、坡度、航向角、曲率、曲率变化率以及偏移量创建参考点的场景
 *
 * @warning 需确保输入的参数有效，否则可能无法正确初始化参考点
 */
ReferencePoint::ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa,
                               const double dkappa, const double offset)
    : Vec3d(pt), slope_(slope), heading_(heading), kappa_(kappa), dkappa_(dkappa), offset_(offset) {}

/**
 * @brief 构造函数，用于初始化参考点
 * @details 根据给定的位置、坡度、航向角、曲率、曲率变化率、边界信息以及偏移量，初始化参考点对象
 *
 * @param[in] pt 参考点的位置，包含x、y、z坐标
 * @param[in] slope 参考点的坡度
 * @param[in] heading 参考点的航向角
 * @param[in] kappa 参考点的曲率
 * @param[in] dkappa 参考点的曲率变化率
 * @param[in] left_bound 参考点的左侧边界
 * @param[in] right_bound 参考点的右侧边界
 * @param[in] road_left_bound 参考点的道路左侧边界
 * @param[in] road_right_bound 参考点的道路右侧边界
 * @param[in] offset 参考点的偏移量
 *
 * @par 关键变量说明:
 * - pt (Vec3d): 参考点的位置，包含x、y、z坐标
 * - slope (float): 参考点的坡度
 * - heading (double): 参考点的航向角
 * - kappa (double): 参考点的曲率
 * - dkappa (double): 参考点的曲率变化率
 * - left_bound (double): 参考点的左侧边界
 * - right_bound (double): 参考点的右侧边界
 * - road_left_bound (double): 参考点的道路左侧边界
 * - road_right_bound (double): 参考点的道路右侧边界
 * - offset (double): 参考点的偏移量
 *
 * @par 处理逻辑:
 * - 调用基类Vec3d的构造函数，初始化参考点的位置
 * - 初始化参考点的坡度
 * - 初始化参考点的航向角
 * - 初始化参考点的曲率
 * - 初始化参考点的曲率变化率
 * - 初始化参考点的左侧边界
 * - 初始化参考点的右侧边界
 * - 初始化参考点的道路左侧边界
 * - 初始化参考点的道路右侧边界
 * - 初始化参考点的偏移量
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用基类Vec3d的构造函数，初始化参考点的位置;
 * :初始化参考点的坡度;
 * :初始化参考点的航向角;
 * :初始化参考点的曲率;
 * :初始化参考点的曲率变化率;
 * :初始化参考点的左侧边界;
 * :初始化参考点的右侧边界;
 * :初始化参考点的道路左侧边界;
 * :初始化参考点的道路右侧边界;
 * :初始化参考点的偏移量;
 * stop
 * @enduml
 *
 * @note
 * 该构造函数用于初始化参考点，适用于需要根据位置、坡度、航向角、曲率、曲率变化率、边界信息以及偏移量创建参考点的场景
 *
 * @warning 需确保输入的参数有效，否则可能无法正确初始化参考点
 */
ReferencePoint::ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa,
                               const double dkappa, const double left_bound, const double right_bound,
                               const double road_left_bound, const double road_right_bound, const double offset)
    : Vec3d(pt),
      slope_(slope),
      heading_(heading),
      kappa_(kappa),
      dkappa_(dkappa),
      left_bound_(left_bound),
      right_bound_(right_bound),
      road_left_bound_(road_left_bound),
      road_right_bound_(road_right_bound),
      offset_(offset) {}

/**
 * @brief 构造函数，用于初始化参考点
 * @details 根据给定的位置、坡度、航向角、曲率、曲率变化率、偏移量以及局部s值，初始化参考点对象
 *
 * @param[in] pt 参考点的位置，包含x、y、z坐标
 * @param[in] slope 参考点的坡度
 * @param[in] heading 参考点的航向角
 * @param[in] kappa 参考点的曲率
 * @param[in] dkappa 参考点的曲率变化率
 * @param[in] offset 参考点的偏移量
 * @param[in] local_s 参考点的局部s值
 *
 * @par 关键变量说明:
 * - pt (Vec3d): 参考点的位置，包含x、y、z坐标
 * - slope (float): 参考点的坡度
 * - heading (double): 参考点的航向角
 * - kappa (double): 参考点的曲率
 * - dkappa (double): 参考点的曲率变化率
 * - offset (double): 参考点的偏移量
 * - local_s (double): 参考点的局部s值
 *
 * @par 处理逻辑:
 * - 调用基类Vec3d的构造函数，初始化参考点的位置
 * - 初始化参考点的坡度
 * - 初始化参考点的航向角
 * - 初始化参考点的曲率
 * - 初始化参考点的曲率变化率
 * - 初始化参考点的偏移量
 * - 初始化参考点的局部s值
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用基类Vec3d的构造函数，初始化参考点的位置;
 * :初始化参考点的坡度;
 * :初始化参考点的航向角;
 * :初始化参考点的曲率;
 * :初始化参考点的曲率变化率;
 * :初始化参考点的偏移量;
 * :初始化参考点的局部s值;
 * stop
 * @enduml
 *
 * @note
 * 该构造函数用于初始化参考点，适用于需要根据位置、坡度、航向角、曲率、曲率变化率、偏移量以及局部s值创建参考点的场景
 *
 * @warning 需确保输入的参数有效，否则可能无法正确初始化参考点
 */
ReferencePoint::ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa,
                               const double dkappa, const double offset, const double local_s)
    : Vec3d(pt), slope_(slope), heading_(heading), kappa_(kappa), dkappa_(dkappa), offset_(offset), local_s_(local_s) {}

/**
 * @brief 构造函数，用于初始化参考点
 * @details 根据给定的位置、坡度、航向角、曲率、曲率变化率、边界信息、偏移量以及局部s值，初始化参考点对象
 *
 * @param[in] pt 参考点的位置，包含x、y、z坐标
 * @param[in] slope 参考点的坡度
 * @param[in] heading 参考点的航向角
 * @param[in] kappa 参考点的曲率
 * @param[in] dkappa 参考点的曲率变化率
 * @param[in] left_bound 参考点的左侧边界
 * @param[in] right_bound 参考点的右侧边界
 * @param[in] road_left_bound 参考点的道路左侧边界
 * @param[in] road_right_bound 参考点的道路右侧边界
 * @param[in] offset 参考点的偏移量
 * @param[in] local_s 参考点的局部s值
 *
 * @par 关键变量说明:
 * - pt (Vec3d): 参考点的位置，包含x、y、z坐标
 * - slope (float): 参考点的坡度
 * - heading (double): 参考点的航向角
 * - kappa (double): 参考点的曲率
 * - dkappa (double): 参考点的曲率变化率
 * - left_bound (double): 参考点的左侧边界
 * - right_bound (double): 参考点的右侧边界
 * - road_left_bound (double): 参考点的道路左侧边界
 * - road_right_bound (double): 参考点的道路右侧边界
 * - offset (double): 参考点的偏移量
 * - local_s (double): 参考点的局部s值
 *
 * @par 处理逻辑:
 * - 调用基类Vec3d的构造函数，初始化参考点的位置
 * - 初始化参考点的坡度
 * - 初始化参考点的航向角
 * - 初始化参考点的曲率
 * - 初始化参考点的曲率变化率
 * - 初始化参考点的左侧边界
 * - 初始化参考点的右侧边界
 * - 初始化参考点的道路左侧边界
 * - 初始化参考点的道路右侧边界
 * - 初始化参考点的偏移量
 * - 初始化参考点的局部s值
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用基类Vec3d的构造函数，初始化参考点的位置;
 * :初始化参考点的坡度;
 * :初始化参考点的航向角;
 * :初始化参考点的曲率;
 * :初始化参考点的曲率变化率;
 * :初始化参考点的左侧边界;
 * :初始化参考点的右侧边界;
 * :初始化参考点的道路左侧边界;
 * :初始化参考点的道路右侧边界;
 * :初始化参考点的偏移量;
 * :初始化参考点的局部s值;
 * stop
 * @enduml
 *
 * @note
 * 该构造函数用于初始化参考点，适用于需要根据位置、坡度、航向角、曲率、曲率变化率、边界信息、偏移量以及局部s值创建参考点的场景
 *
 * @warning 需确保输入的参数有效，否则可能无法正确初始化参考点
 */
ReferencePoint::ReferencePoint(const Vec3d pt, const float slope, const double heading, const double kappa,
                               const double dkappa, const double left_bound, const double right_bound,
                               const double road_left_bound, const double road_right_bound, const double offset,
                               const double local_s)
    : Vec3d(pt),
      slope_(slope),
      heading_(heading),
      kappa_(kappa),
      dkappa_(dkappa),
      left_bound_(left_bound),
      right_bound_(right_bound),
      road_left_bound_(road_left_bound),
      road_right_bound_(road_right_bound),
      offset_(offset),
      local_s_(local_s) {}

/**
 * @brief 设置驾驶管道的缓冲区
 * @details 根据给定的左侧和右侧缓冲区值，设置参考点的驾驶管道缓冲区
 *
 * @param[in] left 左侧缓冲区值
 * @param[in] right 右侧缓冲区值
 *
 * @par 关键变量说明:
 * - left (double): 左侧缓冲区值
 * - right (double): 右侧缓冲区值
 *
 * @par 处理逻辑:
 * - 设置参考点的左侧驾驶管道缓冲区
 * - 设置参考点的右侧驾驶管道缓冲区
 *
 * @par 流程图:
 * @startuml
 * start
 * :设置参考点的左侧驾驶管道缓冲区;
 * :设置参考点的右侧驾驶管道缓冲区;
 * stop
 * @enduml
 *
 * @note 该函数用于设置参考点的驾驶管道缓冲区，适用于需要根据左侧和右侧缓冲区值调整驾驶管道的场景
 *
 * @warning 需确保输入的缓冲区值有效，否则可能无法正确设置驾驶管道缓冲区
 */
void ReferencePoint::setDrivingTubeBuffer(const double left, const double right) {
  driving_tube_left_buffer_ = left;
  driving_tube_right_buffer_ = right;
}

/**
 * @brief 将参考点转换为路径点
 * @details 根据给定的s值，将当前参考点转换为路径点对象
 *
 * @param[in] s 路径点的s值
 * @return 返回转换后的路径点对象
 *
 * @par 关键变量说明:
 * - s (double): 路径点的s值
 *
 * @par 处理逻辑:
 * - 使用当前参考点的位置、坡度、航向角、曲率、曲率变化率等信息创建路径点
 * - 设置路径点的s值
 * - 返回路径点对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :使用参考点信息创建路径点;
 * :设置路径点的s值;
 * :返回路径点对象;
 * stop
 * @enduml
 *
 * @note 该函数用于将参考点转换为路径点，适用于需要根据s值生成路径点的场景
 *
 * @warning 需确保输入的s值有效，否则可能无法正确生成路径点
 */
gpal::proto::PathPoint ReferencePoint::ToPathPoint(double s) const {
  gpal::proto::PathPoint path_point = util::MakePathPoint(x(), y(), z(), slope(), heading(), kappa_, dkappa_, 0.0);
  path_point.set_s(s);
  return path_point;
}

/**
 * @brief 横向偏移参考点
 * @details 根据给定的横向偏移量，生成一个新的参考点，该参考点是当前参考点的横向偏移结果
 *
 * @param[in] l 横向偏移量（正值为向左偏移，负值为向右偏移）
 * @return 返回横向偏移后的参考点对象
 *
 * @par 关键变量说明:
 * - l (double): 横向偏移量
 *
 * @par 处理逻辑:
 * - 复制当前参考点对象
 * - 根据航向角和偏移量计算新的x坐标
 * - 根据航向角和偏移量计算新的y坐标
 * - 返回偏移后的参考点对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :复制当前参考点对象;
 * :根据航向角和偏移量计算新的x坐标;
 * :根据航向角和偏移量计算新的y坐标;
 * :返回偏移后的参考点对象;
 * stop
 * @enduml
 *
 * @note 该函数用于生成当前参考点的横向偏移结果，适用于需要根据横向偏移量调整参考点位置的场景
 *
 * @warning 需确保输入的偏移量有效，否则可能无法正确生成偏移后的参考点
 */
ReferencePoint ReferencePoint::lateralShift(const double l) const {
  ReferencePoint pt(*this);
  pt.x_ -= l * sin(pt.heading_);
  pt.y_ += l * cos(pt.heading_);
  return pt;
}

/**
 * @brief 纵向偏移参考点
 * @details 根据给定的纵向偏移量，生成一个新的参考点，该参考点是当前参考点的纵向偏移结果
 *
 * @param[极客时间](http://gk.link/a/10v9C) in] s 纵向偏移量（正值为向前偏移，负值为向后偏移）
 * @return 返回纵向偏移后的参考点对象
 *
 * @par 关键变量说明:
 * - s (double): 纵向偏移量
 *
 * @par 处理逻辑:
 * - 复制当前参考点对象
 * - 根据航向角和偏移量计算新的x坐标
 * - 根据航向角和偏移量计算新的y坐标
 * - 返回偏移后的参考点对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :复制当前参考点对象;
 * :根据航向角和偏移量计算新的x坐标;
 * :根据航向角和偏移量计算新的y坐标;
 * :返回偏移后的参考点对象;
 * stop
 * @enduml
 *
 * @note 该函数用于生成当前参考点的纵向偏移结果，适用于需要根据纵向偏移量调整参考点位置的场景
 *
 * @warning 需确保输入的偏移量有效，否则可能无法正确生成偏移后的参考点
 */
ReferencePoint ReferencePoint::longitudinalShift(const double s) const {
  ReferencePoint pt(*this);
  pt.x_ += s * cos(pt.heading_);
  pt.y_ += s * sin(pt.heading_);
  return pt;
}

/**
 * @brief 生成参考点的调试信息字符串
 * @details 将参考点的关键信息（如位置、坡度、航向角、曲率、曲率变化率等）格式化为字符串，便于调试和日志记录
 *
 * @return 返回包含参考点调试信息的字符串
 *
 * @par 关键变量说明:
 * - x(): 参考点的x坐标
 * - y(): 参考点的y坐标
 * - z(): 参考点的z坐标
 * - slope(): 参考点的坡度
 * - heading(): 参考点的航向角
 * - kappa(): 参考点的曲率
 * - dkappa(): 参考点的曲率变化率
 *
 * @par 处理逻辑:
 * - 使用StrCat函数将参考点的位置、坡度、航向角、曲率和曲率变化率等信息拼接为字符串
 * - 返回生成的调试信息字符串
 *
 * @par 流程图:
 * @startuml
 * start
 * :使用StrCat拼接参考点信息;
 * :返回调试信息字符串;
 * stop
 * @enduml
 *
 * @note 该函数用于生成参考点的调试信息，适用于需要将参考点信息输出为字符串的场景，如日志记录或调试输出
 *
 * @warning 需确保参考点的各属性值有效，否则可能无法正确生成调试信息
 */
std::string ReferencePoint::DebugString() const {
  return StrCat("{x: ", x(), ", y: ", y(), ", z: ", z(), ", slope: ", slope(), ", heading: ", heading()) +
         StrCat(", kappa: ", kappa(), ", dkappa: ", dkappa(), "}");
}

}  // namespace gpal::pnc::planning
