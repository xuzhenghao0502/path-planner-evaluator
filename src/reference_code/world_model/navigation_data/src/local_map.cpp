/**
 * @file local_map.cpp
 * @brief 局部地图数据核心模块，负责管理局部地图中的道路段和路口信息
 * @details 该模块用于存储和管理局部地图中的道路段和路口信息，并提供快速查询功能，支持导航和路径规划。
 */

#include "navigation_data/local_map.h"

#include "config_manager/config_manager.h"

namespace gpal::pnc::planning {

/**
 * @brief LocalMapRoadSegment构造函数
 * @details 初始化LocalMapRoadSegment对象，并调用clear()函数清空所有数据
 */
LocalMapRoadSegment::LocalMapRoadSegment() { clear(); }

/**
 * @brief LocalMapRoadSegment析构函数
 * @details 释放LocalMapRoadSegment对象占用的资源，确保所有动态分配的内存被正确释放
 */
LocalMapRoadSegment::~LocalMapRoadSegment() {}

/**
 * @brief 初始化LocalMapRoadSegment对象
 * @details 根据道路段的点集初始化AABB包围盒、长度数组和线段数组
 *
 * @par 输入参数说明:
 * - 无显式输入参数，但依赖成员变量points_（道路段点集）进行初始化
 *
 * @par 输出参数说明:
 * - 返回值 (bool): 初始化是否成功，成功返回true，失败返回false
 *
 * @par 关键变量说明:
 * - points_ (std::vector<math::Vec2d>): 道路段点集，必须包含至少2个点
 * - aabox_ (math::AABox2d): 道路段的AABB包围盒，由起点和终点初始化
 * - length_array_ (std::vector<double>): 道路段的长度数组，存储每个点到起点的累计距离
 * - line_segment_array_ (std::vector<math::LineSegment2d>): 道路段的线段列表，由相邻点生成
 *
 * @par 判断条件:
 * - points_的大小必须大于等于2，否则初始化失败
 *
 * @par 流程图:
 * @startuml
 * start
 * if (points_大小是否小于2?) then (是)
 *   :返回false;
 * else (否)
 *   :获取起点和终点;
 *   :初始化aabox_;
 *   :初始化length_array_;
 *   :计算每个点的累计距离;
 *   :初始化line_segment_array_;
 *   :返回true;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于初始化道路段的几何信息，必须在对象创建后调用
 *
 * @warning 调用该函数前需确保points_已正确设置
 */
bool LocalMapRoadSegment::init() {
  if (points_.size() < 2) {
    return false;
  }
  const auto& start_point = points_[0];
  const auto& end_point = points_[points_.size() - 1];
  aabox_ = math::AABox2d(start_point, end_point);
  length_array_.resize(points_.size());
  for (int i = 0; i < points_.size(); ++i) {
    if (i == 0) {
      length_array_[i] = 0.0;
    } else {
      length_array_[i] = length_array_[i - 1] + points_[i - 1].DistanceTo(points_[i]);
    }
  }
  for (int i = 0; i < points_.size() - 1; ++i) {
    math::LineSegment2d seg(points_[i], points_[i + 1]);
    line_segment_array_.emplace_back(std::move(seg));
  }
  return true;
}

/**
 * @brief 清空LocalMapRoadSegment对象的所有数据
 * @details 重置所有成员变量为初始状态，确保对象可以重新使用
 *
 * @par 关键变量说明:
 * - road_segment_id_ (std::string): 道路段ID，重置为空字符串
 * - road_id_ (std::string): 道路ID，重置为空字符串
 * - road_direction_ (RoadDirection): 道路方向，重置为kStartToEnd
 * - driving_direction_in_routing_ (DrivingDirection): 导航中的行驶方向，重置为kDrivingUnknown
 * - road_class_ (RoadClass): 道路等级，重置为kRoadInvalid
 * - road_types_ (std::vector<RoadType>): 道路类型列表，清空所有条目
 * - speed_limit_ (double): 速度限制，重置为0
 * - length_ (double): 道路段长度，重置为0.0
 * - points_ (std::vector<math::Vec2d>): 道路段点集，清空所有条目
 * - is_lane_info_valid_ (bool): 车道信息是否有效，重置为false
 * - lane_num_ (int): 车道数量，重置为0
 * - lanes_ (std::vector<Lane>): 车道信息列表，清空所有条目
 * - stop_lines_ (std::vector<StopLine>): 停止线列表，清空所有条目
 * - cross_in_out_type_ (CrossInOutType): 交叉口进出类型，重置为kCrossInvalid
 * - in_road_segment_ids_ (std::vector<std::string>): 进入道路段ID列表，清空所有条目
 * - out_road_segment_ids_ (std::vector<std::string>): 驶出道路段ID列表，清空所有条目
 * - left_road_segment_id_ (std::string): 左侧道路段ID，重置为空字符串
 * - right_road_segment_id_ (std::string): 右侧道路段ID，重置为空字符串
 * - related_junction_ids_ (std::vector<std::string>): 相关路口ID列表，清空所有条目
 * - aabox_ (math::AABox2d): 道路段的AABB包围盒，重置为空
 * - line_segment_array_ (std::vector<math::LineSegment2d>): 道路段的线段列表，清空所有条目
 * - length_array_ (std::vector<double>): 道路段的长度数组，清空所有条目
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用StampedBase::reset();
 * :重置road_segment_id_为空字符串;
 * :重置road_id_为空字符串;
 * :重置road_direction_为kStartToEnd;
 * :重置driving_direction_in_routing_为kDrivingUnknown;
 * :重置road_class_为kRoadInvalid;
 * :清空road_types_;
 * :重置speed_limit_为0;
 * :重置length_为0.0;
 * :清空points_;
 * :重置is_lane_info_valid_为false;
 * :重置lane_num_为0;
 * :清空lanes_;
 * :清空stop_lines_;
 * :重置cross_in_out_type_为kCrossInvalid;
 * :清空in_road_segment_ids_;
 * :清空out_road_segment_ids_;
 * :重置left_road_segment_id_为空字符串;
 * :重置right_road_segment_id_为空字符串;
 * :清空related_junction_ids_;
 * :重置aabox_为空;
 * :清空line_segment_array_;
 * :清空length_array_;
 * stop
 * @enduml
 *
 * @note 该函数用于重置对象状态，通常在对象重用或销毁前调用
 *
 * @warning 调用该函数后，所有存储的数据将被清空，需谨慎使用
 */
void LocalMapRoadSegment::clear() {
  road_segment_id_ = "";
  road_id_ = "";
  road_direction_ = RoadDirection::kStartToEnd;
  driving_direction_in_routing_ = DrivingDirection::kDrivingUnknown;
  road_class_ = RoadClass::kRoadInvalid;
  road_types_.clear();
  speed_limit_ = 0;
  length_ = 0.0;
  points_.clear();
  is_lane_info_valid_ = false;
  lane_num_ = 0;
  lanes_.clear();
  stop_lines_.clear();
  cross_in_out_type_ = CrossInOutType::kCrossInvalid;
  in_road_segment_ids_.clear();
  out_road_segment_ids_.clear();
  left_road_segment_id_ = "";
  right_road_segment_id_ = "";
  related_junction_ids_.clear();
  aabox_ = math::AABox2d();
  line_segment_array_.clear();
  length_array_.clear();
}

/**
 * @brief 计算点到道路段的最短距离平方
 * @details 通过查找最近的线段，计算点到该线段的距离平方
 *
 * @par 输入参数说明:
 * - point (math::Vec2d): 目标点，类型为二维向量
 *
 * @par 输出参数说明:
 * - 返回值 (double): 点到道路段的最短距离平方，若未找到最近线段则返回最大值
 *
 * @par 关键变量说明:
 * - line_segment_array_ (std::vector<math::LineSegment2d>): 道路段的线段列表，用于计算距离
 *
 * @par 判断条件:
 * - 若未找到最近线段，则返回最大值
 *
 * @par 流程图:
 * @startuml
 * start
 * :查找最近线段索引;
 * if (索引是否有效?) then (否)
 *   :返回最大值;
 * else (是)
 *   :计算点到线段的距离平方;
 *   :返回距离平方;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算点到道路段的最短距离平方，适用于几何计算
 *
 * @warning 调用该函数前需确保line_segment_array_已正确初始化
 */
double LocalMapRoadSegment::DistanceSquareTo(const math::Vec2d& point) const {
  int idx = findLineSegmentIndex(point);
  if (idx < 0) {
    return std::numeric_limits<double>::max();
  }
  return line_segment_array_[idx].DistanceSquareTo(point);
}

/**
 * @brief 计算点到道路段的最短距离平方（考虑方向）
 * @details 通过查找最近的线段，计算点到该线段的距离平方，并根据道路方向调整计算结果
 *
 * @par 输入参数说明:
 * - point (math::Vec2d): 目标点，类型为二维向量
 * - heading (double): 目标点的方向，单位为弧度
 *
 * @par 输出参数说明:
 * - 返回值 (double): 点到道路段的最短距离平方，若未找到最近线段则返回最大值
 *
 * @par 关键变量说明:
 * - line_segment_array_ (std::vector<math::LineSegment2d>): 道路段的线段列表，用于计算距离
 * - road_direction_ (RoadDirection): 道路方向，影响距离计算方式
 *
 * @par 判断条件:
 * - 若未找到最近线段，则返回最大值
 * - 若道路方向为双向，则计算正向和反向距离，取最小值
 *
 * @par 流程图:
 * @startuml
 * start
 * :查找最近线段索引;
 * if (索引是否有效?) then (否)
 *   :返回最大值;
 * else (是)
 *   if (道路方向是否为双向?) then (是)
 *     :计算正向距离平方;
 *     :计算反向距离平方;
 *     :取最小值;
 *   else (否)
 *     :计算正向距离平方;
 *   endif
 *   :返回距离平方;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于计算点到道路段的最短距离平方，适用于考虑方向的几何计算
 *
 * @warning 调用该函数前需确保line_segment_array_和road_direction_已正确初始化
 */
double LocalMapRoadSegment::DistanceSquareTo(const math::Vec2d& point, double heading) const {
  int idx = findLineSegmentIndex(point);
  if (idx < 0) {
    return std::numeric_limits<double>::max();
  }
  auto& nearest_seg = line_segment_array_[idx];
  double result = std::numeric_limits<double>::max();
  if (road_direction_ == RoadDirection::kBothDirection) {
    double dist_1 = nearest_seg.DistanceSquareTo(point, heading);
    double dist_2 = nearest_seg.DistanceSquareTo(point, math::OppositeAngle(heading));
    // double dist_2 = reverse_line_segment.DistanceSquareTo(point, heading);
    result = std::min(dist_1, dist_2);
  } else {
    result = nearest_seg.DistanceSquareTo(point, heading);
  }
  // ERT_PLOG_I << "[DistanceSquareTo] Road ID: " << road_id << ", point heading = " << heading
  //           << ", dist = " << result;
  return result;
}

/**
 * @brief 查找距离给定点最近的线段索引
 * @details 通过遍历所有线段，计算点到每条线段的距离平方，找到距离最小的线段索引
 *
 * @par 输入参数说明:
 * - point (math::Vec2d): 目标点，类型为二维向量
 *
 * @par 输出参数说明:
 * - 返回值 (int): 距离给定点最近的线段索引，若线段列表为空则返回-1
 *
 * @par 关键变量说明:
 * - line_segment_array_ (std::vector<math::LineSegment2d>): 道路段的线段列表，用于计算距离
 * - min_distance (double): 最小距离平方，初始化为无穷大
 * - min_index (int): 最小距离对应的线段索引，初始化为-1
 *
 * @par 判断条件:
 * - 若线段列表为空，则返回-1
 *
 * @par 流程图:
 * @startuml
 * start
 * if (line_segment_array_是否为空?) then (是)
 *   :返回-1;
 * else (否)
 *   :初始化min_distance为无穷大;
 *   :初始化min_index为-1;
 *   :遍历line_segment_array_;
 *   :计算点到线段的距离平方;
 *   if (距离是否小于min_distance?) then (是)
 *     :更新min_distance;
 *     :更新min_index;
 *   endif
 *   :返回min_index;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于查找距离给定点最近的线段索引，适用于几何计算
 *
 * @warning 调用该函数前需确保line_segment_array_已正确初始化
 */
int LocalMapRoadSegment::findLineSegmentIndex(const math::Vec2d& point) const {
  if (line_segment_array_.empty()) {
    return -1;
  }
  double min_distance = std::numeric_limits<double>::infinity();
  int min_index = -1;
  for (int i = 0; i < line_segment_array_.size(); ++i) {
    const double distance = line_segment_array_[i].DistanceSquareTo(point);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  return min_index;
}

/**
 * @brief LocalMap
 * @details 初始化LocalMap对象，并调用clear()函数清空所有数据
 */
LocalMap::LocalMap() { clear(); }

/**
 * @brief 初始化局部地图对象
 * @details 对局部地图中所有道路段进行初始化验证，确保地图数据可用性
 *
 * @par 输入参数说明:
 * - 无显式参数，依赖成员变量road_segment_map_中的道路段数据
 *
 * @par 输出参数说明:
 * - 返回值 (bool): 初始化结果
 *   - true: 所有道路段初始化成功
 *   - false: 存在道路段初始化失败或地图数据无效
 *
 * @par 关键变量说明:
 * - road_segment_map_ (map<string,shared_ptr<LocalMapRoadSegment>>): 道路段集合
 *   - 键: 道路段ID (长度范围[1,256])
 *   - 值: 道路段智能指针 (非空指针)
 * - is_inited_ (bool): 初始化状态标记
 *   - true: 地图数据就绪
 *   - false: 地图数据不可用
 *
 * @par 判断条件:
 * - 地图有效性检查: 通过isValid()验证基础数据完整性
 * - 道路段初始化: 每个road_segment的init()必须返回true
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用isValid()检查;
 * if (地图无效?) then (是)
 *   :设置is_inited_=false;
 *   return false;
 * else (否)
 *   :遍历所有道路段;
 *   -> 循环处理;
 *   if (道路段初始化失败?) then (是)
 *     :设置status=false;
 *     break;
 *   endif
 *   <- 循环结束;
 *   :设置is_inited_=status;
 *   return status;
 * endif
 * stop
 * @enduml
 *
 * @note 初始化顺序要求:
 * 1. 必须在clear()之后调用
 * 2. 需确保road_segment_map_已加载有效数据
 *
 * @warning 出现以下情况将导致初始化失败:
 * - 地图数据错误码非kOK (map_error_code_ != kOK)
 * - 自车所在道路段ID无效 (ego_road_seg_id_为空或不存在)
 * - 道路段点集数量不足 (points_.size() < 2)
 */
bool LocalMap::init() {
  if (!isValid()) {
    is_inited_ = false;
    return is_inited_;
  }
  bool status = true;
  for (auto& road_segment : road_segment_map_) {
    if (!road_segment.second->init()) {
      status = false;
      break;
    }
  }
  is_inited_ = status;
  return is_inited_;
}

/**
 * @brief 初始化LocalMap对象
 * @details 初始化LocalMap对象中的所有道路段，并构建AABB KD树用于快速查询
 *
 * @par 输入参数说明:
 * - tree_params (math::AABoxKDTree2dParams): AABB KD树的构建参数，包含分割策略等信息
 *
 * @par 输出参数说明:
 * - 返回值 (bool): 初始化是否成功，成功返回true，失败返回false
 *
 * @par 关键变量说明:
 * - road_segment_map_ (std::map<std::string, std::shared_ptr<LocalMapRoadSegment>>): 道路段映射表，存储所有道路段
 * - road_segment_tree_ (std::unique_ptr<math::AABoxKDTree2d<LocalMapRoadSegment>>): AABB KD树，用于快速查询最近道路段
 * - road_segment_tree_params_ (math::AABoxKDTree2dParams): AABB KD树的构建参数
 * - is_inited_ (bool): 初始化状态，成功初始化后设置为true
 *
 * @par 判断条件:
 * - 若任一道路段初始化失败，则整个LocalMap初始化失败
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化状态为true;
 * :遍历road_segment_map_;
 * if (道路段初始化是否成功?) then (否)
 *   :设置初始化状态为false;
 *   :跳出循环;
 * endif
 * :构建AABB KD树;
 * :保存AABB KD树参数;
 * :返回初始化状态;
 * stop
 * @enduml
 *
 * @note 该函数用于初始化LocalMap对象，必须在对象创建后调用
 *
 * @warning 调用该函数前需确保road_segment_map_已正确设置
 */
bool LocalMap::init(const math::AABoxKDTree2dParams& tree_params) {
  if (!isValid()) {
    is_inited_ = false;
    return is_inited_;
  }
  bool status = true;
  for (auto& road_segment : road_segment_map_) {
    if (!road_segment.second->init()) {
      status = false;
      break;
    }
  }
  road_segment_tree_.reset(new math::AABoxKDTree2d<LocalMapRoadSegment>(road_segment_map_, tree_params));
  road_segment_tree_params_ = tree_params;
  is_inited_ = status;
  return is_inited_;
}

/**
 * @brief 验证局部地图数据有效性
 * @details 执行三级有效性检查确保地图数据满足导航规划需求：
 * 1. 基础状态验证：地图错误码检测
 * 2. 自车定位验证：自车所在道路段ID有效性
 * 3. 数据结构验证：道路段点集完整性
 *
 * @par 输出参数说明:
 * - 返回值 (bool): 有效性状态
 *   - true: 满足所有有效性条件
 *   - false: 任一条件不满足
 *
 * @par 关键变量说明:
 * - map_error_code_ (MapErrorCode): 地图错误状态标识
 *   - 有效值: kOK (0)
 *   - 无效值范围: [kNoMapData, kMapExpired]
 * - ego_road_seg_id_ (std::string): 自车所在道路段ID
 *   - 长度范围: [1, 256]字符
 *   - 空值表示定位失效
 * - road_segment_map_ (std::map): 道路段存储容器
 *   - 键存在性: 必须包含ego_road_seg_id_
 *   - 值有效性: 对应道路段需通过LocalMapRoadSegment::init()验证
 *
 * @par 判断条件:
 * - 一级条件: map_error_code_ == kOK
 * - 二级条件: !ego_road_seg_id_.empty()
 * - 三级条件: road_segment_map_.count(ego_road_seg_id_) > 0
 *
 * @par 流程图:
 * @startuml
 * start
 * partition 三级验证流程 {
 * if (地图错误码非OK?) then (是)
 *   :返回false;
 * else (否)
 *   if (自车道路段ID为空?) then (是)
 *     :返回false;
 *   else (否)
 *     if (道路段不存在?) then (是)
 *       :返回false;
 *     else (否)
 *       :返回true;
 *     endif
 *   endif
 * endif
 * }
 * stop
 * @enduml
 *
 * @note 有效性层级关系:
 * 1. 基础状态 > 自车定位 > 数据结构
 * 2. 前序条件失败将跳过后续检查
 *
 * @warning 出现以下情况需重新加载地图数据:
 * - 连续3次验证返回false
 * - map_error_code_持续非kOK超过5秒
 */
bool LocalMap::isValid() {
  // ERT_PLOG_I << "map_error_code_ = " << (int)map_error_code_ << ", ego_road_seg_id_ = " << ego_road_seg_id_ ;
  if (map_error_code_ == MapErrorCode::kOK && !ego_road_seg_id_.empty()) {
    if (road_segment_map_.count(ego_road_seg_id_) > 0) {
      return true;
    }
  }
  return false;
}

/**
 * @brief 清空LocalMap对象的所有数据
 * @details 重置所有成员变量为初始状态，确保对象可以重新使用
 *
 * @par 关键变量说明:
 * - is_inited_ (bool): 初始化状态，重置为false
 * - map_error_code_ (MapErrorCode): 地图错误码，重置为kNoMapData
 * - map_range_ (int): 地图范围，重置为0
 * - road_segment_map_ (std::map<std::string, std::shared_ptr<LocalMapRoadSegment>>): 道路段映射表，清空所有条目
 * - junction_map_ (std::map<std::string, std::shared_ptr<LocalMapJunction>>): 路口映射表，清空所有条目
 * - road_segment_tree_ (std::unique_ptr<math::AABoxKDTree2d<LocalMapRoadSegment>>): AABB KD树，重置为空
 *
 * @par 流程图:
 * @startuml
 * start
 * :重置is_inited_为false;
 * :重置map_error_code_为kNoMapData;
 * :重置map_range_为0;
 * :清空road_segment_map_;
 * :清空junction_map_;
 * :重置road_segment_tree_为空;
 * stop
 * @enduml
 *
 * @note 该函数用于重置对象状态，通常在对象重用或销毁前调用
 *
 * @warning 调用该函数后，所有存储的数据将被清空，需谨慎使用
 */
void LocalMap::clear() {
  is_inited_ = false;
  map_error_code_ = MapErrorCode::kNoMapData;
  map_range_ = 0;
  road_segment_map_.clear();
  junction_map_.clear();
  road_segment_tree_.reset();
}

/**
 * @brief 获取距离给定位姿最近的道路段
 * @details 通过AABB KD树查询距离给定位姿最近的道路段，并将结果存储在nearest_road_seg中
 *
 * @par 输入参数说明:
 * - pose (math::Arrow2d): 目标位姿，包含位置和方向信息
 * - nearest_road_seg (LocalMapRoadSegment**): 用于存储最近道路段的指针
 *
 * @par 输出参数说明:
 * - 返回值 (bool): 查询是否成功，成功返回true，失败返回false
 *
 * @par 关键变量说明:
 * - road_segment_tree_ (std::unique_ptr<math::AABoxKDTree2d<LocalMapRoadSegment>>): AABB KD树，用于快速查询最近道路段
 * - is_inited_ (bool): 初始化状态，若未初始化则查询失败
 *
 * @par 判断条件:
 * - 若未初始化，则返回false
 * - 若未找到最近道路段，则返回false
 *
 * @par 流程图:
 * @startuml
 * start
 * if (是否已初始化?) then (否)
 *   :返回false;
 * else (是)
 *   :查询最近道路段;
 *   if (是否找到最近道路段?) then (否)
 *     :返回false;
 *   else (是)
 *     :存储最近道路段指针;
 *     :返回true;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于查询距离给定位姿最近的道路段，适用于导航和规划
 *
 * @warning 调用该函数前需确保LocalMap已正确初始化
 */
bool LocalMap::getNearestRoadSegment(const math::Arrow2d& pose, LocalMapRoadSegment** nearest_road_seg) {
  if (!is_inited_) {
    return false;
  }
  *nearest_road_seg =
      road_segment_tree_->GetNearestObject(math::Vec2d(pose.center_x(), pose.center_y()), pose.heading());
  if (!(*nearest_road_seg)) {
    return false;
  }
  return true;
}

/**
 * @brief 获取距离给定位姿最近的道路段ID
 * @details 通过调用getNearestRoadSegment函数获取最近的道路段，并返回其ID
 *
 * @par 输入参数说明:
 * - pose (math::Arrow2d): 目标位姿，包含位置和方向信息
 *
 * @par 输出参数说明:
 * - 返回值 (std::string): 最近道路段的ID，若未找到则返回空字符串
 *
 * @par 关键变量说明:
 * - nearest_road_seg (LocalMapRoadSegment*): 最近道路段的指针，用于获取道路段ID
 * - is_inited_ (bool): 初始化状态，若未初始化则返回空字符串
 *
 * @par 判断条件:
 * - 若未初始化，则返回空字符串
 * - 若未找到最近道路段，则返回空字符串
 *
 * @par 流程图:
 * @startuml
 * start
 * if (是否已初始化?) then (否)
 *   :返回空字符串;
 * else (是)
 *   :获取最近道路段指针;
 *   if (是否找到最近道路段?) then (否)
 *     :返回空字符串;
 *   else (是)
 *     :返回道路段ID;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数用于查询距离给定位姿最近的道路段ID，适用于导航和规划
 *
 * @warning 调用该函数前需确保LocalMap已正确初始化
 */
std::string LocalMap::getNearestRoadSegmentId(const math::Arrow2d& pose) {
  if (!is_inited_) {
    return "";
  }
  LocalMapRoadSegment* nearest_road_seg = nullptr;
  if (!getNearestRoadSegment(pose, &nearest_road_seg)) {
    return "";
  }
  return nearest_road_seg->getRoadSegmentId();
}

}  // namespace gpal::pnc::planning
