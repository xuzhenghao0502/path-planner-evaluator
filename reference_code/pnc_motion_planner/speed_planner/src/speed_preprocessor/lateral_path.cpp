/**
 * @file lateral_path.cpp
 * @brief 横向路径预处理器
 * @details 本类负责对原始路径进行离散化处理，生成可供速度规划使用的路径组。
 */
#include "speed_preprocessor/lateral_path.h"

namespace gpal::pnc::planning {
/**
 * @brief 生成离散化横向路径组（路径处理入口）
 * @param[in] path 原始参考路径（需非空）
 * @param[in] valid_length 有效路径长度（>=0.0）
 * @param[in] step 路径离散化步长（>0.0）
 * @return PathGroup 包含离散化路径的结构体
 * 
 * @par 输入参数说明:
 * | 参数          | 类型             | 取值范围      | 单位 | 说明                     |
 * |---------------|------------------|-------------|------|--------------------------|
 * | path          | DiscretizedPath | size >=1    | -    | 原始路径点集合           |
 * | valid_length  | double           | >=0.0       | 米   | 最大处理路径长度         |
 * | step          | double           | >0.0        | 米   | 路径离散化采样间隔       |
 * 
 * @par 处理流程:
 * @startuml
 :输入有效性检查;
 if (路径为空?) then (yes)
   :输出错误日志;
   :返回空结构体;
 else (no)
   :调用getPathGroup进行路径处理;
 endif
 :返回处理后的路径组;
 @enduml
 *
 * @note 核心功能特征:
 * 1. 路径有效性前置检查
 * 2. 支持路径长度截断(valid_length)
 * 3. 保证输出路径至少包含2个点
 *
 * @warning 需确保调用前已完成:
 * - 参考路径的预处理（坐标系转换等）
 * - 有效长度参数合理性校验
 */
PathGroup LateralPathProcessor::getDiscretizedPathGroup(const DiscretizedPath& path, const double& valid_length, const double& step, const double& expand_length) {
  PathGroup lateral_path_group;
  if (path.empty()) {
    ERT_PLOG_I << "ERROR: Empty lateral path" ;
    return lateral_path_group;
  }
  getPathGroup(path, valid_length, step, expand_length, lateral_path_group);
  // for(auto& pt : lateral_path_group.origin_path_){
  //   ERT_PLOG_I<<" lateral_path: x = "<<pt.x()<<" y = "<<pt.y()<<" s = "<<pt.s()<<" theta = "<<pt.theta();
  // }
  return lateral_path_group;
}

/**
 * @brief 生成离散化局部路径组（局部路径处理入口）
 * @param[in] path 原始局部路径（需非空）
 * @param[in] valid_length 有效路径长度（>=0.0）
 * @param[in] step 路径离散化步长（>0.0）
 * @return PathGroup 包含离散化路径的结构体
 * 
 * @par 输入参数说明:
 * | 参数          | 类型             | 取值范围      | 单位 | 说明                     |
 * |---------------|------------------|-------------|------|--------------------------|
 * | path          | DiscretizedPath | size >=1    | -    | 原始局部路径点集合       |
 * | valid_length  | double           | >=0.0       | 米   | 最大处理路径长度         |
 * | step          | double           | >0.0        | 米   | 路径离散化采样间隔       |
 * 
 * @par 处理流程:
 * @startuml
 :输入有效性检查;
 if (路径为空?) then (yes)
   :输出"Empty local path"错误;
   :返回空结构体;
 else (no)
   :调用getPathGroup进行路径处理;
 endif
 :返回处理后的路径组;
 @enduml
 *
 * @note 核心功能特征:
 * 1. 面向局部路径的特殊处理
 * 2. 继承getPathGroup核心处理逻辑
 * 3. 支持路径长度动态截断
 *
 * @warning 需确保调用前已完成:
 * - 局部路径坐标系对齐
 * - 有效长度参数与场景匹配
 */
PathGroup LateralPathProcessor::getLocalPathGroup(const DiscretizedPath& path, const double& valid_length, const double& step, const double& expand_length) {
  PathGroup local_path_group;
  if (path.empty()) {
    ERT_PLOG_I << "ERROR: Empty local path" ;
    return local_path_group;
  }
  getPathGroup(path, valid_length, step, expand_length, local_path_group);
  // for(auto& pt : local_path_group.origin_path_){
  //   ERT_PLOG_I<<" lateral_path: x = "<<pt.x()<<" y = "<<pt.y()<<" s = "<<pt.s()<<" theta = "<<pt.theta();
  // }
  return local_path_group;
}
/**
 * @brief 生成路径组核心处理逻辑（多阶段路径处理）
 * @param[in] path 原始路径数据
 * @param[in] valid_length 有效处理长度
 * @param[in] step 路径采样步长
 * @param[out] path_group 输出路径组结构体
 * 
 * @par 输入参数说明:
 * | 参数          | 类型             | 取值范围      | 单位 | 说明                     |
 * |---------------|------------------|-------------|------|--------------------------|
 * | valid_length  | double           | >=0.0       | 米   | 路径最大处理长度         |
 * | step          | double           | >0.0        | 米   | 采样间隔                 |
 * 
 * @par 处理流程:
 * @startuml
 partition 路径采样 {
   :清空现有路径数据;
   :按step采样原始路径;
   if (采样点<2?) then (yes)
     :使用原始路径;
   endif
 }
 partition 关键点检测 {
   :累计角度变化检测关键点;
   :记录分割线段索引对;
 }
 partition 线段生成 {
   :基于关键点生成线段;
   :构建路径扩展结构;
 }
 partition 路径扩展 {
   :向前后无限方向扩展路径;
   :构建扩展路径插值结构;
 }
 @enduml
 *
 * @note 核心算法参数:
 * - 关键点角度阈值: 1.17弧度(~67度)
 * - 路径扩展长度: 100米
 * - 线段插值方式: 线性插值
 *
 * @warning 需确保调用前已完成:
 * - 路径方向标记设置（前进/倒车）
 * - 车辆参数配置加载
 */
void LateralPathProcessor::getPathGroup(const DiscretizedPath& path, const double& valid_length, const double& step, const double& expand_length, PathGroup& path_group) {
  path_group.clear();
  for (double s = 0.0; s <= min(path.back().s(),valid_length); s += step) {
    path_group.origin_path_.emplace_back(path.evaluate(s));
  }
  if (path_group.origin_path_.size() < 2) {
    path_group.origin_path_ = path;
    if (path_group.origin_path_.size() < 2) {
      ERT_PLOG_I << "Fail to get params because of too few path points. path points size: " << path_group.origin_path_.size()
           ;
      return;
    }
  }

  double segment_delta_theta_thres = 1.17F;
  double delta_theta = 0.0;

  // Initial critical_path_pt
  PathPt last_path_pt = path_group.origin_path_.front();

  // The index pairs of segments in path_
  std::vector<std::pair<size_t, size_t>> index_pairs_of_segments;

  // Get the whole segmentation path points
  for (size_t i = 0UL; i < path_group.origin_path_.size(); i++) {
    bool is_critical_path_point = false;
    delta_theta += std::abs(math::NormalizeAngle(path_group.origin_path_[i].theta() - last_path_pt.theta()));
    is_critical_path_point =
        (delta_theta >= segment_delta_theta_thres) || (i == 0UL) || (i + 1UL == path_group.origin_path_.size());
    if (is_critical_path_point) {
      path_group.path_points_for_segmentation_.emplace_back(path_group.origin_path_[i]);
      delta_theta = 0.0;
      if ((i > 0UL) && (index_pairs_of_segments.empty())) {
        index_pairs_of_segments.emplace_back(std::pair<size_t, size_t>(0, i));
      } else if (i > 0UL) {
        size_t last_piese_end_pt_idx = index_pairs_of_segments.back().second;
        index_pairs_of_segments.emplace_back(std::pair<size_t, size_t>(last_piese_end_pt_idx, i));
      } else {
        ;
      }
    }
    last_path_pt = path_group.origin_path_[i];
  }
  // Get the line-segments:
  for (size_t i = 0UL; i + 1UL < path_group.path_points_for_segmentation_.size(); i++) {
    math::Vec2d start(path_group.path_points_for_segmentation_[i].x(), path_group.path_points_for_segmentation_[i].y());
    math::Vec2d end(path_group.path_points_for_segmentation_[i + 1UL].x(),
                    path_group.path_points_for_segmentation_[i + 1UL].y());
    math::LineSegment2d line_segment(start, end);
    path_group.line_segments_for_path_segmentation_.emplace_back(line_segment);
  }

  double front_theta = path_group.path_points_for_segmentation_.front().theta();
  double back_theta = path_group.path_points_for_segmentation_.back().theta();
  if (path.front().direction() == PathPt::Direction::BACKWARD) {
    front_theta = math::NormalizeAngle(front_theta - M_PI);
    back_theta = math::NormalizeAngle(back_theta - M_PI);
  }

  PathPt backward_infty_pt(
      path_group.path_points_for_segmentation_.front().x() + std::cos(front_theta) * (-expand_length),
      path_group.path_points_for_segmentation_.front().y() + std::sin(front_theta) * (-expand_length),
      path_group.path_points_for_segmentation_.front().z(),
      path_group.path_points_for_segmentation_.front().slope(),
      path_group.path_points_for_segmentation_.front().theta());
  backward_infty_pt.set_s(-expand_length);
  path_group.path_points_for_segmentation_after_extand_.emplace_back(backward_infty_pt);

  for (const auto& pt : path_group.path_points_for_segmentation_) {
    path_group.path_points_for_segmentation_after_extand_.emplace_back(pt);
  }

  PathPt forward_infty_pt(path_group.path_points_for_segmentation_.back().x() + std::cos(back_theta) * expand_length,
                          path_group.path_points_for_segmentation_.back().y() + std::sin(back_theta) * expand_length,
                          path_group.path_points_for_segmentation_.back().z(),
                          path_group.path_points_for_segmentation_.back().slope(),
                          path_group.path_points_for_segmentation_.back().theta());
  forward_infty_pt.set_s(path_group.path_points_for_segmentation_after_extand_.back().s() + expand_length);
  path_group.path_points_for_segmentation_after_extand_.emplace_back(forward_infty_pt);

  DiscretizedPath extand_path;
  extand_path.emplace_back(backward_infty_pt);
  extand_path.insert(extand_path.end(), path_group.origin_path_.begin(), path_group.origin_path_.end());
  extand_path.emplace_back(forward_infty_pt);
  path_group.extand_interval_path_ = intervalPath(extand_path, step);

}

}  // namespace gpal::pnc::planning