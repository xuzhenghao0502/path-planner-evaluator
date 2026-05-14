# OpenSpace ROS2 Bridge — 开发记录

## 项目概述

将既有的 `openspace_path_planner` C++ 算法库（Hybrid A\* + 自行车模型）封装为 ROS2 Humble 节点，在 Docker 中运行。核心思路：通过 **stub 头文件** 替代缺失的 GPAL 内部 SDK 依赖，不改动 reference_code 中的原始算法代码。

**GitHub**: `git@github.com:xuzhenghao0502/path-planner-evaluator.git` (develop 分支)

---

## 工作一：Phase 1 — 算法库编译与基础节点搭建

**Commit**: `9f47394`

### 1.1 Stub 头文件体系（77 个）

在 `openspace_ros2_bridge/stubs/` 下创建 77 个 stub 头文件，模拟 GPAL 内部 SDK 的 API，覆盖以下模块：

| 类别 | 模块 |
|------|------|
| 基础 | `base/log.h`, `base/singleton.h`, `base/status.h` |
| 数学 | `math/vec2d.h`, `math/box2d.h`, `math/polygon2d.h`, `math/line_segment2d.h`, `math/circle.h`, `math/math_utils.h`, `math/linear_interpolation.h`, `math/discrete_points_math.h` |
| 算法库 | `basic_algorithm_lib/basic_algorithm_lib.h`, `transfer.h`, `geometry_calculation.h`, `position_conversion.h` |
| 配置 | `config/vehicle_config.pb.h`, `config/freespace_config.pb.h`, `config/openspace/openspace_search.pb.h`, `config_manager/config_manager.h` |
| 世界模型 | `local_view/*`, `navigation_data/*`, `decision_data/*`, `path/*`, `speed/*`, `reference_line/*`, `center_lines/*`, `road_instance/*`, `memorized_route/*`, `adas_data/*`, `obstacle/*` |
| 规划器 | `openspace_path_planner/*`, `path_planner/*`, `ocp/*`, `path_bound_parser/*` |
| GPAL 接口 | `gpal-interface/*` (chassis, localization, perception, planning, map_server) |
| 工具 | `util/timer.h`, `util/task_handler.h`, `util/base_struct.h` |

### 1.2 关键补丁

- `reference_code/pnc_motion_planner/path_planner/include/openspace_path_planner/core/generator/arc_model.h` — 为 `ArcPathNodeKeyHash::operator()` 和 `ArcPathNodeKey` 比较运算符添加 `noexcept`（GCC 11 要求）
- `reference_code/pnc_motion_planner/path_planner/include/openspace_path_planner/core/manager/openspace_search_data.h` — 修复 `.cc` 后缀包含为 `.h`

### 1.3 CMake 构建

```cmake
# openspace_algorithm OBJECT 库（4 个 .cpp）
${WORLD_MODEL_ROOT}/local_view/src/Freespace.cpp
${PNC_MOTION_ROOT}/path_planner/src/openspace_path_planner/core/generator/rs_path.cpp
${PNC_MOTION_ROOT}/path_planner/src/openspace_path_planner/utils/openspace_tools.cpp
${PNC_MOTION_ROOT}/path_planner/src/openspace_path_planner/utils/geometry_utils.cpp

# planner_bridge_node 可执行文件
add_executable(planner_bridge_node src/planner_bridge_node.cpp src/freespace_adapter.cpp)
```

Include 路径顺序：**stubs/ 优先** → reference_code 路径 → Eigen3。

### 1.4 Docker 环境

- 容器: `ros2` (镜像 `ros2-humble-2204`)
- 依赖: `libfmt-dev`（额外安装）
- ROS2 包: `rclcpp`, `nav_msgs`, `visualization_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`, `Eigen3`

---

## 工作二：Phase 2-6 — 完整 ROS2 Bridge 系统

**Commit**: `979139e`（7 文件，+649/−55 行）

### 2.1 Planner Bridge Node（C++）

文件: `openspace_ros2_bridge/src/planner_bridge_node.cpp` (383 行)

**功能**:
- ROS2 参数声明与加载（vehicle / search / map 三类，共 20+ 个参数）
- 通过 `Singleton<ConfigManager>` 注入 `VehicleConfig` 和 `OpenspaceSearchConfig`
- 订阅:
  - `/start_pose` (`geometry_msgs/PoseStamped`) — 起点位姿
  - `/goal_pose` (`geometry_msgs/PoseStamped`) — 终点位姿
  - `/obstacle_polygons` (`visualization_msgs/MarkerArray`) — 障碍物多边形
- 服务: `~/plan_path` (`std_srvs/Trigger`) — 触发规划
- 发布:
  - `/trajectory_path` (`nav_msgs/Path`) — 规划路径
  - `/vehicle_boxes` (`visualization_msgs/MarkerArray`) — 车辆碰撞箱
  - `/planning_result` (`std_msgs/String`) — 规划结果摘要
- 碰撞校验: 规划前检查起终点是否在地图外/障碍物内，返回明确错误信息

**调用链路**:
```
onPlanPath() → buildFreespace() → FreespaceBuilder::build()
             → OpenspaceSearchData 填充
             → PathProviderBaseHAStar<ArcModel>::run()
             → publishPath() + publishVehicleBoxes()
```

### 2.2 Freespace Adapter

- `freespace_adapter.hpp` — `PolygonRegion` 结构体 + `FreespaceBuilder` 类（`init/addPolygon/clear/build`）
- `freespace_adapter.cpp` — 栅格化实现：对每个障碍物多边形，使用射线投射法（point-in-polygon）标记 `GridMap::setOccupy()`，最后调用 `updateGridMap()` 计算距离变换用于碰撞检测

### 2.3 Map Editor Node（Python）

文件: `openspace_ros2_bridge/scripts/map_editor_node.py` (283 行)

**功能**:
- 利用 RViz2 的 **"Publish Point"** 工具绘制多边形顶点
- 发布 `MarkerArray` 到 `/obstacle_polygons` 供 Planner 消费
- 发布可视化数据到 `/polygon_viz`（顶点球 + 线段边框）
- 7 个服务:
  | 服务 | 功能 |
  |------|------|
  | `~/finish_polygon` | 闭合当前多边形（≥3 个顶点） |
  | `~/undo_last_point` | 撤销最后一个顶点 |
  | `~/clear_all` | 清除所有多边形 |
  | `~/save_scene` | 保存场景到 JSON 文件 |
  | `~/load_scene` | 从 JSON 文件加载场景 |
  | `~/set_mode_obstacle` | 切换为障碍物模式（红色） |
  | `~/set_mode_free` | 切换为自由空间模式（绿色） |

JSON 场景格式:
```json
{
  "polygons": [
    {
      "vertices": [[x1,y1], [x2,y2], ...],
      "is_occupied": true
    }
  ]
}
```

### 2.4 启动与配置

- `launch/planner_system.launch.py` — 一键启动 map_editor + planner_bridge + rviz2
- `rviz/polygon_editor.rviz` — 预配置显示: Grid, Obstacle Polygons, Polygon Viz, Vehicle Boxes, Trajectory Path, PublishPoint 工具

---

## 已验证的功能

| # | 验证项 | 状态 |
|---|--------|------|
| 1 | openspace_algorithm 库编译链接 | ✓ |
| 2 | planner_bridge_node / map_editor_node 可执行文件安装 | ✓ |
| 3 | 自定义 ROS2 参数加载（vehicle/search/map） | ✓ |
| 4 | ConfigManager 注入 VehicleConfig / OpenspaceSearchConfig | ✓ |
| 5 | Planner Bridge 服务接口（`~/plan_path`） | ✓ |
| 6 | 缺少起终点时的错误提示 | ✓ |
| 7 | 起终点越界检测 | ✓ |
| 8 | 无障碍场景规划成功（0→10m，11 个路径点） | ✓ |
| 9 | 有障碍场景规划成功（绕行障碍物） | ✓ |
| 10 | Map Editor 7 个服务全部可用 | ✓ |
| 11 | 多边形绘制 → 发布 MarkerArray → Planner 接收 | ✓ |
| 12 | 场景保存/加载（JSON） | ✓ |
| 13 | 规划结果话题发布（Path + VehicleBoxes + Result） | ✓ |
| 14 | FreespaceBuilder API 完整性（栅格化 + 距离变换） | ✓ |

---

## 架构总览

```
┌─────────────────────────────────────────────────────────┐
│                      RViz2                              │
│  PublishPoint ──▶ /clicked_point                        │
│  MarkerArray ◀── /obstacle_polygons, /vehicle_boxes     │
│  Path        ◀── /trajectory_path                       │
└─────────────────────────────────────────────────────────┘
         │                              ▲
         ▼                              │
┌─────────────────┐          ┌──────────────────────┐
│  Map Editor     │          │  Planner Bridge       │
│  (map_editor_   │──Marker──▶  (planner_bridge_     │
│   node.py)      │  Array   │   node.cpp)           │
│                 │          │                       │
│  7 services:    │          │  ~/plan_path service  │
│  finish/undo/   │          │  Params: vehicle/     │
│  clear/save/    │          │  search/map           │
│  load/mode      │          │                       │
└─────────────────┘          └──────────┬────────────┘
                                        │
                          ┌─────────────▼────────────┐
                          │  FreespaceBuilder        │
                          │  (freespace_adapter)     │
                          │  Polygon → GridMap       │
                          └─────────────┬────────────┘
                                        │
                          ┌─────────────▼────────────┐
                          │  PathProviderBaseHAStar  │
                          │  <ArcModel>              │
                          │  Hybrid A* Search        │
                          └──────────────────────────┘
```

---

## 技术要点

1. **Stub 策略**: 用最小化 stub 头文件替换 GPAL SDK，避免侵入 reference_code
2. **Include 顺序**: `stubs/` 优先于 `reference_code`，确保复杂依赖头文件被 stub 覆盖
3. **GCC 11 兼容**: `std::unordered_map` 要求 hash functor 和 key comparison 有 `noexcept` 声明
4. **标准 ROS2 类型**: 避免 `rosidl_generate_interfaces`（Docker 中 CMake Python 扩展问题），使用 `std_srvs::Trigger` 等标准接口
5. **障碍物栅格化**: 射线投射法 + `GridMap::setOccupy()` + 距离变换（`updateGridMap`）

---

## 文件清单

```
openspace_ros2_bridge/
├── CMakeLists.txt                              # 构建定义
├── package.xml                                 # ROS2 包描述
├── include/openspace_ros2_bridge/
│   └── freespace_adapter.hpp                   # FreespaceBuilder 接口
├── src/
│   ├── planner_bridge_node.cpp                 # 规划桥接节点 (383 行)
│   └── freespace_adapter.cpp                   # Freespace 适配器 (66 行)
├── scripts/
│   └── map_editor_node.py                      # 地图编辑节点 (283 行)
├── launch/
│   └── planner_system.launch.py                # 系统启动文件
├── rviz/
│   └── polygon_editor.rviz                     # RViz2 配置
├── scenes/                                     # 场景 JSON 文件目录
├── stubs/                                      # 77 个 GPAL stub 头文件
├── msg/                                        # (未使用，废弃的自定义接口)
└── srv/                                        # (未使用，废弃的自定义接口)
```

---

**开发日期**: 2026-05-14  
**环境**: Docker (ros2-humble-2204), GCC 11, ROS2 Humble
