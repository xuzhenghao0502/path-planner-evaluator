# OpenSpace ROS2 Bridge

基于 Hybrid A\* 算法的停车场路径规划系统，提供 ROS2 接口与 RViz2 交互式地图编辑。

## 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                        RViz2                             │
│  工具栏: Publish Point → /clicked_point                   │
│  显示: /obstacle_polygons, /polygon_viz, /vehicle_boxes  │
│        /trajectory_path                                   │
└──────────────────────────────────────────────────────────┘
          │                              ▲
          ▼                              │
┌──────────────────┐          ┌─────────────────────────┐
│  Map Editor      │  Marker  │  Planner Bridge          │
│  (Python 节点)    │  Array   │  (C++ 节点)              │
│                  │─────────▶│                          │
│  多边形绘制工具    │          │  Hybrid A* 路径规划       │
│  场景保存/加载     │          │  参数化车辆/搜索配置      │
│  障碍物/自由空间   │          │  碰撞检测与错误反馈       │
└──────────────────┘          └─────────────────────────┘
```

---

## 环境要求

| 依赖 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble |
| C++ 编译器 | GCC 11+ |
| CMake | 3.14+ |
| Python | 3.10+ |
| 额外库 | libfmt-dev |

Docker 镜像: `ros2-humble-2204`

---

## 快速开始

### 1. 进入 Docker 容器

```bash
docker start ros2
docker exec -it ros2 bash
```

### 2. 编译

```bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws
colcon build --packages-select openspace_ros2_bridge
source install/setup.bash
```

### 3. 一键启动

```bash
ros2 launch openspace_ros2_bridge planner_system.launch.py
```

这将同时启动:
- **Static TF Publisher** — 发布 `map → odom` 静态坐标系（RViz2 Fixed Frame 依赖）
- **Map Editor Node** — 地图编辑节点
- **Planner Bridge Node** — 路径规划节点
- **RViz2** — 可视化界面（预配置 TopDownOrtho 俯视图）

---

## 使用指南

操作需要**两个终端**（都先 `docker exec -it ros2 bash` 并 source ROS2 环境）：

- **终端 A**：运行 `ros2 launch ...`，观察日志输出
- **终端 B**：执行服务调用和话题发布命令

> 如果 RViz2 工具栏的 **Publish Point** 工具前有红叉无法使用，改用终端 B 手动发 `/clicked_point` 话题即可，两种方式效果相同——参见本章末尾的命令行绘制方式。

### 第一步：绘制场景

RViz2 启动后，你会看到一个 40m×40m 的俯视地图。

**1.1 选择绘制模式**

| 操作 | 命令（在终端 B 执行） |
|------|------|
| 障碍物模式（红色） | `ros2 service call /map_editor_node/set_mode_obstacle std_srvs/srv/Trigger {}` |
| 自由空间模式（绿色） | `ros2 service call /map_editor_node/set_mode_free std_srvs/srv/Trigger {}` |

**1.2 绘制多边形**

**方式一（推荐）**：在 RViz2 工具栏中选择 **"Publish Point"** 工具，在地图上依次点击多边形顶点。每次点击，终端 A 会打印 `Added vertex` 日志。

**方式二（命令行，无 GUI 依赖）**：在终端 B 逐个发送顶点坐标：

```bash
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped '{point: {x: 4, y: -2, z: 0}}'
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped '{point: {x: 6, y: -2, z: 0}}'
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped '{point: {x: 6, y: 2, z: 0}}'
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped '{point: {x: 4, y: 2, z: 0}}'
```

当前绘制的多边形在 RViz2 中显示为虚线预览（半透明），顶点有小球标记。至少需要 **3 个顶点** 才能闭合。

**1.3 闭合多边形**

```bash
ros2 service call /map_editor_node/finish_polygon std_srvs/srv/Trigger {}
```

闭合后，多边形会出现在 `/obstacle_polygons` 话题上，Planner Bridge Node 将自动接收。

---

### 第二步：设置起终点

```bash
# 设置起点（位置 + 朝向）
ros2 topic pub --once /start_pose geometry_msgs/msg/PoseStamped \
  '{pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}'

# 设置终点
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{pose: {position: {x: 10, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}'
```

> **朝向说明**: `orientation` 使用四元数，`{x:0, y:0, z:0, w:1}` 表示朝向正东（0°）。朝向正北为 `{x:0, y:0, z:0.707, w:0.707}`（90°）。

---

### 第三步：执行规划

```bash
ros2 service call /planner_bridge_node/plan_path std_srvs/srv/Trigger {}
```

**成功响应**:
```
success=True, message='Path found: 11 points'
```

**失败响应**:
```
success=False, message='Start is out of map bounds'
success=False, message='Start or goal pose not set'
success=False, message='Planning failed: SEARCH_FAILED'
```

规划结果会自动发布到以下话题:
- `/trajectory_path` — 规划路径（绿色曲线）
- `/vehicle_boxes` — 每个路径点上的车辆碰撞箱（绿色半透明矩形）
- `/planning_result` — 结果摘要（`SUCCESS` 或 `FAILED`）

---

### 第四步：场景管理

| 操作 | 命令 |
|------|------|
| 撤销上一个顶点 | `ros2 service call /map_editor_node/undo_last_point std_srvs/srv/Trigger {}` |
| 清除所有多边形 | `ros2 service call /map_editor_node/clear_all std_srvs/srv/Trigger {}` |
| 保存场景 | `ros2 service call /map_editor_node/save_scene std_srvs/srv/Trigger {}` |
| 加载场景 | `ros2 service call /map_editor_node/load_scene std_srvs/srv/Trigger {}` |

场景文件保存在 `openspace_ros2_bridge/scenes/scene.json`（可通过 ROS2 参数 `scene_dir` 和 `scene_name` 修改路径）。

**JSON 格式**:
```json
{
  "polygons": [
    {
      "vertices": [[1.0, 1.0], [5.0, 1.0], [5.0, 4.0], [1.0, 4.0]],
      "is_occupied": true
    },
    {
      "vertices": [[7.0, 2.0], [9.0, 2.0], [9.0, 5.0], [7.0, 5.0]],
      "is_occupied": false
    }
  ]
}
```

---

## 参数配置

### 车辆参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vehicle.length` | 4.8 | 车辆长度 (m) |
| `vehicle.width` | 1.9 | 车辆宽度 (m) |
| `vehicle.wheel_base` | 2.8 | 轴距 (m) |
| `vehicle.rear_overhang` | 0.9 | 后悬长度 (m) |
| `vehicle.rear_edge_to_ego` | 3.5 | 后轴中心到后缘距离 (m) |
| `vehicle.width_without_rearview_mirror` | 1.9 | 不含后视镜的车宽 (m) |

### 搜索参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `search.step_size` | 0.3 | 搜索步长 (m) |
| `search.steering_radian` | 0.4 | 最大转向角 (rad) |
| `search.steering_angle_discrete_num` | 1 | 转向角离散数量 |
| `search.search_time_limit` | 10.0 | 搜索时间上限 (s) |
| `search.shot_distance` | 8.0 | 目标逼近距离 (m) |
| `search.angular_upper` | 300 | 角度分辨率 |
| `search.map_grid_resolution` | 1.0 | 碰撞检测网格分辨率 (m) |
| `search.min_path_length_limit` | 1.0 | 最短路径约束 (m) |
| `search.steering_penalty` | 1.0 | 转向代价权重 |
| `search.changing_gear_penalty` | 10.0 | 换挡代价权重 |
| `search.width_collision_buff` | 0.1 | 宽度碰撞缓冲 (m) |
| `search.length_collision_buff` | 0.1 | 长度碰撞缓冲 (m) |

### 地图参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map.resolution` | 0.1 | 栅格地图分辨率 (m) |
| `map.size_x` | 40.0 | 地图 X 方向尺寸 (m) |
| `map.size_y` | 40.0 | 地图 Y 方向尺寸 (m) |
| `map.origin_x` | 20.0 | 地图原点 X 偏移 (m) |
| `map.origin_y` | 20.0 | 地图原点 Y 偏移 (m) |

### 命令行覆盖参数示例

```bash
ros2 run openspace_ros2_bridge planner_bridge_node \
  --ros-args \
  -p vehicle.length:=5.0 \
  -p vehicle.width:=2.0 \
  -p search.step_size:=0.25 \
  -p search.steering_radian:=0.5 \
  -p search.search_time_limit:=30.0
```

---

## 话题一览

### 输入话题（Planner Bridge 订阅）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/start_pose` | `geometry_msgs/PoseStamped` | 规划起点 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 规划终点 |
| `/obstacle_polygons` | `visualization_msgs/MarkerArray` | 障碍物多边形（由 Map Editor 发布） |

### 输入话题（Map Editor 订阅）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz2 Publish Point 点击坐标 |

### 输出话题

| 话题 | 类型 | 节点 | 说明 |
|------|------|------|------|
| `/trajectory_path` | `nav_msgs/Path` | Planner | 规划路径 |
| `/vehicle_boxes` | `visualization_msgs/MarkerArray` | Planner | 车辆碰撞箱 |
| `/planning_result` | `std_msgs/String` | Planner | 规划结果摘要 |
| `/obstacle_polygons` | `visualization_msgs/MarkerArray` | Map Editor | 障碍物多边形 |
| `/polygon_viz` | `visualization_msgs/MarkerArray` | Map Editor | 绘图预览（顶点+边框） |
| `/map_editor/mode` | `std_msgs/String` | Map Editor | 当前绘制模式 |

---

## 服务一览

### Planner Bridge Node (`/planner_bridge_node/`)

| 服务 | 类型 | 说明 |
|------|------|------|
| `~/plan_path` | `std_srvs/Trigger` | 触发路径规划 |

### Map Editor Node (`/map_editor_node/`)

| 服务 | 类型 | 说明 |
|------|------|------|
| `~/finish_polygon` | `std_srvs/Trigger` | 闭合当前多边形 |
| `~/undo_last_point` | `std_srvs/Trigger` | 撤销上一个顶点 |
| `~/clear_all` | `std_srvs/Trigger` | 清除所有多边形 |
| `~/save_scene` | `std_srvs/Trigger` | 保存场景到 JSON |
| `~/load_scene` | `std_srvs/Trigger` | 从 JSON 加载场景 |
| `~/set_mode_obstacle` | `std_srvs/Trigger` | 切换为障碍物模式 |
| `~/set_mode_free` | `std_srvs/Trigger` | 切换为自由空间模式 |

---

## RViz2 显示配置

预配置文件 `rviz/polygon_editor.rviz` 已包含以下显示项:

| 显示项 | 话题 | 效果 |
|--------|------|------|
| Grid | — | 参考网格 |
| Obstacle Polygons | `/obstacle_polygons` | 红色=障碍物，绿色=自由空间 |
| Polygon Viz | `/polygon_viz` | 顶点球标记 + 多边形边框 |
| Vehicle Boxes | `/vehicle_boxes` | 绿色半透明车辆长方形 |
| Trajectory Path | `/trajectory_path` | 绿色路径曲线 |

视角默认使用 **TopDownOrtho（俯视正交）**，缩放级别 30。工具栏预置 **Publish Point** 工具用于多边形绘制。

---

## 完整工作流示例（命令行方式）

以下在 **终端 B** 中依次执行，配合 **终端 A** 中运行的 Launch 系统：

```bash
# 1. 障碍物模式
ros2 service call /map_editor_node/set_mode_obstacle std_srvs/srv/Trigger {}

# 2. 逐个发送 4 个顶点（画一个 2m×4m 的矩形障碍物）
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped '{point: {x: 4, y: -2, z: 0}}'
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped '{point: {x: 6, y: -2, z: 0}}'
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped '{point: {x: 6, y: 2, z: 0}}'
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped '{point: {x: 4, y: 2, z: 0}}'

# 3. 闭合多边形
ros2 service call /map_editor_node/finish_polygon std_srvs/srv/Trigger {}

# 4. 设置起点（朝向正东）
ros2 topic pub --once /start_pose geometry_msgs/msg/PoseStamped \
  '{pose: {position: {x: 0, y: 0}, orientation: {w: 1}}}'

# 5. 设置终点
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{pose: {position: {x: 10, y: 0}, orientation: {w: 1}}}'

# 6. 执行规划
ros2 service call /planner_bridge_node/plan_path std_srvs/srv/Trigger {}

# 7. （可选）保存场景
ros2 service call /map_editor_node/save_scene std_srvs/srv/Trigger {}
```

---

## 错误处理

系统在以下情况会返回明确的错误信息:

| 错误 | 响应消息 | 含义 |
|------|----------|------|
| 缺少起终点 | `Start or goal pose not set` | 需要先发布 `/start_pose` 和 `/goal_pose` |
| 起点越界 | `Start is out of map bounds` | 起点超出 40m×40m 地图范围 |
| 终点越界 | `Goal is out of map bounds` | 终点超出地图范围 |
| 碰撞 | `Start/Goal is in collision with obstacle` | 起/终点与障碍物碰撞 |
| 搜索失败 | `Planning failed: SEARCH_FAILED` | A\* 搜索无法找到可行路径 |

---

## 目录结构

```
openspace_ros2_bridge/
├── CMakeLists.txt                           # 构建定义
├── package.xml                              # ROS2 包元数据
├── include/openspace_ros2_bridge/
│   └── freespace_adapter.hpp                # FreespaceBuilder 接口
├── src/
│   ├── planner_bridge_node.cpp              # 规划桥接节点（C++）
│   └── freespace_adapter.cpp                # 障碍物栅格化适配器
├── scripts/
│   └── map_editor_node.py                   # 地图编辑节点（Python）
├── launch/
│   └── planner_system.launch.py             # 一键启动文件
├── rviz/
│   └── polygon_editor.rviz                  # RViz2 窗口配置
├── scenes/                                  # 场景 JSON 文件
├── stubs/                                   # GPAL SDK 桩头文件（77 个）
├── msg/                                     # 废弃
└── srv/                                     # 废弃
```
