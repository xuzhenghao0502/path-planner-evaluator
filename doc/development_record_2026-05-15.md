# 开发记录 — 2026-05-15

## 一、系统架构概览

```
┌─────────────────────────────────────────────────────────────────┐
│                         RViz2 (GUI)                              │
│  Tools: Interact / PublishPoint / SetInitialPose / SetGoal      │
│  Displays: /obstacle_polygons, /polygon_viz, /vehicle_boxes,    │
│            /trajectory_path, /start_goal_viz                     │
└──────┬──────────────┬──────────────────┬────────────────────────┘
       │ /clicked_point│ /initialpose     │ /goal_pose
       ▼               ▼                  ▼
┌──────────────┐  ┌─────────────────────────────────────────────┐
│ map_editor   │  │          planner_bridge_node (C++)           │
│ _node (Py)   │  │  ┌─────────────┐  ┌──────────────────────┐  │
│              │  │  │ConfigManager│  │ OpenspacePathProvider │  │
│ - 多边形绘制  │  │  │(单例,车辆参数│  │ (Hybrid A* + RS扩展) │  │
│ - 场景加载/保存│  │  │ +搜索配置)  │  │                      │  │
│ - 可视化发布  │  │  └──────┬──────┘  └──────────┬───────────┘  │
│              │  │         │                      │              │
│  ↓ publish   │  │         ▼                      ▼              │
│ /obstacle_   │  │  ┌──────────────┐  ┌──────────────────────┐  │
│ polygons     │  │  │FreespaceBuilder│  │  Freespace (GridMap) │  │
│ /polygon_viz │  │  │(多边形→栅格)  │  │  (距离变换+碰撞检测)  │  │
└──────┬───────┘  │  └──────────────┘  └──────────────────────┘  │
       │          └─────────────────────────────────────────────┘
       │                          │
       │          /vehicle_boxes  │  /trajectory_path
       │          /start_goal_viz │  /planning_result
       ▼                          ▼
┌──────────────────────────────────────┐
│           RViz2 Visualization         │
└──────────────────────────────────────┘
```

### 核心文件

| 文件 | 职责 |
|------|------|
| `scripts/map_editor_node.py` | 多边形编辑器：手动绘制、场景文件加载/保存、可视化 |
| `src/planner_bridge_node.cpp` | 规划器桥接：ROS2订阅/发布、参数加载、Freespace构建、规划调用 |
| `src/freespace_adapter.cpp` | Freespace构建器：多边形光栅化（射线投射法）→障碍物栅格 |
| `stubs/config/vehicle_config.pb.h` | 车辆参数：长宽、轴距、前后悬、ego参考点偏移 |
| `stubs/config/openspace/openspace_search.pb.h` | 搜索参数：步长、最大转角、网格分辨率、碰撞缓冲等 |
| `launch/planner_system.launch.py` | 启动文件：启动4个节点（TF发布、地图编辑器、规划器、RViz） |
| `scenes/customer_map.json` | 场景文件：5个凸多边形定义的不可通行区域 |
| `rviz/polygon_editor.rviz` | RViz配置：显示面板、工具、视图 |

---

## 二、场景文件与不可通行区域

### 场景格式

```json
{
  "polygons": [
    {"vertices": [[x1,y1], [x2,y2], ...], "is_occupied": true/false}
  ]
}
```

- `is_occupied: true` → 红色，不可通行（障碍物）
- `is_occupied: false` → 绿色，可通行（自由空间）
- 多边形必须为凸多边形，凹多边形需拆分为多个凸多边形

### 当前场景 (customer_map.json)：5个不可通行多边形

| 编号 | 区域 | 顶点范围 | 说明 |
|------|------|----------|------|
| 1 | 左侧矩形 | (0,0)→(5,39) | 5m×39m 障碍墙 |
| 2 | C形底横条 | (10.4,0)→(40.4,13) | 30m×13m |
| 3 | C形中右竖条 | (30.4,13)→(40.4,26) | 10m×13m |
| 4 | C形顶横条 | (10.4,26)→(40.4,39) | 30m×13m |
| 5 | 切口小方块 | (24.4,17)→(26.4,19) | 2m×2m |

C形区域的切口（可通行空间）：`x∈[10.4, 30.4], y∈[13, 26]`
左侧墙与C形之间的间隙：`x∈[5, 10.4]`（宽度 5.4m）

---

## 三、障碍物→不可通行空间 完整链路

```
场景JSON文件 ──▶ map_editor_node._auto_load_scene()
                      │
                      ▼
              self.polygons (内存)
                      │
              _publish_all() 每1秒定时 + 点击事件触发
                      │
          ┌───────────┴───────────┐
          ▼                       ▼
  /polygon_viz              /obstacle_polygons
  (RViz可视化)              (MarkerArray: LINE_STRIP)
                                   │
                    planner_bridge_node 订阅回调
                                   │
                    polygons_ = *msg  (存储)
                                   │
                    ~/plan_path 服务触发
                                   │
                    buildFreespaceFromPolygons()
                                   │
                    FreespaceBuilder:
                      1. init(resolution, size, origin)
                      2. addPolygon(region) × N  ← 颜色R>0.5=障碍物
                      3. build():
                         - 射线投射法(PIP)遍历bounding box
                         - 步长 res*0.5 子像素采样
                         - grid_map->setOccupy(x, y) 标记占据
                         - freespace->updateGridMap() 计算距离变换
                                   │
                                   ▼
                    Hybrid A* + RS路径扩展
                      pointCollisonCheck():
                        - Box中心 = ego_point + shift*方向
                        - geneCheckCircles: 中线圆+角圆→距离图查询
```

### 关键参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `map.resolution` | 0.1m | 栅格分辨率 |
| `map.size_x / size_y` | 55m | 地图尺寸 |
| `map.origin_x` | 5.0 | X偏移（地图左下角世界坐标X） |
| `map.origin_y` | 50.0 | Y偏移（地图左下角世界坐标Y） |
| `search.map_grid_resolution` | 1.0m | A*搜索网格分辨率 |
| `search.step_size` | 0.3m | 运动基元步长 |
| `search.steering_radian` | 0.535rad | 最大前轮转角 |
| `search.width_collision_buff` | 0.1m | 碰撞宽度缓冲 |
| `search.length_collision_buff` | 0.1m | 碰撞长度缓冲 |

---

## 四、车型参数

| 参数 | 值 | 文件位置 |
|------|-----|----------|
| `length` | 12.0m | `vehicle_config.pb.h` / `planner_bridge_node.cpp:96` |
| `width` | 2.6m | 同上 |
| `width_without_rearview_mirror` | 2.6m | 同上 |
| `wheel_base` | 7.1m | 同上 |
| `front_overhang` | 1.46m | 同上 |
| `rear_overhang` | 3.33m | 同上 |
| `rear_edge_to_ego` | 3.5m | 同上 |

### 最小转弯半径

```
R_min = wheel_base / tan(steering_radian)
      = 7.1 / tan(0.535)
      ≈ 12.0m
```

### 碰撞检测模型

`geneCheckCircles()` 对车辆矩形生成覆盖圆：
- **中线圆**：沿车辆中线排列，半径 = 0.554 × 车宽 ≈ 1.44m。数量 = max(1, ceil((车长-车宽)/step) + 1) ≈ 9个（12m车长）
- **角圆**：4个角圆，半径 ≈ 0.48m，用于覆盖矩形四角超出中线圆的部分
- **总计 ~13个圆**覆盖整个车辆矩形

---

## 五、走过的坑（按时间顺序）

### 坑 #1: PathPt 构造函数参数顺序错误 ✅ 已修复
- **现象**：RS路径曲率始终为0，方向角始终90°
- **根因**：`PathPt(x, y, theta, kappa, s)` 构造函数，调用时传了 `(x, y, 0, 0, theta)` 把theta放到了kappa位置，theta填了0
- **修复**：`653cb67` — 改为 `PathPt(x, y, temp_pose.z(), current_kappa, 0.0)`

### 坑 #2: Freespace::init 未设置 init_ 标志 ✅ 已修复
- **现象**：障碍物多边形光栅化后立即被清空，规划器感知不到障碍物
- **根因**：`Freespace::init(config)` 未设 `init_=true`，后续 `updateGridMap()` → `Init()` 使用默认配置重新初始化GridMap，清空了之前 `setOccupy()` 的所有标记
- **修复**：`6d827e0` — 在 `Freespace::init(const FreespaceConfig&)` 末尾添加 `init_ = true`

### 坑 #3: JSON浮点数类型错误 ✅ 已修复
- **现象**：`ros2 launch` 报错 "The 'x' field must be of type 'float'"
- **根因**：场景JSON中多边形顶点使用了整数（0, 5, 39...），geometry_msgs/Point要求float
- **修复**：`e51bf6d` — 全部改为浮点（0.0, 5.0, 39.0...）

### 坑 #4: 凹多边形的填充可视化错误 ✅ 已修复
- **现象**：C形多边形在RViz中实心填充了切口区域，看起来像一个完整矩形（凸包外观）
- **根因**：`_make_fill_marker` 用质心三角化（triangle fan），对凹多边形会跨切口生成三角形。Ear clipping尝试失败（CCW整体缠绕导致切口边界处凸性判定混乱，ear检测全失败后fallback到顶点扇形）
- **修复**：`a6c6667` — 将C形凹多边形拆分为3个凸矩形（底横条、中右竖条、顶横条），覆盖相同区域，质心三角化正确

### 坑 #5: Vehicle Box 显示位置偏移 ✅ 已修复
- **现象**：轨迹上Vehicle Box看起来与红色不可通行区域碰撞，位置偏后约2.5m
- **根因**：路径点 `(x, y)` 是自车参考点（ego point，距车尾3.5m），不是车辆几何中心。碰撞检测中 box 中心偏移了 `shift = length/2 - rear_edge_to_ego = 2.5m` 到正确位置，但 `publishVehicleBoxes` 可视化直接将 box 画在 `(x, y)`，导致显示位置偏差2.5m
- **修复**：`8447c36` — 可视化 box 中心改为 `(x + shift*cos(theta), y + shift*sin(theta))`，与碰撞检测一致。同时修正了起止点碰撞校验中的相同问题

### 坑 #6: 场景多边形在RViz中不显示 ✅ 已修复
- **现象**：场景文件加载后，RViz中没有显示红色多边形边框
- **根因**：`_auto_load_scene()` 在 `__init__` 中调用 `_publish_all()`，此时节点尚未spin，首次publish可能在RViz订阅之前丢失
- **修复**：`8fc898c` — 添加1Hz定时器 `_on_timer()` 持续重发MarkerArray，确保晚订阅者也能收到

### 经验教训
1. **ROS2 publisher在spin之前publish可能丢失** → 使用定时器重发是稳健做法
2. **凹多边形填充不能依赖质心三角化** → 要么实现ear clipping（注意全局缠绕与局部切口方向差异），要么场景层面拆为凸多边形
3. **路径点坐标系必须明确参考点含义** → ego point ≠ vehicle center，可视化/碰撞检测必须统一使用相同偏移量
4. **GridMap的init/update流程有状态依赖** → `init_` 标志决定是否重新初始化，漏设会导致数据丢失

### 坑 #7: Docker容器与宿主机代码不同步 ✅ 已解决
- **现象**：宿主机修改了代码（特别是Python脚本），但容器中执行时用的仍是旧代码，或改了C++代码后 `colcon build` 报错找不到文件
- **根因**：
  1. 本项目的Docker容器 `ros2-cc` 通过 `-v` 挂载了宿主机目录，Python脚本使用 `--symlink-install`，修改后容器中直接生效
  2. 但如果未设置 volume 挂载或使用了 `--copy-install`，容器内的代码是构建时的快照，与宿主机隔离
  3. C++ 源码修改后必须在容器内重新 `colcon build` 才能生效，直接运行不会更新
  4. 如果在宿主机 `git checkout` 切换分支后，容器内的构建产物可能与新代码不匹配，导致运行时错误
- **解决**：
  1. 确认 `docker run` 或 `docker-compose` 中挂载了项目目录：`-v /home/zhenghao/Program/first-cc:/root/ros2_ws/src/...`
  2. Python脚本修改后无需重建（--symlink-install），但需重启节点
  3. C++代码修改后必须在容器内重建：`docker exec ros2-cc bash -c "cd /root/ros2_ws && colcon build --packages-select openspace_ros2_bridge --symlink-install"`
  4. 养成习惯：修改代码前先确认当前在容器内还是宿主机操作，修改后立即在容器内验证

---

## 六、RViz2 操作指南

### 工具
| 工具 | 用途 | 快捷键 |
|------|------|--------|
| **Interact** (默认) | 平移/缩放视图 | 滚轮缩放，左键拖拽平移 |
| Publish Point | 点击绘制多边形顶点 | 单击添加顶点，靠近首点0.5m自动闭合 |
| 2D Pose Estimate | 设置规划起点 | |
| 2D Goal Pose | 设置规划终点 | |

### 规划流程
1. 启动：`ros2 launch openspace_ros2_bridge planner_system.launch.py`
2. 场景自动加载（`customer_map.json`），红色边框显示不可通行区域
3. 使用 "2D Pose Estimate" 在地图上点击设置起点（带方向箭头）
4. 使用 "2D Goal Pose" 设置终点
5. 调用服务触发规划：`ros2 service call /planner_bridge_node/plan_path std_srvs/srv/Trigger`
6. 轨迹显示为绿色路径，车辆盒子显示为空心绿线框

### 地图编辑器服务
| 服务 | 功能 |
|------|------|
| `~/save_scene` | 保存当前多边形到场景文件 |
| `~/load_scene` | 从文件重新加载场景 |
| `~/clear_all` | 清除所有多边形 |
| `~/finish_polygon` | 手动闭合当前多边形 |
| `~/undo_last_point` | 撤销上一个顶点 |
| `~/set_mode_obstacle` | 切换到障碍物模式（红色） |
| `~/set_mode_free` | 切换到自由空间模式（绿色） |

---

## 七、启动参数

所有参数均可在启动时覆盖：

```bash
ros2 launch openspace_ros2_bridge planner_system.launch.py \
  scene_name:=customer_map.json \
  rviz_config:=/path/to/custom.rviz
```

或直接传ROS2参数给规划器节点：

```bash
ros2 launch openspace_ros2_bridge planner_system.launch.py \
  planner_bridge_node:="{\"vehicle.length\": 10.0, \"search.steering_radian\": 0.6}"
```
