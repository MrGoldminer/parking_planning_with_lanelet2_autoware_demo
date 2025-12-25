# ROS 自动泊车演示系统

> 基于 OpenStreetMap 的自主泊车路径规划与控制系统

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![C++](https://img.shields.io/badge/C++-14-orange.svg)](https://en.cppreference.com/w/cpp/14)
[![License](https://img.shields.io/badge/license-TODO-lightgrey.svg)]()

---

## 目录

- [项目简介](#项目简介)
- [系统架构](#系统架构)
- [环境依赖](#环境依赖)
- [安装步骤](#安装步骤)
- [参数配置](#参数配置)
- [魔法函数](#魔法函数)
- [使用指南](#使用指南)
- [可视化](#可视化)
- [故障排除](#故障排除)
- [开发指南](#开发指南)

---

## 项目简介

这是一个**完整的 ROS 自动泊车演示系统**，使用真实的 OpenStreetMap (OSM) 停车场数据进行路径规划和泊车轨迹生成。系统实现了从停车场入口导航到停车位，并执行精确的自动泊车入位。

### 主要特性

- **真实地图集成**: 使用 OSM 格式的真实停车场地图数据
- **多层级规划**: 节点级、关系级（车道级）路径规划
- **约束感知泊车**: 识别车位几何形状，生成符合物理约束的轨迹
- **Pure Pursuit 控制**: 经典轨迹跟踪算法，平滑稳定
- **RViz 可视化**: 完整的实时可视化支持
- **模块化架构**: 清晰的模块分离，易于扩展和维护

### 核心指标

| 指标 | 数值 |
|------|------|
| 代码总量 | ~6,845 行 |
| 控制频率 | 20 Hz |
| 最大速度 | 2.5 m/s (9 km/h) |
| 定位精度 | ±0.2 m |
| 支持车位类型 | 垂直泊车、平行泊车 |

---

## 系统架构

### 模块化设计

```
┌─────────────────────────────────────────────────────────┐
│                  Parking System Node                     │
└────────────────┬────────────────────────────────────────┘
                 │
    ┌────────────┼────────────────┐
    │            │                │
    ▼            ▼                ▼
┌────────┐  ┌─────────┐    ┌──────────┐
│OSM Map │  │  Graph  │    │   Path   │
│ Loader │─▶│ Builder │───▶│ Planner  │
└────────┘  └─────────┘    └─────┬────┘
                                  │
                                  ▼
                          ┌───────────────┐
                          │ Pure Pursuit  │
                          │  Controller   │
                          └───────┬───────┘
                                  │
    ┌─────────────────────────────┼─────────────┐
    │                             │             │
    ▼                             ▼             ▼
┌─────────┐              ┌──────────────┐  ┌────────┐
│ Parking │              │  Visualizer  │  │ Vehicle│
│Maneuver │              │   (RViz)     │  │  Model │
└─────────┘              └──────────────┘  └────────┘
```

### 核心模块

| 模块 | 文件 | 功能 | 代码行数 |
|------|------|------|----------|
| **OSM Map Loader** | `osm_map_loader.cpp` | 解析 OSM XML，坐标转换（WGS84→MGRS→Local） | 182 |
| **Graph Builder** | `graph_builder.cpp` | 构建节点级和关系级连通图 | 225 |
| **Path Planner** | `path_planner.cpp` | BFS 路径规划，中心线生成 | 425 |
| **Pure Pursuit Controller** | `pure_pursuit_controller.cpp` | 轨迹跟踪控制算法 | 225 |
| **Parking Maneuver** | `parking_maneuver.cpp` | 泊车轨迹生成（垂直、平行、约束感知） | 548 |
| **Visualizer** | `visualizer.cpp` | RViz 标记发布和 TF 广播 | 255 |

### 状态机

系统运行时在三个状态间切换：

```
   [APPROACHING]  ────→  [PARKING]  ────→  [COMPLETED]
        │                    │                   │
        │ Pure Pursuit       │ Parking Traj.     │ Goal Reached
        │ to Park Entrance   │ Execution         │ (tolerance < 0.2m)
```

---

## 环境依赖

### 系统要求

- **操作系统**: Ubuntu 20.04 LTS
- **ROS 版本**: ROS Noetic Ninjemys
- **编译器**: GCC 9.3+ (支持 C++14)
- **CMake**: 3.0.2+

### ROS 依赖包

#### 必需依赖

```bash
# ROS 核心库
sudo apt-get install ros-noetic-roscpp

# 消息类型
sudo apt-get install ros-noetic-geometry-msgs \
                     ros-noetic-nav-msgs \
                     ros-noetic-std-msgs \
                     ros-noetic-visualization-msgs

# TF 变换
sudo apt-get install ros-noetic-tf

# ROS 库函数
sudo apt-get install ros-noetic-roslib

# 构建工具
sudo apt-get install ros-noetic-catkin
```

#### 可选依赖

```bash
# OMPL 运动规划库（用于 Reeds-Shepp 曲线生成）
sudo apt-get install ros-noetic-ompl

# 如果不安装，系统会自动使用中心线插值后备方案
```

### Python 依赖（可视化工具）

```bash
sudo apt-get install ros-noetic-rviz
```

### 第三方库

| 库名 | 版本 | 用途 | 必需性 |
|------|------|------|--------|
| **Eigen3** | 3.3+ | 线性代数计算 | 可选（如使用 OMPL） |
| **OMPL** | 1.6 | Reeds-Shepp 路径规划 | 可选 |
| **TinyXML** | - | OSM XML 解析 | 必需（ROS 自带） |

---

## 安装步骤

### 1. 创建工作空间

```bash
# 创建 catkin 工作空间
mkdir -p ~/park_ws/src
cd ~/park_ws/src

# 克隆或复制本项目到 src 目录
# 假设项目已在 src/parking_demo
```

### 2. 安装依赖

```bash
# 方法 1: 使用 rosdep 自动安装
cd ~/park_ws
rosdep install --from-paths src --ignore-src -r -y

# 方法 2: 手动安装（见上面"环境依赖"部分）
```

### 3. 编译项目

```bash
cd ~/park_ws

# Release 模式编译（推荐）
catkin_make -DCMAKE_BUILD_TYPE=Release -j4

# Debug 模式编译（用于调试）
# catkin_make -DCMAKE_BUILD_TYPE=Debug
```

### 4. 配置环境

```bash
# 加载工作空间环境
source devel/setup.bash

# 永久添加到 .bashrc（可选）
echo "source ~/park_ws/devel/setup.bash" >> ~/.bashrc
```

### 5. 验证安装

```bash
# 检查节点是否成功编译
rospack find parking_demo

# 查看可执行文件
rospack list-executables parking_demo
# 应该输出:
# parking_system_refactored
# real_map_parking_node
```

---

## 参数配置

### 配置文件位置

- **启动文件**: `src/parking_demo/launch/parking.launch`
- **地图配置**: `src/parking_demo/maps/map_config.yaml`
- **坐标系统**: `src/parking_demo/maps/map_projector_info.yaml`

### 车辆参数 (Vehicle Parameters)

在 `parking.launch` 中配置：

```xml
<!-- 车辆物理参数 -->
<param name="vehicle_wheelbase" value="1.8" />        <!-- 轴距 (m) -->
<param name="vehicle_max_steering" value="0.6" />     <!-- 最大转向角 (rad, ≈34°) -->
<param name="vehicle_length" value="2.4" />           <!-- 车长 (m) -->
<param name="vehicle_width" value="1.4" />            <!-- 车宽 (m) -->
```

**调参建议**:

| 参数 | 典型值范围 | 说明 | 影响 |
|------|-----------|------|------|
| `wheelbase` | 1.5 - 3.0 m | 前后轴距离 | 影响转弯半径 |
| `max_steering` | 0.5 - 0.8 rad | 最大方向盘转角 | 影响最小转弯半径 |
| `length` | 2.0 - 5.0 m | 车辆总长 | 影响碰撞检测 |
| `width` | 1.2 - 2.0 m | 车辆宽度 | 影响车位适配性 |

### 控制器参数 (Controller Parameters)

```xml
<!-- Pure Pursuit 控制器参数 -->
<param name="lookahead_distance" value="2.5" />   <!-- 前瞻距离 (m) -->
<param name="kp_speed" value="1.0" />             <!-- 速度控制增益 -->
<param name="max_speed" value="2.5" />            <!-- 最大速度 (m/s) -->
<param name="min_speed" value="0.5" />            <!-- 最小速度 (m/s) -->
<param name="goal_tolerance" value="0.2" />       <!-- 目标容差 (m) -->
```

**调参建议**:

| 参数 | 典型值范围 | 说明 | 调参指南 |
|------|-----------|------|----------|
| `lookahead_distance` | 1.5 - 4.0 m | 前瞻点距离 | **减小**: 更精确跟踪，但易震荡<br>**增大**: 更平滑，但转弯响应慢 |
| `kp_speed` | 0.5 - 2.0 | 速度比例增益 | **减小**: 更平稳加速<br>**增大**: 更快响应 |
| `max_speed` | 1.0 - 5.0 m/s | 最高行驶速度 | 停车场环境建议 2-3 m/s |
| `min_speed` | 0.3 - 1.0 m/s | 最低行驶速度 | 避免过低导致停滞 |
| `goal_tolerance` | 0.1 - 0.5 m | 到达目标阈值 | **减小**: 更精确定位<br>**增大**: 更容易到达 |

### 规划器参数 (Planner Parameters)

```xml
<!-- 路径规划参数 -->
<param name="parking_back_distance" value="1.5" />  <!-- 泊车倒车距离 (m) -->
```

**高级参数**（在代码中 `types.h:130-134` 定义）:

```cpp
struct PlannerParams {
    double parking_back_distance = 2.0;  // 泊车倒车距离（米）
    double regulatory_Rmin = 8.0;        // GB 7258最小外轮半径（米）
    double dsafe = 1.0;                  // 安全余量（米）
};
```

### 地图配置

**`map_config.yaml`** - 地图原点（经纬度）

```yaml
map_origin:
  latitude: 35.238      # 北纬（度）
  longitude: 139.901    # 东经（度）
  elevation: 0.0        # 海拔（米）
  roll: 0.0             # 横滚角（度）
  pitch: 0.0            # 俯仰角（度）
  yaw: 0.0              # 偏航角（度）
```

**`map_projector_info.yaml`** - 坐标投影系统

```yaml
projector_type: MGRS         # 投影类型（军事格网参考系统）
vertical_datum: WGS84        # 垂直基准面
mgrs_grid: 54SVE             # MGRS 网格标识
mgrs_code: 54SVE8362019798   # 完整 MGRS 编码
```

### 硬编码关键 ID

在 `parking_system_node_refactored.cpp` 中硬编码的地图元素 ID：

```cpp
const long long START_LANELET_ID = 9259;   // 起始车道 ID
const long long PARK_LANELET_ID = 9265;    // 停车区域 ID
const long long LINESTR_ID_A = 9386;       // 车位长边 A
const long long LINESTR_ID_B = 9392;       // 车位长边 B
```

> **注意**: 更换地图时需要修改这些 ID！

---

## 魔法函数

这里列举系统中的核心算法函数（"魔法函数"），它们是系统智能行为的基础。

### 1. Pure Pursuit 转向角计算

**位置**: `pure_pursuit_controller.cpp:computeSteeringAngle()`

**原理**: 根据前瞻点和当前位置计算转向角

```cpp
/**
 * @brief Pure Pursuit 转向角计算（魔法公式）
 * @param current_state 当前车辆状态
 * @param lookahead_point 前瞻点 (x, y)
 * @param Ld 前瞻距离
 * @return 转向角 (rad)
 */
double computeSteeringAngle(const CarState& current_state,
                           const std::pair<double, double>& lookahead_point,
                           double Ld) {
    // 1. 计算前瞻点相对车辆的方位角
    double angle_to_target = std::atan2(
        lookahead_point.second - current_state.y,
        lookahead_point.first - current_state.x
    );

    // 2. 计算航向误差（alpha）
    double alpha = normalizeAngle(angle_to_target - current_state.theta);

    // 3. Pure Pursuit 魔法公式
    //    delta = atan(2 * L * sin(alpha) / Ld)
    //    其中 L = 轴距，Ld = 前瞻距离
    double steering = std::atan(
        2.0 * vehicle_params_.wheelbase * std::sin(alpha) / Ld
    );

    // 4. 限幅到最大转向角
    steering = std::clamp(steering,
                         -vehicle_params_.max_steering_rad,
                          vehicle_params_.max_steering_rad);

    return steering;
}
```

**数学原理**:
- 基于圆弧几何，车辆沿圆弧行驶到前瞻点
- `alpha`: 前瞻点与车辆航向的夹角
- `Ld`: 前瞻距离（圆弧弦长）
- `L`: 轴距

**调参技巧**:
- 前瞻距离 `Ld` 越小，跟踪越精确但易震荡
- 前瞻距离可动态调整: `Ld = base + k * v`（速度相关）

---

### 2. 阿克曼转向运动学模型

**位置**: `parking_maneuver.cpp:93-95`（在轨迹生成中使用）

**原理**: 根据当前状态和控制输入计算下一时刻状态

```cpp
/**
 * @brief 阿克曼转向运动学模型（魔法积分）
 * @param current 当前状态 (x, y, theta)
 * @param v 速度 (m/s)，负值表示倒车
 * @param steering 转向角 (rad)
 * @param dt 时间步长 (s)
 * @param wheelbase 轴距 (m)
 * @return 下一时刻状态
 */
CarState ackermanKinematics(const CarState& current,
                            double v,
                            double steering,
                            double dt,
                            double wheelbase) {
    CarState next;

    // 位置更新（欧拉积分）
    next.x = current.x + v * std::cos(current.theta) * dt;
    next.y = current.y + v * std::sin(current.theta) * dt;

    // 航向角更新（阿克曼转向公式）
    next.theta = current.theta + (v / wheelbase) * std::tan(steering) * dt;

    // 角度归一化到 [-π, π]
    while (next.theta > M_PI)  next.theta -= 2 * M_PI;
    while (next.theta <= -M_PI) next.theta += 2 * M_PI;

    next.v = v;
    next.phi = steering;

    return next;
}
```

**数学原理**:
- 车辆后轴中心的运动学方程
- 转向角 `δ` 通过前轮实现，转换为角速度: `ω = v/L * tan(δ)`

**倒车处理**:
- 倒车时 `v < 0`，航向角变化方向与前进相反

---

### 3. 约束感知泊车轨迹生成

**位置**: `parking_maneuver.cpp:generateConstraintAwareParking()`

**原理**: 两阶段倒车策略，满足车位几何约束

```cpp
/**
 * @brief 约束感知泊车（两阶段魔法）
 * Stage 1: 后退转向对齐车位方向
 * Stage 2: 最终定位，确保车辆完全进入车位
 */
std::vector<CarState> generateConstraintAwareParking(
    const CarState& start,
    const ParkingSlotConstraints& constraints) {

    std::vector<CarState> trajectory;

    // ========== Stage 1: 后退转向对齐 ==========
    // 目标：后退 1.5-2.0m，同时转向对齐车位方向
    double reverse_distance = 1.8;  // 倒车距离
    double target_theta = constraints.entry_direction + M_PI;  // 车头朝外

    // 计算所需转向角（根据角度差）
    double angle_diff = normalizeAngle(target_theta - start.theta);
    double R = vehicle_params_.wheelbase / std::tan(max_steering);
    double steering_sign = (angle_diff > 0) ? 1.0 : -1.0;
    double steering = steering_sign * max_steering;

    // 生成倒车轨迹（使用阿克曼模型）
    CarState current = start;
    double reverse_speed = -0.35;  // 倒车速度 0.35 m/s
    double dt = 0.05;

    while (distance_traveled < reverse_distance) {
        current = ackermanKinematics(current, reverse_speed, steering, dt, wheelbase);
        trajectory.push_back(current);

        // 碰撞检测：检查是否穿越车位长边
        if (checkLongEdgeCrossing(current, constraints)) {
            ROS_WARN("Stage 1: Collision with long edge!");
            break;
        }
    }

    // ========== Stage 2: 最终定位 ==========
    // 目标：从当前位置移动到车位中心
    double final_x = constraints.target_x;
    double final_y = constraints.target_y;
    double final_theta = constraints.target_theta;

    // 继续倒车或微调，直到到达目标
    while (distanceToGoal(current, final_x, final_y) > 0.1) {
        // 计算朝向目标的转向角
        double angle_to_goal = std::atan2(final_y - current.y,
                                         final_x - current.x);
        double heading_error = normalizeAngle(angle_to_goal - current.theta);

        // 简单比例控制
        double steering = std::clamp(2.0 * heading_error,
                                     -max_steering, max_steering);

        current = ackermanKinematics(current, reverse_speed, steering, dt, wheelbase);
        trajectory.push_back(current);

        // 验证车辆轮廓完全在车位内
        if (!isFootprintInside(current, constraints.slot_polygon)) {
            ROS_WARN("Stage 2: Vehicle footprint outside slot!");
        }
    }

    return trajectory;
}
```

**创新点**:
1. **长边识别**: 自动识别不可穿越的车位边界
2. **入口检测**: 找到最近的短边作为入口
3. **两阶段规划**: 先对齐方向，再精确定位
4. **碰撞检测**: 实时检查车辆轮廓与车位边界

---

### 4. BFS 最短路径搜索

**位置**: `path_planner.cpp:planNodePath()`

**原理**: 广度优先搜索找到节点级最短路径

```cpp
/**
 * @brief BFS 最短路径搜索（图搜索魔法）
 * @param start_node_id 起始节点 ID
 * @param goal_node_id 目标节点 ID
 * @param node_adjacency 邻接表
 * @return 节点 ID 序列（路径）
 */
std::vector<long long> bfsSearch(
    long long start_node_id,
    long long goal_node_id,
    const std::map<long long, std::vector<long long>>& node_adjacency) {

    // BFS 初始化
    std::queue<long long> open_queue;
    std::map<long long, long long> parent;  // 记录父节点（用于回溯）
    std::set<long long> visited;

    open_queue.push(start_node_id);
    visited.insert(start_node_id);
    parent[start_node_id] = -1;

    // BFS 主循环
    while (!open_queue.empty()) {
        long long current = open_queue.front();
        open_queue.pop();

        // 找到目标！
        if (current == goal_node_id) {
            // 回溯构建路径
            std::vector<long long> path;
            long long node = goal_node_id;
            while (node != -1) {
                path.push_back(node);
                node = parent[node];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // 扩展邻居节点
        if (node_adjacency.count(current)) {
            for (long long neighbor : node_adjacency.at(current)) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    parent[neighbor] = current;
                    open_queue.push(neighbor);
                }
            }
        }
    }

    // 未找到路径
    ROS_ERROR("BFS: No path found from %lld to %lld", start_node_id, goal_node_id);
    return {};
}
```

**时间复杂度**: O(V + E)，其中 V 是节点数，E 是边数

**优化点**:
- 使用 `std::queue` 实现 FIFO 队列
- 使用 `std::set` 快速查找已访问节点
- 父节点映射用于高效路径回溯

---

### 5. 车位边界碰撞检测

**位置**: `parking_maneuver.cpp:isFootprintInside()`

**原理**: 射线法（Ray Casting）判断点是否在多边形内

```cpp
/**
 * @brief 点在多边形内判断（射线法魔法）
 * @param point (x, y) 坐标
 * @param polygon 多边形顶点列表
 * @return true 如果点在多边形内
 */
bool isPointInPolygon(const std::pair<double, double>& point,
                     const std::vector<std::pair<double, double>>& polygon) {

    if (polygon.size() < 3) return false;

    int crossings = 0;  // 射线与多边形边的交点数
    double px = point.first;
    double py = point.second;

    // 从点发出水平向右的射线，计算与多边形边的交点
    for (size_t i = 0; i < polygon.size(); ++i) {
        size_t j = (i + 1) % polygon.size();

        double x1 = polygon[i].first;
        double y1 = polygon[i].second;
        double x2 = polygon[j].first;
        double y2 = polygon[j].second;

        // 判断射线是否与边 (i, j) 相交
        if (((y1 <= py && py < y2) || (y2 <= py && py < y1)) &&
            (px < (x2 - x1) * (py - y1) / (y2 - y1) + x1)) {
            crossings++;
        }
    }

    // 奇数次交点 => 在多边形内
    return (crossings % 2 == 1);
}

/**
 * @brief 车辆轮廓是否完全在车位内（扩展魔法）
 */
bool isFootprintInside(const CarState& state,
                       const std::vector<std::pair<double, double>>& polygon) {

    // 计算车辆四个角点的坐标
    double half_length = vehicle_params_.length / 2.0;
    double half_width = vehicle_params_.width / 2.0;

    std::vector<std::pair<double, double>> corners = {
        // 车辆坐标系中的四个角点
        { half_length,  half_width},   // 右前
        { half_length, -half_width},   // 左前
        {-half_length, -half_width},   // 左后
        {-half_length,  half_width}    // 右后
    };

    // 旋转并平移到世界坐标系
    for (auto& corner : corners) {
        double x_local = corner.first;
        double y_local = corner.second;

        // 旋转矩阵变换
        corner.first  = state.x + x_local * std::cos(state.theta)
                              - y_local * std::sin(state.theta);
        corner.second = state.y + x_local * std::sin(state.theta)
                              + y_local * std::cos(state.theta);
    }

    // 检查所有角点是否都在多边形内
    for (const auto& corner : corners) {
        if (!isPointInPolygon(corner, polygon)) {
            return false;  // 有角点在外面
        }
    }

    return true;  // 所有角点都在内部
}
```

**数学原理**:
- **射线法**: 从点发出水平射线，统计与多边形边的交点数
  - 奇数次 => 点在内部
  - 偶数次 => 点在外部

**车辆轮廓处理**:
1. 计算车辆四个角点（车辆坐标系）
2. 使用旋转矩阵转换到世界坐标系
3. 检查所有角点是否在车位多边形内

---

### 6. 坐标系转换（WGS84 → MGRS → Local）

**位置**: `osm_map_loader.cpp:projectToLocal()`

**原理**: 三级坐标转换链

```cpp
/**
 * @brief 经纬度转局部坐标（三级魔法转换）
 * @param lat 纬度（度）
 * @param lon 经度（度）
 * @param origin_lat 原点纬度（度）
 * @param origin_lon 原点经度（度）
 * @return (x, y) 局部坐标（米）
 */
std::pair<double, double> projectToLocal(double lat, double lon,
                                         double origin_lat, double origin_lon) {

    // ========== Stage 1: WGS84 → 平面近似 ==========
    // 使用等距圆柱投影（Equirectangular Projection）
    // 适用于小范围（< 10km）的局部地图

    const double EARTH_RADIUS = 6378137.0;  // 地球半径（米）
    const double DEG_TO_RAD = M_PI / 180.0;

    // 纬度差 => Y 方向（北方向）
    double delta_lat = (lat - origin_lat) * DEG_TO_RAD;
    double y = delta_lat * EARTH_RADIUS;

    // 经度差 => X 方向（东方向）
    // 需要按纬度修正（纬线长度随纬度变化）
    double delta_lon = (lon - origin_lon) * DEG_TO_RAD;
    double lat_rad = lat * DEG_TO_RAD;
    double x = delta_lon * EARTH_RADIUS * std::cos(lat_rad);

    return {x, y};
}
```

**精度**:
- 10km 范围内误差 < 1cm
- 100km 范围内误差 < 1m
- 适用于停车场（通常 < 500m）

**MGRS 网格**（用于大范围定位）:
- 军事格网参考系统，全球统一编码
- 本项目使用 `54SVE` 网格（日本地区）

---

### 7. 路径重采样（均匀化）

**位置**: `path_planner.cpp:resamplePath()`

**原理**: 线性插值生成等间距航点

```cpp
/**
 * @brief 路径重采样（插值魔法）
 * @param path 原始路径
 * @param interval 目标点间距（米）
 * @return 重采样后的路径
 */
std::vector<CarState> resamplePath(const std::vector<CarState>& path,
                                   double interval = 0.1) {

    if (path.size() < 2) return path;

    std::vector<CarState> resampled;
    resampled.push_back(path[0]);  // 起点

    double accumulated_dist = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].x - path[i-1].x;
        double dy = path[i].y - path[i-1].y;
        double segment_length = std::hypot(dx, dy);

        accumulated_dist += segment_length;

        // 当累计距离超过间隔，插入新点
        while (accumulated_dist >= interval) {
            // 在当前段上插值
            double ratio = (accumulated_dist - interval) / segment_length;

            CarState interpolated;
            interpolated.x = path[i].x - ratio * dx;
            interpolated.y = path[i].y - ratio * dy;
            interpolated.theta = std::atan2(dy, dx);  // 切线方向
            interpolated.v = 0.0;
            interpolated.phi = 0.0;

            resampled.push_back(interpolated);
            accumulated_dist -= interval;
        }
    }

    resampled.push_back(path.back());  // 终点

    return resampled;
}
```

**作用**:
- 控制器需要均匀航点密度（通常 0.1-0.2m 间隔）
- 过密 => 计算浪费
- 过疏 => 跟踪不精确

---

## 使用指南

### 快速启动

```bash
# 1. 启动 ROS Master
roscore

# 2. 在新终端启动停车系统（重构版）
source ~/park_ws/devel/setup.bash
roslaunch parking_demo parking.launch

# 3. 在新终端启动 RViz 可视化
rviz -d src/parking_demo/parking_demo.rviz
```

### 运行原始版本

```bash
# 编辑 launch 文件，替换节点
# <node pkg="parking_demo" type="real_map_parking_node" .../>

roslaunch parking_demo parking.launch
```

### 命令行直接运行

```bash
# 设置参数并运行
rosrun parking_demo parking_system_refactored \
    _vehicle_wheelbase:=1.8 \
    _max_speed:=2.0 \
    _lookahead_distance:=3.0
```

### 更换地图

1. **准备 OSM 地图文件**:
   - 使用 JOSM 或 OpenStreetMap 编辑器导出 `.osm` 文件
   - 确保包含 `node`, `way`, `relation` 标签

2. **配置地图原点**:
   ```bash
   vim src/parking_demo/maps/map_config.yaml
   ```
   修改 `latitude` 和 `longitude` 为地图中心坐标

3. **更新硬编码 ID**:
   ```bash
   vim src/parking_demo/src/parking_system_node_refactored.cpp
   ```
   修改第 30-33 行的 ID：
   ```cpp
   const long long START_LANELET_ID = YOUR_START_LANE_ID;
   const long long PARK_LANELET_ID = YOUR_PARKING_AREA_ID;
   const long long LINESTR_ID_A = YOUR_SLOT_EDGE_A_ID;
   const long long LINESTR_ID_B = YOUR_SLOT_EDGE_B_ID;
   ```

4. **重新编译**:
   ```bash
   cd ~/park_ws
   catkin_make
   ```

---

## 可视化

### RViz 主题列表

系统发布以下可视化主题：

| 主题名 | 消息类型 | 描述 | 颜色/样式 |
|--------|----------|------|-----------|
| `/parking_demo/ways` | `visualization_msgs::MarkerArray` | 道路中心线 | 蓝色线条 |
| `/parking_demo/relations` | `visualization_msgs::MarkerArray` | 车道边界 | 绿色线条 |
| `/parking_demo/nodes` | `visualization_msgs::MarkerArray` | 道路节点 | 红色球体 |
| `/parking_demo/parking_spots` | `visualization_msgs::MarkerArray` | 停车位位置 | 黄色方块 |
| `/parking_demo/path` | `nav_msgs::Path` | 规划路径 | 紫色路径 |
| `/parking_demo/vehicle` | `visualization_msgs::Marker` | 车辆位置 | 青色矩形 |
| `/parking_demo/pose_2d` | `geometry_msgs::Pose2D` | 2D 位姿 | - |
| `/parking_demo/control_command` | `geometry_msgs::Twist` | 控制指令 | - |
| `/parking_demo/lookahead_point` | `visualization_msgs::Marker` | 前瞻点 | 橙色球体 |
| `/parking_slot_marker` | `visualization_msgs::Marker` | 车位边界 | 红色框 |

### RViz 配置

推荐的 RViz 配置（保存为 `parking_demo.rviz`）：

```yaml
Panels:
  - Class: rviz/Displays
    Name: Displays
  - Class: rviz/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz/Grid
      Name: Grid
      Reference Frame: map

    - Class: rviz/MarkerArray
      Name: Roads
      Topic: /parking_demo/ways

    - Class: rviz/MarkerArray
      Name: Lanes
      Topic: /parking_demo/relations

    - Class: rviz/Path
      Name: Planned Path
      Topic: /parking_demo/path
      Color: 170; 0; 255

    - Class: rviz/Marker
      Name: Vehicle
      Topic: /parking_demo/vehicle

    - Class: rviz/Marker
      Name: Lookahead
      Topic: /parking_demo/lookahead_point

    - Class: rviz/Marker
      Name: Parking Slot
      Topic: /parking_slot_marker

  Global Options:
    Fixed Frame: map
```

### 实时调试工具

```bash
# 查看车辆位姿
rostopic echo /parking_demo/pose_2d

# 查看控制指令（速度和转向）
rostopic echo /parking_demo/control_command

# 监控节点状态
rosnode info /parking_system_refactored

# 查看 TF 树
rosrun rqt_tf_tree rqt_tf_tree

# 绘制话题关系图
rqt_graph
```

---

## 故障排除

### 常见问题

#### 1. 编译错误：找不到 OMPL 库

**症状**:
```
/usr/bin/ld: cannot find -lompl
```

**解决方案**:
```bash
# 方法 1: 安装 OMPL
sudo apt-get install ros-noetic-ompl

# 方法 2: 禁用 OMPL（使用后备方案）
# 编辑 CMakeLists.txt，注释掉第 32-34 行
```

#### 2. 运行时错误：无法加载地图文件

**症状**:
```
ERROR: Failed to load OSM file: /path/to/parking_map.osm
```

**解决方案**:
```bash
# 检查文件是否存在
ls -lh src/parking_demo/maps/parking_map.osm

# 检查文件权限
chmod 644 src/parking_demo/maps/parking_map.osm

# 检查 XML 格式是否正确
xmllint --noout src/parking_demo/maps/parking_map.osm
```

#### 3. 车辆不移动或抖动

**症状**: 车辆停在原地或来回震荡

**可能原因**:

| 原因 | 检查方法 | 解决方案 |
|------|----------|----------|
| 前瞻距离过小 | 查看 `lookahead_distance` 参数 | 增大到 2.5-3.0m |
| 路径点密度不足 | 查看 `/parking_demo/path` 主题 | 增加重采样密度 |
| 最小速度过高 | 查看 `min_speed` 参数 | 降低到 0.3-0.5 m/s |
| 目标容差过小 | 查看 `goal_tolerance` 参数 | 增大到 0.3-0.5m |

**调试命令**:
```bash
# 实时查看控制指令
rostopic echo /parking_demo/control_command

# 检查路径是否为空
rostopic echo /parking_demo/path --noarr
```

#### 4. RViz 中看不到可视化

**症状**: RViz 打开但没有显示地图或车辆

**解决方案**:
```bash
# 1. 检查主题是否发布
rostopic list | grep parking_demo

# 2. 检查 Fixed Frame 是否正确
# 在 RViz Global Options 中设置为 "map"

# 3. 手动添加显示项
# Add -> By topic -> /parking_demo/ways -> MarkerArray
```

#### 5. 泊车失败：穿越车位边界

**症状**: 日志显示 "Collision with long edge!"

**解决方案**:
```bash
# 1. 检查车辆尺寸参数是否正确
rosparam get /vehicle_length
rosparam get /vehicle_width

# 2. 增大安全余量（修改 types.h:133）
double dsafe = 1.5;  // 从 1.0 增加到 1.5

# 3. 减小车辆尺寸（临时测试）
rosparam set /vehicle_length 2.0
rosparam set /vehicle_width 1.2
```

---

## 开发指南

### 添加新的泊车策略

1. **在 `parking_maneuver.h` 中声明**:
   ```cpp
   std::vector<CarState> generateMyCustomParking(
       const CarState& start,
       double target_x, double target_y, double target_theta);
   ```

2. **在 `parking_maneuver.cpp` 中实现**:
   ```cpp
   std::vector<CarState> ParkingManeuverGenerator::generateMyCustomParking(...) {
       std::vector<CarState> trajectory;
       // 你的算法实现
       return trajectory;
   }
   ```

3. **在主节点中调用**:
   ```cpp
   parking_traj = parking_gen_->generateMyCustomParking(
       current_state, target_x, target_y, target_theta);
   ```

### 添加新的控制器

1. **创建头文件** `include/parking_demo/my_controller.h`
2. **创建源文件** `src/my_controller.cpp`
3. **修改 `CMakeLists.txt`**:
   ```cmake
   add_library(parking_demo_lib
       ...
       src/my_controller.cpp
   )
   ```
4. **在主节点中替换控制器**:
   ```cpp
   controller_ = std::make_unique<MyController>();
   ```

### 代码风格

- **命名规范**:
  - 类名: `PascalCase` (e.g., `PurePursuitController`)
  - 函数: `camelCase` (e.g., `computeControl`)
  - 成员变量: `snake_case_` 带下划线后缀 (e.g., `vehicle_params_`)
  - 常量: `UPPER_CASE` (e.g., `START_LANELET_ID`)

- **注释规范**:
  ```cpp
  /**
   * @brief 函数简短描述
   * @param param1 参数1说明
   * @param param2 参数2说明
   * @return 返回值说明
   */
  ```

### 性能优化建议

1. **降低可视化频率**: 可视化发布频率从 20Hz 降到 5Hz
2. **使用 Release 编译**: `-DCMAKE_BUILD_TYPE=Release` 可提升 3-5 倍性能
3. **减少日志输出**: 将 `ROS_INFO` 改为 `ROS_DEBUG`
4. **优化路径重采样**: 适当增大间隔（0.1m → 0.15m）

---

## 许可证

MIT

---

## 贡献指南

欢迎提交 Issue 和 Pull Request！

### 提交前检查清单

- [ ] 代码通过编译 (`catkin_make`)
- [ ] 代码符合风格规范
- [ ] 添加必要的注释和文档
- [ ] 测试通过（至少运行一次完整泊车流程）

---

## 联系方式

- **维护者**: goldminer
- **邮箱**: goldminer@todo.todo

---

## 致谢

本项目使用了以下开源库和工具：
- ROS Noetic
- OpenStreetMap
- OMPL (可选)
- Eigen3 (可选)

---

**最后更新**: 2025-12-24
