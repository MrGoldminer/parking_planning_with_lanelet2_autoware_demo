# 调试和修复指南

## 📋 已修复的问题

### 问题1：parking_spots话题显示错误 ✅ 已修复

**错误现象**：
- 显示了5个停车位的中心点
- 不是用户需要的边线A和B组成的矩形

**根本原因**：
系统调用了 `visualizer_->publishParkingSpots()`，发布所有从OSM地图提取的停车位

**解决方案**：
注释掉通用停车位的可视化调用（第104-107行）
```cpp
// 🔧 不发布所有停车位，只显示我们关心的A和B边线组成的矩形
// visualizer_->publishParkingSpots(map_loader_->getParkingSpots(),
//                                 map_loader_->getNodes(),
//                                 map_loader_->getWays());
ROS_INFO("Skipping generic parking spots visualization (will show only target slot)");
```

**结果**：
- ✅ 不再显示无关的停车位点
- ✅ 只显示蓝色停车槽线框（由A和B边线扩展3米形成）
- ✅ 话题 `/parking_slot_marker` 显示正确的矩形

---

### 问题2：小车不沿中心线运动 🔍 添加调试

**现象**：
用户报告小车第一步不沿车道线中心运动

**可能原因**：
1. 车辆初始位置偏离路径起点
2. Pure Pursuit参数不当（lookahead距离）
3. 路径生成问题
4. 控制器索引推进不正确

**添加的调试信息**：

#### A. 路径信息（第490-498行）
```cpp
// 打印前5个路径点
ROS_INFO("   First %d waypoints:", print_count);
for (int i = 0; i < print_count; ++i) {
    ROS_INFO("     [%d]: (%.2f, %.2f, %.1f°)", i, ...);
}
```

#### B. 控制循环详细信息（第700-722行）
```cpp
ROS_INFO("🚗 [APPROACHING] pos=(%.2f,%.2f) θ=%.1f° v=%.2f φ=%.2f° | idx=%d/%lu | dist_to_path=%.3fm | lookahead=%.2fm | dist_to_start=%.2fm",
         vehicle_state_.x, vehicle_state_.y,
         vehicle_state_.theta * 180.0 / M_PI,
         vehicle_state_.v,
         vehicle_state_.phi * 180.0 / M_PI,  // 转向角
         curr_idx,                            // 当前路径索引
         global_path_.size(),
         dist_to_path,                        // 到当前路径点的距离
         lookahead_dist,                      // 前瞻距离
         dist_to_start);                      // 到泊车起点的距离
```

---

## 🔍 诊断步骤

### 第一步：检查路径生成

启动系统后，查看路径信息：

```
[INFO] ✅ Mission planned: X total waypoints
[INFO]    Start: (x, y, θ°)
[INFO]    End: (x, y, θ°)
[INFO]    First 5 waypoints:
[INFO]      [0]: (x, y, θ°)
[INFO]      [1]: (x, y, θ°)
[INFO]      [2]: (x, y, θ°)
[INFO]      [3]: (x, y, θ°)
[INFO]      [4]: (x, y, θ°)
[INFO] ✅ Vehicle initialized at path start: pos=(x, y) θ=angle°
```

**检查点**：
1. ✅ 车辆初始位置应该等于 `[0]` 路径点
2. ✅ 路径点应该沿车道中心线分布
3. ✅ 路径点间距应该约0.5米（重采样距离）

### 第二步：检查控制循环

运行1秒后（20次循环后），查看控制信息：

```
[INFO] 🚗 [APPROACHING] pos=(x,y) θ=angle° v=speed φ=steer° | idx=i/total | dist_to_path=0.XXXm | lookahead=3.XXm | dist_to_start=XX.XXm
```

**检查点**：
1. ✅ `idx` 应该逐渐增加（0→1→2→...）
2. ✅ `dist_to_path` 应该很小（<0.5米）
3. ✅ `lookahead` 应该约等于参数值（3.0米）
4. ✅ `v` 速度应该合理（0.5-3.0 m/s）
5. ✅ `φ` 转向角应该变化（不应该一直是0）

### 第三步：在RViz中可视化

**必须订阅的话题**：
1. `/planned_path` (Path) - 绿色规划路径
2. `/car_marker` (MarkerArray) - 蓝色车辆
3. `/lookahead_marker` (Marker) - 红色前瞻点
4. `/parking_slot_marker` (MarkerArray) - 蓝色停车槽线框

**检查点**：
- ✅ 车辆（蓝色）应该沿绿色路径移动
- ✅ 红色前瞻点应该在车前方3米左右
- ✅ 前瞻点应该始终在绿色路径上

---

## 🐛 常见问题和解决方案

### 问题A：车辆不动或速度很慢

**可能原因**：
- `kp_speed` 参数太小
- `lookahead_distance` 太小
- `min_speed` 太低

**解决方案**：
修改 `launch/parking.launch`：
```xml
<param name="kp_speed" value="1.5"/>           <!-- 从1.2增加到1.5 -->
<param name="lookahead_distance" value="4.0"/> <!-- 从3.0增加到4.0 -->
<param name="min_speed" value="0.5"/>          <!-- 从0.3增加到0.5 -->
```

### 问题B：车辆偏离路径

**症状**：`dist_to_path` 持续增大（>1.0米）

**可能原因**：
- Pure Pursuit转向角计算错误
- 路径索引推进太快或太慢
- 转向速率限制太严格

**检查**：
1. 查看转向角 `φ` 是否合理（不应该一直是0）
2. 查看 `idx` 是否正常推进

**解决方案**：
```cpp
// 在 pure_pursuit_controller.cpp:123 修改pass_threshold
double pass_threshold = std::min(0.5, controller_params_.goal_tolerance * 0.5);
// 改为更大值
double pass_threshold = std::min(1.0, controller_params_.goal_tolerance);
```

### 问题C：索引不推进（idx一直是0）

**症状**：`idx=0/total` 不变

**原因**：车辆距离第一个路径点太远，无法满足推进条件

**解决方案1**：确保车辆初始化在路径起点
```cpp
// parking_system_node_refactored.cpp:120
vehicle_state_ = global_path_[0];  // 确保这行被执行
```

**解决方案2**：放宽推进阈值
```cpp
// pure_pursuit_controller.cpp:123
double pass_threshold = 0.5;  // 改为更大值
```

### 问题D：转向角一直是0

**症状**：`φ=0.0°` 不变

**可能原因**：
- 车辆朝向与路径朝向完全一致（理想情况）
- 前瞻点在车辆正前方（alpha=0）
- 转向角计算有误

**检查**：
- 如果 `dist_to_path` 很小（<0.2米），说明跟踪良好
- 如果 `dist_to_path` 很大，说明有问题

---

## 📊 参数调优指南

### Pure Pursuit参数

| 参数 | 默认值 | 作用 | 调优建议 |
|------|--------|------|---------|
| `lookahead_distance` | 3.0m | 前瞻距离 | 增大→更平滑，减小→更精确 |
| `kp_speed` | 1.2 | 速度增益 | 增大→更快，减小→更慢 |
| `max_speed` | 3.0 m/s | 最大速度 | 根据安全性调整 |
| `min_speed` | 0.3 m/s | 最小速度 | 太小会导致停滞 |
| `goal_tolerance` | 0.3m | 到达判定 | 影响索引推进 |

### 推荐配置（快速跟踪）
```xml
<param name="lookahead_distance" value="4.0"/>
<param name="kp_speed" value="1.5"/>
<param name="max_speed" value="3.5"/>
<param name="min_speed" value="0.5"/>
```

### 推荐配置（精确跟踪）
```xml
<param name="lookahead_distance" value="2.0"/>
<param name="kp_speed" value="1.0"/>
<param name="max_speed" value="2.0"/>
<param name="min_speed" value="0.3"/>
```

---

## ✅ 验证修复成功

### 启动系统
```bash
cd ~/park_ws
source devel/setup.bash
roslaunch parking_demo parking.launch
```

### 预期日志（正常情况）

**初始化阶段**：
```
[INFO] === Parking System (Refactored) Starting ===
[INFO] Map loaded: X nodes, Y ways, Z relations
[INFO] Skipping generic parking spots visualization (will show only target slot)
[INFO] 📍 Parking position computed:
[INFO]    Target: (x, y) θ=angle°
[INFO]    Line A points: (x1, y1) -> (x2, y2)
[INFO]    Line B points: (x1, y1) -> (x2, y2)
[INFO]    Parking slot polygon vertices (expanded by 3.0m):
[INFO]      Vertex 0: (x, y)
[INFO]      Vertex 1: (x, y)
[INFO]      Vertex 2: (x, y)
[INFO]      Vertex 3: (x, y)
[INFO] ✅ Parking slot and target visualized in RViz
[INFO] === Planning Mission ===
[INFO] ✅ Relation path found: N lanelets
[INFO]    Path generated: M waypoints
[INFO]    Path resampled: M points
[INFO]    First 5 waypoints:
[INFO]      [0]: (x, y, θ°)
[INFO]      [1]: (x, y, θ°)
[INFO]      [2]: (x, y, θ°)
[INFO]      [3]: (x, y, θ°)
[INFO]      [4]: (x, y, θ°)
[INFO] ✅ Vehicle initialized at path start: pos=(x, y) θ=angle°
```

**运行阶段（每秒一次）**：
```
[INFO] 🚗 [APPROACHING] pos=(x,y) θ=angle° v=speed φ=steer° | idx=10/200 | dist_to_path=0.123m | lookahead=3.05m | dist_to_start=15.45m
```

**关键指标**：
- ✅ `idx` 逐渐增加：0→10→20→...
- ✅ `dist_to_path` 保持很小：<0.5m
- ✅ `v` 速度合理：0.5~3.0 m/s
- ✅ `φ` 转向角变化（非直线路径时）
- ✅ `lookahead` 稳定在3m左右
- ✅ `dist_to_start` 逐渐减小

---

## 🎯 下一步

运行系统后，请提供以下信息以便进一步诊断：

1. **初始化日志**：
   - 前5个路径点的坐标
   - 车辆初始位置

2. **运行日志**：
   - 前几秒的控制循环输出
   - 特别注意 `idx` 和 `dist_to_path`

3. **RViz截图**（如果可能）：
   - 显示车辆、路径、前瞻点的关系

有了这些信息，我可以精确定位问题！🔍
