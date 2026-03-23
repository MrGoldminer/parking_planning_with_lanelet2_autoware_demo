# 泊车系统修复报告 - 约束感知规划方案

**修复日期**: 2025-12-23
**修复类型**: 约束感知路径规划（方案2）
**状态**: ✅ 编译成功，已实现

---

## 问题诊断

### 原始问题
- ❌ 车辆无法正确驶入车位
- ❌ 泊车轨迹可能穿越长边（不可穿越区域）
- ❌ 碰撞检测被禁用（只记录不拒绝）
- ❌ 车位约束未正确建模（长边和短边未区分）

### 根本原因
1. **泊车算法过于简化**: 使用固定圆弧倒车，不考虑车位几何约束
2. **碰撞检测失效**: 为了让系统"能跑起来"，故意禁用了验证逻辑
3. **约束建模缺失**: 车位只用简单多边形表示，未区分长边（障碍）和短边（入口）
4. **动态重规划问题**: 频繁重规划导致轨迹不连续

---

## 修复方案实施

### 1. 添加车位约束数据结构

**文件**: `src/parking_demo/include/parking_demo/types.h:53-83`

新增 `ParkingSlotConstraints` 结构体：
```cpp
struct ParkingSlotConstraints {
    // 长边（不可穿越）
    std::pair<double, double> long_edge_a_start;
    std::pair<double, double> long_edge_a_end;
    std::pair<double, double> long_edge_b_start;
    std::pair<double, double> long_edge_b_end;

    // 短边（入口）
    std::pair<double, double> short_edge_entry_start;
    std::pair<double, double> short_edge_entry_end;
    std::pair<double, double> short_edge_back_start;
    std::pair<double, double> short_edge_back_end;

    // 入口信息
    double entry_center_x;
    double entry_center_y;
    double entry_direction;  // 入口法向量方向（指向车位内部）
    double entry_width;

    // 车位尺寸
    double slot_length;  // 长边长度
    double slot_width;   // 短边长度

    // 目标停车姿态
    double target_x;
    double target_y;
    double target_theta;
};
```

---

### 2. 车位几何约束建模

**文件**: `src/parking_demo/src/parking_system_node_refactored.cpp:278-382`

新增 `computeParkingSlotConstraints()` 函数：

**关键逻辑**:
- ✅ 区分长边A和B（不可穿越）
- ✅ 智能识别入口短边（选择距离车辆起始位置更近的短边）
- ✅ 计算入口中心和法向量方向
- ✅ 计算车位尺寸

```cpp
// 确定哪个短边是入口
double dist_to_short1 = std::hypot(
    (line_a_pts_[0] + line_b_pts_[0]) / 2.0 - vehicle_state_.x,
    (line_a_pts_[0] + line_b_pts_[0]) / 2.0 - vehicle_state_.y
);
double dist_to_short2 = std::hypot(
    (line_a_pts_[1] + line_b_pts_[1]) / 2.0 - vehicle_state_.x,
    (line_a_pts_[1] + line_b_pts_[1]) / 2.0 - vehicle_state_.y
);

if (dist_to_short1 < dist_to_short2) {
    // 短边1是入口
    slot_constraints_.short_edge_entry_start = line_a_pts_[0];
    slot_constraints_.short_edge_entry_end = line_b_pts_[0];
} else {
    // 短边2是入口
    slot_constraints_.short_edge_entry_start = line_a_pts_[1];
    slot_constraints_.short_edge_entry_end = line_b_pts_[1];
}
```

---

### 3. 约束感知泊车轨迹生成

**文件**: `src/parking_demo/src/parking_maneuver.cpp:354-516`

新增 `generateConstraintAwareParking()` 函数，实现**两阶段泊车策略**：

#### 阶段1：计算入口前准备位置
```cpp
// 准备位置：在入口前方2米，朝向对准入口
double prep_distance = 2.0;
double prep_x = entry_center_x - prep_distance * cos(entry_direction);
double prep_y = entry_center_y - prep_distance * sin(entry_direction);
double prep_theta = entry_direction;
```

#### 阶段2：调整朝向
```cpp
// 如果角度差 > 10度，先倒车调整朝向
if (abs(angle_diff) > 10°) {
    // 使用圆弧倒车调整到准备姿态
}
```

#### 阶段3：从入口倒车进入车位
```cpp
// 沿入口方向倒车，逐步旋转到目标朝向
// 使用Ackermann运动学模型
for (int i = 0; i < max_steps; ++i) {
    s.x = current.x + reverse_speed * cos(theta) * dt;
    s.y = current.y + reverse_speed * sin(theta) * dt;
    s.theta = current.theta + (reverse_speed / wheelbase) * tan(steering) * dt;

    // 检查是否到达目标
    if (dist_to_target < 0.3m && abs(angle_error) < 5°) break;
}
```

#### 阶段4：验证轨迹
```cpp
// 检查轨迹是否穿越长边
if (checkLongEdgeCrossing(trajectory)) {
    ROS_ERROR("❌ Generated trajectory crosses long edges! REJECTING.");
    return {};  // 拒绝违规轨迹
}
```

---

### 4. 长边穿越检测

**文件**: `src/parking_demo/src/parking_maneuver.cpp:313-352`

新增函数：
- `checkLongEdgeCrossing()`: 检查轨迹是否穿越长边
- `lineSegmentsIntersect()`: 使用叉积法判断线段相交

```cpp
bool checkLongEdgeCrossing(const std::vector<CarState>& trajectory) const {
    // 检查轨迹的每一段是否与长边A或长边B相交
    for (size_t i = 1; i < trajectory.size(); ++i) {
        if (lineSegmentsIntersect(traj[i-1], traj[i], long_edge_a)) return true;
        if (lineSegmentsIntersect(traj[i-1], traj[i], long_edge_b)) return true;
    }
    return false;
}
```

---

### 5. 重新启用碰撞检测

**文件**: `src/parking_demo/src/parking_system_node_refactored.cpp:710-769`

修改 `generateDynamicParkingTrajectory()` 函数：

**修改前**:
```cpp
if (slot_violations > 0) {
    ROS_INFO("   ℹ️ %d points outside parking slot bounds (acceptable)",
             slot_violations);
}
// 只记录，不拒绝轨迹
```

**修改后**:
```cpp
// ✅ 优先使用约束感知的泊车规划
std::vector<CarState> parking_traj =
    parking_maneuver_->generateConstraintAwareParking(vehicle_state_);

// ✅ 如果约束感知规划失败，回退到传统方法
if (parking_traj.empty()) {
    ROS_WARN("   ⚠️ Constraint-aware planning failed, falling back");
    parking_traj = parking_maneuver_->generateVerticalParking(...);
}

// ✅ 严格验证：如果有违规，拒绝轨迹
if (frontline_violations > 0) {
    ROS_ERROR("   ❌ %d trajectory points passed frontline - REJECTING", ...);
    return {};  // 拒绝违规轨迹
}

if (slot_violations > parking_traj.size() * 0.1) {  // 允许10%容差
    ROS_ERROR("   ❌ %d points outside slot - REJECTING", ...);
    return {};  // 拒绝违规轨迹
}
```

---

## 修改文件清单

| 文件 | 修改内容 | 行号 |
|------|---------|------|
| `include/parking_demo/types.h` | 添加 `ParkingSlotConstraints` 结构体 | 53-83 |
| `include/parking_demo/parking_maneuver.h` | 添加约束感知规划接口 | 70-102 |
| `src/parking_maneuver.cpp` | 实现约束感知规划和长边检测 | 37-516 |
| `src/parking_system_node_refactored.cpp` | 建模车位约束 + 启用验证 | 278-769 |

**代码行数统计**:
- 新增代码: ~300 行
- 修改代码: ~50 行
- 总计: ~350 行

---

## 技术亮点

### 1. 智能入口识别
系统自动识别距离车辆更近的短边作为入口，适应不同的车辆起始位置。

### 2. 两阶段规划策略
- **阶段1**: 准备姿态（对准入口）
- **阶段2**: 倒车入库（从入口进入）

确保车辆始终从短边进入，避开长边障碍。

### 3. 严格约束验证
- ✅ 长边穿越检测（线段相交算法）
- ✅ Footprint边界检测（点在多边形内算法）
- ✅ 前沿线越界检测（法向量投影算法）
- ✅ 违规轨迹拒绝机制

### 4. 降级策略
如果约束感知规划失败，自动回退到传统方法，确保系统鲁棒性。

---

## 测试验证

### 编译测试
```bash
$ catkin_make -j1
[100%] Built target parking_system_refactored
✅ 编译成功
```

### 运行测试（待执行）
```bash
# 启动ROS核心
roscore

# 启动泊车系统
roslaunch parking_demo parking.launch

# 预期结果：
# - 车辆应该从短边进入车位
# - 不应该穿越长边
# - 泊车轨迹应该通过验证
```

---

## 下一步建议

### 短期优化（1-2天）
1. **实际测试**: 在RViz中运行系统，验证泊车行为
2. **参数调优**: 调整准备位置距离、容差等参数
3. **调试输出**: 观察ROS日志，确认约束识别正确

### 中期改进（1-2周）
1. **轨迹平滑**: 在阶段切换处添加轨迹平滑
2. **速度规划**: 在接近目标时降低速度
3. **多次重试**: 如果规划失败，调整准备位置重试

### 长期升级（1-2月）
1. **Hybrid A***：实现更智能的路径搜索
2. **RRT***：支持复杂环境下的泊车
3. **MPC控制**：更精确的轨迹跟踪

---

## 关键文件位置

**核心实现**:
- 约束建模: `src/parking_system_node_refactored.cpp:278-382`
- 约束感知规划: `src/parking_maneuver.cpp:354-516`
- 轨迹验证: `src/parking_system_node_refactored.cpp:731-763`

**头文件**:
- 数据结构: `include/parking_demo/types.h:53-83`
- 接口定义: `include/parking_demo/parking_maneuver.h:70-102`

**Launch文件**:
- 启动配置: `launch/parking.launch`

---

## 修复完成度

- ✅ **约束建模**: 100%
- ✅ **轨迹生成**: 100%
- ✅ **碰撞检测**: 100%
- ✅ **代码编译**: 100%
- ⏳ **实际测试**: 待进行

**总体完成度**: 90%（代码实现完成，实际测试待进行）

---

## 常见问题

### Q1: 车辆仍然无法进入车位？
**A**: 检查以下项：
1. 车辆起始位置是否在泊车路径终点附近？
2. ROS日志中是否有"约束识别"相关的输出？
3. 轨迹是否被拒绝（检查ERROR日志）？

### Q2: 编译出错？
**A**: 使用单线程编译 `catkin_make -j1`

### Q3: 如何调试？
**A**: 启用详细日志：
```bash
roslaunch parking_demo parking.launch --screen
```

观察以下关键信息：
- `📐 Parking slot constraints computed`
- `🔧 Generating constraint-aware parking trajectory`
- `✅ Trajectory validation passed`

---

**修复完成时间**: 2025-12-23
**修复人员**: Claude Sonnet 4.5
**修复方案**: 约束感知路径规划（方案2）
**修复状态**: ✅ 已实现并编译成功
