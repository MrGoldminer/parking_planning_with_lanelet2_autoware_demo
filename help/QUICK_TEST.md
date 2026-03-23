# 泊车系统快速测试指南

## 修复内容

✅ **简化了泊车算法**
- 移除复杂的准备阶段
- 直接从当前位置倒车进入车位
- 一边倒车，一边旋转调整朝向

✅ **放宽了验证逻辑**
- 不再检查中间轨迹点是否在车位内（因为倒车进入时会穿过短边）
- 只验证最终停车位置
- 保留长边穿越检测（确保不碰撞）

✅ **关键约束**
- ✅ 长边A和B：不可穿越
- ✅ 短边（入口）：可直接驶入
- ✅ 倒车策略：反向进入

---

## 测试步骤

### 1. 启动系统

```bash
# 终端1：启动ROS核心
roscore

# 终端2：source环境并启动泊车系统
cd /home/goldminer/park_ws
source devel/setup.bash
roslaunch parking_demo parking.launch
```

### 2. 观察RViz

打开RViz，应该能看到：
- 停车槽可视化（蓝色多边形框）
- 目标停车位置（红色箭头）
- 车辆模型
- 泊车轨迹（绿色路径）

### 3. 查看关键日志

在启动的终端中，查找以下关键信息：

#### ✅ 约束识别成功
```
📐 Parking slot constraints computed:
   Long edge A: ...
   Long edge B: ...
   Entry (short edge): ...
   Entry center: ...
   Slot dimensions: X.XXm (length) x X.XXm (width)
```

#### ✅ 轨迹生成成功
```
🔧 Generating constraint-aware parking (simple reverse entry):
   Start: (X.XX, X.XX, XXX°)
   Target: (X.XX, X.XX, XXX°)
   Initial angle difference: XXX°
   Steering: XXX°, turning radius: X.XXm
   ✅ Reached target at step XXX
✅ Constraint-aware parking trajectory: XXX states
```

#### ✅ 验证通过
```
✅ Generated XXX parking states
✅ Final position validation passed
✅ Trajectory accepted
```

#### ❌ 如果看到这些错误
```
❌ Generated trajectory crosses long edges! REJECTING.
   → 说明轨迹穿越了长边，需要调整算法

❌ XXX points outside parking slot bounds (XX.X%) - REJECTING
   → 说明验证太严格（这个错误应该已经修复了）

⚠️ Constraint-aware planning failed, falling back to traditional method
   → 约束感知规划失败，系统回退到传统方法
```

---

## 预期行为

### 正确的泊车流程

1. **阶段1：接近车位**
   - 车辆沿着规划路径前进
   - 到达泊车起始点附近
   - 状态切换：APPROACHING → PARKING

2. **阶段2：倒车进入**
   - 车辆开始倒车
   - 一边倒车，一边旋转调整朝向
   - **从短边进入车位**
   - **不穿越长边**

3. **阶段3：完成停车**
   - 到达目标位置
   - 朝向与长边平行
   - 车辆停止
   - 状态切换：PARKING → COMPLETED

---

## 常见问题排查

### Q1: 车辆不移动？

检查：
- ROS核心是否启动？`roscore`
- Launch文件是否正确加载？
- 是否有ERROR日志？

### Q2: 轨迹被拒绝？

检查日志中的拒绝原因：
- 如果是"crosses long edges"：长边穿越检测可能误判
- 如果是"outside parking slot bounds"：这个错误应该已经修复

### Q3: 车辆穿越了长边？

这是**严重错误**，说明：
- 长边定义可能不正确
- 线段相交检测算法有bug
- 需要查看约束计算的日志，确认长边坐标

### Q4: 车辆无法进入车位？

可能原因：
- 起始位置太远，倒车距离不够
- 转弯半径太大，无法转到目标朝向
- 需要调整 `min_turn_radius` 参数

---

## 调试技巧

### 1. 查看详细日志
```bash
roslaunch parking_demo parking.launch --screen
```

### 2. 查看话题
```bash
# 查看所有话题
rostopic list

# 查看车辆状态
rostopic echo /vehicle_state

# 查看控制命令
rostopic echo /control_command
```

### 3. 查看RViz标记
- 停车槽：`/parking_slot_marker`
- 路径：`/global_path`
- 车辆：`/vehicle_footprint`
- 前瞻点：`/lookahead_point`

### 4. 录制和回放
```bash
# 录制
rosbag record -a

# 回放
rosbag play <bag文件>
```

---

## 参数调整

如果需要调整参数，编辑 `launch/parking.launch`:

```xml
<!-- 车辆参数 -->
<param name="wheelbase" value="1.8"/>
<param name="max_steering_rad" value="0.6"/>
<param name="min_turn_radius" value="3.0"/>  <!-- 关键参数 -->

<!-- 控制参数 -->
<param name="lookahead_distance" value="2.5"/>
<param name="max_speed" value="2.5"/>

<!-- 倒车速度（在parking_maneuver.cpp中硬编码为-0.35m/s） -->
```

---

## 修复完成度

- ✅ 代码修改：100%
- ✅ 编译成功：100%
- ⏳ 实际测试：**待进行**

---

## 下一步

1. **运行测试**：按照上述步骤启动系统
2. **观察行为**：确认车辆是否从短边倒车进入
3. **调整参数**：如有需要，微调转弯半径等参数
4. **报告结果**：记录测试结果和任何问题

---

**测试日期**: 2025-12-23
**修复版本**: 简化倒车进入算法 v2
**预期效果**: 车辆从短边直接倒车进入车位，不穿越长边
