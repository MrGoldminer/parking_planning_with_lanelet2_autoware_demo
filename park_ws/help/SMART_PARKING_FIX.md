# 智能泊车轨迹修复

## 🎯 问题描述

用户反馈：小车到达泊车起点时，车库入口就在前方，可以直接倒进去，但系统一直在"绕大圈"（先水平倒车2米再转弯）。

## ✅ 解决方案

改进 `generateVerticalParking()` 函数，采用**智能策略**根据实际朝向选择泊车方式。

---

## 📐 新的智能策略

### 策略1：直接垂直倒车（角度差<10°）
```
如果到达泊车起点时，车辆朝向已经基本对准车库（角度差<10度）：
┌─────────────┐
│   Parking   │
│    Slot     │ ← 直接倒车进入
└─────────────┘
      ↑
      │（车辆）
```

**执行步骤**：
- 一个阶段：直接垂直倒车进入车位
- 速度：0.4 m/s
- 转向角：0（直线）

### 策略2：圆弧+垂直倒车（需要转向）
```
如果需要调整朝向（角度差≥10度）：
┌─────────────┐
│   Parking   │
│    Slot     │ ← 第2阶段：垂直倒车
└─────────────┘
      ↑
      │ 第1阶段：圆弧倒车转向
     ╱
    ╱（车辆）
```

**执行步骤**：
- 第1阶段：圆弧倒车转向（使用Ackermann模型）
  - 速度：0.4 m/s
  - 转向角：根据角度差和转弯半径计算
- 第2阶段：垂直倒车进入车位
  - 速度：0.3 m/s
  - 转向角：0（直线）

---

## 🔧 修改的文件

### `parking_maneuver.cpp`

#### 修改位置：`generateVerticalParking()` 函数（第37-193行）

**关键改进**：
1. **取消固定的"水平倒车2米"阶段** ✅
2. **根据角度差动态选择策略** ✅
3. **改进日志输出，显示选择的策略** ✅

#### 代码逻辑：
```cpp
// 计算初始角度差
double angle_diff = target_theta - current.theta;
// 归一化到[-π, π]

if (std::abs(angle_diff) < 10.0 * M_PI / 180.0) {
    // 策略1：角度差<10度，直接垂直倒车
    ROS_INFO("Strategy: Direct vertical reverse");
    // ... 一个阶段：直接倒车进入

} else {
    // 策略2：需要转向，圆弧+垂直
    ROS_INFO("Strategy: Arc + vertical");

    // 第1阶段：圆弧倒车转向
    // ... 使用Ackermann模型

    // 第2阶段：垂直倒车进入
    // ... 直线倒车到目标
}
```

---

## 📊 日志输出示例

### 场景1：车辆朝向对齐（角度差<10°）
```
[INFO] Generating vertical parking trajectory (smart strategy):
[INFO]    Start: (x, y, θ°)
[INFO]    Target: (x, y, θ°)
[INFO]    Initial angle difference: 5.2°
[INFO]    Strategy: Direct vertical reverse (angle diff < 10°)
[INFO]    ✅ Direct vertical parking trajectory: 25 states
```

### 场景2：需要转向（角度差≥10°）
```
[INFO] Generating vertical parking trajectory (smart strategy):
[INFO]    Start: (x, y, θ°)
[INFO]    Target: (x, y, θ°)
[INFO]    Initial angle difference: 35.8°
[INFO]    Strategy: Arc + vertical (angle diff = 35.8°)
[INFO]    Stage 1: Arc backward (angle_diff=35.8°, R=3.00m, arc_len=1.88m, %d steps)
[INFO]    Stage 1: Reached target orientation early at step 15
[INFO]    Stage 2: Vertical backward into slot (dist=5.23m, %d steps)
[INFO]    ✅ Arc + vertical parking trajectory: 45 states
```

---

## 🎮 测试方法

### 启动系统：
```bash
cd ~/park_ws
source devel/setup.bash
roslaunch parking_demo parking.launch
```

### 观察日志：

**检查策略选择**：
- 查看 `Initial angle difference` 的值
- 确认使用的策略（Direct vertical 或 Arc + vertical）

**验证行为**：
- ✅ 不应该再看到"绕大圈"现象
- ✅ 如果角度差小，应该直接倒车进入
- ✅ 如果需要转向，应该立即开始圆弧倒车（不先水平倒车）

### RViz检查：
1. **绿色路径**：应该从泊车起点直接指向车库入口
2. **蓝色车辆**：到达泊车起点后，应该直接或圆弧倒车进入，不绕圈
3. **红色目标箭头**：在车库内，指向停车朝向

---

## 🔍 技术细节

### 为什么要取消"水平倒车2米"？

**原因**：
- 路径规划器已经生成了从起点到泊车起点的优化路径
- 到达泊车起点时，车辆朝向应该已经基本对准车库入口
- 固定的"水平倒车2米"是多余的，会导致"绕大圈"

### 角度阈值为什么选10度？

**10度的考量**：
- 足够小，确保直线倒车时不会偏离车位
- 足够大，避免微小的角度噪声触发圆弧倒车
- 可以根据实际测试调整（建议范围：5-15度）

### 圆弧倒车使用的模型

**Ackermann转向模型**：
```cpp
s.x = current.x + v * std::cos(current.theta) * dt;
s.y = current.y + v * std::sin(current.theta) * dt;
s.theta = current.theta + (v / wheelbase) * std::tan(steering_angle) * dt;
```

- `v`：速度（倒车为负）
- `wheelbase`：车辆轴距
- `steering_angle`：转向角

---

## ⚙️ 参数调优

### 如果角度阈值需要调整：

**位置**：`parking_maneuver.cpp:59`
```cpp
if (std::abs(angle_diff) < 10.0 * M_PI / 180.0) {  // 修改这里
```

**建议值**：
- 保守（需要更精确对齐）：5度
- 标准（推荐）：10度
- 宽松（允许更大偏差）：15度

### 如果倒车速度需要调整：

**直接倒车速度**：`parking_maneuver.cpp:69`
```cpp
s.v = -0.4;  // 修改这里（单位：m/s）
```

**圆弧倒车速度**：`parking_maneuver.cpp:118`
```cpp
s.v = -0.4;  // 修改这里
```

**垂直倒车速度**：`parking_maneuver.cpp:155`
```cpp
s.v = -0.3;  // 修改这里（更慢更安全）
```

---

## 🚀 系统行为（改进后）

### 阶段1：APPROACHING
```
算法：Pure Pursuit
目标：泊车起点（lanelet 9265）
行为：严格沿中心线前进
```

### 阶段2：PARKING（智能策略）

#### 情况A：角度对齐（<10°）
```
判断：角度差<10度
策略：直接垂直倒车
步骤：一步到位，直接倒入车位
```

#### 情况B：需要转向（≥10°）
```
判断：角度差≥10度
策略：圆弧+垂直倒车
步骤：
  1. 圆弧倒车调整朝向
  2. 垂直倒车进入车位
```

---

## 📋 总结

### 修复内容：
✅ 取消固定的"水平倒车2米"阶段
✅ 实现智能策略选择（根据角度差）
✅ 直接倒车或圆弧倒车，不再绕大圈
✅ 改进日志输出，清晰显示策略

### 系统特点：
✅ **智能判断**：根据实际朝向选择最优策略
✅ **直达目标**：车库入口就在前方时直接倒入
✅ **灵活转向**：需要调整时才使用圆弧倒车
✅ **无需绕圈**：不再执行多余的水平倒车

### 测试检查点：
1. ✅ 查看日志确认角度差和选择的策略
2. ✅ 观察车辆是否直接倒入（角度差小时）
3. ✅ 验证圆弧倒车的流畅性（需要转向时）
4. ✅ 确认车辆不再"绕大圈"

---

## 🎉 预期效果

**之前**：车辆到达泊车起点 → 先水平倒车2米 → 再圆弧转向 → 最后垂直倒车（绕大圈）

**现在**：
- **角度对齐时**：车辆到达泊车起点 → 直接垂直倒车进入 ✨
- **需要转向时**：车辆到达泊车起点 → 圆弧倒车调整朝向 → 垂直倒车进入 ✨

系统已优化，准备测试！🚗💨
