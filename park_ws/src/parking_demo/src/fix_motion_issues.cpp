// ============================================================
// 修复车辆无法运动问题的关键代码段
// 将这些代码段替换到 real_map_parking_node.cpp 中对应位置
// ============================================================

// ====== 修复1: OMPL路径速度设置 (第1122-1125行) ======
// 替换这段代码:
/*
for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    const ob::SE2StateSpace::StateType* st = path.getState(i)->as<ob::SE2StateSpace::StateType>();
    CarState s; s.x = st->getX(); s.y = st->getY(); s.theta = st->getYaw(); s.v = 0.0; s.phi = 0.0; out.push_back(s);
}
*/

// 修复后的代码:
for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    const ob::SE2StateSpace::StateType* st = path.getState(i)->as<ob::SE2StateSpace::StateType>();
    CarState s;
    s.x = st->getX();
    s.y = st->getY();
    s.theta = st->getYaw();

    // ✅ 关键修复：为OMPL路径点设置合理的速度
    // 根据路径段长度动态调整速度
    if (i > 0) {
        const ob::SE2StateSpace::StateType* prev_st = path.getState(i-1)->as<ob::SE2StateSpace::StateType>();
        double dx = st->getX() - prev_st->getX();
        double dy = st->getY() - prev_st->getY();
        double seg_len = std::hypot(dx, dy);

        // 曲线段降速，直线段提速
        if (seg_len < 0.5) {
            s.v = 0.8;  // 密集点 -> 曲线区域 -> 低速
        } else {
            s.v = 2.0;  // 稀疏点 -> 直线区域 -> 高速
        }
    } else {
        s.v = 1.5;  // 起点默认速度
    }

    s.phi = 0.0;
    out.push_back(s);
}


// ====== 修复2: centerline fallback路径速度 (第800-820行) ======
// 在 if (global_plan_.empty()) 分支中，替换这段:
/*
for (size_t i=0;i<centerline_pts.size();++i) {
    CarState s; s.x = centerline_pts[i].first; s.y = centerline_pts[i].second; s.v = std::min(1.0, max_speed_);
    if (i+1 < centerline_pts.size()) s.theta = std::atan2(...);
    global_plan_.push_back(s);
}
*/

// 修复后的代码:
for (size_t i=0;i<centerline_pts.size();++i) {
    CarState s;
    s.x = centerline_pts[i].first;
    s.y = centerline_pts[i].second;

    // ✅ 关键修复：提高centerline路径速度
    s.v = std::min(2.5, max_speed_);  // 从1.0提高到2.5

    // 估计heading
    if (i+1 < centerline_pts.size()) {
        s.theta = std::atan2(centerline_pts[i+1].second - centerline_pts[i].second,
                            centerline_pts[i+1].first - centerline_pts[i].first);
    } else if (i>0) {
        s.theta = std::atan2(centerline_pts[i].second - centerline_pts[i-1].second,
                            centerline_pts[i].first - centerline_pts[i-1].first);
    }

    global_plan_.push_back(s);
}

// ✅ 新增：路径密度检查和重采样
if (!global_plan_.empty()) {
    ROS_INFO("Original path has %lu points", global_plan_.size());

    std::vector<CarState> resampled;
    resampled.push_back(global_plan_[0]);

    for (size_t i = 1; i < global_plan_.size(); ++i) {
        double dx = global_plan_[i].x - resampled.back().x;
        double dy = global_plan_[i].y - resampled.back().y;
        double dist = std::hypot(dx, dy);

        // 如果两点距离超过0.5米，插入中间点
        if (dist > 0.5) {
            int n_insert = (int)(dist / 0.3);  // 每0.3米一个点
            for (int j = 1; j <= n_insert; ++j) {
                double t = double(j) / (n_insert + 1);
                CarState s_new;
                s_new.x = resampled.back().x + dx * t;
                s_new.y = resampled.back().y + dy * t;
                s_new.theta = std::atan2(dy, dx);
                s_new.v = 2.0;
                s_new.phi = 0.0;
                resampled.push_back(s_new);
            }
        }

        resampled.push_back(global_plan_[i]);
    }

    global_plan_ = resampled;
    ROS_INFO("Resampled path to %lu points (density improved)", global_plan_.size());
}


// ====== 修复3: 速度计算逻辑增强 (第2209-2238行) ======
// 替换executeTrackingStep中的速度计算部分:

// 基于最近点计算速度
double speed = 0.0;

// ✅ 改进的速度计算策略
if (closest_dist < 0.8) {
    // 距离路径很近，使用前瞻距离计算速度
    speed = kp_speed_ * Ld;

    // 如果前瞻距离也很小，使用最小速度
    if (Ld < 1.0) {
        speed = std::max(speed, 0.5);  // 保证最小速度0.5 m/s
    }
} else {
    // 距离路径较远，使用更大的速度追赶
    speed = kp_speed_ * closest_dist * 1.5;  // 增加系数加速追赶
}

// 曲率惩罚（降低影响）
double curpen = 1.0 + curvature_speed_gain_ * 0.3 * std::abs(curvature);  // 从0.5降到0.3
speed = speed / curpen;

// 限制最大速度
if (speed > max_speed_) speed = max_speed_;

// ✅ 改进的最小速度控制
double effective_min_speed = 0.3;  // 提高最小速度从0.05到0.3

// 根据曲率调整最小速度
if (std::abs(curvature) > 0.2) {
    effective_min_speed = 0.4;  // 曲线段稍微提速
}

if (std::abs(speed) < effective_min_speed) {
    if (cosang >= 0) {
        speed = effective_min_speed;   // 前进
    } else {
        speed = -effective_min_speed;  // 后退
    }
}


// ====== 修复4: 路径点推进逻辑 (第2277-2285行) ======
// 替换这段代码:
/*
while (current_idx_ < (int)global_plan_.size()) {
    double dx = global_plan_[current_idx_].x - car_.x;
    double dy = global_plan_[current_idx_].y - car_.y;
    if (std::hypot(dx,dy) < std::max(0.5, goal_tolerance_)) current_idx_++; else break;
}
*/

// 修复后的代码:
// ✅ 关键修复：减小推进阈值，避免跳过太多路径点
while (current_idx_ < (int)global_plan_.size()) {
    double dx = global_plan_[current_idx_].x - car_.x;
    double dy = global_plan_[current_idx_].y - car_.y;
    double dist_to_point = std::hypot(dx, dy);

    // 使用更小的阈值：0.2米（原来是0.5米）
    double pass_threshold = std::min(0.2, goal_tolerance_ * 0.5);

    if (dist_to_point < pass_threshold) {
        current_idx_++;
    } else {
        break;
    }
}

// 检查是否到达终点
if (current_idx_ >= (int)global_plan_.size()) {
    car_.v = 0.0;
    car_.phi = 0.0;
    ROS_INFO_THROTTLE(2.0, "Reached final goal!");
}


// ====== 修复5: 添加调试输出 (在executeTrackingStep开头) ======
void executeTrackingStep(double dt) {
    if (global_plan_.empty()) {
        ROS_WARN_THROTTLE(1.0, "❌ Global plan is empty! Cannot execute tracking.");
        return;
    }

    // ✅ 添加详细的调试信息
    static int debug_counter = 0;
    if (++debug_counter % 20 == 0) {  // 每秒输出一次 (20Hz)
        double dist_to_current = std::hypot(
            global_plan_[current_idx_].x - car_.x,
            global_plan_[current_idx_].y - car_.y
        );

        ROS_INFO("🚗 Car: pos=(%.2f, %.2f) θ=%.1f° v=%.2f m/s | "
                 "📍 Plan: %lu pts, idx=%d/%lu | "
                 "📏 Dist to current=%.2f m",
                 car_.x, car_.y, car_.theta * 180.0 / M_PI, car_.v,
                 global_plan_.size(), current_idx_, global_plan_.size(),
                 dist_to_current);
    }

    // 原有代码继续...
}


// ====== 修复6: 构造函数参数调整 ======
// 在构造函数中，修改这些参数的默认值:
/*
double max_speed_ = 3.0;
double min_speed_ = 0.05;
double kp_speed_ = 0.8;
double lookahead_dist_ = 2.0;
*/

// 修改为:
double max_speed_ = 3.0;
double min_speed_ = 0.3;        // ✅ 从0.05提高到0.3
double kp_speed_ = 1.2;         // ✅ 从0.8提高到1.2
double lookahead_dist_ = 2.5;   // ✅ 从2.0提高到2.5
