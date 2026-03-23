/**
 * @file parking_maneuver.cpp
 * @brief 泊车轨迹生成器实现
 */

#include "parking_demo/parking_maneuver.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace parking_demo {

ParkingManeuverGenerator::ParkingManeuverGenerator() {
    ROS_INFO("ParkingManeuverGenerator initialized");
}

ParkingManeuverGenerator::~ParkingManeuverGenerator() {
}

void ParkingManeuverGenerator::setVehicleParams(const VehicleParams& params) {
    vehicle_params_ = params;
    ROS_INFO("Parking: Vehicle params set (length=%.2f, width=%.2f, wheelbase=%.2f)",
             params.length, params.width, params.wheelbase);
}

void ParkingManeuverGenerator::setPlannerParams(const PlannerParams& params) {
    planner_params_ = params;
    ROS_INFO("Parking: Planner params set (back_distance=%.2f, Rmin=%.2f)",
             params.parking_back_distance, params.regulatory_Rmin);
}

void ParkingManeuverGenerator::setParkingSlotPolygon(const std::vector<std::pair<double, double>>& polygon) {
    parking_slot_polygon_ = polygon;
    ROS_INFO("Parking slot polygon set: %lu vertices", polygon.size());
}

void ParkingManeuverGenerator::setParkingSlotConstraints(const ParkingSlotConstraints& constraints) {
    slot_constraints_ = constraints;
    has_constraints_ = true;
    ROS_INFO("[FIX] Parking slot constraints set:");
    ROS_INFO("   Entry center: (%.2f, %.2f), direction: %.1f°",
             constraints.entry_center_x, constraints.entry_center_y,
             constraints.entry_direction * 180 / M_PI);
    ROS_INFO("   Slot dimensions: %.2fm x %.2fm", constraints.slot_length, constraints.slot_width);
}

std::vector<CarState> ParkingManeuverGenerator::generateVerticalParking(const CarState& start,
                                                                         double target_x,
                                                                         double target_y,
                                                                         double target_theta) {
    ROS_INFO("Generating vertical parking trajectory (single arc reverse):");
    ROS_INFO("   Start: (%.2f, %.2f, %.2f°)", start.x, start.y, start.theta * 180 / M_PI);
    ROS_INFO("   Target: (%.2f, %.2f, %.2f°)", target_x, target_y, target_theta * 180 / M_PI);

    std::vector<CarState> trajectory;
    double dt = 0.05;  // 时间步长

    // 计算初始角度差
    CarState current = start;
    double angle_diff = target_theta - current.theta;
    // 归一化角度差
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff <= -M_PI) angle_diff += 2 * M_PI;

    ROS_INFO("   Angle difference: %.1f°", angle_diff * 180 / M_PI);

    // [FIX] 新策略：单一圆弧倒车，沿车道线旋转90度入库
    // 使用车辆最小转弯半径
    double R = vehicle_params_.min_turn_radius;
    if (R < 0.1) R = 3.0;  // 默认3米

    // 计算转向方向（右转或左转）
    double steering_sign = (angle_diff > 0) ? 1.0 : -1.0;
    double steering_angle = steering_sign * std::atan(vehicle_params_.wheelbase / R);

    // 限制在最大转向角内
    if (std::abs(steering_angle) > vehicle_params_.max_steering_rad) {
        steering_angle = steering_sign * vehicle_params_.max_steering_rad;
        R = vehicle_params_.wheelbase / std::tan(vehicle_params_.max_steering_rad);
    }

    ROS_INFO("   Single arc: R=%.2fm, steering=%.1f°", R, steering_angle * 180 / M_PI);

    // 一边倒车，一边旋转，直到到达目标
    double reverse_speed = -0.35;  // 倒车速度0.35m/s
    int max_steps = 5000;  // 最大步数，防止无限循环

    for (int i = 0; i < max_steps; ++i) {
        CarState s = current;
        s.v = reverse_speed;
        s.phi = steering_angle;

        // 使用阿克曼转向模型（倒车）
        s.x = current.x + reverse_speed * std::cos(current.theta) * dt;
        s.y = current.y + reverse_speed * std::sin(current.theta) * dt;
        s.theta = current.theta + (reverse_speed / vehicle_params_.wheelbase) * std::tan(steering_angle) * dt;

        // 归一化角度
        while (s.theta > M_PI) s.theta -= 2 * M_PI;
        while (s.theta <= -M_PI) s.theta += 2 * M_PI;

        trajectory.push_back(s);
        current = s;

        // 检查是否到达目标位置和朝向
        double dist_to_target = std::hypot(target_x - current.x, target_y - current.y);
        double angle_remaining = target_theta - current.theta;
        while (angle_remaining > M_PI) angle_remaining -= 2 * M_PI;
        while (angle_remaining <= -M_PI) angle_remaining += 2 * M_PI;

        // 提前终止条件：距离<0.3m 且 角度误差<5度
        if (dist_to_target < 0.3 && std::abs(angle_remaining) < 5.0 * M_PI / 180.0) {
            ROS_INFO("   Reached target at step %d (dist=%.2fm, angle=%.1f°)",
                     i, dist_to_target, angle_remaining * 180 / M_PI);
            break;
        }
    }

    // 确保最终状态精确
    if (!trajectory.empty()) {
        trajectory.back().x = target_x;
        trajectory.back().y = target_y;
        trajectory.back().theta = target_theta;
        trajectory.back().v = 0.0;
        trajectory.back().phi = 0.0;
    }

    ROS_INFO("[OK] Single arc parking trajectory: %lu states", trajectory.size());
    if (!trajectory.empty()) {
        ROS_INFO("   Final position: (%.2f, %.2f, %.2f°)",
                 trajectory.back().x, trajectory.back().y, trajectory.back().theta * 180 / M_PI);
    }

    return trajectory;
}

std::vector<CarState> ParkingManeuverGenerator::generateParallelParking(const CarState& start,
                                                                         double target_x,
                                                                         double target_y,
                                                                         double target_theta) {
    ROS_INFO("Generating parallel parking trajectory");

    std::vector<CarState> trajectory;

    // 简化的平行泊车：多段倒车弧
    double R = std::max(3.0, vehicle_params_.min_turn_radius);

    // 阶段1: 倒车右转
    double arc_angle = M_PI / 3;  // 60度
    int n_arc1 = 20;
    for (int i = 0; i <= n_arc1; ++i) {
        double t = static_cast<double>(i) / n_arc1;
        double theta = start.theta - arc_angle * t;

        CarState s;
        s.x = start.x - R * (std::sin(theta) - std::sin(start.theta));
        s.y = start.y + R * (std::cos(theta) - std::cos(start.theta));
        s.theta = theta;
        s.v = -0.4;
        s.phi = std::atan(vehicle_params_.wheelbase / R);
        trajectory.push_back(s);
    }

    // 阶段2: 倒车左转
    CarState pivot = trajectory.back();
    int n_arc2 = 20;
    for (int i = 1; i <= n_arc2; ++i) {
        double t = static_cast<double>(i) / n_arc2;
        double theta = pivot.theta + arc_angle * t;

        CarState s;
        s.x = pivot.x - R * (std::sin(theta) - std::sin(pivot.theta));
        s.y = pivot.y - R * (std::cos(theta) - std::cos(pivot.theta));
        s.theta = theta;
        s.v = -0.4;
        s.phi = -std::atan(vehicle_params_.wheelbase / R);
        trajectory.push_back(s);
    }

    // 阶段3: 调整到目标
    if (!trajectory.empty()) {
        CarState last = trajectory.back();
        double dx = target_x - last.x;
        double dy = target_y - last.y;
        double dist = std::hypot(dx, dy);

        if (dist > 0.1) {
            int n_adj = std::max(5, static_cast<int>(dist / 0.1));
            for (int i = 1; i <= n_adj; ++i) {
                double t = static_cast<double>(i) / n_adj;
                CarState s;
                s.x = last.x + dx * t;
                s.y = last.y + dy * t;
                s.theta = target_theta;
                s.v = -0.2;
                s.phi = 0.0;
                trajectory.push_back(s);
            }
        }
    }

    ROS_INFO("Parallel parking trajectory generated: %lu states", trajectory.size());
    return trajectory;
}

std::vector<CarState> ParkingManeuverGenerator::generateReedsSheppParking(const CarState& start,
                                                                          const CarState& goal) {
    ROS_WARN("Reeds-Shepp parking requires OMPL library (not implemented in this build)");
    // TODO: 如果有OMPL，在这里实现Reeds-Shepp规划
    return {};
}

std::vector<CarState> ParkingManeuverGenerator::generateTwoArcIngress(double x0, double y0, double theta0,
                                                                       double xt, double yt, double thetat) {
    // 两段圆弧倒车轨迹生成（简化版）
    std::vector<CarState> trajectory;

    double R = std::max(3.0, vehicle_params_.min_turn_radius);
    double step = 0.02;

    // 第一段圆弧
    double C1x = x0;
    double C1y = y0 + R;
    double a0 = std::atan2(y0 - C1y, x0 - C1x);
    double a_mid = M_PI / 2;

    double len1 = R * std::abs(a_mid - a0);
    int n1 = std::max(1, static_cast<int>(std::ceil(len1 / step)));

    for (int i = 0; i <= n1; ++i) {
        double t = static_cast<double>(i) / n1;
        double ang = a0 + t * (a_mid - a0);

        CarState s;
        s.x = C1x + R * std::cos(ang);
        s.y = C1y + R * std::sin(ang);
        s.theta = ang - M_PI / 2;
        s.v = -0.4;
        s.phi = std::atan(vehicle_params_.wheelbase / R);

        // 简单的碰撞检测
        if (!parking_slot_polygon_.empty() && !isFootprintInside(s)) {
            ROS_WARN("Two-arc trajectory collides with parking slot boundary");
            return {};
        }

        trajectory.push_back(s);
    }

    ROS_INFO("Two-arc ingress generated: %lu states", trajectory.size());
    return trajectory;
}

bool ParkingManeuverGenerator::isFootprintInside(const CarState& state) const {
    if (parking_slot_polygon_.empty()) return true;

    double l2 = vehicle_params_.length * 0.5;
    double w2 = vehicle_params_.width * 0.5;

    // 车辆四个角点（局部坐标系）
    std::vector<std::pair<double, double>> corners = {
        { l2,  w2}, { l2, -w2}, {-l2, -w2}, {-l2,  w2}
    };

    double c = std::cos(state.theta);
    double s = std::sin(state.theta);

    // 检查每个角点是否在多边形内
    for (const auto& pt : corners) {
        double wx = state.x + pt.first * c - pt.second * s;
        double wy = state.y + pt.first * s + pt.second * c;

        if (!pointInPolygon(parking_slot_polygon_, wx, wy)) {
            return false;
        }
    }

    return true;
}

bool ParkingManeuverGenerator::pointInPolygon(const std::vector<std::pair<double, double>>& poly,
                                              double x, double y) const {
    if (poly.empty()) return false;

    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        double xi = poly[i].first, yi = poly[i].second;
        double xj = poly[j].first, yj = poly[j].second;

        bool intersect = ((yi > y) != (yj > y)) &&
                        (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);
        if (intersect) inside = !inside;
    }

    return inside;
}

/**
 * @brief 检查线段是否相交
 */
bool ParkingManeuverGenerator::lineSegmentsIntersect(double x1, double y1, double x2, double y2,
                                                     double x3, double y3, double x4, double y4) const {
    // 使用叉积判断线段相交
    auto ccw = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax);
    };

    bool result = (ccw(x1, y1, x3, y3, x4, y4) != ccw(x2, y2, x3, y3, x4, y4)) &&
                  (ccw(x1, y1, x2, y2, x3, y3) != ccw(x1, y1, x2, y2, x4, y4));

    if (!result) {
        return false;
    }

    auto nearEndpoint = [](double ax, double ay, double bx, double by) {
        double dx = ax - bx;
        double dy = ay - by;
        return (dx * dx + dy * dy) < 1e-4; // 1cm阈值
    };

    // 允许端点接触，避免将可接受的贴边操作视为碰撞
    if (nearEndpoint(x1, y1, x3, y3) || nearEndpoint(x1, y1, x4, y4) ||
        nearEndpoint(x2, y2, x3, y3) || nearEndpoint(x2, y2, x4, y4)) {
        return false;
    }

    return true;
}

/**
 * @brief 检查轨迹是否穿越长边
 */
bool ParkingManeuverGenerator::checkLongEdgeCrossing(const std::vector<CarState>& trajectory) const {
    if (!has_constraints_ || trajectory.size() < 2) {
        return false;  // 没有约束或轨迹太短，不检查
    }

    // 长边A的两个端点
    double ax1 = slot_constraints_.long_edge_a_start.first;
    double ay1 = slot_constraints_.long_edge_a_start.second;
    double ax2 = slot_constraints_.long_edge_a_end.first;
    double ay2 = slot_constraints_.long_edge_a_end.second;

    // 长边B的两个端点
    double bx1 = slot_constraints_.long_edge_b_start.first;
    double by1 = slot_constraints_.long_edge_b_start.second;
    double bx2 = slot_constraints_.long_edge_b_end.first;
    double by2 = slot_constraints_.long_edge_b_end.second;

    // 检查轨迹的每一段是否与长边相交
    for (size_t i = 1; i < trajectory.size(); ++i) {
        double x1 = trajectory[i-1].x;
        double y1 = trajectory[i-1].y;
        double x2 = trajectory[i].x;
        double y2 = trajectory[i].y;

        // 检查与长边A相交
        if (lineSegmentsIntersect(x1, y1, x2, y2, ax1, ay1, ax2, ay2)) {
            return true;
        }

        // 检查与长边B相交
        if (lineSegmentsIntersect(x1, y1, x2, y2, bx1, by1, bx2, by2)) {
            return true;
        }
    }

    return false;
}

/**
 * @brief 生成约束感知的泊车轨迹（两段式倒车：直线倒车 + 转90度入库）
 */
std::vector<CarState> ParkingManeuverGenerator::generateConstraintAwareParking(const CarState& start) {
    if (!has_constraints_) {
        ROS_ERROR("[ERR] Cannot generate constraint-aware parking: constraints not set!");
        return {};
    }

    ROS_INFO("[FIX] Generating constraint-aware parking (two-phase: straight reverse + 90° turn):");
    ROS_INFO("   Start: (%.2f, %.2f, %.2f°)", start.x, start.y, start.theta * 180 / M_PI);
    ROS_INFO("   Target: (%.2f, %.2f, %.2f°)",
             slot_constraints_.target_x, slot_constraints_.target_y,
             slot_constraints_.target_theta * 180 / M_PI);
    ROS_INFO("   Entry center: (%.2f, %.2f)",
             slot_constraints_.entry_center_x, slot_constraints_.entry_center_y);

    std::vector<CarState> trajectory;
    double dt = 0.05;
    double reverse_speed = -0.35;  // 倒车速度
    CarState current = start;

    // [FIX] 三阶段泊车策略
    ROS_INFO("   ========================================");
    ROS_INFO("   Three-phase parking strategy");
    ROS_INFO("   Current: (%.2f, %.2f, %.1f°)", current.x, current.y, current.theta * 180 / M_PI);
    ROS_INFO("   Target:  (%.2f, %.2f, %.1f°)",
             slot_constraints_.target_x, slot_constraints_.target_y,
             slot_constraints_.target_theta * 180 / M_PI);

    // 计算到目标的距离
    double dx_to_target = slot_constraints_.target_x - current.x;
    double dy_to_target = slot_constraints_.target_y - current.y;
    double dist_to_target = std::hypot(dx_to_target, dy_to_target);

    ROS_INFO("   Distance to target: %.2fm", dist_to_target);

    // 计算需要的转弯半径（基于目标距离和90度转弯）
    // 假设转弯半径 R，90度转弯的弧长约为 R*π/2
    // 我们需要：直线倒车距离 + 转弯距离 ≈ 到目标的距离
    double R = 1.5;  // 使用1.5米转弯半径
    double turn_arc_length = R * M_PI / 2.0;  // 90度转弯弧长

    ROS_INFO("   Using turning radius: %.2fm (arc length: %.2fm)", R, turn_arc_length);

    // ========== Phase 1: 水平直线倒车 ==========
    double straight_reverse_dist = dist_to_target - turn_arc_length - 3.2;  // 留1.5米余量（减少0.5米）
    double min_straight_travel = 1.0;
    if (slot_constraints_.entry_width > 1e-3) {
        // 至少倒到入口短边附近，保证接下来可垂直入库
        min_straight_travel = std::max(min_straight_travel,
                                       slot_constraints_.entry_width * 0.5 + 0.5);
    }
    if (straight_reverse_dist < min_straight_travel) {
        straight_reverse_dist = min_straight_travel;
    }

    ROS_INFO("   Phase 1: Straight reverse %.2fm", straight_reverse_dist);

    double traveled = 0.0;
    while (traveled < straight_reverse_dist) {
        CarState s = current;
        s.v = reverse_speed;
        s.phi = 0.0;  // 转向角为0，保持直线

        s.x = current.x + reverse_speed * std::cos(current.theta) * dt;
        s.y = current.y + reverse_speed * std::sin(current.theta) * dt;
        s.theta = current.theta;  // 朝向不变

        trajectory.push_back(s);
        traveled += std::abs(reverse_speed * dt);
        current = s;
    }

    ROS_INFO("   Phase 1 completed: traveled %.2fm", traveled);
    ROS_INFO("   After Phase 1: pos=(%.2f, %.2f, %.1f°)",
             current.x, current.y, current.theta * 180 / M_PI);

    // ========== Phase 2: 边倒车边转向，直到与库位平行 ==========
    ROS_INFO("   Phase 2: Turning while reversing until parallel to slot");

    // 计算需要转的角度
    double target_theta = slot_constraints_.target_theta;
    double angle_to_turn = target_theta - current.theta;
    while (angle_to_turn > M_PI) angle_to_turn -= 2 * M_PI;
    while (angle_to_turn <= -M_PI) angle_to_turn += 2 * M_PI;

    ROS_INFO("   Need to rotate: %.1f°", angle_to_turn * 180 / M_PI);

    // 计算转向角
    double steering_sign = (angle_to_turn > 0) ? -1.0 : 1.0;
    double steering_angle = steering_sign * std::atan(vehicle_params_.wheelbase / R);

    // 限制转向角
    double max_steering = vehicle_params_.max_steering_rad;
    if (max_steering < 0.1) max_steering = 0.5;
    if (std::abs(steering_angle) > max_steering) {
        steering_angle = (steering_angle > 0) ? max_steering : -max_steering;
    }

    ROS_INFO("   Phase 2 steering angle: %.1f°", steering_angle * 180 / M_PI);

    // 持续转向直到与目标朝向平行（误差小于5度）
    double angle_tolerance = 3.0 * M_PI / 180.0;  // 5度容差
    int max_turn_steps = 2000;

    for (int i = 0; i < max_turn_steps; ++i) {
        CarState s = current;
        s.v = reverse_speed;
        s.phi = steering_angle;

        s.x = current.x + reverse_speed * std::cos(current.theta) * dt;
        s.y = current.y + reverse_speed * std::sin(current.theta) * dt;
        s.theta = current.theta + (reverse_speed / vehicle_params_.wheelbase) * std::tan(steering_angle) * dt;

        // 归一化角度
        while (s.theta > M_PI) s.theta -= 2 * M_PI;
        while (s.theta <= -M_PI) s.theta += 2 * M_PI;

        trajectory.push_back(s);
        current = s;

        // 检查是否已经平行
        double current_angle_diff = target_theta - current.theta;
        while (current_angle_diff > M_PI) current_angle_diff -= 2 * M_PI;
        while (current_angle_diff <= -M_PI) current_angle_diff += 2 * M_PI;

        if (i % 50 == 0) {
            ROS_INFO("   Phase 2 step %d: θ=%.1f° angle_diff=%.1f°",
                     i, current.theta * 180 / M_PI, current_angle_diff * 180 / M_PI);
        }

        double center_dx = current.x - slot_constraints_.entry_center_x;
        double center_dy = current.y - slot_constraints_.entry_center_y;
        double entry_normal_x = std::cos(slot_constraints_.entry_direction);
        double entry_normal_y = std::sin(slot_constraints_.entry_direction);
        double along_normal = center_dx * entry_normal_x + center_dy * entry_normal_y;
        double lateral_offset = -center_dx * entry_normal_y + center_dy * entry_normal_x;
        double lateral_limit = (slot_constraints_.entry_width > 1e-3)
                                   ? (slot_constraints_.entry_width * 0.5 - 0.1)
                                   : 0.4;
        bool heading_aligned = std::abs(current_angle_diff) < angle_tolerance;
        bool centered = std::abs(lateral_offset) < lateral_limit;
        bool at_entry = std::abs(along_normal) < 0.4;

        if (heading_aligned && centered && at_entry) {
            ROS_INFO("   [OK] Phase 2 completed at step %d: aligned & centered at entry", i);
            break;
        }
    }

    ROS_INFO("   After Phase 2: pos=(%.2f, %.2f, %.1f°)",
             current.x, current.y, current.theta * 180 / M_PI);

    // ========== Phase 3: 直线倒车进入库中 ==========
    ROS_INFO("   Phase 3: Straight reverse into slot");

    double dist_to_target_now = std::hypot(slot_constraints_.target_x - current.x,
                                           slot_constraints_.target_y - current.y);
    ROS_INFO("   Current distance to target: %.2fm", dist_to_target_now);

    double straight_speed = reverse_speed * 0.7;
    double step_distance = std::abs(straight_speed * dt);
    if (step_distance < 1e-4) {
        step_distance = 0.01;
    }

    int planned_steps = static_cast<int>(std::ceil(dist_to_target_now / step_distance));
    int phase3_steps = std::min(2000, std::max(planned_steps + 40, 80));
    ROS_INFO("   Planning %d straight-reverse steps (step=%.3fm)", phase3_steps, step_distance);

    for (int i = 0; i < phase3_steps; ++i) {
        CarState s = current;
        s.v = straight_speed;  // 稍微减速
        s.phi = 0.0;  // 直线倒车，转向角为0

        s.x = current.x + s.v * std::cos(current.theta) * dt;
        s.y = current.y + s.v * std::sin(current.theta) * dt;
        s.theta = current.theta;  // 保持朝向

        trajectory.push_back(s);
        current = s;

        // 检查是否到达目标
        double dist = std::hypot(slot_constraints_.target_x - current.x,
                                 slot_constraints_.target_y - current.y);

        if (i % 50 == 0) {
            ROS_INFO("   Phase 3 step %d/%d: dist=%.2fm", i, phase3_steps, dist);
        }

        if (dist < 0.3) {  // 距离目标小于30cm
            ROS_INFO("   [OK] Phase 3 completed early: dist=%.2fm", dist);
            break;
        }
    }

    double dist_after_phase3 = std::hypot(slot_constraints_.target_x - current.x,
                                          slot_constraints_.target_y - current.y);
    if (dist_after_phase3 > 0.05) {
        CarState final_reverse = current;
        final_reverse.x = slot_constraints_.target_x;
        final_reverse.y = slot_constraints_.target_y;
        // 保持 Phase 2 结束时的朝向，不强制跳变到 target_theta
        // 避免末端朝向不连续导致 Pure Pursuit 大幅打舵
        final_reverse.theta = current.theta;
        final_reverse.v = straight_speed;
        final_reverse.phi = 0.0;
        trajectory.push_back(final_reverse);
        current = final_reverse;
        ROS_INFO("   Phase 3 final approach added (dist=%.2fm)", dist_after_phase3);
    }

    // 添加停稳状态，确保最后一个点速度为0
    CarState stop_state = current;
    stop_state.v = 0.0;
    stop_state.phi = 0.0;
    trajectory.push_back(stop_state);

    ROS_INFO("   Final position: (%.2f, %.2f, %.1f°)",
             current.x, current.y, current.theta * 180 / M_PI);
    ROS_INFO("   Total trajectory states: %lu", trajectory.size());
    ROS_INFO("   ========================================");

    // [FIX] 设置最终状态：速度和转向归零
    if (!trajectory.empty()) {
        ROS_INFO("   [STOP] Final position: (%.2f, %.2f, %.1f°)",
                 trajectory.back().x, trajectory.back().y,
                 trajectory.back().theta * 180 / M_PI);
    }

    // [FIX] 验证轨迹不穿越长边（关键约束）
    if (checkLongEdgeCrossing(trajectory)) {
        ROS_ERROR("[ERR] Generated trajectory crosses long edges! REJECTING.");
        return {};  // 拒绝违规轨迹
    }

    ROS_INFO("[OK] Constraint-aware parking trajectory: %lu states (Phase1: straight, Phase2: turn+entry)",
             trajectory.size());
    return trajectory;
}

} // namespace parking_demo
