/**
 * @file pure_pursuit_controller.cpp
 * @brief Pure Pursuit控制器实现
 */

#include "parking_demo/pure_pursuit_controller.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace parking_demo {

PurePursuitController::PurePursuitController() {
    ROS_INFO("PurePursuitController initialized");
}

PurePursuitController::~PurePursuitController() {
}

void PurePursuitController::setVehicleParams(const VehicleParams& params) {
    vehicle_params_ = params;
    ROS_INFO("Vehicle params set: wheelbase=%.2f m, max_steering=%.3f rad",
             params.wheelbase, params.max_steering_rad);
}

void PurePursuitController::setControllerParams(const ControllerParams& params) {
    controller_params_ = params;
    ROS_INFO("Controller params set: lookahead=%.2f m, kp_speed=%.2f, max_speed=%.2f",
             params.lookahead_dist, params.kp_speed, params.max_speed);
}

void PurePursuitController::setGlobalPath(const std::vector<CarState>& path) {
    global_path_ = path;
    current_idx_ = 0;
    last_motion_sign_ = 1.0;
    ROS_INFO("Global path set: %lu waypoints", path.size());
}

void PurePursuitController::reset() {
    current_idx_ = 0;
    last_steering_ = 0.0;
    last_motion_sign_ = 1.0;
    ROS_INFO("Controller reset");
}

CarState PurePursuitController::computeControl(const CarState& current_state, double dt) {
    if (global_path_.empty()) {
        ROS_WARN_THROTTLE(1.0, "No global path set!");
        return current_state;
    }

    double dist_to_goal = std::hypot(global_path_.back().x - current_state.x,
                                     global_path_.back().y - current_state.y);
    bool near_goal = dist_to_goal < std::max(1.0, controller_params_.goal_tolerance * 4.0);

    // 1. 找前瞻点
    double lookahead = controller_params_.lookahead_dist;
    lookahead = std::min(std::max(lookahead, controller_params_.lookahead_min),
                         controller_params_.lookahead_max);
    if (near_goal) {
        // 收紧前瞻距离，避免在车位内部获取过远的目标点
        double reduced = std::max(controller_params_.lookahead_min * 0.5, dist_to_goal * 0.7);
        lookahead = std::min(lookahead, reduced);
    }
    int look_idx = findLookaheadPoint(current_state, lookahead);

    if (look_idx >= static_cast<int>(global_path_.size())) {
        look_idx = global_path_.size() - 1;
    }

    lookahead_point_ = {global_path_[look_idx].x, global_path_[look_idx].y};

    // 2. 计算转向角
    double Ld = std::hypot(lookahead_point_.first - current_state.x,
                          lookahead_point_.second - current_state.y);

    double steering = computeSteeringAngle(current_state, lookahead_point_, Ld);
    if (near_goal) {
        double fade = std::min(std::max(dist_to_goal / 1.0, 0.0), 1.0);
        steering *= fade;
    }

    // 3. 计算速度
    double angle_to_target = std::atan2(lookahead_point_.second - current_state.y,
                                        lookahead_point_.first - current_state.x);
    double alpha = normalizeAngle(angle_to_target - current_state.theta);
    double curvature = 0.0;
    if (Ld > 1e-6) {
        curvature = 2.0 * std::sin(alpha) / Ld;
    }

    double speed = computeTargetSpeed(current_state, Ld, curvature);
    if (near_goal) {
        double slow_factor = std::min(std::max(dist_to_goal / 1.5, 0.2), 1.0);
        speed *= slow_factor;
        double terminal_min = controller_params_.min_speed * 0.5;
        if (dist_to_goal < 0.8) {
            speed = std::min(speed, std::max(terminal_min, dist_to_goal * 0.8));
        }
    }

    // [FIX] 修正：直接使用轨迹点的速度方向，不根据alpha判断
    // 检查轨迹点的目标速度符号，保持一致
    double motion_sign = last_motion_sign_;
    if (look_idx < static_cast<int>(global_path_.size())) {
        double target_v = global_path_[look_idx].v;
        if (target_v < -1e-3) {
            motion_sign = -1.0;
        } else if (target_v > 1e-3) {
            motion_sign = 1.0;
        }
    }
    speed = motion_sign * std::abs(speed);
    last_motion_sign_ = motion_sign;

    // 4. 转向速率限制
    double max_delta = controller_params_.steering_rate_limit * dt;
    double delta = steering - last_steering_;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    steering = last_steering_ + delta;

    // 饱和到最大转向角
    if (steering > vehicle_params_.max_steering_rad) {
        steering = vehicle_params_.max_steering_rad;
    }
    if (steering < -vehicle_params_.max_steering_rad) {
        steering = -vehicle_params_.max_steering_rad;
    }

    last_steering_ = steering;

    // 5. 更新车辆状态（运动学模型）
    CarState next_state = current_state;

    double xdot = speed * std::cos(current_state.theta);
    double ydot = speed * std::sin(current_state.theta);
    double thetadot = 0.0;
    if (std::abs(std::cos(steering)) > 1e-6) {
        thetadot = speed / vehicle_params_.wheelbase * std::tan(steering);
    }

    next_state.x += xdot * dt;
    next_state.y += ydot * dt;
    next_state.theta = normalizeAngle(current_state.theta + thetadot * dt);
    next_state.v = speed;
    next_state.phi = steering;

    // 6. 推进路径索引
    while (current_idx_ < static_cast<int>(global_path_.size())) {
        double dx = global_path_[current_idx_].x - next_state.x;
        double dy = global_path_[current_idx_].y - next_state.y;
        double dist = std::hypot(dx, dy);

        // [FIX] 修正：放宽推进阈值，让路径点更容易推进，避免卡住
        double pass_threshold = 0.5;  // 固定0.5米，更容易推进

        if (dist < pass_threshold && current_idx_ < static_cast<int>(global_path_.size()) - 1) {
            current_idx_++;
        } else {
            break;
        }
    }

    return next_state;
}

bool PurePursuitController::isGoalReached() const {
    if (global_path_.empty()) return false;
    return current_idx_ >= static_cast<int>(global_path_.size()) - 1;
}

int PurePursuitController::findLookaheadPoint(const CarState& current_state, double lookahead_dist) {
    int look_idx = current_idx_;

    for (int i = current_idx_; i < static_cast<int>(global_path_.size()); ++i) {
        double dx = global_path_[i].x - current_state.x;
        double dy = global_path_[i].y - current_state.y;
        double dist = std::hypot(dx, dy);

        if (dist >= lookahead_dist) {
            look_idx = i;
            break;
        }
    }

    // fallback: 如果没找到，使用最后一个点
    if (look_idx == current_idx_ && current_idx_ < static_cast<int>(global_path_.size()) - 1) {
        look_idx = global_path_.size() - 1;
    }

    return look_idx;
}

double PurePursuitController::computeSteeringAngle(const CarState& current_state,
                                                   const std::pair<double, double>& target_point,
                                                   double lookahead_dist) {
    double angle_to_target = std::atan2(target_point.second - current_state.y,
                                        target_point.first - current_state.x);
    double alpha = normalizeAngle(angle_to_target - current_state.theta);

    // Pure Pursuit curvature
    double curvature = 0.0;
    if (lookahead_dist > 1e-6) {
        curvature = 2.0 * std::sin(alpha) / lookahead_dist;
    }

    // 转换为转向角
    double steering = std::atan(vehicle_params_.wheelbase * curvature);

    return steering;
}

double PurePursuitController::computeTargetSpeed(const CarState& current_state,
                                                double lookahead_dist,
                                                double curvature) {
    // 基于前瞻距离的比例控制
    double speed = controller_params_.kp_speed * lookahead_dist;

    // 曲率惩罚（降低曲线速度）
    double curpen = 1.0 + controller_params_.curvature_speed_gain * 0.3 * std::abs(curvature);
    speed = speed / curpen;

    // 限制在最大最小速度之间
    if (speed > controller_params_.max_speed) {
        speed = controller_params_.max_speed;
    }

    // 改进的最小速度控制
    double effective_min_speed = controller_params_.min_speed;

    // 根据曲率调整最小速度
    if (std::abs(curvature) > 0.2) {
        effective_min_speed = controller_params_.min_speed * 1.2;
    }

    if (std::abs(speed) < effective_min_speed) {
        speed = effective_min_speed;
    }

    return speed;
}

double PurePursuitController::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle <= -M_PI) angle += 2 * M_PI;
    return angle;
}

} // namespace parking_demo
