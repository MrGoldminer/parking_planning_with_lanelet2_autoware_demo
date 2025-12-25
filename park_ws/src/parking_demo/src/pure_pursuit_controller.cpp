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
    ROS_INFO("Global path set: %lu waypoints", path.size());
}

void PurePursuitController::reset() {
    current_idx_ = 0;
    last_steering_ = 0.0;
    ROS_INFO("Controller reset");
}

CarState PurePursuitController::computeControl(const CarState& current_state, double dt) {
    if (global_path_.empty()) {
        ROS_WARN_THROTTLE(1.0, "No global path set!");
        return current_state;
    }

    // 1. 找前瞻点
    double lookahead = controller_params_.lookahead_dist;
    int look_idx = findLookaheadPoint(current_state, lookahead);

    if (look_idx >= static_cast<int>(global_path_.size())) {
        look_idx = global_path_.size() - 1;
    }

    lookahead_point_ = {global_path_[look_idx].x, global_path_[look_idx].y};

    // 2. 计算转向角
    double Ld = std::hypot(lookahead_point_.first - current_state.x,
                          lookahead_point_.second - current_state.y);

    double steering = computeSteeringAngle(current_state, lookahead_point_, Ld);

    // 3. 计算速度
    double angle_to_target = std::atan2(lookahead_point_.second - current_state.y,
                                        lookahead_point_.first - current_state.x);
    double alpha = normalizeAngle(angle_to_target - current_state.theta);
    double curvature = 0.0;
    if (Ld > 1e-6) {
        curvature = 2.0 * std::sin(alpha) / Ld;
    }

    double speed = computeTargetSpeed(current_state, Ld, curvature);

    // 🔧 修正：直接使用轨迹点的速度方向，不根据alpha判断
    // 检查轨迹点的目标速度符号，保持一致
    if (look_idx < static_cast<int>(global_path_.size())) {
        double target_v = global_path_[look_idx].v;
        if (target_v < 0) {
            // 轨迹要求倒车，确保速度为负
            speed = -std::abs(speed);
            // 倒车时转向角符号不变（轨迹已经考虑了倒车的转向）
        } else {
            // 轨迹要求前进，确保速度为正
            speed = std::abs(speed);
        }
    }

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

        // 🔧 修正：放宽推进阈值，让路径点更容易推进，避免卡住
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
