#ifndef PARKING_DEMO_PURE_PURSUIT_CONTROLLER_H
#define PARKING_DEMO_PURE_PURSUIT_CONTROLLER_H

#include "types.h"
#include <vector>

namespace parking_demo {

/**
 * @brief Pure Pursuit路径跟踪控制器
 *
 * 职责：
 * - 计算前瞻点
 * - 计算转向角
 * - 速度控制
 * - 轨迹跟踪
 */
class PurePursuitController {
public:
    PurePursuitController();
    ~PurePursuitController();

    /**
     * @brief 设置车辆参数
     */
    void setVehicleParams(const VehicleParams& params);

    /**
     * @brief 设置控制器参数
     */
    void setControllerParams(const ControllerParams& params);

    /**
     * @brief 设置全局路径
     */
    void setGlobalPath(const std::vector<CarState>& path);

    /**
     * @brief 执行一步控制
     * @param current_state 当前车辆状态
     * @param dt 时间步长（秒）
     * @return 更新后的车辆状态（包含控制指令v, phi）
     */
    CarState computeControl(const CarState& current_state, double dt);

    /**
     * @brief 检查是否到达目标
     * @return true如果到达终点
     */
    bool isGoalReached() const;

    /**
     * @brief 获取当前路径索引
     */
    int getCurrentIndex() const { return current_idx_; }

    /**
     * @brief 获取前瞻点（用于可视化）
     */
    std::pair<double, double> getLookaheadPoint() const { return lookahead_point_; }

    /**
     * @brief 重置控制器
     */
    void reset();

private:
    /**
     * @brief 查找前瞻点
     * @return 前瞻点在路径中的索引
     */
    int findLookaheadPoint(const CarState& current_state, double lookahead_dist);

    /**
     * @brief 计算Pure Pursuit转向角
     */
    double computeSteeringAngle(const CarState& current_state,
                               const std::pair<double, double>& target_point,
                               double lookahead_dist);

    /**
     * @brief 计算目标速度
     */
    double computeTargetSpeed(const CarState& current_state,
                             double lookahead_dist,
                             double curvature);

    /**
     * @brief 角度归一化到[-π, π]
     */
    double normalizeAngle(double angle);

    VehicleParams vehicle_params_;
    ControllerParams controller_params_;

    std::vector<CarState> global_path_;
    int current_idx_ = 0;

    std::pair<double, double> lookahead_point_{0, 0};  // 前瞻点坐标
    double last_steering_ = 0.0;                       // 上一次转向角（用于速率限制）
};

} // namespace parking_demo

#endif // PARKING_DEMO_PURE_PURSUIT_CONTROLLER_H
