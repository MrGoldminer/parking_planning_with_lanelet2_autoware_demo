#ifndef PARKING_DEMO_PARKING_MANEUVER_H
#define PARKING_DEMO_PARKING_MANEUVER_H

#include "types.h"
#include <vector>

namespace parking_demo {

/**
 * @brief 泊车轨迹生成器
 *
 * 职责：
 * - 生成垂直泊车轨迹
 * - 生成平行泊车轨迹
 * - 基于GB 7258法规生成合规轨迹
 * - 碰撞检测
 */
class ParkingManeuverGenerator {
public:
    ParkingManeuverGenerator();
    ~ParkingManeuverGenerator();

    /**
     * @brief 设置车辆参数
     */
    void setVehicleParams(const VehicleParams& params);

    /**
     * @brief 设置规划器参数
     */
    void setPlannerParams(const PlannerParams& params);

    /**
     * @brief 生成垂直泊车轨迹
     * @param start 起始状态
     * @param target_x 目标X坐标
     * @param target_y 目标Y坐标
     * @param target_theta 目标朝向
     * @return 轨迹序列
     */
    std::vector<CarState> generateVerticalParking(const CarState& start,
                                                   double target_x,
                                                   double target_y,
                                                   double target_theta);

    /**
     * @brief 生成平行泊车轨迹
     * @param start 起始状态
     * @param target_x 目标X坐标
     * @param target_y 目标Y坐标
     * @param target_theta 目标朝向
     * @return 轨迹序列
     */
    std::vector<CarState> generateParallelParking(const CarState& start,
                                                   double target_x,
                                                   double target_y,
                                                   double target_theta);

    /**
     * @brief 生成Reeds-Shepp泊车轨迹（需要OMPL）
     */
    std::vector<CarState> generateReedsSheppParking(const CarState& start,
                                                     const CarState& goal);

    /**
     * @brief 设置停车槽多边形（用于碰撞检测）
     */
    void setParkingSlotPolygon(const std::vector<std::pair<double, double>>& polygon);

    /**
     * @brief 设置车位几何约束（用于约束感知路径规划）
     */
    void setParkingSlotConstraints(const ParkingSlotConstraints& constraints);

    /**
     * @brief 生成约束感知的泊车轨迹（两阶段策略）
     * @param start 起始状态
     * @return 轨迹序列
     */
    std::vector<CarState> generateConstraintAwareParking(const CarState& start);

    /**
     * @brief 检查车辆footprint是否在停车槽内
     */
    bool isFootprintInside(const CarState& state) const;

private:
    /**
     * @brief 生成两段圆弧倒车轨迹（GB 7258合规）
     */
    std::vector<CarState> generateTwoArcIngress(double x0, double y0, double theta0,
                                               double xt, double yt, double thetat);

    /**
     * @brief 检查轨迹是否穿越长边
     */
    bool checkLongEdgeCrossing(const std::vector<CarState>& trajectory) const;

    /**
     * @brief 检查线段是否相交
     */
    bool lineSegmentsIntersect(double x1, double y1, double x2, double y2,
                              double x3, double y3, double x4, double y4) const;

    /**
     * @brief 点是否在多边形内
     */
    bool pointInPolygon(const std::vector<std::pair<double, double>>& poly,
                       double x, double y) const;

    VehicleParams vehicle_params_;
    PlannerParams planner_params_;
    std::vector<std::pair<double, double>> parking_slot_polygon_;
    ParkingSlotConstraints slot_constraints_;
    bool has_constraints_ = false;  // 是否已设置约束
};

} // namespace parking_demo

#endif // PARKING_DEMO_PARKING_MANEUVER_H
