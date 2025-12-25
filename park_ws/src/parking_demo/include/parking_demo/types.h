#ifndef PARKING_DEMO_TYPES_H
#define PARKING_DEMO_TYPES_H

#include <vector>
#include <string>
#include <map>

namespace parking_demo {

// ============ 基础数据结构 ============

/**
 * @brief OSM节点（带经纬度和局部坐标）
 */
struct NodePoint {
    long long id;
    double lat;        // 纬度
    double lon;        // 经度
    double x;          // 局部X坐标（米）
    double y;          // 局部Y坐标（米）
};

/**
 * @brief OSM Way（路径/车道线）
 */
struct Way {
    long long id;
    std::vector<long long> node_refs;  // 组成该way的节点ID列表
    std::string relation;              // 关系类型（如"Successor"）
    std::string subtype;               // 子类型（如"dashed", "solid"）
    std::string wtype;                 // way类型（如"line_thin"）
};

/**
 * @brief OSM Relation（车道/区域组合）
 */
struct Relation {
    long long id;
    std::vector<long long> member_way_refs;  // 成员way的ID列表
    std::string type;                        // relation类型（如"multipolygon"）
};

/**
 * @brief 停车位
 */
struct ParkingSpot {
    long long id;
    double x = 0.0;
    double y = 0.0;
    bool from_way = false;  // true=基于way, false=基于node
};

/**
 * @brief 车位几何约束（用于约束感知路径规划）
 */
struct ParkingSlotConstraints {
    // 长边（不可穿越）
    std::pair<double, double> long_edge_a_start;  // 长边A起点
    std::pair<double, double> long_edge_a_end;    // 长边A终点
    std::pair<double, double> long_edge_b_start;  // 长边B起点
    std::pair<double, double> long_edge_b_end;    // 长边B终点

    // 短边（入口）
    std::pair<double, double> short_edge_entry_start;  // 入口短边起点
    std::pair<double, double> short_edge_entry_end;    // 入口短边终点
    std::pair<double, double> short_edge_back_start;   // 后短边起点
    std::pair<double, double> short_edge_back_end;     // 后短边终点

    // 入口信息
    double entry_center_x = 0.0;      // 入口中心X坐标
    double entry_center_y = 0.0;      // 入口中心Y坐标
    double entry_direction = 0.0;     // 入口法向量方向（指向车位内部）
    double entry_width = 0.0;         // 入口宽度

    // 车位尺寸
    double slot_length = 0.0;         // 车位长度（长边长度）
    double slot_width = 0.0;          // 车位宽度（短边长度）

    // 目标停车姿态
    double target_x = 0.0;
    double target_y = 0.0;
    double target_theta = 0.0;
};

/**
 * @brief 车辆状态
 */
struct CarState {
    double x = 0.0;      // X位置（米）
    double y = 0.0;      // Y位置（米）
    double theta = 0.0;  // 航向角（弧度）
    double v = 0.0;      // 速度（米/秒）
    double phi = 0.0;    // 转向角（弧度）
};

/**
 * @brief 车辆参数
 */
struct VehicleParams {
    double wheelbase = 2.7;              // 轴距（米）
    double max_steering_rad = 0.6;       // 最大转向角（弧度）
    double length = 4.6;                 // 车长（米）
    double width = 1.85;                 // 车宽（米）
    double rear_overhang = 1.0;          // 后悬（米）
    double front_overhang = 0.9;         // 前悬（米）
    double min_turn_radius = 2.8;        // 最小转弯半径（米）
};

/**
 * @brief 控制器参数
 */
struct ControllerParams {
    double lookahead_dist = 2.5;         // 前瞻距离（米）
    double lookahead_min = 1.0;          // 最小前瞻距离
    double lookahead_max = 4.0;          // 最大前瞻距离
    double lookahead_scale = 1.0;        // 前瞻缩放因子

    double kp_speed = 1.2;               // 速度控制增益
    double max_speed = 3.0;              // 最大速度（米/秒）
    double min_speed = 0.3;              // 最小速度（米/秒）

    double curvature_speed_gain = 3.0;   // 曲率速度惩罚系数
    double steering_rate_limit = 0.6;    // 转向速率限制（弧度/秒）
    double goal_tolerance = 0.25;        // 目标容差（米）
};

/**
 * @brief 规划参数
 */
struct PlannerParams {
    double parking_back_distance = 2.0;  // 泊车倒车距离（米）
    double regulatory_Rmin = 8.0;        // GB 7258最小外轮半径（米）
    double dsafe = 1.0;                  // 安全余量（米）
};

} // namespace parking_demo

#endif // PARKING_DEMO_TYPES_H
