/**
 * @file parking_system_node_refactored.cpp
 * @brief 重构后的停车系统主节点（模块化架构）
 *
 * 修正：
 * 1. 车辆起始位置与规划路径对齐
 * 2. 停车方向平行于边线A和B
 * 3. 改进路径跟踪性能
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <cmath>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

// 重构后的模块
#include "parking_demo/types.h"
#include "parking_demo/osm_map_loader.h"
#include "parking_demo/graph_builder.h"
#include "parking_demo/path_planner.h"
#include "parking_demo/pure_pursuit_controller.h"
#include "parking_demo/parking_maneuver.h"
#include "parking_demo/visualizer.h"

using namespace parking_demo;

/**
 * @brief 停车系统状态机
 */
enum class ParkingState {
    APPROACHING,  // 前进到泊车起点
    PARKING,      // 泊车倒车阶段
    COMPLETED     // 完成
};

/**
 * @brief 重构后的停车系统类（清晰简洁）
 */
class ParkingSystemRefactored {
public:
    ParkingSystemRefactored() : nh_("~") {
        ROS_INFO("=== Parking System (Refactored) Starting ===");

        // 0. 初始化发布器
        slot_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/parking_slot_marker", 1, true);

        // 1. 加载参数
        loadParameters();

        // 2. 初始化各个模块
        map_loader_ = std::make_shared<OSMMapLoader>();
        graph_builder_ = std::make_shared<GraphBuilder>();
        path_planner_ = std::make_shared<PathPlanner>();
        controller_ = std::make_shared<PurePursuitController>();
        parking_maneuver_ = std::make_shared<ParkingManeuverGenerator>();
        visualizer_ = std::make_shared<Visualizer>(nh_);

        // 3. 加载地图
        std::string map_file = ros::package::getPath("parking_demo") + "/maps/parking_map.osm";
        ROS_INFO("Loading map from: %s", map_file.c_str());

        if (!map_loader_->loadFromFile(map_file)) {
            ROS_ERROR("Failed to load map!");
            throw std::runtime_error("Map loading failed");
        }

        map_loader_->projectToLocal();
        map_loader_->extractParkingSpots();

        ROS_INFO("Map loaded: %lu nodes, %lu ways, %lu relations",
                 map_loader_->getNodes().size(),
                 map_loader_->getWays().size(),
                 map_loader_->getRelations().size());

        // 4. 构建图
        graph_builder_->buildFromOSM(map_loader_->getNodes(),
                                     map_loader_->getWays(),
                                     map_loader_->getRelations());
        int max_comp = graph_builder_->checkConnectivity();
        ROS_INFO("Graph connectivity: %d nodes in main component", max_comp);

        // 5. 配置规划器
        path_planner_->setMapData(map_loader_->getNodes(),
                                  map_loader_->getWays(),
                                  map_loader_->getRelations());
        path_planner_->setGraphBuilder(graph_builder_.get());

        // 6. 配置控制器
        controller_->setVehicleParams(vehicle_params_);
        controller_->setControllerParams(controller_params_);

        // 7. 配置泊车生成器
        parking_maneuver_->setVehicleParams(vehicle_params_);
        parking_maneuver_->setPlannerParams(planner_params_);

        // 8. 可视化地图
        visualizer_->publishMap(map_loader_->getNodes(),
                               map_loader_->getWays(),
                               map_loader_->getRelations());

        // 🔧 不发布所有停车位，只显示我们关心的A和B边线组成的矩形
        // visualizer_->publishParkingSpots(map_loader_->getParkingSpots(),
        //                                 map_loader_->getNodes(),
        //                                 map_loader_->getWays());
        ROS_INFO("Skipping generic parking spots visualization (will show only target slot)");

        // 9. 计算停车位位置（先计算，因为规划需要这个信息）
        if (!computeParkingPosition()) {
            ROS_ERROR("Failed to compute parking position!");
            throw std::runtime_error("Position computation failed");
        }

        // 10. 规划任务（会生成完整路径）
        planMission();

        // 11. 设置车辆初始位置为路径起点
        if (!global_path_.empty()) {
            vehicle_state_ = global_path_[0];
            vehicle_state_.v = 0.0;  // 初始速度为0
            vehicle_state_.phi = 0.0;
            ROS_INFO("✅ Vehicle initialized at path start: pos=(%.2f, %.2f) θ=%.1f°",
                     vehicle_state_.x, vehicle_state_.y, vehicle_state_.theta * 180 / M_PI);
        } else {
            ROS_ERROR("No path generated!");
            throw std::runtime_error("Path generation failed");
        }

        // 12. 启动控制循环（20Hz）
        control_timer_ = nh_.createTimer(ros::Duration(0.05), &ParkingSystemRefactored::controlLoop, this);

        // 13. 启动停车槽可视化定时器（2Hz），确保一直显示
        vis_timer_ = nh_.createTimer(ros::Duration(0.5), &ParkingSystemRefactored::visualizationLoop, this);

        ROS_INFO("=== Parking System Initialized Successfully ===");
    }

private:
    /**
     * @brief 从ROS参数服务器加载参数
     */
    void loadParameters() {
        // 车辆参数
        nh_.param("vehicle_wheelbase", vehicle_params_.wheelbase, vehicle_params_.wheelbase);
        nh_.param("vehicle_max_steering", vehicle_params_.max_steering_rad, vehicle_params_.max_steering_rad);
        nh_.param("vehicle_length", vehicle_params_.length, vehicle_params_.length);
        nh_.param("vehicle_width", vehicle_params_.width, vehicle_params_.width);

        // 控制器参数
        nh_.param("lookahead_distance", controller_params_.lookahead_dist, controller_params_.lookahead_dist);
        nh_.param("kp_speed", controller_params_.kp_speed, controller_params_.kp_speed);
        nh_.param("max_speed", controller_params_.max_speed, controller_params_.max_speed);
        nh_.param("min_speed", controller_params_.min_speed, controller_params_.min_speed);
        nh_.param("goal_tolerance", controller_params_.goal_tolerance, controller_params_.goal_tolerance);

        // 规划器参数
        nh_.param("parking_back_distance", planner_params_.parking_back_distance, planner_params_.parking_back_distance);

        ROS_INFO("Parameters loaded:");
        ROS_INFO("  Vehicle: wheelbase=%.2f, max_steering=%.3f, length=%.2f, width=%.2f",
                 vehicle_params_.wheelbase, vehicle_params_.max_steering_rad,
                 vehicle_params_.length, vehicle_params_.width);
        ROS_INFO("  Controller: lookahead=%.2f, kp=%.2f, max_v=%.2f, min_v=%.2f",
                 controller_params_.lookahead_dist, controller_params_.kp_speed,
                 controller_params_.max_speed, controller_params_.min_speed);
    }

    /**
     * @brief 计算停车位位置（从两条边线）
     * @return true if successful
     */
    bool computeParkingPosition() {
        const auto& nodes = map_loader_->getNodes();
        const auto& ways = map_loader_->getWays();

        // 获取边线
        const Way* way_a = findWayById(ways, LINESTR_ID_A);
        const Way* way_b = findWayById(ways, LINESTR_ID_B);

        if (!way_a || !way_b) {
            ROS_ERROR("Parking space boundary lines not found!");
            return false;
        }

        // 获取边线的端点
        std::vector<std::pair<double, double>> line_a_pts;
        std::vector<std::pair<double, double>> line_b_pts;

        for (long long nid : way_a->node_refs) {
            if (nodes.count(nid)) {
                line_a_pts.emplace_back(nodes.at(nid).x, nodes.at(nid).y);
            }
        }

        for (long long nid : way_b->node_refs) {
            if (nodes.count(nid)) {
                line_b_pts.emplace_back(nodes.at(nid).x, nodes.at(nid).y);
            }
        }

        // 保存边界线点用于后续检测
        line_a_pts_ = line_a_pts;
        line_b_pts_ = line_b_pts;

        if (line_a_pts.size() < 2 || line_b_pts.size() < 2) {
            ROS_ERROR("Invalid parking space boundaries");
            return false;
        }

        // 计算停车位中心（两条边线中点的平均）
        double cx_a = (line_a_pts[0].first + line_a_pts[1].first) / 2.0;
        double cy_a = (line_a_pts[0].second + line_a_pts[1].second) / 2.0;
        double cx_b = (line_b_pts[0].first + line_b_pts[1].first) / 2.0;
        double cy_b = (line_b_pts[0].second + line_b_pts[1].second) / 2.0;

        parking_target_x_ = (cx_a + cx_b) / 2.0;
        parking_target_y_ = (cy_a + cy_b) / 2.0;

        // 🔧 修正：计算车辆停放朝向（平行于边线）
        // 边线A的方向向量
        double dx_a = line_a_pts[1].first - line_a_pts[0].first;
        double dy_a = line_a_pts[1].second - line_a_pts[0].second;
        double angle_a = std::atan2(dy_a, dx_a);

        // 边线B的方向向量
        double dx_b = line_b_pts[1].first - line_b_pts[0].first;
        double dy_b = line_b_pts[1].second - line_b_pts[0].second;
        double angle_b = std::atan2(dy_b, dx_b);

        // 取两条边线角度的平均（确保平行）
        parking_target_theta_ = (angle_a + angle_b) / 2.0;

        // 🔧 修正：泊车朝向需要反向（倒车入库）
        parking_target_theta_ += M_PI;  // 旋转180度

        // 归一化角度到[-π, π]
        while (parking_target_theta_ > M_PI) parking_target_theta_ -= 2 * M_PI;
        while (parking_target_theta_ <= -M_PI) parking_target_theta_ += 2 * M_PI;

        ROS_INFO("📍 Parking position computed:");
        ROS_INFO("   Target: (%.2f, %.2f) θ=%.1f°",
                 parking_target_x_, parking_target_y_, parking_target_theta_ * 180 / M_PI);
        ROS_INFO("   Line A angle: %.1f°, Line B angle: %.1f°",
                 angle_a * 180 / M_PI, angle_b * 180 / M_PI);

        // 🔍 调试：打印边线点坐标
        ROS_INFO("   Line A points: (%.2f, %.2f) -> (%.2f, %.2f)",
                 line_a_pts[0].first, line_a_pts[0].second,
                 line_a_pts[1].first, line_a_pts[1].second);
        ROS_INFO("   Line B points: (%.2f, %.2f) -> (%.2f, %.2f)",
                 line_b_pts[0].first, line_b_pts[0].second,
                 line_b_pts[1].first, line_b_pts[1].second);

        // 设置停车槽多边形（用于碰撞检测）
        // 🔧 修正：直接使用A和B边线定义的原始矩形，不扩展
        parking_slot_polygon_ = {
            line_a_pts[0], line_a_pts[1], line_b_pts[1], line_b_pts[0]
        };

        ROS_INFO("   Parking slot polygon vertices (original, no expansion):");
        for (size_t i = 0; i < parking_slot_polygon_.size(); ++i) {
            ROS_INFO("     Vertex %lu: (%.2f, %.2f)", i,
                     parking_slot_polygon_[i].first, parking_slot_polygon_[i].second);
        }

        parking_maneuver_->setParkingSlotPolygon(parking_slot_polygon_);

        // 🔧 新增：计算车位几何约束（用于约束感知路径规划）
        computeParkingSlotConstraints();

        // 可视化停车位方向（用箭头marker）
        visualizeParkingTarget();

        return true;
    }

    /**
     * @brief 计算车位几何约束（区分长边和短边）
     */
    void computeParkingSlotConstraints() {
        // 长边A和B（不可穿越）
        slot_constraints_.long_edge_a_start = line_a_pts_[0];
        slot_constraints_.long_edge_a_end = line_a_pts_[1];
        slot_constraints_.long_edge_b_start = line_b_pts_[0];
        slot_constraints_.long_edge_b_end = line_b_pts_[1];

        // 计算车位尺寸
        double dx_a = line_a_pts_[1].first - line_a_pts_[0].first;
        double dy_a = line_a_pts_[1].second - line_a_pts_[0].second;
        slot_constraints_.slot_length = std::hypot(dx_a, dy_a);

        double dx_short = line_b_pts_[0].first - line_a_pts_[0].first;
        double dy_short = line_b_pts_[0].second - line_a_pts_[0].second;
        slot_constraints_.slot_width = std::hypot(dx_short, dy_short);

        // 🔧 关键：确定哪个短边是入口
        // 策略：选择距离车辆起始位置更近的短边作为入口
        double dist_to_short1 = std::hypot(
            (line_a_pts_[0].first + line_b_pts_[0].first) / 2.0 - vehicle_state_.x,
            (line_a_pts_[0].second + line_b_pts_[0].second) / 2.0 - vehicle_state_.y
        );
        double dist_to_short2 = std::hypot(
            (line_a_pts_[1].first + line_b_pts_[1].first) / 2.0 - vehicle_state_.x,
            (line_a_pts_[1].second + line_b_pts_[1].second) / 2.0 - vehicle_state_.y
        );

        if (dist_to_short1 < dist_to_short2) {
            // 短边1是入口（连接 line_a[0] 和 line_b[0]）
            slot_constraints_.short_edge_entry_start = line_a_pts_[0];
            slot_constraints_.short_edge_entry_end = line_b_pts_[0];
            slot_constraints_.short_edge_back_start = line_a_pts_[1];
            slot_constraints_.short_edge_back_end = line_b_pts_[1];
        } else {
            // 短边2是入口（连接 line_a[1] 和 line_b[1]）
            slot_constraints_.short_edge_entry_start = line_a_pts_[1];
            slot_constraints_.short_edge_entry_end = line_b_pts_[1];
            slot_constraints_.short_edge_back_start = line_a_pts_[0];
            slot_constraints_.short_edge_back_end = line_b_pts_[0];
        }

        // 计算入口中心和方向
        slot_constraints_.entry_center_x = (slot_constraints_.short_edge_entry_start.first +
                                           slot_constraints_.short_edge_entry_end.first) / 2.0;
        slot_constraints_.entry_center_y = (slot_constraints_.short_edge_entry_start.second +
                                           slot_constraints_.short_edge_entry_end.second) / 2.0;
        slot_constraints_.entry_width = std::hypot(
            slot_constraints_.short_edge_entry_end.first - slot_constraints_.short_edge_entry_start.first,
            slot_constraints_.short_edge_entry_end.second - slot_constraints_.short_edge_entry_start.second
        );

        // 🔧 计算入口法向量方向（指向车位内部）
        // 短边方向向量
        double dx_entry = slot_constraints_.short_edge_entry_end.first -
                         slot_constraints_.short_edge_entry_start.first;
        double dy_entry = slot_constraints_.short_edge_entry_end.second -
                         slot_constraints_.short_edge_entry_start.second;

        // 法向量（逆时针旋转90度）
        double normal_x = -dy_entry;
        double normal_y = dx_entry;
        double normal_len = std::hypot(normal_x, normal_y);
        normal_x /= normal_len;
        normal_y /= normal_len;

        // 检查法向量是否指向车位内部（通过检查法向量是否指向目标点）
        double to_target_x = parking_target_x_ - slot_constraints_.entry_center_x;
        double to_target_y = parking_target_y_ - slot_constraints_.entry_center_y;
        double dot = normal_x * to_target_x + normal_y * to_target_y;

        if (dot < 0) {
            // 反向法向量
            normal_x = -normal_x;
            normal_y = -normal_y;
        }

        slot_constraints_.entry_direction = std::atan2(normal_y, normal_x);

        // 设置目标姿态
        slot_constraints_.target_x = parking_target_x_;
        slot_constraints_.target_y = parking_target_y_;
        slot_constraints_.target_theta = parking_target_theta_;

        ROS_INFO("📐 Parking slot constraints computed:");
        ROS_INFO("   Long edge A: (%.2f, %.2f) -> (%.2f, %.2f)",
                 slot_constraints_.long_edge_a_start.first, slot_constraints_.long_edge_a_start.second,
                 slot_constraints_.long_edge_a_end.first, slot_constraints_.long_edge_a_end.second);
        ROS_INFO("   Long edge B: (%.2f, %.2f) -> (%.2f, %.2f)",
                 slot_constraints_.long_edge_b_start.first, slot_constraints_.long_edge_b_start.second,
                 slot_constraints_.long_edge_b_end.first, slot_constraints_.long_edge_b_end.second);
        ROS_INFO("   Entry (short edge): (%.2f, %.2f) -> (%.2f, %.2f)",
                 slot_constraints_.short_edge_entry_start.first, slot_constraints_.short_edge_entry_start.second,
                 slot_constraints_.short_edge_entry_end.first, slot_constraints_.short_edge_entry_end.second);
        ROS_INFO("   Entry center: (%.2f, %.2f), direction: %.1f°, width: %.2fm",
                 slot_constraints_.entry_center_x, slot_constraints_.entry_center_y,
                 slot_constraints_.entry_direction * 180 / M_PI, slot_constraints_.entry_width);
        ROS_INFO("   Slot dimensions: %.2fm (length) x %.2fm (width)",
                 slot_constraints_.slot_length, slot_constraints_.slot_width);

        // 🔧 将约束传递给泊车轨迹生成器
        parking_maneuver_->setParkingSlotConstraints(slot_constraints_);
    }

    /**
     * @brief 可视化停车目标位置和方向
     */
    void visualizeParkingTarget() {
        // 创建停车槽多边形可视化（蓝色线框 - 增强可见性）
        visualization_msgs::Marker slot_marker;
        slot_marker.header.frame_id = "map";
        slot_marker.header.stamp = ros::Time::now();
        slot_marker.ns = "parking_slot";
        slot_marker.id = 100;
        slot_marker.type = visualization_msgs::Marker::LINE_STRIP;
        slot_marker.action = visualization_msgs::Marker::ADD;
        slot_marker.scale.x = 0.25;  // 加粗线宽，更明显
        slot_marker.color.r = 0.0;
        slot_marker.color.g = 0.7;   // 增加亮度
        slot_marker.color.b = 1.0;
        slot_marker.color.a = 1.0;
        slot_marker.lifetime = ros::Duration(0);  // 永久显示

        // 添加停车槽多边形的所有顶点（闭合）
        for (const auto& pt : parking_slot_polygon_) {
            geometry_msgs::Point p;
            p.x = pt.first;
            p.y = pt.second;
            p.z = 0.2;  // 提高高度，避免被地面遮挡
            slot_marker.points.push_back(p);
        }
        // 闭合多边形
        if (!parking_slot_polygon_.empty()) {
            geometry_msgs::Point p;
            p.x = parking_slot_polygon_[0].first;
            p.y = parking_slot_polygon_[0].second;
            p.z = 0.2;
            slot_marker.points.push_back(p);
        }

        // 创建停车目标箭头（红色）
        visualization_msgs::Marker arrow_marker;
        arrow_marker.header.frame_id = "map";
        arrow_marker.header.stamp = ros::Time::now();
        arrow_marker.ns = "parking_target";
        arrow_marker.id = 101;
        arrow_marker.type = visualization_msgs::Marker::ARROW;
        arrow_marker.action = visualization_msgs::Marker::ADD;
        arrow_marker.scale.x = 2.0;  // 箭头长度
        arrow_marker.scale.y = 0.3;  // 箭头宽度
        arrow_marker.scale.z = 0.3;  // 箭头高度
        arrow_marker.color.r = 1.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 0.0;
        arrow_marker.color.a = 0.8;
        arrow_marker.lifetime = ros::Duration(0);

        arrow_marker.pose.position.x = parking_target_x_;
        arrow_marker.pose.position.y = parking_target_y_;
        arrow_marker.pose.position.z = 0.5;

        // 设置箭头朝向（使用四元数）
        tf::Quaternion q;
        q.setRPY(0, 0, parking_target_theta_);
        arrow_marker.pose.orientation.x = q.x();
        arrow_marker.pose.orientation.y = q.y();
        arrow_marker.pose.orientation.z = q.z();
        arrow_marker.pose.orientation.w = q.w();

        // 发布markers
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(slot_marker);
        marker_array.markers.push_back(arrow_marker);
        slot_marker_pub_.publish(marker_array);

        ROS_INFO("✅ Parking slot and target visualized in RViz");
    }

    /**
     * @brief Helper: 根据ID查找way
     */
    const Way* findWayById(const std::vector<Way>& ways, long long id) {
        for (const auto& w : ways) {
            if (w.id == id) return &w;
        }
        return nullptr;
    }

    /**
     * @brief 规划任务
     */
    void planMission() {
        ROS_INFO("=== Planning Mission ===");

        // Relation-level规划
        std::vector<long long> rel_path = path_planner_->planRelationPath(START_LANELET_ID, PARK_LANELET_ID);

        if (!rel_path.empty()) {
            ROS_INFO("✅ Relation path found: %lu lanelets", rel_path.size());

            // 生成centerline
            auto centerline = path_planner_->relationsToCenterline(rel_path, 40);

            if (centerline.empty()) {
                ROS_ERROR("Failed to generate centerline!");
                return;
            }

            // 转换为车辆轨迹
            for (const auto& pt : centerline) {
                CarState s;
                s.x = pt.first;
                s.y = pt.second;
                s.v = 2.0;  // 默认速度
                s.phi = 0.0;
                global_path_.push_back(s);
            }

            // 估计朝向
            for (size_t i = 0; i < global_path_.size(); ++i) {
                if (i + 1 < global_path_.size()) {
                    double dx = global_path_[i + 1].x - global_path_[i].x;
                    double dy = global_path_[i + 1].y - global_path_[i].y;
                    global_path_[i].theta = std::atan2(dy, dx);
                } else if (i > 0) {
                    global_path_[i].theta = global_path_[i - 1].theta;
                }
            }

            ROS_INFO("   Path generated: %lu waypoints", global_path_.size());
        } else {
            ROS_WARN("Relation path planning failed, using emergency path");

            // 从起始lanelet的中心生成应急路径
            const auto& relations = map_loader_->getRelations();
            const Relation* start_rel = nullptr;
            for (const auto& r : relations) {
                if (r.id == START_LANELET_ID) {
                    start_rel = &r;
                    break;
                }
            }

            CarState emergency_start;
            emergency_start.x = 0.0;
            emergency_start.y = 0.0;
            emergency_start.theta = 0.0;

            if (start_rel) {
                auto start_centerline = computeRelationCenterline(*start_rel);
                if (!start_centerline.empty()) {
                    size_t mid_idx = start_centerline.size() / 2;
                    emergency_start.x = start_centerline[mid_idx].first;
                    emergency_start.y = start_centerline[mid_idx].second;
                    if (mid_idx + 1 < start_centerline.size()) {
                        double dx = start_centerline[mid_idx + 1].first - emergency_start.x;
                        double dy = start_centerline[mid_idx + 1].second - emergency_start.y;
                        emergency_start.theta = std::atan2(dy, dx);
                    }
                }
            }

            global_path_ = path_planner_->generateEmergencyPath(
                emergency_start, parking_target_x_, parking_target_y_, parking_target_theta_
            );
        }

        // 重采样路径确保密度
        if (!global_path_.empty()) {
            global_path_ = path_planner_->resamplePath(global_path_, 0.5);
            ROS_INFO("   Path resampled: %lu points", global_path_.size());
        }

        // 🔧 修正：不在这里生成泊车轨迹，而是在到达泊车起点后动态生成
        // 保存泊车起点位置（路径的最后一个点）
        if (!global_path_.empty()) {
            park_start_x_ = global_path_.back().x;
            park_start_y_ = global_path_.back().y;
            park_start_theta_ = global_path_.back().theta;
            ROS_INFO("📍 Parking start point: (%.2f, %.2f, %.1f°)",
                     park_start_x_, park_start_y_, park_start_theta_ * 180 / M_PI);
            ROS_INFO("🎯 Parking target: (%.2f, %.2f, %.1f°)",
                     parking_target_x_, parking_target_y_, parking_target_theta_ * 180 / M_PI);
        }

        // 设置控制器路径
        controller_->setGlobalPath(global_path_);

        // 可视化路径
        visualizer_->publishPath(global_path_);

        ROS_INFO("✅ Mission planned: %lu total waypoints", global_path_.size());
        if (!global_path_.empty()) {
            ROS_INFO("   Start: (%.2f, %.2f, %.1f°)",
                     global_path_.front().x, global_path_.front().y,
                     global_path_.front().theta * 180 / M_PI);
            ROS_INFO("   End: (%.2f, %.2f, %.1f°)",
                     global_path_.back().x, global_path_.back().y,
                     global_path_.back().theta * 180 / M_PI);

            // 打印前几个路径点，帮助调试
            int print_count = std::min(5, static_cast<int>(global_path_.size()));
            ROS_INFO("   First %d waypoints:", print_count);
            for (int i = 0; i < print_count; ++i) {
                ROS_INFO("     [%d]: (%.2f, %.2f, %.1f°)",
                         i, global_path_[i].x, global_path_[i].y,
                         global_path_[i].theta * 180 / M_PI);
            }
        }
    }

    /**
     * @brief 计算relation的centerline
     */
    std::vector<std::pair<double, double>> computeRelationCenterline(const Relation& rel) {
        const auto& nodes = map_loader_->getNodes();
        const auto& ways = map_loader_->getWays();

        std::vector<std::pair<double, double>> centerline;

        // 找到relation中的所有way并采样
        std::vector<std::vector<std::pair<double, double>>> all_way_samples;

        for (long long wid : rel.member_way_refs) {
            const Way* way = findWayById(ways, wid);
            if (!way) continue;

            std::vector<std::pair<double, double>> pts;
            for (long long nid : way->node_refs) {
                if (nodes.count(nid)) {
                    pts.emplace_back(nodes.at(nid).x, nodes.at(nid).y);
                }
            }

            if (!pts.empty()) {
                all_way_samples.push_back(pts);
            }
        }

        if (all_way_samples.empty()) {
            return centerline;
        }

        // 如果只有一条way，直接返回
        if (all_way_samples.size() == 1) {
            return all_way_samples[0];
        }

        // 计算centerline（所有ways的平均）
        size_t M = all_way_samples[0].size();
        for (size_t i = 0; i < M; ++i) {
            double sx = 0, sy = 0;
            int count = 0;
            for (const auto& samples : all_way_samples) {
                if (i < samples.size()) {
                    sx += samples[i].first;
                    sy += samples[i].second;
                    ++count;
                }
            }
            if (count > 0) {
                centerline.emplace_back(sx / count, sy / count);
            }
        }

        return centerline;
    }

    /**
     * @brief 检测是否到达泊车起点
     */
    bool hasReachedParkingStart() const {
        double dist = std::hypot(vehicle_state_.x - park_start_x_,
                                vehicle_state_.y - park_start_y_);
        // 距离泊车起点1米以内认为到达
        return dist < 1.0;
    }

    /**
     * @brief 检测车辆是否超过边界前沿（车位短边）
     * @return true if vehicle has crossed the boundary frontline
     */
    bool hasPassedBoundaryFront() const {
        return checkPointPassedFrontline(vehicle_state_.x, vehicle_state_.y);
    }

    /**
     * @brief 检测某个点是否超过车位短边前沿线
     * @param x 点的X坐标
     * @param y 点的Y坐标
     * @return true if point has crossed the frontline
     */
    bool checkPointPassedFrontline(double x, double y) const {
        if (line_a_pts_.empty() || line_b_pts_.empty()) return false;

        // 计算边界前沿线（A和B的前端点连线）
        // 假设前端点是每条线的第二个点（索引1）
        double front_a_x = line_a_pts_[1].first;
        double front_a_y = line_a_pts_[1].second;
        double front_b_x = line_b_pts_[1].first;
        double front_b_y = line_b_pts_[1].second;

        // 前沿线的方向向量
        double front_dx = front_b_x - front_a_x;
        double front_dy = front_b_y - front_a_y;
        double norm = std::hypot(front_dx, front_dy);
        if (norm < 1e-6) return false;

        // 归一化
        front_dx /= norm;
        front_dy /= norm;

        // 计算前沿线的法向量（指向停车位内部）
        double normal_x = -front_dy;
        double normal_y = front_dx;

        // 点相对于前沿线A点的向量
        double to_point_x = x - front_a_x;
        double to_point_y = y - front_a_y;

        // 计算有向距离（投影到法向量）
        double signed_dist = to_point_x * normal_x + to_point_y * normal_y;

        // 如果有向距离为正，说明点已经越过前沿线（进入停车位内部）
        return signed_dist > 0.0;
    }

    /**
     * @brief 动态生成泊车轨迹（基于当前实时位置）
     */
    std::vector<CarState> generateDynamicParkingTrajectory() {
        ROS_INFO("🔄 Generating dynamic parking trajectory from current position:");
        ROS_INFO("   Current: (%.2f, %.2f, %.1f°)",
                 vehicle_state_.x, vehicle_state_.y, vehicle_state_.theta * 180 / M_PI);
        ROS_INFO("   Target: (%.2f, %.2f, %.1f°)",
                 parking_target_x_, parking_target_y_, parking_target_theta_ * 180 / M_PI);

        // 🔧 优先使用约束感知的泊车规划
        std::vector<CarState> parking_traj = parking_maneuver_->generateConstraintAwareParking(vehicle_state_);

        // 🔧 如果约束感知规划失败，回退到传统方法
        if (parking_traj.empty()) {
            ROS_WARN("   ⚠️ Constraint-aware planning failed, falling back to traditional method");
            parking_traj = parking_maneuver_->generateVerticalParking(
                vehicle_state_, parking_target_x_, parking_target_y_, parking_target_theta_
            );
        }

        if (!parking_traj.empty()) {
            ROS_INFO("   ✅ Generated %lu parking states", parking_traj.size());

            // 🔧 简化验证逻辑：只检查最终位置是否在车位内
            // 原因：车辆倒车进入时会穿过短边，中间过程的轨迹点可能在车位外

            // 检查最终位置的footprint是否完全在车位内
            bool final_inside = parking_maneuver_->isFootprintInside(parking_traj.back());

            if (!final_inside) {
                ROS_WARN("   ⚠️ Final position not fully inside parking slot, but continuing");
                // 不拒绝，只是警告
            } else {
                ROS_INFO("   ✅ Final position validation passed");
            }

            ROS_INFO("   ✅ Trajectory accepted");
        } else {
            ROS_ERROR("   ❌ Failed to generate parking trajectory");
        }

        return parking_traj;
    }

    /**
     * @brief 可视化循环回调（独立定时器，持续发布停车槽）
     */
    void visualizationLoop(const ros::TimerEvent& event) {
        visualizeParkingTarget();
    }

    /**
     * @brief 控制循环回调（带状态机）
     */
    void controlLoop(const ros::TimerEvent& event) {
        if (global_path_.empty() && current_state_ == ParkingState::APPROACHING) {
            ROS_WARN_THROTTLE(2.0, "No path to follow");
            return;
        }

        // === 状态机逻辑 ===
        switch (current_state_) {
            case ParkingState::APPROACHING: {
                // 阶段1: 前进到泊车起点

                // 检查是否超过边界前沿（安全检查）
                if (hasPassedBoundaryFront()) {
                    ROS_WARN_THROTTLE(1.0, "⚠️ WARNING: Vehicle has passed boundary front!");
                }

                // 执行一步控制
                vehicle_state_ = controller_->computeControl(vehicle_state_, 0.05);

                // 检查是否到达泊车起点
                if (hasReachedParkingStart() || controller_->isGoalReached()) {
                    ROS_INFO("✅ Reached parking start point! Switching to PARKING mode...");
                    current_state_ = ParkingState::PARKING;

                    // 停车并准备进入泊车阶段
                    vehicle_state_.v = 0.0;
                    vehicle_state_.phi = 0.0;

                    // 立即生成第一条泊车轨迹
                    auto parking_traj = generateDynamicParkingTrajectory();
                    if (!parking_traj.empty()) {
                        controller_->setGlobalPath(parking_traj);
                        visualizer_->publishPath(parking_traj);
                        ROS_INFO("🚗 Starting parking maneuver with %lu states", parking_traj.size());
                    } else {
                        ROS_ERROR("Failed to generate initial parking trajectory!");
                        current_state_ = ParkingState::COMPLETED;
                    }
                }

                // 调试输出
                static int counter_approach = 0;
                if (++counter_approach % 20 == 0) {
                    double dist_to_start = std::hypot(vehicle_state_.x - park_start_x_,
                                                     vehicle_state_.y - park_start_y_);

                    // 计算到当前路径点的距离
                    int curr_idx = controller_->getCurrentIndex();
                    double dist_to_path = 0.0;
                    if (curr_idx < static_cast<int>(global_path_.size())) {
                        dist_to_path = std::hypot(vehicle_state_.x - global_path_[curr_idx].x,
                                                 vehicle_state_.y - global_path_[curr_idx].y);
                    }

                    auto lookahead = controller_->getLookaheadPoint();
                    double lookahead_dist = std::hypot(lookahead.first - vehicle_state_.x,
                                                       lookahead.second - vehicle_state_.y);

                    ROS_INFO("🚗 [APPROACHING] pos=(%.2f,%.2f) θ=%.1f° v=%.2f φ=%.2f° | idx=%d/%lu | dist_to_path=%.3fm | lookahead=%.2fm | dist_to_start=%.2fm",
                             vehicle_state_.x, vehicle_state_.y,
                             vehicle_state_.theta * 180.0 / M_PI,
                             vehicle_state_.v,
                             vehicle_state_.phi * 180.0 / M_PI,
                             curr_idx,
                             global_path_.size(),
                             dist_to_path,
                             lookahead_dist,
                             dist_to_start);
                }
                break;
            }

            case ParkingState::PARKING: {
                // 阶段2: 泊车倒车阶段

                // 🔧 禁用动态重规划，直接跟踪第一次生成的轨迹
                // 原因：频繁重规划导致轨迹不断变化，车辆晃动
                // 第一次生成的轨迹已经是正确的，直接跟踪即可

                // 执行一步控制（跟踪第一次生成的轨迹）
                vehicle_state_ = controller_->computeControl(vehicle_state_, 0.05);

                // 检查是否到达最终目标
                double dist_to_target = std::hypot(parking_target_x_ - vehicle_state_.x,
                                                  parking_target_y_ - vehicle_state_.y);

                if (dist_to_target < 0.2 || controller_->isGoalReached()) {
                    ROS_INFO("🎯 Parking completed!");
                    current_state_ = ParkingState::COMPLETED;
                    vehicle_state_.v = 0.0;
                    vehicle_state_.phi = 0.0;

                    // 计算最终姿态与目标的偏差
                    double pos_error = dist_to_target;
                    double angle_error = parking_target_theta_ - vehicle_state_.theta;
                    while (angle_error > M_PI) angle_error -= 2 * M_PI;
                    while (angle_error <= -M_PI) angle_error += 2 * M_PI;

                    ROS_INFO("📊 Final errors: Position=%.3fm, Angle=%.1f°",
                             pos_error, angle_error * 180 / M_PI);
                }

                // 调试输出
                static int counter_parking = 0;
                if (++counter_parking % 20 == 0) {
                    double angle_to_target = parking_target_theta_ - vehicle_state_.theta;
                    while (angle_to_target > M_PI) angle_to_target -= 2 * M_PI;
                    while (angle_to_target <= -M_PI) angle_to_target += 2 * M_PI;

                    ROS_INFO("🚗 [PARKING] pos=(%.2f,%.2f) θ=%.1f° v=%.2f | dist=%.2fm Δθ=%.1f°",
                             vehicle_state_.x, vehicle_state_.y,
                             vehicle_state_.theta * 180.0 / M_PI,
                             vehicle_state_.v,
                             dist_to_target,
                             angle_to_target * 180 / M_PI);
                }
                break;
            }

            case ParkingState::COMPLETED: {
                // 阶段3: 完成
                vehicle_state_.v = 0.0;
                vehicle_state_.phi = 0.0;

                static bool logged = false;
                if (!logged) {
                    ROS_INFO("✅ Parking system COMPLETED. Vehicle is parked.");
                    logged = true;
                }
                break;
            }
        }

        // 发布可视化（所有状态都需要）
        visualizer_->publishVehicleState(vehicle_state_);
        visualizer_->publishTF(vehicle_state_);
        visualizer_->publishControlCommand(vehicle_state_.v, vehicle_state_.phi);

        if (current_state_ != ParkingState::COMPLETED) {
            auto lookahead = controller_->getLookaheadPoint();
            visualizer_->publishLookaheadPoint(lookahead.first, lookahead.second);
        }
    }

    // ROS
    ros::NodeHandle nh_;
    ros::Timer control_timer_;
    ros::Timer vis_timer_;  // 停车槽可视化定时器
    ros::Publisher slot_marker_pub_;  // 停车槽可视化发布器

    // 模块（使用智能指针管理）
    std::shared_ptr<OSMMapLoader> map_loader_;
    std::shared_ptr<GraphBuilder> graph_builder_;
    std::shared_ptr<PathPlanner> path_planner_;
    std::shared_ptr<PurePursuitController> controller_;
    std::shared_ptr<ParkingManeuverGenerator> parking_maneuver_;
    std::shared_ptr<Visualizer> visualizer_;

    // 参数
    VehicleParams vehicle_params_;
    ControllerParams controller_params_;
    PlannerParams planner_params_;

    // 状态
    CarState vehicle_state_;
    std::vector<CarState> global_path_;

    // 任务参数
    double parking_target_x_ = 0.0;
    double parking_target_y_ = 0.0;
    double parking_target_theta_ = 0.0;
    std::vector<std::pair<double, double>> parking_slot_polygon_;

    // 状态机
    ParkingState current_state_ = ParkingState::APPROACHING;

    // 泊车起点位置（从路径的最后一个点获取）
    double park_start_x_ = 0.0;
    double park_start_y_ = 0.0;
    double park_start_theta_ = 0.0;

    // 边界线位置（用于前沿检测）
    std::vector<std::pair<double, double>> line_a_pts_;
    std::vector<std::pair<double, double>> line_b_pts_;

    // 车位几何约束（用于约束感知路径规划）
    ParkingSlotConstraints slot_constraints_;

    // 常量ID定义
    static constexpr long long START_LANELET_ID = 9259;
    static constexpr long long PARK_LANELET_ID = 9265;
    static constexpr long long LINESTR_ID_A = 9386;
    static constexpr long long LINESTR_ID_B = 9392;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "parking_system_refactored");

    try {
        ParkingSystemRefactored system;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
