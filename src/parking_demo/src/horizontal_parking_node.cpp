/**
 * @file horizontal_parking_node.cpp
 * @brief 水平泊车（侧方位停车）系统节点
 *
 * 功能：
 * 1. 从lane119出发
 * 2. 到lane121开始计算泊车轨迹
 * 3. 基于way 10295和10298边界进行侧方位停车
 * 4. 使用车辆动力学模型和车身尺寸计算揉库轨迹
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <cmath>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose2D.h>

// 重构后的模块
#include "parking_demo/types.h"
#include "parking_demo/osm_map_loader.h"
#include "parking_demo/graph_builder.h"
#include "parking_demo/path_planner.h"
#include "parking_demo/pure_pursuit_controller.h"
#include "parking_demo/parking_maneuver.h"
#include "parking_demo/visualizer.h"

using namespace parking_demo;

// 硬编码的地图ID（用户指定）
static constexpr long long START_LANELET_ID = 44;      // 起始车道（从44出发）
static constexpr long long PARKING_LANELET_ID = 121;   // 泊车计算起点车道
static constexpr long long PARKING_BOUNDARY_WAY_1 = 10295;  // 停车位边界1
static constexpr long long PARKING_BOUNDARY_WAY_2 = 10298;  // 停车位边界2

/**
 * @brief 停车系统状态机
 */
enum class ParkingState {
    APPROACHING,  // 前进到泊车起点（lane119 -> lane121）
    PARKING,      // 侧方位停车倒车阶段（揉库）
    COMPLETED     // 完成
};

/**
 * @brief 水平泊车系统类
 */
class HorizontalParkingSystem {
public:
    HorizontalParkingSystem() : nh_("~") {
        ROS_INFO("=== Horizontal Parking System (Parallel Parking) Starting ===");

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

        // 9. 计算停车位位置（基于way 10295和10298）
        if (!computeParkingPosition()) {
            ROS_ERROR("Failed to compute parking position!");
            throw std::runtime_error("Position computation failed");
        }

        // 10. 规划任务（从lane119到lane121）
        planMission();

        // 11. 设置车辆初始位置为路径起点
        if (!global_path_.empty()) {
            vehicle_state_ = global_path_[0];
            vehicle_state_.v = 0.0;
            vehicle_state_.phi = 0.0;

            // 设置控制器的全局路径
            controller_->setGlobalPath(global_path_);

            ROS_INFO("Vehicle initialized at path start: pos=(%.2f, %.2f) theta=%.1f deg",
                     vehicle_state_.x, vehicle_state_.y, vehicle_state_.theta * 180 / M_PI);
        } else {
            ROS_ERROR("No path generated!");
            throw std::runtime_error("Path generation failed");
        }

        // 12. 启动控制循环（20Hz）
        control_timer_ = nh_.createTimer(ros::Duration(0.05), &HorizontalParkingSystem::controlLoop, this);

        // 13. 启动停车槽可视化定时器（2Hz）
        vis_timer_ = nh_.createTimer(ros::Duration(0.5), &HorizontalParkingSystem::visualizationLoop, this);

        ROS_INFO("=== Horizontal Parking System Initialized Successfully ===");
    }

private:
    /**
     * @brief 从ROS参数服务器加载参数
     */
    void loadParameters() {
        // 车辆参数
        nh_.param("vehicle_wheelbase", vehicle_params_.wheelbase, 1.8);
        nh_.param("vehicle_max_steering", vehicle_params_.max_steering_rad, 0.6);
        nh_.param("vehicle_length", vehicle_params_.length, 2.4);
        nh_.param("vehicle_width", vehicle_params_.width, 1.4);

        // 计算最小转弯半径
        vehicle_params_.min_turn_radius = vehicle_params_.wheelbase /
                                         std::tan(vehicle_params_.max_steering_rad);

        // 控制器参数
        nh_.param("lookahead_distance", controller_params_.lookahead_dist, 2.0);
        nh_.param("kp_speed", controller_params_.kp_speed, 0.8);
        nh_.param("max_speed", controller_params_.max_speed, 1.5);
        nh_.param("min_speed", controller_params_.min_speed, 0.3);
        nh_.param("goal_tolerance", controller_params_.goal_tolerance, 0.15);

        // 规划器参数
        nh_.param("parking_back_distance", planner_params_.parking_back_distance, 1.5);

        ROS_INFO("Vehicle params: L=%.2f W=%.2f wheelbase=%.2f R_min=%.2f",
                 vehicle_params_.length, vehicle_params_.width,
                 vehicle_params_.wheelbase, vehicle_params_.min_turn_radius);
    }

    /**
     * @brief 计算停车位位置（基于way 10295和10298）
     */
    bool computeParkingPosition() {
        ROS_INFO("=== Computing Parking Position ===");

        // 获取way 10295的节点（停车位边界1）
        const Way* boundary_1 = nullptr;
        const Way* boundary_2 = nullptr;

        for (const auto& way : map_loader_->getWays()) {
            if (way.id == PARKING_BOUNDARY_WAY_1) {
                boundary_1 = &way;
            }
            if (way.id == PARKING_BOUNDARY_WAY_2) {
                boundary_2 = &way;
            }
        }

        if (!boundary_1) {
            ROS_ERROR("Way %lld not found!", PARKING_BOUNDARY_WAY_1);
            return false;
        }

        if (!boundary_2) {
            ROS_ERROR("Way %lld not found!", PARKING_BOUNDARY_WAY_2);
            return false;
        }

        if (boundary_1->node_refs.size() < 2 || boundary_2->node_refs.size() < 2) {
            ROS_ERROR("Parking boundaries have insufficient nodes!");
            return false;
        }

        // 获取边界1的两个节点（侧方位停车的前边界）
        long long node_id_1_start = boundary_1->node_refs[0];
        long long node_id_1_end = boundary_1->node_refs[1];

        // 获取边界2的两个节点（侧方位停车的后边界）
        long long node_id_2_start = boundary_2->node_refs[0];
        long long node_id_2_end = boundary_2->node_refs[1];

        // 获取节点坐标
        const auto& nodes = map_loader_->getNodes();
        auto it_1_start = nodes.find(node_id_1_start);
        auto it_1_end = nodes.find(node_id_1_end);
        auto it_2_start = nodes.find(node_id_2_start);
        auto it_2_end = nodes.find(node_id_2_end);

        if (it_1_start == nodes.end() || it_1_end == nodes.end() ||
            it_2_start == nodes.end() || it_2_end == nodes.end()) {
            ROS_ERROR("Parking boundary nodes not found!");
            return false;
        }

        // 提取边界坐标
        boundary_1_start_ = std::make_pair(it_1_start->second.x, it_1_start->second.y);
        boundary_1_end_ = std::make_pair(it_1_end->second.x, it_1_end->second.y);
        boundary_2_start_ = std::make_pair(it_2_start->second.x, it_2_start->second.y);
        boundary_2_end_ = std::make_pair(it_2_end->second.x, it_2_end->second.y);

        // 计算停车位中心
        parking_target_.x = (boundary_1_start_.first + boundary_1_end_.first +
                            boundary_2_start_.first + boundary_2_end_.first) / 4.0;
        parking_target_.y = (boundary_1_start_.second + boundary_1_end_.second +
                            boundary_2_start_.second + boundary_2_end_.second) / 4.0;

        // 计算停车位朝向（车身垂直于边界，保持水平）
        double dx = boundary_1_end_.first - boundary_1_start_.first;
        double dy = boundary_1_end_.second - boundary_1_start_.second;
        double boundary_angle = std::atan2(dy, dx);  // 停车位边界角度

        // 车身应该垂直于边界（即加90度），保持水平姿态
        parking_target_.theta = boundary_angle + M_PI / 2.0;

        // 归一化角度到 [-π, π]
        while (parking_target_.theta > M_PI) parking_target_.theta -= 2 * M_PI;
        while (parking_target_.theta < -M_PI) parking_target_.theta += 2 * M_PI;

        ROS_INFO("Parking slot boundary angle: %.1f deg", boundary_angle * 180 / M_PI);
        ROS_INFO("Vehicle target angle: %.1f deg (PERPENDICULAR to boundary, HORIZONTAL)",
                 parking_target_.theta * 180 / M_PI);
        ROS_INFO("Vehicle will park HORIZONTALLY (perpendicular to boundary)");

        // 计算停车位尺寸
        parking_slot_length_ = std::hypot(boundary_1_end_.first - boundary_1_start_.first,
                                         boundary_1_end_.second - boundary_1_start_.second);

        double dist_between_boundaries = std::hypot(
            (boundary_2_start_.first + boundary_2_end_.first) / 2.0 -
            (boundary_1_start_.first + boundary_1_end_.first) / 2.0,
            (boundary_2_start_.second + boundary_2_end_.second) / 2.0 -
            (boundary_1_start_.second + boundary_1_end_.second) / 2.0
        );
        parking_slot_width_ = dist_between_boundaries;

        // 计算停车位中心线（两次转向的目标线）
        // 中心线起点：停车位入口中心（两边界中点的中点）
        double entry_center_x = (boundary_1_start_.first + boundary_1_end_.first +
                                 boundary_2_start_.first + boundary_2_end_.first) / 4.0;
        double entry_center_y = (boundary_1_start_.second + boundary_1_end_.second +
                                 boundary_2_start_.second + boundary_2_end_.second) / 4.0;

        // 中心线方向：垂直于边界（即parking_target_.theta方向）
        centerline_direction_x_ = std::cos(parking_target_.theta);
        centerline_direction_y_ = std::sin(parking_target_.theta);

        // 中心线起点（稍微靠外一点，方便对齐）
        centerline_start_x_ = entry_center_x + 1.0 * centerline_direction_x_;
        centerline_start_y_ = entry_center_y + 1.0 * centerline_direction_y_;

        ROS_INFO("Parking slot computed:");
        ROS_INFO("  Center: (%.2f, %.2f)", parking_target_.x, parking_target_.y);
        ROS_INFO("  Orientation: %.1f deg", parking_target_.theta * 180 / M_PI);
        ROS_INFO("  Size: %.2f x %.2f m", parking_slot_length_, parking_slot_width_);
        ROS_INFO("  Centerline direction: (%.2f, %.2f)", centerline_direction_x_, centerline_direction_y_);
        ROS_INFO("  Centerline start: (%.2f, %.2f)", centerline_start_x_, centerline_start_y_);
        ROS_INFO("  Boundary 1: (%.2f, %.2f) -> (%.2f, %.2f)",
                 boundary_1_start_.first, boundary_1_start_.second,
                 boundary_1_end_.first, boundary_1_end_.second);
        ROS_INFO("  Boundary 2: (%.2f, %.2f) -> (%.2f, %.2f)",
                 boundary_2_start_.first, boundary_2_start_.second,
                 boundary_2_end_.first, boundary_2_end_.second);

        // 验证停车位尺寸是否足够
        if (parking_slot_length_ < vehicle_params_.length + 0.5 ||
            parking_slot_width_ < vehicle_params_.width + 0.3) {
            ROS_WARN("Parking slot may be too small! Slot: %.2f x %.2f, Vehicle: %.2f x %.2f",
                     parking_slot_length_, parking_slot_width_,
                     vehicle_params_.length, vehicle_params_.width);
        }

        return true;
    }

    /**
     * @brief 规划任务（从lane44到lane121）
     */
    void planMission() {
        ROS_INFO("=== Planning Mission: Lane %lld -> Lane %lld ===",
                 START_LANELET_ID, PARKING_LANELET_ID);

        // 规划路径：44 -> 121
        std::vector<long long> rel_path = path_planner_->planRelationPath(
            START_LANELET_ID, PARKING_LANELET_ID);

        if (rel_path.empty()) {
            ROS_ERROR("Failed to plan relation path from lane %lld to lane %lld!",
                     START_LANELET_ID, PARKING_LANELET_ID);
            ROS_ERROR("Cannot generate path! Please check lanelet IDs.");
            return;
        }

        ROS_INFO("Relation path found: %lu lanelets", rel_path.size());

        // 生成centerline
        auto centerline = path_planner_->relationsToCenterline(rel_path, 40);

        if (centerline.empty()) {
            ROS_ERROR("Failed to generate centerline!");
            return;
        }

        ROS_INFO("Centerline generated: %lu points", centerline.size());

        // 转换为CarState轨迹
        global_path_.clear();
        for (size_t i = 0; i < centerline.size(); ++i) {
            CarState state;
            state.x = centerline[i].first;
            state.y = centerline[i].second;

            // 计算朝向（基于相邻点）
            if (i + 1 < centerline.size()) {
                double dx = centerline[i + 1].first - state.x;
                double dy = centerline[i + 1].second - state.y;
                state.theta = std::atan2(dy, dx);
            } else if (i > 0) {
                // 最后一个点使用前一个点的朝向
                state.theta = global_path_[i - 1].theta;
            }

            state.v = 0.0;
            state.phi = 0.0;
            global_path_.push_back(state);
        }

        if (!global_path_.empty()) {
            ROS_INFO("Path planned successfully: %lu waypoints", global_path_.size());
            visualizer_->publishPath(global_path_);
        } else {
            ROS_ERROR("Failed to generate trajectory!");
        }
    }


    /**
     * @brief 控制循环（20Hz）
     */
    void controlLoop(const ros::TimerEvent&) {
        if (state_ == ParkingState::COMPLETED) {
            // 停止时继续发布车辆位姿
            visualizer_->publishVehicleState(vehicle_state_);
            return;
        }

        if (state_ == ParkingState::APPROACHING) {
            // 前进阶段：使用Pure Pursuit跟随全局路径
            vehicle_state_ = controller_->computeControl(vehicle_state_, 0.05);

            // 检查是否到达泊车起点（lane121附近）
            if (controller_->isGoalReached()) {
                ROS_INFO("=== Reached parking start position (lane 121) ===");
                ROS_INFO("Current pose: (%.2f, %.2f) theta=%.1f deg",
                        vehicle_state_.x, vehicle_state_.y,
                        vehicle_state_.theta * 180 / M_PI);

                state_ = ParkingState::PARKING;

                // 停车准备
                vehicle_state_.v = 0.0;
                vehicle_state_.phi = 0.0;

                // 停车位目标朝向已在初始化时计算（水平，与车道平行）
                ROS_INFO("Target parking orientation: %.1f deg (horizontal, parallel to lane)",
                        parking_target_.theta * 180 / M_PI);

                // 到达位置后，现在开始规划水平泊车轨迹
                ROS_INFO("Now planning horizontal parking trajectory...");
                parking_trajectory_ = generateParallelParkingTrajectory();
                parking_traj_index_ = 0;

                if (parking_trajectory_.empty()) {
                    ROS_ERROR("Failed to generate parking trajectory!");
                    state_ = ParkingState::COMPLETED;
                } else {
                    ROS_INFO("Horizontal parking trajectory generated: %lu waypoints",
                            parking_trajectory_.size());
                }
            }
        }
        else if (state_ == ParkingState::PARKING) {
            // 泊车阶段：跟随预计算的侧方位停车轨迹
            if (parking_traj_index_ < parking_trajectory_.size()) {
                const CarState& target = parking_trajectory_[parking_traj_index_];

                // 直接设置车辆状态（预计算轨迹）
                vehicle_state_ = target;
                parking_traj_index_++;

                ROS_INFO_THROTTLE(1.0, "Parking progress: %lu / %lu",
                                 parking_traj_index_, parking_trajectory_.size());
            } else {
                // 泊车完成
                ROS_INFO("=== Horizontal Parking Completed! ===");
                ROS_INFO("Final position: (%.2f, %.2f) theta=%.1f deg (HORIZONTAL)",
                        vehicle_state_.x, vehicle_state_.y,
                        vehicle_state_.theta * 180 / M_PI);
                state_ = ParkingState::COMPLETED;
            }
        }

        // 发布可视化
        visualizer_->publishVehicleState(vehicle_state_);
        visualizer_->publishTF(vehicle_state_);
    }

    /**
     * @brief 生成水平泊车轨迹（4阶段）
     *
     * 核心思路：
     * - 停车位有一条中心线（垂直于边界，指向中心）
     * - Phase 2 + Phase 3：两次转向将车辆移动到中心线上
     * - Phase 4：沿中心线直线倒车到停车位中心
     *
     * 算法步骤：
     * 1. 直线前进到停车位附近
     * 2. 左打死倒车（第一段圆弧）
     * 3. 右打死倒车（对齐到中心线）
     * 4. 沿中心线直线倒车到库中心
     */
    std::vector<CarState> generateParallelParkingTrajectory() {
        ROS_INFO("=== Generating Horizontal Parking Trajectory (4 Phases) ===");
        ROS_INFO("Strategy: Two steering maneuvers move vehicle to centerline, then reverse straight");
        ROS_INFO("Key: Vehicle aligns to parking slot centerline for final approach");

        std::vector<CarState> trajectory;

        // 当前车辆位置（已到达lane 121，停车位左侧）
        CarState start = vehicle_state_;

        // 停车位目标位置（车身保持水平，与车道121平行）
        CarState target = parking_target_;

        ROS_INFO("Start: (%.2f, %.2f) theta=%.1f deg",
                 start.x, start.y, start.theta * 180 / M_PI);
        ROS_INFO("Target: (%.2f, %.2f) theta=%.1f deg (HORIZONTAL, parallel to lane)",
                 target.x, target.y, target.theta * 180 / M_PI);
        ROS_INFO("Parking slot: %.2f m × %.2f m", parking_slot_length_, parking_slot_width_);

        // 计算转弯半径
        double R = vehicle_params_.min_turn_radius * 1.2;
        double max_steering = std::atan(vehicle_params_.wheelbase / R);

        ROS_INFO("Turn radius: %.2f m, max steering: %.1f deg", R, max_steering * 180 / M_PI);

        // 计算边界2（linestring 10298）的中点作为参考
        double boundary2_mid_x = (boundary_2_start_.first + boundary_2_end_.first) / 2.0;
        double boundary2_mid_y = (boundary_2_start_.second + boundary_2_end_.second) / 2.0;

        // 计算从当前位置到边界2中点的距离（沿行驶方向）
        double dx_to_boundary = boundary2_mid_x - start.x;
        double dy_to_boundary = boundary2_mid_y - start.y;
        double dist_along_lane = dx_to_boundary * std::cos(start.theta) +
                                 dy_to_boundary * std::sin(start.theta);

        // 驶过边界30%车身长度
        double overshoot_dist = 0.3 * vehicle_params_.length;
        double forward_dist = dist_along_lane + overshoot_dist;

        ROS_INFO("Boundary 2 (10298) distance: %.2f m", dist_along_lane);
        ROS_INFO("Overshoot: %.2f m (30%% of vehicle length)", overshoot_dist);
        ROS_INFO("Total forward distance: %.2f m", forward_dist);

        // ===== 阶段1: 直线前进到停车位附近 =====
        ROS_INFO("Phase 1: Move STRAIGHT forward to parking position");

        int phase1_steps = std::max(15, static_cast<int>(forward_dist / 0.08));

        for (int i = 1; i <= phase1_steps; ++i) {
            double alpha = static_cast<double>(i) / phase1_steps;

            CarState s1;
            s1.x = start.x + forward_dist * alpha * std::cos(start.theta);
            s1.y = start.y + forward_dist * alpha * std::sin(start.theta);
            s1.theta = start.theta;
            s1.v = 0.5;  // 慢速前进
            s1.phi = 0.0;

            trajectory.push_back(s1);
        }

        CarState phase1_end = trajectory.back();
        ROS_INFO("After Phase 1: pos=(%.2f, %.2f) theta=%.1f deg, waypoints=%d",
                 phase1_end.x, phase1_end.y, phase1_end.theta * 180 / M_PI, phase1_steps);
        ROS_INFO("Phase 1 summary: All waypoints should be STRAIGHT forward (phi=0, theta=const)");

        // ===== 阶段2: 左打死倒车（第一段圆弧）=====
        // 从372点立即开始倒车（v < 0）
        ROS_INFO("Phase 2: Left-steer reverse (first arc) - START REVERSING immediately at point 372");

        double phase2_angle = 0.6;  // 左转约34度
        int phase2_steps = 40;  // 左打死倒车40个点

        for (int i = 1; i <= phase2_steps; ++i) {
            double alpha = static_cast<double>(i) / phase2_steps;
            double theta_change = alpha * phase2_angle;

            CarState s2;
            // 倒车左打死：theta减小（车尾向左）
            s2.theta = phase1_end.theta - theta_change;

            // 原来的圆弧计算（保持不变，镜像到倒车）
            double cx = phase1_end.x - R * std::sin(phase1_end.theta);
            double cy = phase1_end.y + R * std::cos(phase1_end.theta);
            double arc_angle = phase1_end.theta - theta_change;  // 跟随theta变化

            s2.x = cx + R * std::sin(arc_angle);
            s2.y = cy - R * std::cos(arc_angle);
            s2.v = -0.5;  // 倒车
            s2.phi = max_steering;  // 左打死

            trajectory.push_back(s2);
        }

        CarState phase2_end = trajectory.back();
        ROS_INFO("After Phase 2 (left-steer reverse): pos=(%.2f, %.2f) theta=%.1f deg, total waypoints=%lu",
                 phase2_end.x, phase2_end.y, phase2_end.theta * 180 / M_PI, trajectory.size());

        // ===== 阶段3: 右打死倒车（第二段圆弧，对齐到停车位中心线）=====
        // 目标：基于第一段倒车结束后直接反向打死，将车辆移动到中心线
        ROS_INFO("Phase 3: Right-steer reverse (second arc) - mirror of Phase 2");

        // 直接反向打死：使用与Phase 2相同的角度，但方向相反
        double phase3_angle = 0.6;  // 右转约34度（与Phase 2对称）
        int phase3_steps = 40;  // 右打死倒车40个点

        ROS_INFO("Phase 3: Rotate %.1f deg right (mirror of Phase 2 left turn)",
                 phase3_angle * 180 / M_PI);
        ROS_INFO("  Starting from Phase 2 end: theta=%.1f deg", phase2_end.theta * 180 / M_PI);

        for (int i = 1; i <= phase3_steps; ++i) {
            double alpha = static_cast<double>(i) / phase3_steps;
            double theta_change = alpha * phase3_angle;

            CarState s3;
            // theta逐渐回到水平
            s3.theta = phase2_end.theta + theta_change;

            // 右打死倒车，圆心在车辆右侧
            double cx = phase2_end.x + R * std::sin(phase2_end.theta);
            double cy = phase2_end.y - R * std::cos(phase2_end.theta);
            double arc_angle = phase2_end.theta + theta_change;

            s3.x = cx + R * std::sin(arc_angle);
            s3.y = cy - R * std::cos(arc_angle);
            s3.v = -0.4;  // 倒车
            s3.phi = -max_steering;  // 右打死

            trajectory.push_back(s3);
        }

        CarState phase3_end = trajectory.back();
        ROS_INFO("After Phase 3: pos=(%.2f, %.2f) theta=%.1f deg (final steering complete), total waypoints=%lu",
                 phase3_end.x, phase3_end.y, phase3_end.theta * 180 / M_PI, trajectory.size());

        // ===== 阶段4: 沿中心线直线倒车到停车位中心 =====
        ROS_INFO("Phase 4: Reverse straight along centerline to parking slot center");

        // 计算到中心的距离
        double dx_to_center = target.x - phase3_end.x;
        double dy_to_center = target.y - phase3_end.y;
        double dist_to_center = std::hypot(dx_to_center, dy_to_center);

        ROS_INFO("Distance to center along centerline: %.2f m", dist_to_center);

        int phase4_steps = std::max(20, static_cast<int>(dist_to_center / 0.08));

        for (int i = 1; i <= phase4_steps; ++i) {
            double alpha = static_cast<double>(i) / phase4_steps;

            CarState s4;
            s4.x = phase3_end.x + dx_to_center * alpha;
            s4.y = phase3_end.y + dy_to_center * alpha;
            s4.theta = target.theta;  // 保持目标朝向
            s4.v = -0.3;  // 慢速倒车
            s4.phi = 0.0;  // 直线

            trajectory.push_back(s4);
        }

        ROS_INFO("=== Horizontal Parking Trajectory Generated ===");
        ROS_INFO("Total waypoints: %lu", trajectory.size());
        ROS_INFO("Final: pos=(%.2f, %.2f) theta=%.1f deg (HORIZONTAL, parallel to lane)",
                 trajectory.back().x, trajectory.back().y,
                 trajectory.back().theta * 180 / M_PI);

        // 可视化泊车轨迹
        visualizer_->publishPath(trajectory);

        return trajectory;
    }

    /**
     * @brief 可视化停车槽
     */
    void visualizationLoop(const ros::TimerEvent&) {
        visualization_msgs::MarkerArray markers;

        // 可视化停车位边界
        visualization_msgs::Marker boundary_marker;
        boundary_marker.header.frame_id = "map";
        boundary_marker.header.stamp = ros::Time::now();
        boundary_marker.ns = "parking_boundaries";
        boundary_marker.id = 0;
        boundary_marker.type = visualization_msgs::Marker::LINE_LIST;
        boundary_marker.action = visualization_msgs::Marker::ADD;
        boundary_marker.scale.x = 0.1;
        boundary_marker.color.r = 1.0;
        boundary_marker.color.g = 0.0;
        boundary_marker.color.b = 0.0;
        boundary_marker.color.a = 1.0;

        // 边界1
        geometry_msgs::Point p1, p2, p3, p4;
        p1.x = boundary_1_start_.first;
        p1.y = boundary_1_start_.second;
        p1.z = 0.0;
        p2.x = boundary_1_end_.first;
        p2.y = boundary_1_end_.second;
        p2.z = 0.0;

        // 边界2
        p3.x = boundary_2_start_.first;
        p3.y = boundary_2_start_.second;
        p3.z = 0.0;
        p4.x = boundary_2_end_.first;
        p4.y = boundary_2_end_.second;
        p4.z = 0.0;

        boundary_marker.points.push_back(p1);
        boundary_marker.points.push_back(p2);
        boundary_marker.points.push_back(p3);
        boundary_marker.points.push_back(p4);

        // 连接形成矩形
        boundary_marker.points.push_back(p1);
        boundary_marker.points.push_back(p3);
        boundary_marker.points.push_back(p2);
        boundary_marker.points.push_back(p4);

        markers.markers.push_back(boundary_marker);

        // 可视化目标位置
        visualization_msgs::Marker target_marker;
        target_marker.header.frame_id = "map";
        target_marker.header.stamp = ros::Time::now();
        target_marker.ns = "parking_target";
        target_marker.id = 1;
        target_marker.type = visualization_msgs::Marker::ARROW;
        target_marker.action = visualization_msgs::Marker::ADD;
        target_marker.pose.position.x = parking_target_.x;
        target_marker.pose.position.y = parking_target_.y;
        target_marker.pose.position.z = 0.5;
        target_marker.pose.orientation = tf::createQuaternionMsgFromYaw(parking_target_.theta);
        target_marker.scale.x = 1.0;
        target_marker.scale.y = 0.2;
        target_marker.scale.z = 0.2;
        target_marker.color.r = 0.0;
        target_marker.color.g = 1.0;
        target_marker.color.b = 0.0;
        target_marker.color.a = 0.8;

        markers.markers.push_back(target_marker);

        slot_marker_pub_.publish(markers);
    }

    // ROS相关
    ros::NodeHandle nh_;
    ros::Timer control_timer_;
    ros::Timer vis_timer_;
    ros::Publisher slot_marker_pub_;

    // 模块
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
    ParkingState state_ = ParkingState::APPROACHING;
    CarState vehicle_state_;
    std::vector<CarState> global_path_;
    std::vector<CarState> parking_trajectory_;
    size_t parking_traj_index_ = 0;

    // 停车位信息
    CarState parking_target_;
    double parking_slot_length_ = 0.0;
    double parking_slot_width_ = 0.0;
    std::pair<double, double> boundary_1_start_;
    std::pair<double, double> boundary_1_end_;
    std::pair<double, double> boundary_2_start_;
    std::pair<double, double> boundary_2_end_;

    // 停车位中心线（两次转向的目标线）
    double centerline_start_x_ = 0.0;
    double centerline_start_y_ = 0.0;
    double centerline_direction_x_ = 0.0;
    double centerline_direction_y_ = 0.0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "horizontal_parking_node");

    try {
        HorizontalParkingSystem system;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
