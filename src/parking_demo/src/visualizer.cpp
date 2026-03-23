/**
 * @file visualizer.cpp
 * @brief ROS可视化管理器实现
 */

#include "parking_demo/visualizer.h"
#include <ros/ros.h>

namespace parking_demo {

Visualizer::Visualizer(ros::NodeHandle& nh) : nh_(nh) {
    // 初始化发布器
    ways_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/osm/ways", 1, true);
    relations_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/osm/relations", 1, true);
    nodes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/osm/nodes", 1, true);
    parking_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/parking_spots", 1, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1, true);
    car_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/car_marker", 10, false);
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/pose2d", 10);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    lookahead_pub_ = nh_.advertise<visualization_msgs::Marker>("/lookahead_marker", 5);

    ROS_INFO("Visualizer initialized");
}

Visualizer::~Visualizer() {
}

void Visualizer::publishMap(const std::map<long long, NodePoint>& nodes,
                            const std::vector<Way>& ways,
                            const std::vector<Relation>& relations) {
    ROS_INFO("Publishing map visualization...");

    // 发布Ways（LINE_LIST）
    visualization_msgs::MarkerArray ways_arr;
    visualization_msgs::Marker ways_marker;
    ways_marker.header.frame_id = "map";
    ways_marker.header.stamp = ros::Time::now();
    ways_marker.ns = "lanelet_ways";
    ways_marker.type = visualization_msgs::Marker::LINE_LIST;
    ways_marker.scale.x = 0.1;
    ways_marker.color.a = 0.6;
    ways_marker.color.r = 1.0;
    ways_marker.color.g = 1.0;
    ways_marker.color.b = 1.0;
    ways_marker.id = 0;

    for (const auto& w : ways) {
        for (size_t i = 0; i + 1 < w.node_refs.size(); ++i) {
            long long id1 = w.node_refs[i];
            long long id2 = w.node_refs[i + 1];

            if (nodes.count(id1) && nodes.count(id2)) {
                geometry_msgs::Point p1, p2;
                p1.x = nodes.at(id1).x;
                p1.y = nodes.at(id1).y;
                p1.z = 0.0;
                p2.x = nodes.at(id2).x;
                p2.y = nodes.at(id2).y;
                p2.z = 0.0;

                ways_marker.points.push_back(p1);
                ways_marker.points.push_back(p2);
            }
        }
    }

    ways_arr.markers.push_back(ways_marker);
    ways_pub_.publish(ways_arr);

    // 发布Relations（LINE_STRIP）
    visualization_msgs::MarkerArray rels_arr;
    int rel_id = 0;
    for (const auto& r : relations) {
        visualization_msgs::Marker rel_marker;
        rel_marker.header.frame_id = "map";
        rel_marker.header.stamp = ros::Time::now();
        rel_marker.ns = std::string("relation_") + std::to_string(r.id);
        rel_marker.type = visualization_msgs::Marker::LINE_STRIP;
        rel_marker.scale.x = 0.15;
        rel_marker.color.a = 0.7;
        rel_marker.color.r = 1.0;
        rel_marker.color.g = 0.0;
        rel_marker.color.b = 1.0;
        rel_marker.id = rel_id++;

        // 连接member ways
        for (long long way_id : r.member_way_refs) {
            for (const auto& w : ways) {
                if (w.id == way_id) {
                    for (long long nid : w.node_refs) {
                        if (nodes.count(nid)) {
                            geometry_msgs::Point p;
                            p.x = nodes.at(nid).x;
                            p.y = nodes.at(nid).y;
                            p.z = 0.0;
                            rel_marker.points.push_back(p);
                        }
                    }
                    break;
                }
            }
        }

        rels_arr.markers.push_back(rel_marker);
    }
    relations_pub_.publish(rels_arr);

    // 发布Nodes（SPHERE）
    visualization_msgs::MarkerArray nodes_arr;
    int nid_idx = 0;
    for (const auto& kv : nodes) {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "osm_nodes";
        m.id = nid_idx++;
        m.type = visualization_msgs::Marker::SPHERE;
        m.pose.position.x = kv.second.x;
        m.pose.position.y = kv.second.y;
        m.pose.position.z = 0.0;
        m.scale.x = 0.2;
        m.scale.y = 0.2;
        m.scale.z = 0.2;
        m.color.a = 0.4;
        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        nodes_arr.markers.push_back(m);
    }
    nodes_pub_.publish(nodes_arr);

    ROS_INFO("Map visualization published");
}

void Visualizer::publishParkingSpots(const std::vector<ParkingSpot>& spots,
                                     const std::map<long long, NodePoint>& nodes,
                                     const std::vector<Way>& ways) {
    visualization_msgs::MarkerArray park_arr;
    int pid = 0;

    for (const auto& spot : spots) {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "parking_spots";
        m.id = pid++;
        m.type = visualization_msgs::Marker::CUBE;
        m.pose.position.x = spot.x;
        m.pose.position.y = spot.y;
        m.pose.position.z = 0.05;
        m.scale.x = 2.5;
        m.scale.y = 1.25;
        m.scale.z = 0.1;
        m.color.a = 0.6;
        m.color.g = 1.0;
        m.color.r = 0.0;
        m.color.b = 0.0;
        park_arr.markers.push_back(m);
    }

    parking_pub_.publish(park_arr);
    ROS_INFO("Published %lu parking spots", spots.size());
}

void Visualizer::publishPath(const std::vector<CarState>& path) {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    for (const auto& state : path) {
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = state.x;
        ps.pose.position.y = state.y;
        ps.pose.position.z = 0.0;
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(state.theta);
        path_msg.poses.push_back(ps);
    }

    path_pub_.publish(path_msg);
    // ROS_INFO("Published path with %lu poses", path.size());
}

void Visualizer::publishVehicleState(const CarState& state, const std::string& frame_id) {
    // 发布车辆Marker（CUBE）
    visualization_msgs::MarkerArray car_marker;
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = "car_body";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE;
    m.pose.position.x = state.x;
    m.pose.position.y = state.y;
    m.pose.position.z = 0.5;
    m.pose.orientation = tf::createQuaternionMsgFromYaw(state.theta);
    m.scale.x = 3.5;
    m.scale.y = 1.6;
    m.scale.z = 1.0;
    m.color.a = 0.8;
    m.color.b = 1.0;
    car_marker.markers.push_back(m);
    car_pub_.publish(car_marker);

    // 发布Pose2D
    geometry_msgs::Pose2D p2d;
    p2d.x = state.x;
    p2d.y = state.y;
    p2d.theta = state.theta;
    pose_pub_.publish(p2d);
}

void Visualizer::publishTF(const CarState& state,
                          const std::string& parent_frame,
                          const std::string& child_frame) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(state.x, state.y, 0.0));

    tf::Quaternion q;
    q.setRPY(0, 0, state.theta);
    transform.setRotation(q);

    tf_broadcaster_.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
}

void Visualizer::publishLookaheadPoint(double x, double y) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "lookahead";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.color.a = 0.9;
    mk.color.r = 1.0;
    mk.color.g = 0.1;
    mk.color.b = 0.1;
    mk.pose.position.x = x;
    mk.pose.position.y = y;
    mk.pose.position.z = 0.1;

    lookahead_pub_.publish(mk);
}

void Visualizer::publishControlCommand(double v, double steering_angle) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = steering_angle;
    cmd_pub_.publish(cmd);
}

} // namespace parking_demo
