/**
 * @file path_planner.cpp
 * @brief 路径规划器实现
 */

#include "parking_demo/path_planner.h"
#include <ros/ros.h>
#include <queue>
#include <algorithm>
#include <cmath>

namespace parking_demo {

PathPlanner::PathPlanner() {
    ROS_INFO("PathPlanner initialized");
}

PathPlanner::~PathPlanner() {
}

void PathPlanner::setMapData(const std::map<long long, NodePoint>& nodes,
                            const std::vector<Way>& ways,
                            const std::vector<Relation>& relations) {
    nodes_ = &nodes;
    ways_ = &ways;
    relations_ = &relations;
    ROS_INFO("PathPlanner: map data set (%lu nodes, %lu ways, %lu relations)",
             nodes.size(), ways.size(), relations.size());
}

void PathPlanner::setGraphBuilder(const GraphBuilder* graph_builder) {
    graph_builder_ = graph_builder;
    ROS_INFO("PathPlanner: graph builder set");
}

std::vector<long long> PathPlanner::planNodePath(long long start_node, long long goal_node) {
    if (!graph_builder_) {
        ROS_ERROR("GraphBuilder not set!");
        return {};
    }

    const auto& adjacency = graph_builder_->getAdjacencyList();

    ROS_INFO("Planning node path: start=%lld, goal=%lld", start_node, goal_node);

    // 检查起点和终点是否在图中
    if (adjacency.find(start_node) == adjacency.end()) {
        ROS_ERROR("Start node %lld not in graph!", start_node);
        return {};
    }

    if (adjacency.find(goal_node) == adjacency.end()) {
        ROS_ERROR("Goal node %lld not in graph!", goal_node);
        return {};
    }

    // BFS搜索
    std::queue<long long> q;
    std::map<long long, long long> came_from;
    q.push(start_node);
    came_from[start_node] = start_node;

    int visited_count = 0;
    while (!q.empty()) {
        long long current = q.front();
        q.pop();
        visited_count++;

        if (current == goal_node) {
            ROS_INFO("Path found! Visited %d nodes.", visited_count);
            break;
        }

        if (adjacency.find(current) != adjacency.end()) {
            for (long long next : adjacency.at(current)) {
                if (came_from.find(next) == came_from.end()) {
                    q.push(next);
                    came_from[next] = current;
                }
            }
        }
    }

    // 重建路径
    std::vector<long long> path;
    if (came_from.find(goal_node) == came_from.end()) {
        ROS_ERROR("No path found from %lld to %lld (visited %d nodes)",
                  start_node, goal_node, visited_count);
        return path;
    }

    long long curr = goal_node;
    while (curr != start_node) {
        path.push_back(curr);
        curr = came_from[curr];
    }
    path.push_back(start_node);
    std::reverse(path.begin(), path.end());

    ROS_INFO("Path found with %lu waypoints", path.size());
    return path;
}

std::vector<long long> PathPlanner::planRelationPath(long long start_rel, long long goal_rel) {
    if (!graph_builder_) {
        ROS_ERROR("GraphBuilder not set!");
        return {};
    }

    const auto& rel_adjacency = graph_builder_->getRelationAdjacency();

    ROS_INFO("Planning relation path: start=%lld, goal=%lld", start_rel, goal_rel);

    // 检查起点和终点
    if (rel_adjacency.find(start_rel) == rel_adjacency.end()) {
        ROS_ERROR("Start relation %lld not in graph!", start_rel);
        return {};
    }

    if (rel_adjacency.find(goal_rel) == rel_adjacency.end()) {
        ROS_ERROR("Goal relation %lld not in graph!", goal_rel);
        return {};
    }

    // BFS搜索
    std::queue<long long> q;
    std::map<long long, long long> came_from;
    q.push(start_rel);
    came_from[start_rel] = start_rel;

    int visited = 0;
    while (!q.empty()) {
        long long cur = q.front();
        q.pop();
        visited++;

        if (cur == goal_rel) {
            ROS_INFO("Relation path found! Visited %d relations.", visited);
            break;
        }

        if (rel_adjacency.find(cur) != rel_adjacency.end()) {
            for (long long nxt : rel_adjacency.at(cur)) {
                if (came_from.find(nxt) == came_from.end()) {
                    q.push(nxt);
                    came_from[nxt] = cur;
                }
            }
        }
    }

    // 重建路径
    std::vector<long long> path;
    if (came_from.find(goal_rel) == came_from.end()) {
        ROS_ERROR("No relation path found (visited %d relations)", visited);
        return path;
    }

    long long cur = goal_rel;
    while (cur != start_rel) {
        path.push_back(cur);
        cur = came_from[cur];
    }
    path.push_back(start_rel);
    std::reverse(path.begin(), path.end());

    ROS_INFO("Relation path: %lu lanelets", path.size());
    return path;
}

std::vector<CarState> PathPlanner::nodesToTrajectory(const std::vector<long long>& node_ids,
                                                     double default_speed) {
    if (!nodes_) {
        ROS_ERROR("Nodes data not set!");
        return {};
    }

    std::vector<CarState> trajectory;

    for (size_t i = 0; i < node_ids.size(); ++i) {
        long long nid = node_ids[i];
        if (!nodes_->count(nid)) {
            ROS_WARN("Node %lld not found, skipping", nid);
            continue;
        }

        CarState s;
        s.x = nodes_->at(nid).x;
        s.y = nodes_->at(nid).y;
        s.v = default_speed;

        // 估计朝向
        if (i + 1 < node_ids.size() && nodes_->count(node_ids[i + 1])) {
            double dx = nodes_->at(node_ids[i + 1]).x - s.x;
            double dy = nodes_->at(node_ids[i + 1]).y - s.y;
            s.theta = std::atan2(dy, dx);
        } else if (i > 0 && !trajectory.empty()) {
            s.theta = trajectory.back().theta;
        }

        trajectory.push_back(s);
    }

    ROS_INFO("Generated trajectory with %lu states", trajectory.size());
    return trajectory;
}

std::vector<std::pair<double, double>> PathPlanner::relationsToCenterline(
    const std::vector<long long>& rel_ids,
    int samples_per_rel) {

    std::vector<std::pair<double, double>> centerline;

    for (long long rel_id : rel_ids) {
        auto rel_centerline = computeRelationCenterline(rel_id, samples_per_rel);
        centerline.insert(centerline.end(), rel_centerline.begin(), rel_centerline.end());
    }

    // 去重：移除过于接近的点
    std::vector<std::pair<double, double>> filtered;
    for (size_t i = 0; i < centerline.size(); ++i) {
        if (i == 0) {
            filtered.push_back(centerline[i]);
        } else {
            double dx = centerline[i].first - filtered.back().first;
            double dy = centerline[i].second - filtered.back().second;
            double dist = std::hypot(dx, dy);
            if (dist > 0.05) {  // 5cm阈值
                filtered.push_back(centerline[i]);
            }
        }
    }

    ROS_INFO("Centerline: %lu points (filtered from %lu)", filtered.size(), centerline.size());
    return filtered;
}

std::vector<CarState> PathPlanner::resamplePath(const std::vector<CarState>& path,
                                                double max_spacing) {
    if (path.empty()) return {};

    std::vector<CarState> resampled;
    resampled.push_back(path[0]);

    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].x - resampled.back().x;
        double dy = path[i].y - resampled.back().y;
        double dist = std::hypot(dx, dy);

        // 如果两点距离超过max_spacing，插入中间点
        if (dist > max_spacing) {
            int n_insert = static_cast<int>(std::ceil(dist / max_spacing)) - 1;
            for (int j = 1; j <= n_insert; ++j) {
                double t = static_cast<double>(j) / (n_insert + 1);
                CarState s_new;
                s_new.x = resampled.back().x + dx * t;
                s_new.y = resampled.back().y + dy * t;
                s_new.theta = std::atan2(dy, dx);
                s_new.v = 2.0;  // 默认速度
                s_new.phi = 0.0;
                resampled.push_back(s_new);
            }
        }

        resampled.push_back(path[i]);
    }

    ROS_INFO("Resampled path: %lu points (from %lu)", resampled.size(), path.size());
    return resampled;
}

std::vector<CarState> PathPlanner::generateEmergencyPath(const CarState& start,
                                                         double goal_x,
                                                         double goal_y,
                                                         double goal_theta) {
    ROS_WARN("Generating emergency straight-line path");

    std::vector<CarState> path;

    double dx = goal_x - start.x;
    double dy = goal_y - start.y;
    double dist = std::hypot(dx, dy);

    int n_points = std::max(10, static_cast<int>(dist / 0.5));

    for (int i = 0; i <= n_points; ++i) {
        double t = static_cast<double>(i) / n_points;
        CarState s;
        s.x = start.x + dx * t;
        s.y = start.y + dy * t;
        s.theta = (i < n_points) ? std::atan2(dy, dx) : goal_theta;
        s.v = 1.5;
        s.phi = 0.0;
        path.push_back(s);
    }

    ROS_INFO("Emergency path generated: %lu points", path.size());
    return path;
}

std::vector<std::pair<double, double>> PathPlanner::computeRelationCenterline(long long rel_id, int samples) {
    if (!relations_ || !ways_ || !nodes_) {
        ROS_ERROR("Map data not set!");
        return {};
    }

    // 查找relation
    const Relation* rel = nullptr;
    for (const auto& r : *relations_) {
        if (r.id == rel_id) {
            rel = &r;
            break;
        }
    }

    if (!rel || rel->member_way_refs.empty()) {
        return {};
    }

    // 对每个member way进行采样
    std::vector<std::vector<std::pair<double, double>>> all_way_samples;
    for (long long wid : rel->member_way_refs) {
        // 查找way
        const Way* way = nullptr;
        for (const auto& w : *ways_) {
            if (w.id == wid) {
                way = &w;
                break;
            }
        }

        if (way) {
            auto way_samples = resampleWay(*way, samples);
            if (!way_samples.empty()) {
                all_way_samples.push_back(way_samples);
            }
        }
    }

    if (all_way_samples.empty()) {
        return {};
    }

    // 计算centerline（所有ways的平均）
    size_t M = all_way_samples[0].size();
    std::vector<std::pair<double, double>> centerline(M, {0.0, 0.0});

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
            centerline[i].first = sx / count;
            centerline[i].second = sy / count;
        }
    }

    return centerline;
}

std::vector<std::pair<double, double>> PathPlanner::resampleWay(const Way& way, int num_samples) {
    if (!nodes_) {
        return {};
    }

    // 收集way的所有节点坐标
    std::vector<std::pair<double, double>> pts;
    for (long long nid : way.node_refs) {
        if (nodes_->count(nid)) {
            pts.emplace_back(nodes_->at(nid).x, nodes_->at(nid).y);
        }
    }

    if (pts.empty() || pts.size() == 1) {
        return pts;
    }

    // 计算累积距离
    std::vector<double> dists(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i) {
        double dx = pts[i].first - pts[i - 1].first;
        double dy = pts[i].second - pts[i - 1].second;
        dists[i] = dists[i - 1] + std::hypot(dx, dy);
    }

    double total_length = dists.back();
    if (total_length < 1e-6) {
        return {pts[0]};
    }

    // 线性插值重采样
    std::vector<std::pair<double, double>> resampled;
    for (int k = 0; k < num_samples; ++k) {
        double s = (total_length * k) / (num_samples - 1);

        // 找到对应的区间
        size_t idx = 0;
        while (idx + 1 < dists.size() && dists[idx + 1] < s) {
            ++idx;
        }

        if (idx + 1 >= pts.size()) {
            resampled.push_back(pts.back());
            continue;
        }

        double seg_length = dists[idx + 1] - dists[idx];
        double t = (seg_length > 1e-9) ? ((s - dists[idx]) / seg_length) : 0.0;

        double x = pts[idx].first * (1 - t) + pts[idx + 1].first * t;
        double y = pts[idx].second * (1 - t) + pts[idx + 1].second * t;

        resampled.emplace_back(x, y);
    }

    return resampled;
}

} // namespace parking_demo
