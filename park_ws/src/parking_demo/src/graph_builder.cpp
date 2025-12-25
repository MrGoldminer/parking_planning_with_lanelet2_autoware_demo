/**
 * @file graph_builder.cpp
 * @brief 图构建器实现
 */

#include "parking_demo/graph_builder.h"
#include <ros/ros.h>
#include <queue>
#include <algorithm>
#include <cmath>

namespace parking_demo {

GraphBuilder::GraphBuilder() {
    ROS_INFO("GraphBuilder initialized");
}

GraphBuilder::~GraphBuilder() {
}

void GraphBuilder::buildFromOSM(const std::map<long long, NodePoint>& nodes,
                                const std::vector<Way>& ways,
                                const std::vector<Relation>& relations) {
    ROS_INFO("Building graph from OSM data...");

    nodes_ptr_ = &nodes;

    // 清空旧数据
    adjacency_list_.clear();
    relation_adjacency_.clear();

    // 1. 构建节点邻接图
    buildNodeAdjacency(ways);

    // 2. 构建relation邻接图
    buildRelationAdjacency(ways, relations);

    ROS_INFO("Graph built: %lu nodes, %d edges",
             adjacency_list_.size(),
             checkConnectivity());
}

void GraphBuilder::buildNodeAdjacency(const std::vector<Way>& ways) {
    ROS_INFO("Building node adjacency graph...");

    for (const auto& w : ways) {
        // 检查是否为单向（Successor关系）
        bool is_successor = false;
        std::string lr = w.relation;
        std::transform(lr.begin(), lr.end(), lr.begin(), ::tolower);
        if (lr.find("successor") != std::string::npos) {
            is_successor = true;
        }

        // 连接way中的相邻节点
        for (size_t i = 0; i + 1 < w.node_refs.size(); ++i) {
            long long n1 = w.node_refs[i];
            long long n2 = w.node_refs[i + 1];

            // 正向连接
            adjacency_list_[n1].push_back(n2);

            // 如果不是单向，添加反向连接
            if (!is_successor) {
                adjacency_list_[n2].push_back(n1);
            }
        }
    }

    ROS_INFO("Node adjacency built: %lu nodes", adjacency_list_.size());
}

void GraphBuilder::buildRelationAdjacency(const std::vector<Way>& ways,
                                         const std::vector<Relation>& relations) {
    ROS_INFO("Building relation adjacency graph...");

    // Helper: 计算relation的质心
    auto computeCentroid = [&](const Relation& r) -> std::pair<double, double> {
        double sx = 0, sy = 0;
        int count = 0;

        for (long long wid : r.member_way_refs) {
            // 查找way
            for (const auto& w : ways) {
                if (w.id == wid) {
                    // 累加way中所有节点的坐标
                    for (long long nid : w.node_refs) {
                        if (nodes_ptr_->count(nid)) {
                            sx += nodes_ptr_->at(nid).x;
                            sy += nodes_ptr_->at(nid).y;
                            ++count;
                        }
                    }
                    break;
                }
            }
        }

        if (count == 0) return {0.0, 0.0};
        return {sx / count, sy / count};
    };

    // 策略1: 共享way的relations相邻
    std::map<long long, std::vector<long long>> way_to_relations;
    for (const auto& r : relations) {
        for (long long wid : r.member_way_refs) {
            way_to_relations[wid].push_back(r.id);
        }
    }

    for (const auto& kv : way_to_relations) {
        const auto& rels = kv.second;
        for (size_t i = 0; i < rels.size(); ++i) {
            for (size_t j = i + 1; j < rels.size(); ++j) {
                relation_adjacency_[rels[i]].push_back(rels[j]);
                relation_adjacency_[rels[j]].push_back(rels[i]);
            }
        }
    }

    // 策略2: 质心距离<8m的relations相邻
    for (size_t i = 0; i < relations.size(); ++i) {
        for (size_t j = i + 1; j < relations.size(); ++j) {
            auto ci = computeCentroid(relations[i]);
            auto cj = computeCentroid(relations[j]);

            double dx = ci.first - cj.first;
            double dy = ci.second - cj.second;
            double dist = std::hypot(dx, dy);

            if (dist < 8.0) {
                relation_adjacency_[relations[i].id].push_back(relations[j].id);
                relation_adjacency_[relations[j].id].push_back(relations[i].id);
            }
        }
    }

    ROS_INFO("Relation adjacency built: %lu relations", relation_adjacency_.size());
}

int GraphBuilder::checkConnectivity() {
    if (adjacency_list_.empty()) return 0;

    std::set<long long> visited;
    int max_component_size = 0;

    for (const auto& kv : adjacency_list_) {
        if (visited.count(kv.first)) continue;

        // BFS查找连通分量
        std::queue<long long> q;
        q.push(kv.first);
        visited.insert(kv.first);
        int component_size = 0;

        while (!q.empty()) {
            long long curr = q.front();
            q.pop();
            component_size++;

            if (adjacency_list_.count(curr)) {
                for (long long next : adjacency_list_.at(curr)) {
                    if (!visited.count(next)) {
                        visited.insert(next);
                        q.push(next);
                    }
                }
            }
        }

        max_component_size = std::max(max_component_size, component_size);
    }

    double connectivity_ratio = 100.0 * max_component_size / adjacency_list_.size();
    ROS_INFO("Graph connectivity: %d/%lu nodes (%.1f%%) in main component",
             max_component_size, adjacency_list_.size(), connectivity_ratio);

    if (connectivity_ratio < 80.0) {
        ROS_WARN("Graph is fragmented! Only %.1f%% nodes are connected.", connectivity_ratio);
    }

    return max_component_size;
}

long long GraphBuilder::findNearestGraphNode(double x, double y) const {
    if (!nodes_ptr_ || adjacency_list_.empty()) {
        ROS_WARN("Cannot find nearest graph node: data not initialized");
        return -1;
    }

    long long best_node = -1;
    double best_dist = std::numeric_limits<double>::max();

    // 只搜索在图中有连接的节点
    for (const auto& kv : adjacency_list_) {
        long long nid = kv.first;

        // 跳过孤立节点
        if (kv.second.empty()) continue;

        // 检查节点是否存在
        if (!nodes_ptr_->count(nid)) continue;

        const NodePoint& node = nodes_ptr_->at(nid);
        double dx = node.x - x;
        double dy = node.y - y;
        double dist = dx * dx + dy * dy;

        if (dist < best_dist) {
            best_dist = dist;
            best_node = nid;
        }
    }

    if (best_node != -1) {
        ROS_INFO("Found nearest graph node: id=%lld, dist=%.2fm",
                 best_node, std::sqrt(best_dist));
    } else {
        ROS_WARN("No valid graph node found near (%.2f, %.2f)", x, y);
    }

    return best_node;
}

} // namespace parking_demo
