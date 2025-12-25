// ============================================================
// 修复路径规划失败问题的关键代码段
// ============================================================

// ====== 问题诊断1: 图连通性检查 ======
// 在buildGraph()函数末尾添加连通性检查:

void buildGraph() {
    // ... 原有的图构建代码 ...

    // ✅ 新增：检查图的连通性
    ROS_INFO("=== Graph Connectivity Check ===");
    ROS_INFO("Total adjacency nodes: %lu", adjacency_list_.size());

    // 统计每个节点的邻居数量
    int isolated_nodes = 0;
    int total_edges = 0;
    for (const auto& kv : adjacency_list_) {
        int neighbors = kv.second.size();
        total_edges += neighbors;
        if (neighbors == 0) {
            isolated_nodes++;
            ROS_WARN("Isolated node found: id=%lld at (%.2f, %.2f)",
                     kv.first,
                     nodes_map_[kv.first].x,
                     nodes_map_[kv.first].y);
        }
    }
    ROS_INFO("Total edges: %d, Isolated nodes: %d", total_edges, isolated_nodes);

    // ✅ 使用DFS/BFS检查最大连通分量
    std::set<long long> visited;
    int max_component_size = 0;
    long long max_component_root = -1;

    for (const auto& kv : adjacency_list_) {
        if (visited.count(kv.first)) continue;

        // BFS to find component size
        std::queue<long long> q;
        q.push(kv.first);
        visited.insert(kv.first);
        int component_size = 0;
        long long component_root = kv.first;

        while (!q.empty()) {
            long long curr = q.front(); q.pop();
            component_size++;

            for (long long next : adjacency_list_[curr]) {
                if (!visited.count(next)) {
                    visited.insert(next);
                    q.push(next);
                }
            }
        }

        if (component_size > max_component_size) {
            max_component_size = component_size;
            max_component_root = component_root;
        }

        ROS_INFO("Found connected component: root=%lld, size=%d", component_root, component_size);
    }

    ROS_INFO("Largest connected component: root=%lld, size=%d/%lu (%.1f%%)",
             max_component_root,
             max_component_size,
             adjacency_list_.size(),
             100.0 * max_component_size / adjacency_list_.size());

    // 如果最大连通分量太小，发出警告
    if (max_component_size < adjacency_list_.size() * 0.8) {
        ROS_WARN("⚠️ Graph is fragmented! Only %.1f%% nodes are in the main component.",
                 100.0 * max_component_size / adjacency_list_.size());
        ROS_WARN("This may cause planning failures. Check OSM data for disconnected ways.");
    }

    // Build relation adjacency graph
    // ... 原有的relation adjacency构建代码 ...
}


// ====== 问题诊断2: 改进的BFS with路径验证 ======
// 替换原有的bfs函数:

std::vector<long long> bfs(long long start, long long goal) {
    ROS_INFO("=== BFS Planning: start=%lld, goal=%lld ===", start, goal);

    // ✅ 检查起点和终点是否在图中
    if (adjacency_list_.find(start) == adjacency_list_.end()) {
        ROS_ERROR("❌ Start node %lld not found in adjacency_list!", start);
        // 尝试找到最近的替代起点
        double min_dist = 1e18;
        long long alt_start = -1;
        if (nodes_map_.count(start)) {
            for (const auto& kv : adjacency_list_) {
                if (!nodes_map_.count(kv.first)) continue;
                double dx = nodes_map_[kv.first].x - nodes_map_[start].x;
                double dy = nodes_map_[kv.first].y - nodes_map_[start].y;
                double d = std::hypot(dx, dy);
                if (d < min_dist) {
                    min_dist = d;
                    alt_start = kv.first;
                }
            }
            if (alt_start != -1 && min_dist < 10.0) {
                ROS_WARN("Using alternative start node %lld (%.2fm away)", alt_start, min_dist);
                start = alt_start;
            } else {
                return std::vector<long long>();
            }
        } else {
            return std::vector<long long>();
        }
    }

    if (adjacency_list_.find(goal) == adjacency_list_.end()) {
        ROS_ERROR("❌ Goal node %lld not found in adjacency_list!", goal);
        // 尝试找到最近的替代终点
        double min_dist = 1e18;
        long long alt_goal = -1;
        if (nodes_map_.count(goal)) {
            for (const auto& kv : adjacency_list_) {
                if (!nodes_map_.count(kv.first)) continue;
                double dx = nodes_map_[kv.first].x - nodes_map_[goal].x;
                double dy = nodes_map_[kv.first].y - nodes_map_[goal].y;
                double d = std::hypot(dx, dy);
                if (d < min_dist) {
                    min_dist = d;
                    alt_goal = kv.first;
                }
            }
            if (alt_goal != -1 && min_dist < 10.0) {
                ROS_WARN("Using alternative goal node %lld (%.2fm away)", alt_goal, min_dist);
                goal = alt_goal;
            } else {
                return std::vector<long long>();
            }
        } else {
            return std::vector<long long>();
        }
    }

    // ✅ 标准BFS搜索
    std::queue<long long> q;
    std::map<long long, long long> came_from;
    q.push(start);
    came_from[start] = start;

    int visited_count = 0;
    while (!q.empty()) {
        long long current = q.front();
        q.pop();
        visited_count++;

        if (current == goal) {
            ROS_INFO("✅ Path found! Visited %d nodes.", visited_count);
            break;
        }

        // ✅ 检查adjacency_list_[current]是否存在
        if (adjacency_list_.find(current) == adjacency_list_.end()) {
            ROS_WARN("Node %lld has no adjacency entry", current);
            continue;
        }

        for (long long next : adjacency_list_[current]) {
            if (came_from.find(next) == came_from.end()) {
                q.push(next);
                came_from[next] = current;
            }
        }
    }

    // 重建路径
    std::vector<long long> path;
    if (came_from.find(goal) == came_from.end()) {
        ROS_ERROR("❌ No path found from %lld to %lld (visited %d nodes)",
                  start, goal, visited_count);

        // ✅ 诊断：打印起点和终点的邻居信息
        if (adjacency_list_.count(start)) {
            ROS_INFO("Start node %lld has %lu neighbors", start, adjacency_list_[start].size());
        }
        if (adjacency_list_.count(goal)) {
            ROS_INFO("Goal node %lld has %lu neighbors", goal, adjacency_list_[goal].size());
        }

        return path;
    }

    long long curr = goal;
    while (curr != start) {
        path.push_back(curr);
        curr = came_from[curr];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    ROS_INFO("✅ Path found with %lu waypoints", path.size());
    return path;
}


// ====== 问题诊断3: 改进relation-level规划 ======
// 替换findRelationPath函数:

std::vector<long long> findRelationPath(long long start_rel, long long goal_rel) {
    ROS_INFO("=== Relation-Level Planning: start=%lld, goal=%lld ===", start_rel, goal_rel);

    // ✅ 检查relation adjacency
    if (relation_adjacency_.find(start_rel) == relation_adjacency_.end()) {
        ROS_ERROR("❌ Start relation %lld not found in relation_adjacency_!", start_rel);
        return std::vector<long long>();
    }

    if (relation_adjacency_.find(goal_rel) == relation_adjacency_.end()) {
        ROS_ERROR("❌ Goal relation %lld not found in relation_adjacency_!", goal_rel);
        return std::vector<long long>();
    }

    ROS_INFO("Start relation has %lu neighbors", relation_adjacency_[start_rel].size());
    ROS_INFO("Goal relation has %lu neighbors", relation_adjacency_[goal_rel].size());

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
            ROS_INFO("✅ Relation path found! Visited %d relations.", visited);
            break;
        }

        if (relation_adjacency_.find(cur) == relation_adjacency_.end()) {
            continue;
        }

        for (long long nxt : relation_adjacency_[cur]) {
            if (came_from.find(nxt) == came_from.end()) {
                q.push(nxt);
                came_from[nxt] = cur;
            }
        }
    }

    std::vector<long long> path;
    if (came_from.find(goal_rel) == came_from.end()) {
        ROS_ERROR("❌ No relation path found (visited %d relations)", visited);
        return path;
    }

    long long cur = goal_rel;
    while (cur != start_rel) {
        path.push_back(cur);
        cur = came_from[cur];
    }
    path.push_back(start_rel);
    std::reverse(path.begin(), path.end());

    ROS_INFO("✅ Relation path: %lu lanelets", path.size());
    return path;
}


// ====== 问题诊断4: 改进起点和终点选择逻辑 ======
// 在planTask()中，改进节点选择:

void planTask() {
    // ... 前面的代码 ...

    // ✅ 改进的起点选择
    long long start_node_id = -1;
    if (start_rel) {
        // 优先选择relation中有邻接边的节点
        std::vector<long long> candidate_starts;
        for (long long wid : start_rel->member_way_refs) {
            Way* w = findWayById(wid);
            if (!w) continue;
            for (long long nid : w->node_refs) {
                if (adjacency_list_.count(nid) && !adjacency_list_[nid].empty()) {
                    candidate_starts.push_back(nid);
                }
            }
        }

        if (!candidate_starts.empty()) {
            // 从候选中选择离车辆最近的
            double best_d = 1e18;
            for (long long cand : candidate_starts) {
                if (!nodes_map_.count(cand)) continue;
                double dx = nodes_map_[cand].x - car_.x;
                double dy = nodes_map_[cand].y - car_.y;
                double d = dx*dx + dy*dy;
                if (d < best_d) {
                    best_d = d;
                    start_node_id = cand;
                }
            }
            ROS_INFO("✅ Selected start node %lld with %lu neighbors (dist=%.2fm)",
                     start_node_id,
                     adjacency_list_[start_node_id].size(),
                     std::sqrt(best_d));
        }
    }

    // fallback
    if (start_node_id == -1) {
        // 找最近的有邻接边的节点
        double best_d = 1e18;
        for (const auto& kv : adjacency_list_) {
            if (kv.second.empty()) continue;  // 跳过孤立节点
            if (!nodes_map_.count(kv.first)) continue;

            double dx = nodes_map_[kv.first].x - car_.x;
            double dy = nodes_map_[kv.first].y - car_.y;
            double d = dx*dx + dy*dy;

            if (d < best_d) {
                best_d = d;
                start_node_id = kv.first;
            }
        }
        if (start_node_id != -1) {
            ROS_WARN("Using fallback start node %lld (dist=%.2fm, neighbors=%lu)",
                     start_node_id,
                     std::sqrt(best_d),
                     adjacency_list_[start_node_id].size());
        }
    }

    // ✅ 改进的终点选择（类似逻辑）
    long long target_node = -1;
    if (park_rel) {
        std::vector<long long> candidate_goals;
        for (long long wid : park_rel->member_way_refs) {
            Way* w = findWayById(wid);
            if (!w) continue;
            for (long long nid : w->node_refs) {
                if (adjacency_list_.count(nid) && !adjacency_list_[nid].empty()) {
                    candidate_goals.push_back(nid);
                }
            }
        }

        if (!candidate_goals.empty()) {
            double best_d = 1e18;
            for (long long cand : candidate_goals) {
                if (!nodes_map_.count(cand)) continue;
                double dx = nodes_map_[cand].x - park_x;
                double dy = nodes_map_[cand].y - park_y;
                double d = dx*dx + dy*dy;
                if (d < best_d) {
                    best_d = d;
                    target_node = cand;
                }
            }
            ROS_INFO("✅ Selected goal node %lld with %lu neighbors (dist=%.2fm)",
                     target_node,
                     adjacency_list_[target_node].size(),
                     std::sqrt(best_d));
        }
    }

    // fallback
    if (target_node == -1) {
        double best_d = 1e18;
        for (const auto& kv : adjacency_list_) {
            if (kv.second.empty()) continue;
            if (!nodes_map_.count(kv.first)) continue;

            double dx = nodes_map_[kv.first].x - park_x;
            double dy = nodes_map_[kv.first].y - park_y;
            double d = dx*dx + dy*dy;

            if (d < best_d) {
                best_d = d;
                target_node = kv.first;
            }
        }
        if (target_node != -1) {
            ROS_WARN("Using fallback goal node %lld (dist=%.2fm, neighbors=%lu)",
                     target_node,
                     std::sqrt(best_d),
                     adjacency_list_[target_node].size());
        }
    }

    // ... 继续原有的路径规划代码 ...
}


// ====== 问题诊断5: 完全失败时的应急路径 ======
// 在planTask()的最后，如果所有规划都失败，生成一条简单直线路径:

// 在planTask()末尾添加:
if (global_plan_.empty()) {
    ROS_ERROR("❌ All planning methods failed! Generating emergency straight-line path.");

    // 生成从当前位置到停车位的简单直线路径
    double dx = park_x - car_.x;
    double dy = park_y - car_.y;
    double dist = std::hypot(dx, dy);
    int n_points = std::max(10, (int)(dist / 0.5));  // 每0.5米一个点

    for (int i = 0; i <= n_points; ++i) {
        double t = double(i) / n_points;
        CarState s;
        s.x = car_.x + dx * t;
        s.y = car_.y + dy * t;
        s.theta = std::atan2(dy, dx);
        s.v = 1.5;
        s.phi = 0.0;
        global_plan_.push_back(s);
    }

    ROS_WARN("⚠️ Emergency path generated with %lu points", global_plan_.size());
}
