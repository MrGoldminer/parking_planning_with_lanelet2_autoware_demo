#ifndef PARKING_DEMO_GRAPH_BUILDER_H
#define PARKING_DEMO_GRAPH_BUILDER_H

#include "types.h"
#include <vector>
#include <map>
#include <set>

namespace parking_demo {

/**
 * @brief 图构建器
 *
 * 职责：
 * - 从OSM数据构建节点邻接图
 * - 构建relation-level的邻接关系
 * - 提供图查询接口
 */
class GraphBuilder {
public:
    GraphBuilder();
    ~GraphBuilder();

    /**
     * @brief 从OSM数据构建邻接图
     * @param nodes 节点映射
     * @param ways Way列表
     * @param relations Relation列表
     */
    void buildFromOSM(const std::map<long long, NodePoint>& nodes,
                     const std::vector<Way>& ways,
                     const std::vector<Relation>& relations);

    /**
     * @brief 检查图的连通性
     * @return 最大连通分量大小
     */
    int checkConnectivity();

    /**
     * @brief 查找最近的图节点
     * @param x X坐标
     * @param y Y坐标
     * @return 节点ID，如果失败返回-1
     */
    long long findNearestGraphNode(double x, double y) const;

    // Getters
    const std::map<long long, std::vector<long long>>& getAdjacencyList() const {
        return adjacency_list_;
    }

    const std::map<long long, std::vector<long long>>& getRelationAdjacency() const {
        return relation_adjacency_;
    }

private:
    /**
     * @brief 构建节点级邻接图
     */
    void buildNodeAdjacency(const std::vector<Way>& ways);

    /**
     * @brief 构建relation级邻接图
     */
    void buildRelationAdjacency(const std::vector<Way>& ways,
                               const std::vector<Relation>& relations);

    std::map<long long, std::vector<long long>> adjacency_list_;        // 节点邻接
    std::map<long long, std::vector<long long>> relation_adjacency_;    // relation邻接

    const std::map<long long, NodePoint>* nodes_ptr_ = nullptr;  // 保存节点引用
};

} // namespace parking_demo

#endif // PARKING_DEMO_GRAPH_BUILDER_H
