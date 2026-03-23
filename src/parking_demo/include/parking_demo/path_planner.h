#ifndef PARKING_DEMO_PATH_PLANNER_H
#define PARKING_DEMO_PATH_PLANNER_H

#include "types.h"
#include "graph_builder.h"
#include <vector>
#include <map>

namespace parking_demo {

/**
 * @brief 路径规划器
 *
 * 职责：
 * - BFS/Dijkstra节点级路径规划
 * - Relation-level车道规划
 * - Centerline路径生成
 * - 路径平滑与重采样
 */
class PathPlanner {
public:
    PathPlanner();
    ~PathPlanner();

    /**
     * @brief 设置地图数据
     */
    void setMapData(const std::map<long long, NodePoint>& nodes,
                   const std::vector<Way>& ways,
                   const std::vector<Relation>& relations);

    /**
     * @brief 设置图构建器
     */
    void setGraphBuilder(const GraphBuilder* graph_builder);

    /**
     * @brief BFS节点级路径规划
     * @param start_node 起始节点ID
     * @param goal_node 目标节点ID
     * @return 节点ID序列（空表示失败）
     */
    std::vector<long long> planNodePath(long long start_node, long long goal_node);

    /**
     * @brief Relation-level路径规划
     * @param start_rel 起始relation ID
     * @param goal_rel 目标relation ID
     * @return Relation ID序列
     */
    std::vector<long long> planRelationPath(long long start_rel, long long goal_rel);

    /**
     * @brief 从节点ID序列生成车辆轨迹
     * @param node_ids 节点ID列表
     * @param default_speed 默认速度（米/秒）
     * @return 车辆状态序列
     */
    std::vector<CarState> nodesToTrajectory(const std::vector<long long>& node_ids,
                                            double default_speed = 2.0);

    /**
     * @brief 从relation序列生成centerline路径
     * @param rel_ids Relation ID列表
     * @param samples_per_rel 每个relation的采样点数
     * @return (x, y)坐标序列
     */
    std::vector<std::pair<double, double>> relationsToCenterline(
        const std::vector<long long>& rel_ids,
        int samples_per_rel = 40);

    /**
     * @brief 路径重采样（确保路径点密度）
     * @param path 原始路径
     * @param max_spacing 最大点间距（米）
     * @return 重采样后的路径
     */
    std::vector<CarState> resamplePath(const std::vector<CarState>& path,
                                      double max_spacing = 0.5);

    /**
     * @brief 生成应急直线路径（当所有规划失败时）
     * @param start 起点
     * @param goal_x 目标X
     * @param goal_y 目标Y
     * @param goal_theta 目标朝向
     * @return 轨迹
     */
    std::vector<CarState> generateEmergencyPath(const CarState& start,
                                                 double goal_x,
                                                 double goal_y,
                                                 double goal_theta);

private:
    /**
     * @brief 计算relation的中心线
     */
    std::vector<std::pair<double, double>> computeRelationCenterline(long long rel_id, int samples);

    /**
     * @brief 对way进行重采样
     */
    std::vector<std::pair<double, double>> resampleWay(const Way& way, int num_samples);

    const std::map<long long, NodePoint>* nodes_ = nullptr;
    const std::vector<Way>* ways_ = nullptr;
    const std::vector<Relation>* relations_ = nullptr;
    const GraphBuilder* graph_builder_ = nullptr;
};

} // namespace parking_demo

#endif // PARKING_DEMO_PATH_PLANNER_H
