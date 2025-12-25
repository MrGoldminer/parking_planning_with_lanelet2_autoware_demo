#ifndef PARKING_DEMO_VISUALIZER_H
#define PARKING_DEMO_VISUALIZER_H

#include "types.h"
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <map>

namespace parking_demo {

/**
 * @brief ROS可视化管理器
 *
 * 职责：
 * - 发布地图可视化
 * - 发布车辆状态
 * - 发布规划路径
 * - 发布TF变换
 */
class Visualizer {
public:
    Visualizer(ros::NodeHandle& nh);
    ~Visualizer();

    /**
     * @brief 发布OSM地图（ways, relations, nodes）
     */
    void publishMap(const std::map<long long, NodePoint>& nodes,
                   const std::vector<Way>& ways,
                   const std::vector<Relation>& relations);

    /**
     * @brief 发布停车位
     */
    void publishParkingSpots(const std::vector<ParkingSpot>& spots,
                            const std::map<long long, NodePoint>& nodes,
                            const std::vector<Way>& ways);

    /**
     * @brief 发布规划路径
     */
    void publishPath(const std::vector<CarState>& path);

    /**
     * @brief 发布车辆状态
     * @param state 车辆状态
     * @param frame_id 坐标系ID（默认"map"）
     */
    void publishVehicleState(const CarState& state, const std::string& frame_id = "map");

    /**
     * @brief 发布TF变换
     */
    void publishTF(const CarState& state,
                  const std::string& parent_frame = "map",
                  const std::string& child_frame = "base_link");

    /**
     * @brief 发布前瞻点（用于调试）
     */
    void publishLookaheadPoint(double x, double y);

    /**
     * @brief 发布控制指令（速度、转向角）
     */
    void publishControlCommand(double v, double steering_angle);

private:
    ros::NodeHandle& nh_;

    // Publishers
    ros::Publisher ways_pub_;
    ros::Publisher relations_pub_;
    ros::Publisher nodes_pub_;
    ros::Publisher parking_pub_;
    ros::Publisher path_pub_;
    ros::Publisher car_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher cmd_pub_;
    ros::Publisher lookahead_pub_;

    // TF broadcaster
    tf::TransformBroadcaster tf_broadcaster_;
};

} // namespace parking_demo

#endif // PARKING_DEMO_VISUALIZER_H
