#ifndef PARKING_DEMO_OSM_MAP_LOADER_H
#define PARKING_DEMO_OSM_MAP_LOADER_H

#include "types.h"
#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

namespace parking_demo {

/**
 * @brief OSM地图加载器
 *
 * 职责：
 * - 解析OSM XML文件
 * - 提取nodes, ways, relations
 * - 坐标转换（经纬度 -> 局部坐标）
 */
class OSMMapLoader {
public:
    OSMMapLoader();
    ~OSMMapLoader();

    /**
     * @brief 从文件加载OSM地图
     * @param filename OSM文件路径
     * @return 成功返回true
     */
    bool loadFromFile(const std::string& filename);

    /**
     * @brief 将经纬度坐标转换为局部坐标
     */
    void projectToLocal();

    /**
     * @brief 提取停车位信息
     */
    void extractParkingSpots();

    // Getters
    const std::map<long long, NodePoint>& getNodes() const { return nodes_; }
    const std::vector<Way>& getWays() const { return ways_; }
    const std::vector<Relation>& getRelations() const { return relations_; }
    const std::vector<ParkingSpot>& getParkingSpots() const { return parking_spots_; }

    double getOriginLat() const { return origin_lat_; }
    double getOriginLon() const { return origin_lon_; }

private:
    /**
     * @brief 从XML行中提取属性值
     */
    std::string extractAttribute(const std::string& line, const std::string& key);

    std::map<long long, NodePoint> nodes_;
    std::vector<Way> ways_;
    std::vector<Relation> relations_;
    std::vector<ParkingSpot> parking_spots_;

    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
};

} // namespace parking_demo

#endif // PARKING_DEMO_OSM_MAP_LOADER_H
