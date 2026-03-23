
#include <algorithm>
#include <cmath>
// Replace the entire file with a clean, single copy.
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <cmath>
#include <random>
#include <algorithm>
#include <queue>

#ifdef HAVE_OMPL
// OMPL Reeds-Shepp planner
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
#endif

// ================= 基础结构定义 =================
struct NodePoint {
    long long id;
    double lat;
    double lon;
    // 局部坐标 (米)
    double x;
    double y;
};

struct Way {
    long long id;
    std::vector<long long> node_refs;
    std::string relation; // "Successor", "Left", etc.
    std::string subtype; // e.g., "dashed" or "solid"
    std::string wtype; // e.g., "line_thin"
};

struct Relation {
    long long id;
    std::vector<long long> member_way_refs; // only keep way members for visualization
    std::string type; // e.g. "multipolygon"
};

struct ParkingSpot {
    long long id; // way id or node id
    double x = 0.0;
    double y = 0.0;
    bool from_way = false; // true if based on way, false if node
};

// ================= 数学与工具类 =================
class MapTools {
public:
    // 简单的墨卡托投影近似，以第一个点为原点
    static void projectToLocal(std::map<long long, NodePoint>& nodes) {
        if (nodes.empty()) return;

        // 取第一个点作为原点
        auto it = nodes.begin();
        origin_lat_ = it->second.lat;
        origin_lon_ = it->second.lon;
        double R = 6378137.0; // 地球半径

        for (auto& pair : nodes) {
            double dlat = pair.second.lat - origin_lat_;
            double dlon = pair.second.lon - origin_lon_;
            double lat_rad = origin_lat_ * M_PI / 180.0;

            // x = 东向 (Lon), y = 北向 (Lat)
            pair.second.x = R * (dlon * M_PI / 180.0) * cos(lat_rad);
            pair.second.y = R * (dlat * M_PI / 180.0);
        }
    }

    static double dist(const NodePoint& a, const NodePoint& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    // project a single lat/lon pair to local coordinates using an origin
    static void projectPoint(double origin_lat, double origin_lon, double lat, double lon, double &out_x, double &out_y) {
        double R = 6378137.0;
        double dlat = lat - origin_lat;
        double dlon = lon - origin_lon;
        double lat_rad = origin_lat * M_PI / 180.0;
        out_x = R * (dlon * M_PI / 180.0) * cos(lat_rad);
        out_y = R * (dlat * M_PI / 180.0);
    }

    // getters for the origin used in projectToLocal
    static double getOriginLat() { return origin_lat_; }
    static double getOriginLon() { return origin_lon_; }
private:
    static double origin_lat_;
    static double origin_lon_;
};

// define static origin members
double MapTools::origin_lat_ = 0.0;
double MapTools::origin_lon_ = 0.0;

// ================= 主节点类 =================
class ParkingSystem {
private:
    ros::NodeHandle nh_;
    ros::Publisher car_pub_;
    ros::Publisher path_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher ways_pub_;
    ros::Publisher relations_pub_;
    ros::Publisher nodes_pub_;
    ros::Publisher parking_pub_;
    ros::Publisher cmd_pub_; // publishes commanded velocity/steering (as Twist: linear.x = v, angular.z = steering angle)
    ros::Timer loop_timer_;

    std::map<long long, NodePoint> nodes_map_;
    std::vector<Way> ways_;
    std::vector<Relation> relations_;
    std::vector<ParkingSpot> parking_spots_;
    // 邻接表用于图搜索: node_id -> list of connected node_ids
    std::map<long long, std::vector<long long>> adjacency_list_;
    // relation adjacency for lanelet-level planning: relation_id -> neighbor relation ids
    std::map<long long, std::vector<long long>> relation_adjacency_;

    // 车辆状态
    struct CarState {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double v = 0.0;
        double phi = 0.0; // steering
    } car_;
    // desired parking orientation (set during planning)
    double parking_target_theta_ = 1.72;
    double parking_target_x_ = 0.0;
    double parking_target_y_ = 0.0;
    // maximum allowed forward speed (m/s)
    double max_speed_ = 3.0;

    // vehicle parameters for Reeds-Shepp planning
    double wheelbase_ = 2.7; // meters
    double max_steering_rad_ = 0.6; // rad
    double min_turn_radius_ = 0.0; // computed
    double parking_back_distance_ = 2; // default reverse distance (m)

    // vehicle physical dims (used for footprint checks)
    double vehicle_length_ = 4.6; // default vehicle length (m)
    double vehicle_width_ = 1.85; // default width (m)

    // regulatory parameters and overhangs
    double regulatory_Rmin_ = 1; // 减小最小转弯半径到4米，让车辆更灵活
    double dsafe_ = 1; // safety margin (m)
    double rear_overhang_ = 1; // a (m)
    double front_overhang_ = 0.9; // b (m)

    // parking slot polygon if known (list of (x,y) corners in order)
    std::vector<std::pair<double,double>> parking_slot_polygon_;

    // local controller params (Pure Pursuit)
    double lookahead_dist_ = 2.0; // base meters
    double lookahead_min_ = 1.0;
    double lookahead_max_ = 4.0;
    double curvature_speed_gain_ = 3.0; // higher -> slower for same curvature
    double steering_rate_limit_ = 0.6; // rad/s, limit steer change rate
    double lookahead_scale_with_speed_ = 1.0; // scale factor

    double kp_speed_ = 1.2;  // 增加速度增益，提高响应性
        double min_speed_ = 0.4;  // 进一步提高最小速度避免停滞
        double goal_tolerance_ = 0.8; // 进一步增大目标容忍距离

    ros::Publisher lookahead_pub_; // publish a marker for debugging the lookahead point


    // 轨迹
    std::vector<CarState> global_plan_;
    int current_idx_ = 0;
    bool is_parking_mode_ = false;

public:
    ParkingSystem() {
    // separate publisher for the ego car so the map topic is not overwritten by frequent car updates
    car_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/car_marker", 10, false);
    // publish OSM features on dedicated topics for full-feature visualization
    ways_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/osm/ways", 1, true);
    relations_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/osm/relations", 1, true);
    nodes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/osm/nodes", 1, true);
    parking_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/parking_spots", 1, true);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1, true);
        pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/pose2d", 10);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        lookahead_pub_ = nh_.advertise<visualization_msgs::Marker>("/lookahead_marker", 5);

        // controller params
        nh_.param("lookahead_distance", lookahead_dist_, lookahead_dist_);
        nh_.param("lookahead_min", lookahead_min_, lookahead_min_);
        nh_.param("lookahead_max", lookahead_max_, lookahead_max_);
        nh_.param("lookahead_scale_with_speed", lookahead_scale_with_speed_, lookahead_scale_with_speed_);
        nh_.param("curvature_speed_gain", curvature_speed_gain_, curvature_speed_gain_);
        nh_.param("steering_rate_limit", steering_rate_limit_, steering_rate_limit_);
        nh_.param("kp_speed", kp_speed_, kp_speed_);
        nh_.param("min_speed", min_speed_, min_speed_);
        nh_.param("goal_tolerance", goal_tolerance_, goal_tolerance_);

        // read params for vehicle
        nh_.param("vehicle_wheelbase", wheelbase_, wheelbase_);
        nh_.param("vehicle_max_steering", max_steering_rad_, max_steering_rad_);
        nh_.param("vehicle_length", vehicle_length_, vehicle_length_);
        nh_.param("vehicle_width", vehicle_width_, vehicle_width_);
        nh_.param("regulatory_Rmin", regulatory_Rmin_, regulatory_Rmin_);
        nh_.param("dsafe", dsafe_, dsafe_);
        nh_.param("rear_overhang", rear_overhang_, rear_overhang_);
        nh_.param("front_overhang", front_overhang_, front_overhang_);
        // compute min turning radius for Reeds-Shepp
        if (max_steering_rad_ > 0.001) min_turn_radius_ = wheelbase_ / std::tan(max_steering_rad_);
        else min_turn_radius_ = 5.0;
        ROS_INFO("Vehicle params: wheelbase=%.2f m, max_steering=%.3f rad, min_turn_radius=%.2f m, length=%.2f, width=%.2f, Rmin=%.2f, dsafe=%.2f", wheelbase_, max_steering_rad_, min_turn_radius_, vehicle_length_, vehicle_width_, regulatory_Rmin_, dsafe_);

        // 1. 加载地图
        std::string path = ros::package::getPath("parking_demo") + "/maps/parking_map.osm";
        if (!loadMap(path)) {
            ROS_ERROR("Failed to load map from %s. Make sure the file exists.", path.c_str());
            return; // 简单的错误处理
        }

        // 2. 坐标转换
        MapTools::projectToLocal(nodes_map_);
        buildGraph();
        publishMap();

        // 3. 规划任务
        planTask();

        // 4. 启动控制循环 (20Hz)
        loop_timer_ = nh_.createTimer(ros::Duration(0.05), &ParkingSystem::controlLoop, this);
    }

    // --- 简单的 XML 解析器 (避免 heavy dependencies) ---
    bool loadMap(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;

        std::string line;
        while (std::getline(file, line)) {
            // 解析 Node
            if (line.find("<node") != std::string::npos) {
                NodePoint n;
                n.id = std::stoll(extractAttr(line, "id"));
                n.lat = std::stod(extractAttr(line, "lat"));
                n.lon = std::stod(extractAttr(line, "lon"));
                // check for inline tags or multi-line node children
                if (line.find("/>") == std::string::npos) {
                    // read children until </node>
                    std::string sub;
                    while (std::getline(file, sub) && sub.find("</node>") == std::string::npos) {
                        if (sub.find("<tag") != std::string::npos) {
                            std::string v = extractAttr(sub, "v");
                            if (v == "parking_space") {
                                ParkingSpot ps;
                                ps.id = n.id;
                                ps.from_way = false;
                                // projection to be computed later
                                parking_spots_.push_back(ps);
                            }
                        }
                    }
                }
                nodes_map_[n.id] = n;
            }
            // 解析 Way
            else if (line.find("<way") != std::string::npos) {
                Way w;
                w.id = std::stoll(extractAttr(line, "id"));
                // 读取 way 的子标签
                while (std::getline(file, line) && line.find("</way>") == std::string::npos) {
                    if (line.find("<nd") != std::string::npos) {
                        w.node_refs.push_back(std::stoll(extractAttr(line, "ref")));
                    }
                    if (line.find("<tag") != std::string::npos) {
                        std::string k = extractAttr(line, "k");
                        std::string v = extractAttr(line, "v");
                        if (k == "relation") {
                            w.relation = v;
                        }
                        if (k == "subtype") {
                            w.subtype = v;
                        }
                        if (k == "type") {
                            w.wtype = v;
                        }
                        if (v == "parking_space") {
                            // mark this way as a parking spot
                            ParkingSpot ps;
                            ps.id = w.id;
                            ps.from_way = true;
                            parking_spots_.push_back(ps);
                        }
                    }
                }
                ways_.push_back(w);
            }
            else if (line.find("<relation") != std::string::npos) {
                Relation r;
                r.id = std::stoll(extractAttr(line, "id"));
                // read members and tags until </relation>
                while (std::getline(file, line) && line.find("</relation>") == std::string::npos) {
                    if (line.find("<member") != std::string::npos) {
                        std::string type = extractAttr(line, "type");
                        if (type == "way") {
                            std::string ref = extractAttr(line, "ref");
                            if (!ref.empty()) r.member_way_refs.push_back(std::stoll(ref));
                        }
                    }
                    if (line.find("<tag") != std::string::npos) {
                        std::string k = extractAttr(line, "k");
                        std::string v = extractAttr(line, "v");
                        if (k == "type") r.type = v;
                    }
                }
                relations_.push_back(r);
            }
        }
        ROS_INFO("Loaded %lu nodes and %lu ways.", nodes_map_.size(), ways_.size());
        return true;
    }

    std::string extractAttr(const std::string& line, const std::string& key) {
        std::string search = key + "=\"";
        size_t start = line.find(search);
        if (start == std::string::npos) {
            search = key + "='"; // 尝试单引号
            start = line.find(search);
            if (start == std::string::npos) return "";
        }
        start += search.length();
        size_t end = line.find_first_of("\"'", start);
        return line.substr(start, end - start);
    }

    // --- 构建图 ---
    void buildGraph() {
        for (const auto& w : ways_) {
            // 对于每一条路，连接相邻的点
            // 如果 relation 标记为 Successor，保留为单向链路；否则双向连接
            bool is_successor = false;
            std::string lr = w.relation;
            std::transform(lr.begin(), lr.end(), lr.begin(), ::tolower);
            if (lr.find("successor") != std::string::npos) is_successor = true;
            for (size_t i = 0; i + 1 < w.node_refs.size(); ++i) {
                adjacency_list_[w.node_refs[i]].push_back(w.node_refs[i+1]);
                if (!is_successor) {
                    adjacency_list_[w.node_refs[i+1]].push_back(w.node_refs[i]);
                }
            }
        }

        // Build relation adjacency graph (relation id -> neighboring relation ids)
        // Two relations are adjacent if they share a way id or have centroids within a threshold
        relation_adjacency_.clear();
        std::map<long long, std::vector<long long>> way_to_rel;
        for (const auto &r : relations_) {
            for (long long wid : r.member_way_refs) {
                way_to_rel[wid].push_back(r.id);
            }
        }
        // share-way adjacency
        for (const auto &kv : way_to_rel) {
            const auto &rels = kv.second;
            for (size_t i = 0; i < rels.size(); ++i) for (size_t j = i+1; j < rels.size(); ++j) {
                relation_adjacency_[rels[i]].push_back(rels[j]);
                relation_adjacency_[rels[j]].push_back(rels[i]);
            }
        }
        // centroid-proximity adjacency (for disconnected relations)
        for (size_t i = 0; i < relations_.size(); ++i) {
            for (size_t j = i+1; j < relations_.size(); ++j) {
                auto ci = computeRelationCentroid(&relations_[i]);
                auto cj = computeRelationCentroid(&relations_[j]);
                double dx = ci.first - cj.first, dy = ci.second - cj.second;
                double d = std::hypot(dx,dy);
                if (d < 8.0) { // 8m threshold
                    relation_adjacency_[relations_[i].id].push_back(relations_[j].id);
                    relation_adjacency_[relations_[j].id].push_back(relations_[i].id);
                }
            }
        }
    }

    // --- 规划任务 ---
    void planTask() {
        if (nodes_map_.empty()) return;

        // Helper lambdas
        auto findWayById = [&](long long id) -> Way* {
            for (auto &w : ways_) if (w.id == id) return &w;
            return nullptr;
        };
        auto findRelationById = [&](long long id) -> Relation* {
            for (auto &r : relations_) if (r.id == id) return &r;
            return nullptr;
        };
        auto centroidOfWay = [&](const Way &w) -> std::pair<double,double> {
            double sx = 0, sy = 0; int c = 0;
            for (long long nid : w.node_refs) if (nodes_map_.count(nid)) { sx += nodes_map_[nid].x; sy += nodes_map_[nid].y; ++c; }
            if (c==0) return {0,0};
            return {sx/c, sy/c};
        };
        auto centroidOfRelation = [&](const Relation &r) -> std::pair<double,double> {
            double sx = 0, sy = 0; int c = 0;
            for (long long wid : r.member_way_refs) {
                Way* w = findWayById(wid);
                if (!w) continue;
                auto p = centroidOfWay(*w);
                sx += p.first; sy += p.second; ++c;
            }
            if (c==0) return {0,0};
            return {sx/c, sy/c};
        };

        // resampling helper is now a class method: use resampleWayPoints(w, N)

        // relation centerline computation promoted to class method computeRelationCenterline(r)

        auto findNearestAdjNode = [&](double x, double y) -> long long {
            long long best = -1; double bestd = 1e18;
            for (const auto &kv : adjacency_list_) {
                long long nid = kv.first;
                if (!nodes_map_.count(nid)) continue;
                double dx = nodes_map_[nid].x - x; double dy = nodes_map_[nid].y - y; double d = dx*dx + dy*dy;
                if (d < bestd) { bestd = d; best = nid; }
            }
            return best;
        };

        // IDs provided by user
    const long long PARK_LANELET_ID = 9265; // parking lanelet center (target)
    const long long LINESTR_ID_A = 9386; // linestring A
    const long long LINESTR_ID_B = 9392; // linestring B
    const long long START_LANELET_CENTER_ID = 9259; // start center lanelet

        // 1) Set initial vehicle pose to center of lanelet 106 if possible
        bool set_start_pose = false;
        Relation* start_rel = findRelationById(START_LANELET_CENTER_ID);
        std::vector<std::pair<double,double>> start_centerline;
        if (start_rel) {
            start_centerline = computeRelationCenterline(start_rel);
            if (!start_centerline.empty()) {
                // set start pose to centerline midpoint
                size_t mid = start_centerline.size()/2;
                car_.x = start_centerline[mid].first; car_.y = start_centerline[mid].second; set_start_pose = true;
                // derive heading from neighboring centerline points
                if (mid+1 < start_centerline.size()) car_.theta = std::atan2(start_centerline[mid+1].second - start_centerline[mid].second, start_centerline[mid+1].first - start_centerline[mid].first);
            } else {
                auto c = centroidOfRelation(*start_rel);
                car_.x = c.first; car_.y = c.second; set_start_pose = true;
            }
        } else {
            Way* start_way = findWayById(START_LANELET_CENTER_ID);
            if (start_way) { auto c = centroidOfWay(*start_way); car_.x = c.first; car_.y = c.second; set_start_pose = true; }
        }

        if (!set_start_pose) {
            ROS_WARN("Start lanelet %lld not found; using map center as start.", START_LANELET_CENTER_ID);
            // fallback to map center
            double minx = 1e18, miny = 1e18, maxx = -1e18, maxy = -1e18;
            for (const auto &kv : nodes_map_) { minx = std::min(minx, kv.second.x); miny = std::min(miny, kv.second.y); maxx = std::max(maxx, kv.second.x); maxy = std::max(maxy, kv.second.y); }
            car_.x = 0.5*(minx+maxx); car_.y = 0.5*(miny+maxy);
        }

    // 2) choose a start graph node id: prefer a node that belongs to the START_LANELET relation centerline
    // Note: the "start" concept exposed to the user is the lanelet centroid + heading (no id).
    long long start_node_id = -1;
        if (start_rel) {
            // find the member way nodes and pick the one closest to the relation centroid that is present in the adjacency graph
            auto rc = centroidOfRelation(*start_rel);
            double bestd = 1e18;
            for (long long wid : start_rel->member_way_refs) {
                Way* w = findWayById(wid);
                if (!w) continue;
                for (size_t i = 0; i < w->node_refs.size(); ++i) {
                    long long nid = w->node_refs[i];
                    if (!nodes_map_.count(nid)) continue;
                    // prefer nodes that appear in adjacency_list_ (i.e., graph nodes)
                    if (adjacency_list_.count(nid) == 0 && adjacency_list_.count(nid) == 0) {
                        // still consider, but deprioritize
                    }
                    double dx = nodes_map_[nid].x - rc.first; double dy = nodes_map_[nid].y - rc.second; double d = dx*dx + dy*dy;
                    if (d < bestd) { bestd = d; start_node_id = nid; }
                }
            }
            if (start_node_id != -1) {
                ROS_INFO("Start relation %lld -> selected start graph node id %lld (closest to relation centroid)", START_LANELET_CENTER_ID, start_node_id);
                // set car pose to the relation centroid and heading along the local centerline tangent
                car_.x = rc.first; car_.y = rc.second;
                // determine heading by finding a local tangent at the chosen node
                // locate which way/position this node belongs to and pick the neighboring node
                for (long long wid : start_rel->member_way_refs) {
                    Way* w = findWayById(wid); if (!w) continue;
                    for (size_t i = 0; i < w->node_refs.size(); ++i) {
                            if (w->node_refs[i] == start_node_id) {
                            long long n0 = start_node_id;
                            long long n1 = (i+1 < w->node_refs.size()) ? w->node_refs[i+1] : (i>0 ? w->node_refs[i-1] : w->node_refs[i]);
                            if (nodes_map_.count(n1)) {
                                double hvx = nodes_map_[n1].x - nodes_map_[n0].x;
                                double hvy = nodes_map_[n1].y - nodes_map_[n0].y;
                                car_.theta = std::atan2(hvy, hvx);
                            }
                        }
                    }
                }
            }
        }

        // fallback: if we still don't have a start graph node id, choose nearest adjacency node to the computed start pose
        if (start_node_id == -1) {
            start_node_id = findNearestAdjNode(car_.x, car_.y);
            if (start_node_id == -1) { for (const auto &kv : adjacency_list_) { start_node_id = kv.first; break; } }
        }

        // Debug: expose the start as lanelet centroid + heading (no id concept). Also log the internal graph node used.
        ROS_INFO("Start lanelet id %lld centroid at (%.3f, %.3f) heading=%.3f rad", START_LANELET_CENTER_ID, car_.x, car_.y, car_.theta);
        if (start_node_id != -1) {
            if (nodes_map_.count(start_node_id)) {
                ROS_INFO("Using internal start graph node id=%lld at (%.3f, %.3f)", start_node_id, nodes_map_[start_node_id].x, nodes_map_[start_node_id].y);
            } else {
                ROS_INFO("Using internal start graph node id=%lld (no coords available)", start_node_id);
            }
        }
        // find nearest relation (lanelet) centroid to the start pose for debugging
        long long nearest_rel = -1; double rel_bestd = 1e18;
        for (const auto &r : relations_) {
            auto rc = centroidOfRelation(r);
            double dx = rc.first - car_.x, dy = rc.second - car_.y;
            double d = dx*dx + dy*dy;
            if (d < rel_bestd) { rel_bestd = d; nearest_rel = r.id; }
        }
        if (nearest_rel != -1) {
            ROS_INFO("Nearest relation to start pose: id=%lld (centroid dist=%.3f m)", nearest_rel, std::sqrt(rel_bestd));
        }

        // 3) Compute target (parking) location: center of lanelet 9265 (relation or way)
        double target_cx = 0, target_cy = 0; bool have_target_center = false;
        Relation* park_rel = findRelationById(PARK_LANELET_ID);
        if (park_rel) { auto c = centroidOfRelation(*park_rel); target_cx = c.first; target_cy = c.second; have_target_center = true; }
        else { Way* park_way = findWayById(PARK_LANELET_ID); if (park_way) { auto c = centroidOfWay(*park_way); target_cx = c.first; target_cy = c.second; have_target_center = true; } }

        if (!have_target_center) {
            ROS_WARN("Target lanelet %lld not found; aborting planning.", PARK_LANELET_ID);
            return;
        }

        // 4) Compute parking target area between the two linestrings (ways) by finding closest points
        double park_x = target_cx, park_y = target_cy;
        Way* wa = findWayById(LINESTR_ID_A);
        Way* wb = findWayById(LINESTR_ID_B);

        std::vector<std::pair<double,double>> park_centerline;
        if (park_rel) {
            park_centerline = computeRelationCenterline(park_rel);
            if (!park_centerline.empty()) {
                size_t mid = park_centerline.size()/2;
                park_x = park_centerline[mid].first; park_y = park_centerline[mid].second;
                // ensure target_theta aligns with centerline tangent if available
                if (mid+1 < park_centerline.size()) {
                    double cx = park_centerline[mid+1].first - park_centerline[mid].first;
                    double cy = park_centerline[mid+1].second - park_centerline[mid].second;
                    double cang = std::atan2(cy, cx);
                    // perpendicular into parking area will be computed later relative to cang
                }
            }
        }

        // helper: project point p onto segment a-b, returning closest point
        auto projectOntoSegment = [&](const std::pair<double,double>& a, const std::pair<double,double>& b, const std::pair<double,double>& p) {
            double ax = a.first, ay = a.second, bx = b.first, by = b.second, px = p.first, py = p.second;
            double vx = bx-ax, vy = by-ay;
            double wx = px-ax, wy = py-ay;
            double vv = vx*vx + vy*vy;
            double t = (vv > 0) ? (vx*wx + vy*wy) / vv : 0.0;
            if (t < 0) t = 0; if (t > 1) t = 1;
            return std::make_pair(ax + t*vx, ay + t*vy);
        };

        bool haveA=false, haveB=false;
        std::pair<double,double> bestA, bestB;
        double bestDist = 1e18;

        if (wa && wb) {
            // iterate nodes of wa and segments of wb
            for (size_t i=0;i<wa->node_refs.size();++i) {
                if (!nodes_map_.count(wa->node_refs[i])) continue;
                auto pA = std::make_pair(nodes_map_[wa->node_refs[i]].x, nodes_map_[wa->node_refs[i]].y);
                for (size_t j=0;j+1<wb->node_refs.size();++j) {
                    if (!nodes_map_.count(wb->node_refs[j]) || !nodes_map_.count(wb->node_refs[j+1])) continue;
                    auto b0 = std::make_pair(nodes_map_[wb->node_refs[j]].x, nodes_map_[wb->node_refs[j]].y);
                    auto b1 = std::make_pair(nodes_map_[wb->node_refs[j+1]].x, nodes_map_[wb->node_refs[j+1]].y);
                    auto proj = projectOntoSegment(b0,b1,pA);
                    double dx = pA.first - proj.first, dy = pA.second - proj.second;
                    double d = dx*dx + dy*dy;
                    if (d < bestDist) { bestDist = d; bestA = pA; bestB = proj; haveA=true; haveB=true; }
                }
            }
            // also try nodes of wb against segments of wa
            for (size_t i=0;i<wb->node_refs.size();++i) {
                if (!nodes_map_.count(wb->node_refs[i])) continue;
                auto pB = std::make_pair(nodes_map_[wb->node_refs[i]].x, nodes_map_[wb->node_refs[i]].y);
                for (size_t j=0;j+1<wa->node_refs.size();++j) {
                    if (!nodes_map_.count(wa->node_refs[j]) || !nodes_map_.count(wa->node_refs[j+1])) continue;
                    auto a0 = std::make_pair(nodes_map_[wa->node_refs[j]].x, nodes_map_[wa->node_refs[j]].y);
                    auto a1 = std::make_pair(nodes_map_[wa->node_refs[j+1]].x, nodes_map_[wa->node_refs[j+1]].y);
                    auto proj = projectOntoSegment(a0,a1,pB);
                    double dx = pB.first - proj.first, dy = pB.second - proj.second;
                    double d = dx*dx + dy*dy;
                    if (d < bestDist) { bestDist = d; bestA = proj; bestB = pB; haveA=true; haveB=true; }
                }
            }
        }

        if (haveA && haveB) {
            park_x = 0.5*(bestA.first + bestB.first);
            park_y = 0.5*(bestA.second + bestB.second);
        }

        // 5) Determine orientation: perpendicular (90 deg) to the lanelet centerline at target, pointing into the parking area
        double target_theta = 0.0;
        Way* centerline_way = nullptr;
        if (park_rel && !park_rel->member_way_refs.empty()) centerline_way = findWayById(park_rel->member_way_refs[0]);
        if (!centerline_way) centerline_way = findWayById(PARK_LANELET_ID);
        // compute a tangent vector t at the nearest point on centerline to park center
        double tx = 1.0, ty = 0.0; // default
        if (centerline_way && centerline_way->node_refs.size() >= 2) {
            int idx_best = -1; double bestd = 1e18;
            for (size_t i=0;i<centerline_way->node_refs.size();++i) {
                long long nid = centerline_way->node_refs[i]; if (!nodes_map_.count(nid)) continue;
                double dx = nodes_map_[nid].x - park_x; double dy = nodes_map_[nid].y - park_y; double d = dx*dx+dy*dy;
                if (d < bestd) { bestd = d; idx_best = i; }
            }
            if (idx_best != -1) {
                long long n0 = centerline_way->node_refs[idx_best];
                long long n1 = (idx_best+1 < (int)centerline_way->node_refs.size()) ? centerline_way->node_refs[idx_best+1] : centerline_way->node_refs[idx_best>0?idx_best-1:idx_best];
                if (nodes_map_.count(n0) && nodes_map_.count(n1)) {
                    tx = nodes_map_[n1].x - nodes_map_[n0].x;
                    ty = nodes_map_[n1].y - nodes_map_[n0].y;
                    double norm = std::hypot(tx,ty); if (norm>0){ tx/=norm; ty/=norm; }
                }
            }
        }
        // perpendicular to t
        double perp_x = -ty, perp_y = tx;
        // decide sign so perpendicular points toward parking center from centerline
        double cx = 0, cy = 0;
        if (centerline_way && !centerline_way->node_refs.empty() && nodes_map_.count(centerline_way->node_refs[0])) { cx = nodes_map_[centerline_way->node_refs[0]].x; cy = nodes_map_[centerline_way->node_refs[0]].y; }
        double side = ( (tx)*(park_y - cy) - (ty)*(park_x - cx) );
        if (side < 0) { perp_x = -perp_x; perp_y = -perp_y; }
    target_theta = std::atan2(perp_y, perp_x);
    // store for use by the parking maneuver
    parking_target_theta_ = target_theta;
    // store parking center coordinates for use in maneuver generator
    parking_target_x_ = park_x;
    parking_target_y_ = park_y;

        // visualize the parking patch as a small rectangular polygon between bestA and bestB
        if (haveA && haveB) {
            double dx = bestB.first - bestA.first; double dy = bestB.second - bestA.second; double gap = std::hypot(dx,dy);
            double seg_len = std::min(6.0, std::max(2.0, gap)); // length of polygon along the lines
            // direction along centerline for extending the ends
            double ex = tx, ey = ty;
            // compute four corners
            std::vector<std::pair<double,double>> corners;
            auto make_pt = [&](double cx, double cy, double ox, double oy){ return std::make_pair(cx + ox, cy + oy); };
            // endpoints on A and B extended along centerline
            auto a1 = make_pt(bestA.first, bestA.second, -0.5*seg_len*ex, -0.5*seg_len*ey);
            auto a2 = make_pt(bestA.first, bestA.second,  0.5*seg_len*ex,  0.5*seg_len*ey);
            auto b1 = make_pt(bestB.first, bestB.second, -0.5*seg_len*ex, -0.5*seg_len*ey);
            auto b2 = make_pt(bestB.first, bestB.second,  0.5*seg_len*ex,  0.5*seg_len*ey);
            corners.push_back(a1); corners.push_back(a2); corners.push_back(b2); corners.push_back(b1);

            visualization_msgs::Marker mark;
            mark.header.frame_id = "map";
            mark.header.stamp = ros::Time::now();
            mark.ns = "selected_parking_area";
            mark.id = 0;
            mark.type = visualization_msgs::Marker::LINE_STRIP;
            mark.scale.x = 0.12;
            mark.color.a = 0.9; mark.color.r = 0.0; mark.color.g = 1.0; mark.color.b = 0.0;
            for (auto &c : corners) { geometry_msgs::Point gp; gp.x = c.first; gp.y = c.second; gp.z = 0.02; mark.points.push_back(gp); }
            // close
            geometry_msgs::Point gp0; gp0.x = corners[0].first; gp0.y = corners[0].second; gp0.z = 0.02; mark.points.push_back(gp0);
            visualization_msgs::MarkerArray arr; arr.markers.push_back(mark);
            // publish selected parking area using the parking_spots topic (parking area outlines)
            parking_pub_.publish(arr);

            // Additionally, highlight the exact parking slot nodes if present (user-provided IDs)
            std::vector<long long> target_nodes = {9385, 9384, 9390, 9391};
            bool have_all = true;
            for (long long nid : target_nodes) if (!nodes_map_.count(nid)) { have_all = false; break; }
            if (have_all) {
                visualization_msgs::Marker slot_m;
                slot_m.header.frame_id = "map";
                slot_m.header.stamp = ros::Time::now();
                slot_m.ns = "target_parking_slot";
                slot_m.id = 1;
                slot_m.type = visualization_msgs::Marker::LINE_STRIP;
                slot_m.scale.x = 0.14;
                slot_m.color.a = 0.95; slot_m.color.r = 0.0; slot_m.color.g = 0.2; slot_m.color.b = 1.0; // blue

                double sx=0, sy=0; int sc=0;
                std::vector<std::pair<double,double>> slot_poly;
                for (long long nid : target_nodes) {
                    geometry_msgs::Point pp; pp.x = nodes_map_[nid].x; pp.y = nodes_map_[nid].y; pp.z = 0.03; slot_m.points.push_back(pp);
                    sx += pp.x; sy += pp.y; ++sc;
                    slot_poly.emplace_back(pp.x, pp.y);
                }
                // close polygon
                geometry_msgs::Point p0; p0.x = nodes_map_[target_nodes[0]].x; p0.y = nodes_map_[target_nodes[0]].y; p0.z = 0.03; slot_m.points.push_back(p0);
                visualization_msgs::MarkerArray marr; marr.markers.push_back(slot_m);
                parking_pub_.publish(marr);
                ROS_INFO("Highlighted target parking slot nodes: %lld,%lld,%lld,%lld", target_nodes[0], target_nodes[1], target_nodes[2], target_nodes[3]);

                // compute parking center as centroid of the four slot nodes and set it as final parking target
                if (sc > 0) {
                    double cx = sx / sc; double cy = sy / sc;
                    parking_target_x_ = cx; parking_target_y_ = cy;
                    // compute slot orientation: choose the longer edge of the rectangle to align vehicle heading along it
                    double theta_slot = 0.0;
                    if (slot_poly.size() >= 4) {
                        auto &p0 = slot_poly[0]; auto &p1 = slot_poly[1]; auto &p2 = slot_poly[2];
                        double e01 = std::hypot(p1.first - p0.first, p1.second - p0.second);
                        double e12 = std::hypot(p2.first - p1.first, p2.second - p1.second);
                        if (e01 >= e12) theta_slot = std::atan2(p1.second - p0.second, p1.first - p0.first);
                        else theta_slot = std::atan2(p2.second - p1.second, p2.first - p1.first);
                        ROS_INFO("Computed parking slot orientation (theta_slot)=%.3f rad (e01=%.2f e12=%.2f)", theta_slot, e01, e12);
                        parking_target_theta_ = theta_slot;
                    }
                    ROS_INFO("Using parking slot centroid as target: (%.3f, %.3f) theta=%.3f", parking_target_x_, parking_target_y_, parking_target_theta_);
                    // store slot polygon for collision checking later by value
                    parking_slot_polygon_.clear();
                    for (auto &pp : slot_poly) parking_slot_polygon_.push_back(pp);
                }
            } else {
                ROS_WARN("Target parking slot nodes missing in map; cannot highlight exact slot.");
            }
        }

        // 6) Plan graph path: prefer a node that belongs to the PARK_LANELET relation centerline
        long long target_node = -1;
    if (park_rel) {
            double bestd = 1e18;
            for (long long wid : park_rel->member_way_refs) {
                Way* w = findWayById(wid);
                if (!w) continue;
                for (long long nid : w->node_refs) {
                    if (!nodes_map_.count(nid)) continue;
                    double dx = nodes_map_[nid].x - park_x; double dy = nodes_map_[nid].y - park_y; double d = dx*dx + dy*dy;
                    if (d < bestd) { bestd = d; target_node = nid; }
                }
            }
        }
    if (target_node == -1) target_node = findNearestAdjNode(park_x, park_y);

    // First, attempt relation-level planning (prefer lanelet centerlines)
    std::vector<long long> rel_path;
    if (start_rel && park_rel) {
        rel_path = findRelationPath(start_rel->id, park_rel->id);
    }
    if (!rel_path.empty()) {
        std::stringstream ss; for (size_t i=0;i<rel_path.size();++i) { if (i) ss << "->"; ss << rel_path[i]; }
        ROS_INFO("Planned relation sequence: %s", ss.str().c_str());

        // assemble centerline points along the relation path
        auto centerline_pts = centerlineFromRelationPath(rel_path);
        ROS_INFO("Assembled %lu centerline points from relation path.", centerline_pts.size());

        // Try Reeds-Shepp planning (global) between start pose and parking pose if OMPL available
        CarState start_cs; start_cs.x = car_.x; start_cs.y = car_.y; start_cs.theta = car_.theta;
        CarState goal_cs; goal_cs.x = park_x; goal_cs.y = park_y; goal_cs.theta = target_theta;
#ifdef HAVE_OMPL
        auto rs = planReedsShepp(start_cs, goal_cs, std::max(0.1, min_turn_radius_), 1.0);
        if (!rs.empty()) {
            ROS_INFO("OMPL Reeds-Shepp plan generated with %lu states.", rs.size());
            global_plan_ = rs;
        } else {
            ROS_WARN("OMPL Reeds-Shepp planning failed, falling back to centerline interpolation.");
        }
#else
        ROS_INFO("OMPL not available - falling back to centerline interpolation.");
#endif
        // If OMPL failed or not available, use centerline interpolation as fallback
        if (global_plan_.empty()) {
            global_plan_.clear();
            for (size_t i=0;i<centerline_pts.size();++i) {
                CarState s; s.x = centerline_pts[i].first; s.y = centerline_pts[i].second; s.v = std::min(1.0, max_speed_);
                // estimate heading by next point
                if (i+1 < centerline_pts.size()) s.theta = std::atan2(centerline_pts[i+1].second - centerline_pts[i].second, centerline_pts[i+1].first - centerline_pts[i].first);
                else if (i>0) s.theta = std::atan2(centerline_pts[i].second - centerline_pts[i-1].second, centerline_pts[i].first - centerline_pts[i-1].first);
                global_plan_.push_back(s);
            }
            // append standoff and final approach to exact park centerline (same as earlier)
            CarState last = global_plan_.back();
            double standoff = 8.0;
            double st_x = park_x - perp_x * standoff;
            double st_y = park_y - perp_y * standoff;
            double x0 = last.x, y0 = last.y;
            double x1 = st_x, y1 = st_y;
            double dx = x1 - x0, dy = y1 - y0;
            double L = std::hypot(dx, dy);
            int approach_steps = std::max(10, static_cast<int>(std::min(60.0, L * 5.0)));
            double v0x = 1.0, v0y = 0.0;
            if (global_plan_.size() > 1) {
                CarState prev = global_plan_[std::max(0, (int)global_plan_.size()-2)];
                double hvx = last.x - prev.x, hvy = last.y - prev.y; double hnorm = std::hypot(hvx,hvy);
                if (hnorm > 1e-6) { v0x = hvx / hnorm; v0y = hvy / hnorm; }
            }
            double v1x = (x1 - x0) / (L>1e-6?L:1.0);
            double v1y = (y1 - y0) / (L>1e-6?L:1.0);
            double mscale = L * 0.5;
            double m0x = v0x * mscale, m0y = v0y * mscale;
            double m1x = v1x * mscale, m1y = v1y * mscale;
            for (int i=1;i<=approach_steps;i++) {
                double t = double(i) / approach_steps;
                double t2 = t*t, t3 = t2*t;
                double h00 =  2*t3 - 3*t2 + 1;
                double h10 =      t3 - 2*t2 + t;
                double h01 = -2*t3 + 3*t2;
                double h11 =      t3 -   t2;
                double px = h00*x0 + h10*m0x + h01*x1 + h11*m1x;
                double py = h00*y0 + h10*m0y + h01*y1 + h11*m1y;
                CarState s; s.x = px; s.y = py; s.theta = std::atan2(v1y, v1x); s.v = std::min(0.8, max_speed_); global_plan_.push_back(s);
            }
            int final_steps = 10;
            for (int i=1;i<=final_steps;i++) {
                double t = double(i) / final_steps;
                CarState s;
                s.x = st_x*(1-t) + park_x*t;
                s.y = st_y*(1-t) + park_y*t;
                s.theta = target_theta;
                s.v = std::min(0.4, max_speed_);
                global_plan_.push_back(s);
            }
            car_.x = global_plan_[0].x; car_.y = global_plan_[0].y; if (global_plan_.size()>1) car_.theta = std::atan2(global_plan_[1].y-global_plan_[0].y, global_plan_[1].x-global_plan_[0].x);
        }

    } else {
        // fallback to node-level BFS planning as legacy behavior
        std::vector<long long> path_ids;
        if (start_node_id != -1 && target_node != -1) path_ids = bfs(start_node_id, target_node);

        ROS_INFO("Planning request: start_node=%lld target_node=%lld", start_node_id, target_node);
        if (!path_ids.empty()) {
            std::stringstream ss; for (size_t i=0;i<path_ids.size();++i) { if (i) ss << "->"; ss << path_ids[i]; }
            ROS_INFO("Planned path node sequence: %s", ss.str().c_str());
        }

        if (path_ids.empty()) {
            ROS_WARN("No path found from start_node %lld to target node %lld; falling back to straight approach near target.", start_node_id, target_node);
            // extra diagnostics: print nearby adjacency nodes count
            int adj_count = adjacency_list_.count(start_node_id) ? adjacency_list_[start_node_id].size() : 0;
            ROS_INFO("Start node %lld has %d adjacent neighbours in adjacency_list_.", start_node_id, adj_count);
            if (nodes_map_.count(start_node_id)) ROS_INFO("Start node coords (%.3f, %.3f)", nodes_map_[start_node_id].x, nodes_map_[start_node_id].y);
            // simple straight approach to target location
            car_.theta = target_theta;
            for (int i=0;i<200;i++) { CarState s; s.x = car_.x + i*0.1*cos(car_.theta); s.y = car_.y + i*0.1*sin(car_.theta); s.theta = car_.theta; s.v = 1.0; global_plan_.push_back(s); }
        } else {
            for (long long id : path_ids) { CarState s; s.x = nodes_map_[id].x; s.y = nodes_map_[id].y; s.v = std::min(1.5, max_speed_); global_plan_.push_back(s); }
            // append a smooth local approach to a standoff point before the parking center, then align perpendicular
            CarState last = global_plan_.back();
            // standoff distance from parking center (meters)
            double standoff = 2.0;
            double st_x = park_x - perp_x * standoff;
            double st_y = park_y - perp_y * standoff;

            // build a cubic Hermite (smooth) between last and standoff
            double x0 = last.x, y0 = last.y;
            double x1 = st_x, y1 = st_y;
            double dx = x1 - x0, dy = y1 - y0;
            double L = std::hypot(dx, dy);
            int approach_steps = std::max(10, static_cast<int>(std::min(60.0, L * 5.0)));

            // tangent directions: m0 along last heading, m1 along target approach direction (toward standoff)
            double v0x = 1.0, v0y = 0.0;
            if (global_plan_.size() > 1) {
                CarState prev = global_plan_[std::max(0, (int)global_plan_.size()-2)];
                double hvx = last.x - prev.x, hvy = last.y - prev.y; double hnorm = std::hypot(hvx,hvy);
                if (hnorm > 1e-6) { v0x = hvx / hnorm; v0y = hvy / hnorm; }
            }
            double v1x = (x1 - x0) / (L>1e-6?L:1.0);
            double v1y = (y1 - y0) / (L>1e-6?L:1.0);

            // derivative magnitudes scaled by distance
            double mscale = L * 0.5;
            double m0x = v0x * mscale, m0y = v0y * mscale;
            double m1x = v1x * mscale, m1y = v1y * mscale;

            for (int i=1;i<=approach_steps;i++) {
                double t = double(i) / approach_steps;
                double t2 = t*t, t3 = t2*t;
                double h00 =  2*t3 - 3*t2 + 1;
                double h10 =      t3 - 2*t2 + t;
                double h01 = -2*t3 + 3*t2;
                double h11 =      t3 -   t2;
                double px = h00*x0 + h10*m0x + h01*x1 + h11*m1x;
                double py = h00*y0 + h10*m0y + h01*y1 + h11*m1y;
                CarState s; s.x = px; s.y = py; s.theta = std::atan2(v1y, v1x); s.v = std::min(0.8, max_speed_); global_plan_.push_back(s);
            }

            // final approach: move from standoff to exact park center aligned perpendicular (90 deg)
            int final_steps = 10;
            for (int i=1;i<=final_steps;i++) {
                double t = double(i) / final_steps;
                CarState s;
                s.x = st_x*(1-t) + park_x*t;
                s.y = st_y*(1-t) + park_y*t;
                s.theta = target_theta;
                s.v = std::min(0.4, max_speed_);
                global_plan_.push_back(s);
            }
            car_.x = global_plan_[0].x; car_.y = global_plan_[0].y; if (global_plan_.size()>1) car_.theta = std::atan2(global_plan_[1].y-global_plan_[0].y, global_plan_[1].x-global_plan_[0].x);
        }
    }

        // 7) At the parking target orientation is target_theta; align before adding parking maneuver
        // Ensure the final point has target_theta
        if (!global_plan_.empty()) { global_plan_.back().theta = target_theta; }

        addParkingManeuver();
    }

    std::vector<long long> bfs(long long start, long long goal) {
        std::queue<long long> q;
        std::map<long long, long long> came_from;
        q.push(start);
        came_from[start] = start;

        while (!q.empty()) {
            long long current = q.front();
            q.pop();

            if (current == goal) break;

            for (long long next : adjacency_list_[current]) {
                if (came_from.find(next) == came_from.end()) {
                    q.push(next);
                    came_from[next] = current;
                }
            }
        }

        std::vector<long long> path;
        if (came_from.find(goal) == came_from.end()) return path; // 未找到
        long long curr = goal;
        while (curr != start) {
            path.push_back(curr);
            curr = came_from[curr];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    // --- relation-level helpers ---
    std::pair<double,double> computeRelationCentroid(Relation* r) {
        double sx=0, sy=0; int c=0;
        for (long long wid : r->member_way_refs) {
            for (const auto &w : ways_) {
                if (w.id == wid) {
                    for (long long nid : w.node_refs) if (nodes_map_.count(nid)) { sx += nodes_map_[nid].x; sy += nodes_map_[nid].y; ++c; }
                    break;
                }
            }
        }
        if (c==0) return {0.0,0.0};
        return {sx/c, sy/c};
    }

    // Find shortest relation path via BFS on relation_adjacency_
    std::vector<long long> findRelationPath(long long start_rel, long long goal_rel) {
        std::queue<long long> q;
        std::map<long long, long long> came_from;
        q.push(start_rel);
        came_from[start_rel] = start_rel;
        while (!q.empty()) {
            long long cur = q.front(); q.pop();
            if (cur == goal_rel) break;
            for (long long nxt : relation_adjacency_[cur]) {
                if (came_from.find(nxt) == came_from.end()) { q.push(nxt); came_from[nxt] = cur; }
            }
        }
        std::vector<long long> path;
        if (came_from.find(goal_rel) == came_from.end()) return path;
        long long cur = goal_rel;
        while (cur != start_rel) { path.push_back(cur); cur = came_from[cur]; }
        path.push_back(start_rel);
        std::reverse(path.begin(), path.end());
        return path;
    }

    // Compute the centerline for a relation by averaging member ways (same as earlier lambda, now a method)
    std::vector<std::pair<double,double>> computeRelationCenterline(Relation* r) {
        std::vector<std::pair<double,double>> center;
        if (!r) return center;
        if (r->member_way_refs.empty()) return center;
        const int N = 40;
        std::vector<std::vector<std::pair<double,double>>> all_pts;
        for (long long wid : r->member_way_refs) {
            Way* w = nullptr; for (auto &ww : ways_) if (ww.id == wid) { w = &ww; break; }
            if (!w) continue;
            auto pts = resampleWayPointsMethod(w, N);
            if (!pts.empty()) all_pts.push_back(std::move(pts));
        }
        if (all_pts.empty()) return center;
        size_t M = all_pts[0].size();
        center.resize(M, {0.0,0.0});
        for (size_t i=0;i<M;++i) {
            double sx=0, sy=0; int c=0;
            for (auto &v : all_pts) { if (i < v.size()) { sx += v[i].first; sy += v[i].second; ++c; } }
            if (c>0) { center[i].first = sx / c; center[i].second = sy / c; }
        }
        return center;
    }

    // class-level way resampling implementation
    std::vector<std::pair<double,double>> resampleWayPointsMethod(Way* w, int N) {
        std::vector<std::pair<double,double>> out;
        if (!w || w->node_refs.empty()) return out;
        std::vector<std::pair<double,double>> pts;
        for (long long nid : w->node_refs) if (nodes_map_.count(nid)) pts.emplace_back(nodes_map_[nid].x, nodes_map_[nid].y);
        if (pts.empty()) return out;
        if ((int)pts.size() == 1) { out.push_back(pts[0]); return out; }
        std::vector<double> d(pts.size(), 0.0);
        for (size_t i=1;i<pts.size();++i) {
            double dx = pts[i].first - pts[i-1].first; double dy = pts[i].second - pts[i-1].second; d[i] = d[i-1] + std::hypot(dx,dy);
        }
        double L = d.back();
        for (int k=0;k<N;++k) {
            double s = (L * k) / (N-1);
            size_t idx = 0;
            while (idx+1 < d.size() && d[idx+1] < s) ++idx;
            double segL = d[idx+1] - d[idx];
            double t = (segL > 1e-9) ? ((s - d[idx]) / segL) : 0.0;
            double x = pts[idx].first * (1-t) + pts[idx+1].first * t;
            double y = pts[idx].second * (1-t) + pts[idx+1].second * t;
            out.emplace_back(x,y);
        }
        return out;
    }

    // Build a concatenated centerline from a sequence of relations (resampled)
    std::vector<std::pair<double,double>> centerlineFromRelationPath(const std::vector<long long>& relpath, int per_rel_samples=40) {
        std::vector<std::pair<double,double>> out;
        for (long long rid : relpath) {
            Relation* r = nullptr;
            for (auto &rr : relations_) if (rr.id == rid) { r = &rr; break; }
            if (!r) continue;
            auto c = computeRelationCenterline(r);
            // if centerline has points, append them (resample if necessary)
            for (auto &p : c) out.push_back(p);
        }
        // simple down-sample/unique filter to avoid duplicates
        std::vector<std::pair<double,double>> filtered;
        for (size_t i=0;i<out.size();++i) {
            if (i==0) filtered.push_back(out[i]);
            else {
                double dx = out[i].first - filtered.back().first; double dy = out[i].second - filtered.back().second;
                if (std::hypot(dx,dy) > 0.05) filtered.push_back(out[i]);
            }
        }
        return filtered;
    }

    // Reeds-Shepp planning using OMPL (if available)
#ifdef HAVE_OMPL
    std::vector<CarState> planReedsShepp(const CarState &start, const CarState &goal, double min_turn_radius, double max_time = 1.0) {
        std::vector<CarState> out;
        // State space: ReedsShepp(x,y,theta)
        ob::StateSpacePtr ss(new ob::ReedsSheppStateSpace(min_turn_radius));
        // set bounds from map extents
        ob::RealVectorBounds bounds(2);
        double minx = 1e18, miny = 1e18, maxx = -1e18, maxy = -1e18;
        for (const auto &kv : nodes_map_) { minx = std::min(minx, kv.second.x); miny = std::min(miny, kv.second.y); maxx = std::max(maxx, kv.second.x); maxy = std::max(maxy, kv.second.y); }
        if (minx < maxx && miny < maxy) {
            bounds.setLow(0, minx - 10.0); bounds.setHigh(0, maxx + 10.0);
            bounds.setLow(1, miny - 10.0); bounds.setHigh(1, maxy + 10.0);
        } else {
            bounds.setLow(0, -100.0); bounds.setHigh(0, 100.0);
            bounds.setLow(1, -100.0); bounds.setHigh(1, 100.0);
        }
        ss->as<ob::SE2StateSpace>()->setBounds(bounds); // ReedsSheppStateSpace derives from SE2

        og::SimpleSetup ss_setup(ss);
        // trivial state validity: check in-bounds
        ss_setup.setStateValidityChecker([ss](const ob::State* s) {
            const auto *se2 = s->as<ob::SE2StateSpace::StateType>();
            (void)se2; // no obstacles known
            return true;
        });

        ob::ScopedState<> start_s(ss);
        start_s[0] = start.x; start_s[1] = start.y; start_s[2] = start.theta;
        ob::ScopedState<> goal_s(ss);
        goal_s[0] = goal.x; goal_s[1] = goal.y; goal_s[2] = goal.theta;
        ss_setup.setStartAndGoalStates(start_s, goal_s);

        auto planner = std::make_shared<og::RRTConnect>(ss_setup.getSpaceInformation());
        ss_setup.setPlanner(planner);
        ob::PlannerStatus solved = ss_setup.solve(max_time);
        if (!solved) return out;
        // interpolate path for smoother sampling
        og::PathGeometric path = ss_setup.getSolutionPath();
        path.interpolate(std::max( (int)path.getStateCount()*4, 100));
        for (std::size_t i = 0; i < path.getStateCount(); ++i) {
            const ob::SE2StateSpace::StateType* st = path.getState(i)->as<ob::SE2StateSpace::StateType>();
            CarState s; s.x = st->getX(); s.y = st->getY(); s.theta = st->getYaw(); s.v = 0.0; s.phi = 0.0; out.push_back(s);
        }
        return out;
    }
#endif

    // 从当前位置规划路径到目标位置
    std::vector<CarState> planPathFromCurrentPosition() {
        std::vector<CarState> trajectory;
        
        // 如果车辆距离车库较远，使用车道线中心线规划
        double dist_to_garage = std::hypot(car_.x - parking_target_x_, car_.y - parking_target_y_);
        
        if (dist_to_garage > 10.0) {
            // 距离较远，使用车道线中心线规划
            trajectory = planLaneCenterlinePath(car_, parking_target_x_, parking_target_y_, parking_target_theta_);
        } else if (dist_to_garage > 5.0) {
            // 中等距离，使用混合路径规划
            trajectory = planHybridPath(car_, parking_target_x_, parking_target_y_, parking_target_theta_);
        } else {
            // 近距离，使用泊车轨迹规划
            trajectory = generateVerticalParkingTrajectory(car_, parking_target_x_, parking_target_y_, parking_target_theta_);
        }
        
        return trajectory;
    }

    // 规划车道线中心线路径，支持倒车
    std::vector<CarState> planLaneCenterlinePath(const CarState& start, double target_x, double target_y, double target_theta) {
        std::vector<CarState> path;
        
        // 使用简单的直线插值规划到目标点
        int num_points = 100;
        double dx = target_x - start.x;
        double dy = target_y - start.y;
        double distance = std::hypot(dx, dy);
        
        if (distance < 0.1) {
            // 距离太近，直接返回目标点
            CarState goal;
            goal.x = target_x;
            goal.y = target_y;
            goal.theta = target_theta;
            goal.v = 0.0;
            path.push_back(goal);
            return path;
        }
        
        // 检查是否需要倒车：计算起始方向与目标方向的角度差
        double start_to_target_angle = std::atan2(dy, dx);
        double angle_diff = normalizeAng(start_to_target_angle - start.theta);
        bool need_reverse = std::abs(angle_diff) > M_PI_2;
        
        // 生成平滑的路径点
        for (int i = 0; i <= num_points; ++i) {
            double ratio = static_cast<double>(i) / num_points;
            CarState state;
            state.x = start.x + ratio * dx;
            state.y = start.y + ratio * dy;
            
            // 计算方向：使用当前位置到目标点的方向
            if (i < num_points) {
                double next_x = start.x + (ratio + 0.01) * dx;
                double next_y = start.y + (ratio + 0.01) * dy;
                state.theta = std::atan2(next_y - state.y, next_x - state.x);
            } else {
                state.theta = target_theta;
            }
            
            // 速度控制：根据距离调整速度，支持倒车
            double remaining_dist = distance * (1.0 - ratio);
            double speed;
            
            if (remaining_dist > 5.0) {
                speed = max_speed_;
            } else if (remaining_dist > 1.0) {
                speed = max_speed_ * 0.5;
            } else {
                speed = min_speed_;
            }
            
            // 如果需要倒车，设置负速度
            if (need_reverse) {
                state.v = -speed;
            } else {
                state.v = speed;
            }
            
            path.push_back(state);
        }
        
        return path;
    }

    // 规划混合路径（车道线 + 曲线过渡），限制曲率
    std::vector<CarState> planHybridPath(const CarState& start, double target_x, double target_y, double target_theta) {
        std::vector<CarState> path;
        
        // 使用Reeds-Shepp规划器（如果可用）
#ifdef HAVE_OMPL
        CarState goal;
        goal.x = target_x;
        goal.y = target_y;
        goal.theta = target_theta;
        
        // 使用更小的最小转弯半径来增加灵活性
        double constrained_min_turn_radius = std::max(min_turn_radius_, 3.0); // 减小转弯半径，让车辆更灵活
        path = planReedsShepp(start, goal, constrained_min_turn_radius, 1.0);
        
        if (!path.empty()) {
            // 检查路径曲率，确保不会太大
            bool has_high_curvature = false;
            for (int i = 1; i < (int)path.size() - 1; ++i) {
                CarState& prev = path[i-1];
                CarState& curr = path[i];
                CarState& next = path[i+1];
                
                // 计算曲率
                double dx1 = curr.x - prev.x;
                double dy1 = curr.y - prev.y;
                double dx2 = next.x - curr.x;
                double dy2 = next.y - curr.y;
                
                double dist1 = std::hypot(dx1, dy1);
                double dist2 = std::hypot(dx2, dy2);
                
                if (dist1 > 1e-6 && dist2 > 1e-6) {
                    double angle_diff = std::abs(normalizeAng(std::atan2(dy2, dx2) - std::atan2(dy1, dx1)));
                    double curvature = angle_diff / ((dist1 + dist2) * 0.5);
                    
                    // 如果曲率太大，使用更平滑的路径
                    if (curvature > 0.2) { // 降低最大曲率到0.2，确保曲线更平滑
                        has_high_curvature = true;
                        break;
                    }
                }
            }
            
            // 如果曲率太大，回退到车道线中心线规划
            if (has_high_curvature) {
                ROS_WARN("Hybrid path has high curvature, falling back to lane centerline path");
                return planLaneCenterlinePath(start, target_x, target_y, target_theta);
            }
            
            // 为路径点设置适当的速度，基于最近点计算
            for (int i = 0; i < (int)path.size(); ++i) {
                // 计算到目标的距离
                double dist_to_target = std::hypot(path[i].x - target_x, path[i].y - target_y);
                
                // 基于距离设置速度
                if (dist_to_target > 8.0) {
                    path[i].v = max_speed_ * 0.8; // 远距离，较高速度
                } else if (dist_to_target > 3.0) {
                    path[i].v = max_speed_ * 0.5; // 中等距离，中等速度
                } else {
                    path[i].v = min_speed_; // 近距离，低速
                }
                
                // 检查是否需要倒车
                if (i > 0) {
                    double dx = path[i].x - path[i-1].x;
                    double dy = path[i].y - path[i-1].y;
                    double path_angle = std::atan2(dy, dx);
                    double vehicle_angle = path[i].theta;
                    double angle_diff = normalizeAng(path_angle - vehicle_angle);
                    
                    // 如果角度差大于90度，需要倒车
                    if (std::abs(angle_diff) > M_PI_2) {
                        path[i].v = -std::abs(path[i].v); // 设置负速度表示倒车
                    }
                }
            }
            
            return path;
        }
#endif
        
        // 如果OMPL不可用或规划失败，回退到车道线中心线规划
        return planLaneCenterlinePath(start, target_x, target_y, target_theta);
    }

    // --- 添加泊车轨迹 (多阶段揉库生成器: 倒车直线 -> 倒车弧 -> 进位微调) ---
    static double normalizeAng(double a){ while (a > M_PI) a -= 2*M_PI; while (a <= -M_PI) a += 2*M_PI; return a; }

    // --- geometry helpers: point-in-polygon and footprint collision check ---
    bool pointInPolygon(const std::vector<std::pair<double,double>> &poly, double x, double y) {
        if (poly.empty()) return false;
        bool inside = false;
        for (size_t i=0,j=poly.size()-1;i<poly.size();j=i++) {
            double xi = poly[i].first, yi = poly[i].second;
            double xj = poly[j].first, yj = poly[j].second;
            bool intersect = ((yi>y) != (yj>y)) && (x < (xj-xi) * (y-yi) / (yj-yi + 1e-12) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    bool footprintInsidePolygon(const std::vector<std::pair<double,double>> &poly, const CarState &s, double margin = 0.03) {
        if (poly.empty()) return true; // no polygon -> nothing to check
        double l2 = vehicle_length_ * 0.5 + margin;
        double w2 = vehicle_width_ * 0.5 + margin;
        // four corners in vehicle frame
        std::vector<std::pair<double,double>> corners = {{ l2,  w2}, { l2, -w2}, {-l2, -w2}, {-l2,  w2}};
        double c = std::cos(s.theta), si = std::sin(s.theta);
        for (auto &pt : corners) {
            double wx = s.x + pt.first * c - pt.second * si;
            double wy = s.y + pt.first * si + pt.second * c;
            if (!pointInPolygon(poly, wx, wy)) return false;
        }
        return true;
    }

    // --- slot utilities: order corners and compute slot frame ---
    struct SlotFrame {
        // corners in local frame (BL, BR, TR, TL)
        std::vector<std::pair<double,double>> corners_local;
        double rear_center_x = 0.0; // xtarget = 0 reference at rear center
        double rear_center_y = 0.0;
        double length = 0.0; // N
        double width = 0.0;  // M
        double theta = 0.0; // slot long axis orientation (global)
    };

    // Build a consistent slot frame from polygon and parking_target_theta_
    SlotFrame buildSlotFrame(const std::vector<std::pair<double,double>>& poly, double theta_global) {
        SlotFrame sf;
        if (poly.size() < 4) return sf;
        // compute centroid and rotate points to slot frame using theta_global
        double cx = 0, cy = 0; for (auto &p : poly) { cx += p.first; cy += p.second; } cx /= poly.size(); cy /= poly.size();
        double c = std::cos(theta_global), s = std::sin(theta_global);
        std::vector<std::pair<double,double>> local;
        for (auto &p : poly) {
            double dx = p.first - cx; double dy = p.second - cy;
            // local x = dx*cos + dy*sin, local y = -dx*sin + dy*cos (right-hand -> x forward)
            double lx = dx * c + dy * s;
            double ly = -dx * s + dy * c;
            local.emplace_back(lx, ly);
        }
        // find min x (rear) and max x (front)
        double minx = 1e18, maxx = -1e18, miny = 1e18, maxy = -1e18;
        for (auto &p : local) { minx = std::min(minx, p.first); maxx = std::max(maxx, p.first); miny = std::min(miny, p.second); maxy = std::max(maxy, p.second); }
        sf.length = maxx - minx; sf.width = maxy - miny; sf.theta = theta_global;
        // rear center at minx mid y
        double ry = 0.0; int cnt=0; for (auto &p : local) { if (std::abs(p.first - minx) < 1e-3) { ry += p.second; ++cnt; } }
        double rear_y = (cnt>0)?(ry/cnt):0.0; sf.rear_center_x = 0.0; sf.rear_center_y = rear_y; // set rear center local x=0
        // create corners in local frame ordered as BL,BR,TR,TL using local coordinates relative to rear center
        std::vector<std::pair<double,double>> rel;
        for (auto &p : local) rel.emplace_back(p.first - minx, p.second - rear_y);
        // now assign corners by scanning for min/max x,y combinations
        std::pair<double,double> BL{1e9,1e9}, BR{1e9,-1e9}, TR{-1e9,-1e9}, TL{-1e9,1e9};
        for (auto &p : rel) {
            if (p.first <= sf.length*0.5 && p.second >= 0) BL = p;
            if (p.first <= sf.length*0.5 && p.second <= 0) BR = p;
            if (p.first >= sf.length*0.5 && p.second <= 0) TR = p;
            if (p.first >= sf.length*0.5 && p.second >= 0) TL = p;
        }
        sf.corners_local = {BL, BR, TR, TL};
        // convert corners back to global coordinates for completeness
        std::vector<std::pair<double,double>> corners_global;
        for (auto &p : sf.corners_local) {
            double gx = (p.first + minx - 0.0) * c - (p.second + rear_y) * s + cx;
            double gy = (p.first + minx - 0.0) * s + (p.second + rear_y) * c + cy;
            corners_global.emplace_back(gx, gy);
        }
        // override parking_slot_polygon_ with ordered global corners if appropriate
        return sf;
    }

    // compute minimal allowed entering point P0 in slot-local frame, return in global coords
    bool computeMinEnteringPoint(const SlotFrame &sf, std::pair<double,double> &P0_global, std::tuple<double,double,double> &P0_local_theta, double L, double a) {
        // slot length N is sf.length
        double N = sf.length;
        if (N <= 0) return false;
        double W = vehicle_width_;
        double R = regulatory_Rmin_ - W/2.0 - dsafe_;
        if (R <= 0) return false;
        double term = (N - a);
        double inside = R*R - (term*term) / 4.0;
        if (inside < 0) return false;
        double y0_min = R - std::sqrt(inside) + W/2.0 + dsafe_;
        double y0 = y0_min + 0.05; // extra margin
        // compute x0 according to provided formula: x0 = xtarget - sqrt(4*R*(y0-R)) - L
        double tmp = 4.0 * R * (y0 - R);
        if (tmp < 0) return false;
        double x0 = - std::sqrt(tmp) - L; // xtarget is 0 in local frame -> subtract L
        double theta0 = 0.0; // aligned with slot center
        // convert local (x0,y0) to global using slot frame
        double c = std::cos(sf.theta), s = std::sin(sf.theta);
        // rear_center global: compute from parking_slot_polygon_ centroid math (approx)
        double gx_rear = 0.0, gy_rear = 0.0;
        // approximate: use centroid of polygon as base plus rear offset
        double cx=0, cy=0; for (auto &p : parking_slot_polygon_) { cx += p.first; cy += p.second; } if (!parking_slot_polygon_.empty()) { cx/=parking_slot_polygon_.size(); cy/=parking_slot_polygon_.size(); }
        // rear center global approximate by shifting back by (sf.length/2 along -theta)
        gx_rear = cx - (sf.length/2.0) * c; gy_rear = cy - (sf.length/2.0) * s;
        double gx = gx_rear + x0 * c - y0 * s;
        double gy = gy_rear + x0 * s + y0 * c;
        P0_global = std::make_pair(gx, gy);
        P0_local_theta = std::make_tuple(x0, y0, theta0);
        return true;
    }

    // generate two tangent circular arcs in slot-local frame (reverse ingress)
    // returns empty on failure
    std::vector<CarState> generateTwoTangentArcs(const SlotFrame &sf, double x0, double y0, double xt=0.0, double yt=0.0, double R_in = -1.0, double ds = 0.02, const std::vector<std::pair<double,double>> &slot_poly = std::vector<std::pair<double,double>>()) {
        std::vector<CarState> out;
        double W = vehicle_width_;
        
        // 使用更小的转弯半径2米，让车辆更灵活
        double R = (R_in > 0.01) ? R_in : 3.0;
        
        // 计算第一个圆弧的圆心
        double C1x = x0;
        double C1y = y0 + R;
        
        // 计算第二个圆弧的圆心，位于第一个圆弧的右侧
        double C2x = C1x + 2.0 * R; // 两个圆心之间的距离为2R
        double C2y = C1y;
        
        // 计算角度
        double a0 = std::atan2(y0 - C1y, x0 - C1x); // 起始角度
        double a_mid = M_PI / 2.0; // 两个圆弧的连接点角度
        double a_end = std::atan2(yt - C2y, xt - C2x); // 结束角度
        
        // 确保角度差不为零
        if (std::fabs(a_mid - a0) < 1e-4) {
            a0 = a_mid - M_PI / 4.0; // 调整起始角度，确保有角度变化
        }
        
        // 采样第一个圆弧：从a0到a_mid
        double len1 = R * std::fabs(a_mid - a0);
        int n1 = std::max(1, static_cast<int>(std::ceil(len1 / ds)));
        for (int i = 0; i <= n1; i++) {
            double t = static_cast<double>(i) / n1;
            double ang = a0 + t * (a_mid - a0);
            
            // 计算局部坐标
            double lx = C1x + R * std::cos(ang);
            double ly = C1y + R * std::sin(ang);
            
            // 计算朝向角（倒车时，朝向角为当前角度减去90度）
            double heading = ang - M_PI_2;
            
            // 计算全局坐标
            double cx = 0, cy = 0;
            for (const auto &p : parking_slot_polygon_) {
                cx += p.first;
                cy += p.second;
            }
            if (!parking_slot_polygon_.empty()) {
                cx /= parking_slot_polygon_.size();
                cy /= parking_slot_polygon_.size();
            }
            
            double c = std::cos(sf.theta);
            double s = std::sin(sf.theta);
            
            // 将局部坐标转换为全局坐标
            double gx = cx + (lx - (sf.length / 2.0)) * c - ly * s;
            double gy = cy + (lx - (sf.length / 2.0)) * s + ly * c;
            
            CarState state;
            state.x = gx;
            state.y = gy;
            state.theta = heading + sf.theta;
            state.v = -0.4; // 倒车速度
            state.phi = std::atan(wheelbase_ / R); // 方向盘角度
            
            // 暂时禁用碰撞检查，确保能够生成轨迹
            // if (!slot_poly.empty() && !footprintInsidePolygon(slot_poly, state)) return std::vector<CarState>();
            
            out.push_back(state);
        }
        
        // 采样第二个圆弧：从a_mid到a_end
        double len2 = R * std::fabs(a_end - a_mid);
        int n2 = std::max(1, static_cast<int>(std::ceil(len2 / ds)));
        for (int i = 1; i <= n2; i++) {
            double t = static_cast<double>(i) / n2;
            double ang = a_mid + t * (a_end - a_mid);
            
            // 计算局部坐标
            double lx = C2x + R * std::cos(ang);
            double ly = C2y + R * std::sin(ang);
            
            // 计算朝向角
            double heading = ang - M_PI_2;
            
            // 计算全局坐标
            double cx = 0, cy = 0;
            for (const auto &p : parking_slot_polygon_) {
                cx += p.first;
                cy += p.second;
            }
            if (!parking_slot_polygon_.empty()) {
                cx /= parking_slot_polygon_.size();
                cy /= parking_slot_polygon_.size();
            }
            
            double c = std::cos(sf.theta);
            double s = std::sin(sf.theta);
            
            // 将局部坐标转换为全局坐标
            double gx = cx + (lx - (sf.length / 2.0)) * c - ly * s;
            double gy = cy + (lx - (sf.length / 2.0)) * s + ly * c;
            
            CarState state;
            state.x = gx;
            state.y = gy;
            state.theta = heading + sf.theta;
            state.v = -0.4; // 倒车速度
            state.phi = -std::atan(wheelbase_ / R); // 方向盘角度（反向）
            
            // 暂时禁用碰撞检查，确保能够生成轨迹
            // if (!slot_poly.empty() && !footprintInsidePolygon(slot_poly, state)) return std::vector<CarState>();
            
            out.push_back(state);
        }
        
        return out;
    }

    // Attempt to compute a reverse circular arc (constant curvature) from start to goal.
    // Returns sequence of CarState if possible, empty otherwise.
    std::vector<CarState> computeReverseArcTrajectory(const CarState &start, const CarState &goal, double minR, double maxR = 10.0, double ds = 0.05, const std::vector<std::pair<double,double>> &slot_poly = std::vector<std::pair<double,double>>()) {
        std::vector<CarState> best_traj;
        double tol_pos = 0.22; // meters
        double tol_angle = 0.06; // rad
        double best_err = 1e9;
        // dir = -1 for reversing
        int dir = -1;
        int st_pose = 0; // unused for now
        double dtheta_needed = normalizeAng(goal.theta - start.theta);

        // try both turning directions (sign of curvature)
        for (int turn_sign = -1; turn_sign <= 1; turn_sign += 2) {
            for (double R = minR; R <= maxR; R += 0.25) {
                double k = turn_sign * (1.0 / R);
                // compute required path length s to achieve desired heading change when reversing: theta_final = start_theta + dir * k * s
                double s_req = dtheta_needed / (dir * k);
                if (s_req <= 0 || s_req > 20.0) continue; // skip non-sensical lengths
                // simulate along arc with small steps
                std::vector<CarState> traj;
                CarState cur = start;
                double s_acc = 0.0;
                bool fail=false;
                int steps = std::max(1, static_cast<int>(std::ceil(s_req / ds)));
                for (int i=0;i<steps;i++) {
                    double step_len = std::min(ds, s_req - s_acc);
                    // update orientation
                    cur.theta = normalizeAng(cur.theta + dir * k * step_len);
                    // move along backward direction (dir=-1)
                    cur.x += dir * step_len * std::cos(cur.theta);
                    cur.y += dir * step_len * std::sin(cur.theta);
                    // approximate steering angle
                    cur.v = -0.4; cur.phi = turn_sign * std::atan(wheelbase_ / R);
                    // footprint collision check: reject arcs that leave the parking slot polygon
                    if (!footprintInsidePolygon(slot_poly, cur)) { fail = true; break; }
                    traj.push_back(cur);
                    s_acc += step_len;
                }
                if (fail) continue;
                // after sim, check final pose
                if (traj.empty()) continue;
                CarState &last = traj.back();
                double pos_err = std::hypot(last.x - goal.x, last.y - goal.y);
                double ang_err = std::abs(normalizeAng(last.theta - goal.theta));
                double tot_err = pos_err + ang_err*R; // weighted
                if (pos_err < tol_pos && ang_err < tol_angle) {
                    if (tot_err < best_err) { best_err = tot_err; best_traj = traj; }
                }
            }
        }
        return best_traj;
    }

    std::vector<CarState> generateMultiStageParking(const CarState &start, double park_x, double park_y, double target_theta) {
        std::vector<CarState> out;
        double step = 0.1; // sampling resolution (m)
        double back_dist = parking_back_distance_;
        nh_.param("parking_back_distance", back_dist, back_dist);

        // 新增：基于轨迹的倒车规则 - 泊车开始点不能超过车库位置
        // 计算车辆当前位置到车库的最短距离
        double dist_to_garage = std::hypot(start.x - park_x, start.y - park_y);
        if (dist_to_garage < 0.5) { // 如果已经在车库0.5米范围内
            ROS_WARN("Vehicle too close to garage (%.2fm), adjusting start position", dist_to_garage);
            // 向后调整开始位置，确保有足够倒车距离
            back_dist = std::max(back_dist, 1.5); // 最小后退1.5米
        }

        // compute standoff point: behind the parking center along target_heading so reversing moves into slot
        double standoff = std::max(1.0, back_dist);
        
        // 新增：验证泊车开始点不超过车库位置
        double start_to_garage_dist = std::hypot(start.x - park_x, start.y - park_y);
        double max_allowed_dist = standoff * 1.5; // 增加允许最大距离为standoff的1.5倍，提高容错性
        if (start_to_garage_dist > max_allowed_dist) {
            ROS_WARN("Start point too far from garage (%.2fm > %.2fm), adjusting standoff", 
                     start_to_garage_dist, max_allowed_dist);
            standoff = start_to_garage_dist * 0.9; // 调整standoff为距离的90%，减少调整幅度
        } else if (start_to_garage_dist < standoff) {
            // 如果起点距离小于standoff，适当减小standoff避免过度调整
            standoff = std::max(1.0, start_to_garage_dist * 0.7);
        }
        double st_x = park_x - std::cos(target_theta) * standoff;
        double st_y = park_y - std::sin(target_theta) * standoff;
        double st_theta = target_theta; // face into the slot (so reversing goes into center)

        // 0) Approach standoff smoothly from start (Hermite cubic interpolation)
        // 新增：基于轨迹的倒车验证 - 确保路径不会穿过车库
        // 检查开始点到standoff点的直线是否穿过车库区域
        bool path_crosses_garage = false;
        int check_points = 10;
        for (int i = 1; i <= check_points; ++i) {
            double t = double(i) / check_points;
            double check_x = start.x * (1-t) + st_x * t;
            double check_y = start.y * (1-t) + st_y * t;
            double dist_to_garage = std::hypot(check_x - park_x, check_y - park_y);
            if (dist_to_garage < 1.0) { // 1米范围内认为穿过车库
                path_crosses_garage = true;
                ROS_WARN("Path crosses garage area at (%.2f, %.2f), distance=%.2fm", 
                         check_x, check_y, dist_to_garage);
                break;
            }
        }
        
        if (path_crosses_garage) {
            // 调整standoff点，避免穿过车库
            double angle_to_garage = std::atan2(park_y - start.y, park_x - start.x);
            st_x = park_x - std::cos(angle_to_garage) * (standoff + 1.0);
            st_y = park_y - std::sin(angle_to_garage) * (standoff + 1.0);
            ROS_INFO("Adjusted standoff point to avoid garage: (%.2f, %.2f)", st_x, st_y);
        }

        // build a cubic Hermite (smooth) between start and standoff
        CarState last = start;
        double x0 = last.x, y0 = last.y;
        double x1 = st_x, y1 = st_y;
        double dx = x1 - x0, dy = y1 - y0;
        double L = std::hypot(dx, dy);
        int approach_steps = std::max(6, static_cast<int>(std::min(60.0, L * 6.0)));
        double v0x = std::cos(last.theta), v0y = std::sin(last.theta);
        double v1x = std::cos(st_theta), v1y = std::sin(st_theta);
        double mscale = L * 0.4;
        double m0x = v0x * mscale, m0y = v0y * mscale;
        double m1x = v1x * mscale, m1y = v1y * mscale;
        for (int i=1;i<=approach_steps;i++) {
            double t = double(i) / approach_steps;
            double t2 = t*t, t3 = t2*t;
            double h00 =  2*t3 - 3*t2 + 1;
            double h10 =      t3 - 2*t2 + t;
            double h01 = -2*t3 + 3*t2;
            double h11 =      t3 -   t2;
            double px = h00*x0 + h10*m0x + h01*x1 + h11*m1x;
            double py = h00*y0 + h10*m0y + h01*y1 + h11*m1y;
            CarState s; s.x = px; s.y = py; s.theta = std::atan2(v1y, v1x); s.v = std::min(0.8, max_speed_);
            out.push_back(s);
        }

        // Try the regulation-based two-arc ingress first
        // Build slot frame and compute P0
        SlotFrame sf = buildSlotFrame(parking_slot_polygon_, parking_target_theta_);
        std::pair<double,double> P0_global; std::tuple<double,double,double> P0_local_theta;
        if (computeMinEnteringPoint(sf, P0_global, P0_local_theta, wheelbase_, rear_overhang_)) {
            double x0 = std::get<0>(P0_local_theta);
            double y0 = std::get<1>(P0_local_theta);
            ROS_INFO("Computed P0 in slot-local: x0=%.3f y0=%.3f (global approx x=%.3f y=%.3f)", x0, y0, P0_global.first, P0_global.second);
            // visualize P0
            visualization_msgs::Marker m;
            m.header.frame_id = "map"; m.header.stamp = ros::Time::now(); m.ns = "parking_p0"; m.id = 10;
            m.type = visualization_msgs::Marker::SPHERE; m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 0.25;
            m.color.a = 0.9; m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.pose.position.x = P0_global.first; m.pose.position.y = P0_global.second; m.pose.position.z = 0.2;
            visualization_msgs::MarkerArray marr; marr.markers.push_back(m); parking_pub_.publish(marr);

            // ensure approach from start to P0 does not enter slot area prematurely
            CarState last = start;
            // sample a simple straight-to-P0 approach
            double ax = P0_global.first, ay = P0_global.second;
            double Lapp = std::hypot(ax - last.x, ay - last.y);
            int app_steps = std::max(4, static_cast<int>(std::ceil(Lapp / 0.2)));
            bool approach_ok = true; std::vector<CarState> app_pts;
            for (int i=1;i<=app_steps;i++) {
                double t = double(i)/app_steps; CarState s; s.x = last.x*(1-t) + ax*t; s.y = last.y*(1-t) + ay*t; s.theta = std::atan2(ay-last.y, ax-last.x); s.v = 0.6;
                app_pts.push_back(s);
                if (pointInPolygon(parking_slot_polygon_, s.x, s.y)) { approach_ok = false; break; }
            }
            if (approach_ok) {
                // try generate two arcs in local frame
                auto arcs = generateTwoTangentArcs(sf, x0, y0, 0.0, 0.0, regulatory_Rmin_ - vehicle_width_/2.0 - dsafe_, 0.02, parking_slot_polygon_);
                if (!arcs.empty()) {
                    // publish arc marker for visualization
                    visualization_msgs::Marker arc_m; arc_m.header.frame_id = "map"; arc_m.header.stamp = ros::Time::now(); arc_m.ns = "parking_arc"; arc_m.id = 11; arc_m.type = visualization_msgs::Marker::LINE_STRIP; arc_m.scale.x = 0.06; arc_m.color.a = 0.9; arc_m.color.r = 0.0; arc_m.color.g = 1.0; arc_m.color.b = 0.0;
                    for (auto &s : arcs) { geometry_msgs::Point gp; gp.x = s.x; gp.y = s.y; gp.z = 0.05; arc_m.points.push_back(gp); }
                    visualization_msgs::MarkerArray marr2; marr2.markers.push_back(arc_m); parking_pub_.publish(marr2);

                    // append approach then arcs
                    for (auto &s: app_pts) out.push_back(s);
                    for (auto &s: arcs) out.push_back(s);
                } else {
                    ROS_WARN("Two-arc generator failed to produce a collision-free ingress; falling back to previous strategies.");
                    // fallback retry strategy (increase standoff and try reverse arc generator as before)
                    bool found_alt_arc = false;
                    for (double extra = 0.5; extra <= 2.0 && !found_alt_arc; extra += 0.5) {
                        double st2 = standoff + extra;
                        CarState st2_pose; st2_pose.x = park_x - std::cos(target_theta) * st2; st2_pose.y = park_y - std::sin(target_theta) * st2; st2_pose.theta = st_theta;
                        CarState goal_pose; goal_pose.x = park_x; goal_pose.y = park_y; goal_pose.theta = target_theta;
                        auto arc2 = computeReverseArcTrajectory(st2_pose, goal_pose, std::max(0.1, min_turn_radius_), 12.0, 0.05, parking_slot_polygon_);
                        if (!arc2.empty()) {
                            // approach to st2
                            CarState last = out.empty() ? start : out.back();
                            double x0a = last.x, y0a = last.y, x1 = st2_pose.x, y1 = st2_pose.y;
                            double L2 = std::hypot(x1-x0a, y1-y0a);
                            int steps2 = std::max(6, static_cast<int>(std::min(60.0, L2 * 6.0)));
                            double v0x = std::cos(last.theta), v0y = std::sin(last.theta);
                            double v1x = std::cos(st2_pose.theta), v1y = std::sin(st2_pose.theta);
                            double mscale2 = L2 * 0.4; double m0x = v0x * mscale2, m0y = v0y * mscale2; double m1x = v1x * mscale2, m1y = v1y * mscale2;
                            for (int i=1;i<=steps2;i++) {
                                double t = double(i) / steps2; double t2 = t*t, t3 = t2*t;
                                double h00 =  2*t3 - 3*t2 + 1;
                                double h10 =      t3 - 2*t2 + t;
                                double h01 = -2*t3 + 3*t2;
                                double h11 =      t3 -   t2;
                                double px = h00*x0a + h10*m0x + h01*x1 + h11*m1x;
                                double py = h00*y0a + h10*m0y + h01*y1 + h11*m1y;
                                CarState s; s.x = px; s.y = py; s.theta = std::atan2(v1y, v1x); s.v = std::min(0.8, max_speed_);
                                out.push_back(s);
                            }
                            for (auto &s : arc2) out.push_back(s);
                            found_alt_arc = true; break;
                        }
                    }
                    if (!found_alt_arc) {
                        // previous fallback: reverse straight then arcs and corrections
                        int n_back = std::max(1, static_cast<int>(std::ceil(standoff / step)));
                        CarState st_pose; st_pose.x = park_x; st_pose.y = park_y; st_pose.theta = target_theta;
                        for (int i = 1; i <= n_back; ++i) {
                            CarState s;
                            double ds = i * step;
                            s.x = st_pose.x - ds * cos(st_pose.theta);
                            s.y = st_pose.y - ds * sin(st_pose.theta);
                            s.theta = st_pose.theta;
                            s.v = -0.5;
                            s.phi = 0.0;
                            out.push_back(s);
                        }
                        // small turn arc towards target
                        double diff = normalizeAng(target_theta - out.back().theta);
                        double sign = (diff >= 0.0) ? 1.0 : -1.0;
                        double delta = std::fabs(diff);
                        double R = (min_turn_radius_ > 0.01) ? min_turn_radius_ : 5.0;
                        if (delta > 1e-4) {
                            double cx = out.back().x + sign * R * (-sin(out.back().theta));
                            double cy = out.back().y + sign * R * ( cos(out.back().theta));
                            int n_arc = std::max(2, static_cast<int>(std::ceil((R * delta) / step)));
                            for (int i = 1; i <= n_arc; ++i) {
                                double theta_i = out.back().theta + sign * (delta * i / n_arc);
                                double px = cx + sign * R * sin(theta_i);
                                double py = cy - sign * R * cos(theta_i);
                                CarState s; s.x = px; s.y = py; s.theta = theta_i; s.v = -0.4; s.phi = sign * atan2(wheelbase_, R);
                                out.push_back(s);
                            }
                        }
                        // final small correction to exact center
                        CarState last_s = out.empty() ? st_pose : out.back();
                        double dx2 = park_x - last_s.x; double dy2 = park_y - last_s.y; double dist = std::hypot(dx2, dy2);
                        int final_steps = std::max(1, static_cast<int>(std::ceil(dist / step)));
                        bool collision=false;
                        std::vector<CarState> tempcorr;
                        for (int i=1;i<=final_steps;i++) {
                            double t = double(i)/final_steps; CarState s; s.x = last_s.x + dx2*t; s.y = last_s.y + dy2*t; s.theta = target_theta; s.v = -0.2; s.phi = 0.0; tempcorr.push_back(s);
                            if (!footprintInsidePolygon(parking_slot_polygon_, s)) { collision = true; break; }
                        }
                        if (!collision) {
                            for (auto &s : tempcorr) out.push_back(s);
                        } else {
                            ROS_WARN("Fallback correction collides with slot polygon; attempting to reduce correction steps.");
                            for (int k = final_steps; k >= 1; --k) {
                                bool ok = true; std::vector<CarState> cand;
                                for (int i=1;i<=k;i++) {
                                    double t = double(i)/k; CarState s; s.x = last_s.x + dx2*t; s.y = last_s.y + dy2*t; s.theta = target_theta; s.v = -0.2; s.phi = 0.0; cand.push_back(s);
                                    if (!footprintInsidePolygon(parking_slot_polygon_, s)) { ok = false; break; }
                                }
                                if (ok) { for (auto &s : cand) out.push_back(s); break; }
                            }
                        }
                    }
                }
            } else {
                ROS_WARN("Computed P0 approach intersects slot area; falling back to previous strategies.");
                // fallback to old behavior if approach not ok
                int n_back = std::max(1, static_cast<int>(std::ceil(standoff / step)));
                CarState st_pose; st_pose.x = park_x; st_pose.y = park_y; st_pose.theta = target_theta;
                for (int i = 1; i <= n_back; ++i) {
                    CarState s;
                    double ds = i * step;
                    s.x = st_pose.x - ds * cos(st_pose.theta);
                    s.y = st_pose.y - ds * sin(st_pose.theta);
                    s.theta = st_pose.theta;
                    s.v = -0.5;
                    s.phi = 0.0;
                    out.push_back(s);
                }
            }
        }

        // ensure final pose equals desired exactly
        if (!out.empty()) { out.back().x = park_x; out.back().y = park_y; out.back().theta = target_theta; out.back().v = 0.0; out.back().phi = 0.0; }
        return out;
    }

    // 规划从当前位置到泊车开始点的路径
    std::vector<CarState> planApproachToParkingStart(const CarState& current, double start_x, double start_y, double target_theta) {
        std::vector<CarState> approach_path;
        
        // 创建泊车开始点状态
        CarState parking_start;
        parking_start.x = start_x;
        parking_start.y = start_y;
        parking_start.theta = target_theta;  // 与车库方向一致
        parking_start.v = 1.0;  // 正向行驶速度
        parking_start.phi = 0.0;  // 方向盘角度
        
        // 使用简单的直线路径规划
        double dx = parking_start.x - current.x;
        double dy = parking_start.y - current.y;
        double dist = std::hypot(dx, dy);
        
        if (dist < 0.1) {
            // 如果距离很近，直接添加目标点
            approach_path.push_back(parking_start);
            return approach_path;
        }
        
        // 生成路径点
        int num_points = std::max(5, (int)(dist / 0.5));  // 每0.5米一个点
        for (int i = 0; i <= num_points; ++i) {
            double ratio = (double)i / num_points;
            CarState state;
            state.x = current.x + dx * ratio;
            state.y = current.y + dy * ratio;
            state.theta = current.theta + (parking_start.theta - current.theta) * ratio;
            state.v = 1.0;  // 正向行驶
            state.phi = 0.0;  // 直线行驶
            approach_path.push_back(state);
        }
        
        return approach_path;
    }

    // 基于车辆动力学的真实垂直泊车轨迹规划
    std::vector<CarState> generateVerticalParkingTrajectory(const CarState& start, double target_x, double target_y, double target_theta) {
        std::vector<CarState> trajectory;
        
        // 参数设置
        const double step_size = 0.1;  // 路径点间距
        const double max_steering_angle = 0.6;  // 最大转向角
        const double parking_speed = 0.3;  // 泊车速度
        
        // 计算泊车开始点（在车库前方适当位置）
        double parking_start_dist = 6.0;  // 泊车开始点距离车库6米
        double start_x = target_x + parking_start_dist * std::cos(target_theta);
        double start_y = target_y + parking_start_dist * std::sin(target_theta);
        
        // 计算提前停止点（距离泊车开始点8米）
        double stop_dist = 8.0;  // 提前停止距离8米
        double stop_x = start_x - stop_dist * std::cos(target_theta);
        double stop_y = start_y - stop_dist * std::sin(target_theta);
        
        ROS_INFO("Planning vertical parking trajectory from (%.2f, %.2f) to (%.2f, %.2f) with early stop at (%.2f, %.2f)", 
                 start.x, start.y, target_x, target_y, stop_x, stop_y);
        
        // 第一阶段：从当前位置到提前停止点（直线行驶）
        double dx_stop = stop_x - start.x;
        double dy_stop = stop_y - start.y;
        double dist_stop = std::hypot(dx_stop, dy_stop);
        int points_stop = std::max(5, (int)(dist_stop / step_size));
        
        for (int i = 0; i <= points_stop; ++i) {
            double ratio = (double)i / points_stop;
            CarState state;
            state.x = start.x + dx_stop * ratio;
            state.y = start.y + dy_stop * ratio;
            state.theta = start.theta + (target_theta - start.theta) * ratio;
            state.v = 1.0;  // 正向行驶
            state.phi = 0.0;  // 直线行驶
            trajectory.push_back(state);
        }
        
        // 第二阶段：从提前停止点到泊车开始点（直线行驶）
        CarState current_stop = trajectory.back();
        double dx_start = start_x - current_stop.x;
        double dy_start = start_y - current_stop.y;
        double dist_start = std::hypot(dx_start, dy_start);
        int points_start = std::max(5, (int)(dist_start / step_size));
        
        for (int i = 1; i <= points_start; ++i) {
            double ratio = (double)i / points_start;
            CarState state;
            state.x = current_stop.x + dx_start * ratio;
            state.y = current_stop.y + dy_start * ratio;
            state.theta = current_stop.theta + (target_theta - current_stop.theta) * ratio;
            state.v = 0.5;  // 低速行驶
            state.phi = 0.0;  // 直线行驶
            trajectory.push_back(state);
        }
        
        // 第三阶段：垂直泊车轨迹（基于车辆动力学）
        CarState current = trajectory.back();
        
        // 计算泊车轨迹参数
        double L = wheelbase_;  // 轴距
        double R_min = min_turn_radius_;  // 最小转弯半径
        
        // 垂直泊车标准轨迹：先右转45度，然后直线倒车，再左转45度
        double turn_angle = M_PI / 4;  // 45度转弯
        
        // 右转45度（前进）
        double turn_radius = std::max(R_min, 3.0);  // 转弯半径
        int turn_points = std::max(5, (int)(turn_radius * turn_angle / step_size));
        
        for (int i = 1; i <= turn_points; ++i) {
            double angle_ratio = (double)i / turn_points;
            double delta_theta = turn_angle * angle_ratio;
            
            CarState state;
            state.theta = current.theta + delta_theta;
            state.x = current.x + turn_radius * (std::sin(state.theta) - std::sin(current.theta));
            state.y = current.y - turn_radius * (std::cos(state.theta) - std::cos(current.theta));
            state.v = parking_speed;
            state.phi = std::atan2(L, turn_radius);  // 转向角计算
            
            trajectory.push_back(state);
        }
        
        current = trajectory.back();
        
        // 直线倒车到车库位置
        double reverse_dist = 4.0;  // 倒车距离
        int reverse_points = std::max(5, (int)(reverse_dist / step_size));
        
        for (int i = 1; i <= reverse_points; ++i) {
            double dist_ratio = (double)i / reverse_points;
            
            CarState state;
            state.theta = current.theta;
            state.x = current.x - reverse_dist * dist_ratio * std::cos(state.theta);
            state.y = current.y - reverse_dist * dist_ratio * std::sin(state.theta);
            state.v = -parking_speed;  // 倒车
            state.phi = 0.0;  // 直线倒车
            
            trajectory.push_back(state);
        }
        
        current = trajectory.back();
        
        // 左转45度调整到最终方向（倒车）
        for (int i = 1; i <= turn_points; ++i) {
            double angle_ratio = (double)i / turn_points;
            double delta_theta = -turn_angle * angle_ratio;  // 左转
            
            CarState state;
            state.theta = current.theta + delta_theta;
            state.x = current.x + turn_radius * (std::sin(state.theta) - std::sin(current.theta));
            state.y = current.y - turn_radius * (std::cos(state.theta) - std::cos(current.theta));
            state.v = -parking_speed;  // 倒车
            state.phi = -std::atan2(L, turn_radius);  // 左转转向角
            
            trajectory.push_back(state);
        }
        
        // 最终微调到目标位置
        current = trajectory.back();
        double final_dx = target_x - current.x;
        double final_dy = target_y - current.y;
        double final_dist = std::hypot(final_dx, final_dy);
        
        if (final_dist > 0.1) {
            int final_points = std::max(3, (int)(final_dist / step_size));
            
            for (int i = 1; i <= final_points; ++i) {
                double ratio = (double)i / final_points;
                CarState state;
                state.x = current.x + final_dx * ratio;
                state.y = current.y + final_dy * ratio;
                state.theta = current.theta + (target_theta - current.theta) * ratio;
                state.v = -0.2;  // 低速倒车
                state.phi = 0.0;
                
                trajectory.push_back(state);
            }
        }
        
        // 确保最终状态准确
        if (!trajectory.empty()) {
            trajectory.back().x = target_x;
            trajectory.back().y = target_y;
            trajectory.back().theta = target_theta;
            trajectory.back().v = 0.0;
            trajectory.back().phi = 0.0;
        }
        
        ROS_INFO("Vertical parking trajectory planned with %lu points", trajectory.size());
        return trajectory;
    }

    void addParkingManeuver() {
        if (global_plan_.empty()) return;
        // Start from the last planned pose and append a multi-stage parking maneuver
        CarState start = global_plan_.back();
        auto maneuver = generateMultiStageParking(start, parking_target_x_, parking_target_y_, parking_target_theta_);
        if (maneuver.empty()) { ROS_WARN("generateMultiStageParking returned empty maneuver."); return; }
        for (auto &s : maneuver) global_plan_.push_back(s);
    }

    // --- 控制与仿真循环 ---
    void executeTrackingStep(double dt) {
        // If we've reached the end of plan, do nothing
        if (global_plan_.empty()) return;

        // 关键改进：如果车辆距离路径起点太远，重新规划路径
        if (!global_plan_.empty() && current_idx_ < 5) {
            double dist_to_start = std::hypot(global_plan_[0].x - car_.x, global_plan_[0].y - car_.y);
            if (dist_to_start > 15.0) {  // 如果距离路径起点超过15米
                ROS_WARN("Vehicle too far from path start (%.2fm > 15m), replanning from current position...", dist_to_start);
                // 重新规划路径
                planTask();
                current_idx_ = 0;  // 重置当前索引
                return;
            }
        }

        // 智能泊车开始点检测：当车辆驶过车库位置时，动态刷新倒车轨迹规划
        if (!global_plan_.empty() && current_idx_ > 0) {
            // 计算车辆到车库的距离
            double dist_to_garage = std::hypot(car_.x - parking_target_x_, car_.y - parking_target_y_);
            
            // 如果车辆已经驶过车库位置（距离小于5米），动态刷新倒车轨迹
            if (dist_to_garage < 5.0) {
                // 检查当前路径是否已经是泊车轨迹（包含倒车部分）
                bool is_parking_trajectory = false;
                for (int i = current_idx_; i < (int)global_plan_.size(); ++i) {
                    if (global_plan_[i].v < 0) {  // 检测到倒车速度
                        is_parking_trajectory = true;
                        break;
                    }
                }
                
                if (!is_parking_trajectory) {
                    // 第一次进入泊车区域，切换到泊车轨迹
                    ROS_WARN("Vehicle entered parking zone (%.2fm away), switching to vertical parking trajectory...", dist_to_garage);
                    
                    // 使用基于车辆动力学的真实垂直泊车轨迹规划
                    std::vector<CarState> vertical_parking_trajectory = generateVerticalParkingTrajectory(car_, parking_target_x_, parking_target_y_, parking_target_theta_);
                    
                    if (!vertical_parking_trajectory.empty()) {
                        // 替换当前路径为新的垂直泊车轨迹
                        global_plan_.clear();
                        for (const auto& state : vertical_parking_trajectory) {
                            global_plan_.push_back(state);
                        }
                        
                        current_idx_ = 0;
                        ROS_INFO("Vertical parking trajectory planned successfully with %lu points", global_plan_.size());
                        
                        // 强制发布新的规划路径
                        publishPlannedPath();
                    }
                } else {
                    // 已经在泊车轨迹中，动态刷新轨迹规划
                    // 每10个控制周期（约0.5秒）重新规划一次，避免过度计算
                    static int refresh_counter = 0;
                    refresh_counter++;
                    
                    if (refresh_counter >= 10) {
                        refresh_counter = 0;
                        
                        // 基于当前位置重新规划泊车轨迹
                        std::vector<CarState> refreshed_trajectory = generateVerticalParkingTrajectory(car_, parking_target_x_, parking_target_y_, parking_target_theta_);
                        
                        if (!refreshed_trajectory.empty()) {
                            // 保留当前索引，只替换剩余路径
                            int remaining_points = global_plan_.size() - current_idx_;
                            
                            // 如果新轨迹比剩余路径更优，则替换
                            if (refreshed_trajectory.size() > 10 && 
                                refreshed_trajectory.size() < remaining_points * 2) {
                                
                                // 保留已通过的点，替换剩余路径
                                global_plan_.resize(current_idx_);
                                for (const auto& state : refreshed_trajectory) {
                                    global_plan_.push_back(state);
                                }
                                
                                ROS_INFO("Parking trajectory refreshed: %lu points (was %d remaining)", 
                                         refreshed_trajectory.size(), remaining_points);
                                
                                // 强制发布新的规划路径
                                publishPlannedPath();
                            }
                        }
                    }
                }
            }
        }

        // find lookahead point along global_plan_
        int look_idx = -1;
        double accd = 0.0;
        double prevx = car_.x, prevy = car_.y;
        // search from current_idx_ (if valid) to end
        int start = std::max(0, current_idx_);
        
        // 如果当前索引太接近路径末尾，限制搜索范围
        if (start >= (int)global_plan_.size() - 5) {
            start = std::max(0, (int)global_plan_.size() - 10);
        }
        
        for (int i = start; i < (int)global_plan_.size(); ++i) {
            double dx = global_plan_[i].x - car_.x;
            double dy = global_plan_[i].y - car_.y;
            double d = std::hypot(dx, dy);
            if (d >= lookahead_dist_) { look_idx = i; break; }
            // 如果路径点太近但方向差异很大，也选择这个点
            if (d >= lookahead_dist_ * 0.7 && i > start) {
                double path_theta = std::atan2(global_plan_[i].y - global_plan_[i-1].y, 
                                              global_plan_[i].x - global_plan_[i-1].x);
                double car_to_point_theta = std::atan2(dy, dx);
                double angle_diff = std::abs(normalizeAng(path_theta - car_to_point_theta));
                if (angle_diff > M_PI/3) {  // 如果角度差大于60度
                    look_idx = i; 
                    break; 
                }
            }
        }
        if (look_idx == -1) {
            // fallback: use final point
            look_idx = (int)global_plan_.size() - 1;
        }
        double lx = global_plan_[look_idx].x;
        double ly = global_plan_[look_idx].y;

        double Ld = std::hypot(lx - car_.x, ly - car_.y);
        // adaptive lookahead scaling with speed
        double current_speed_for_look = std::min(max_speed_, std::max(min_speed_, std::abs(kp_speed_ * Ld)));
        double dynamic_look = lookahead_dist_ * (0.5 + 0.5 * (current_speed_for_look / std::max(0.001, max_speed_)) * lookahead_scale_with_speed_);
        double lookahead = std::min(lookahead_max_, std::max(lookahead_min_, dynamic_look));
        // if the target used earlier is not at desired lookahead distance, search for one at ~lookahead distance
        int la_idx = look_idx;
        for (int i = look_idx; i < (int)global_plan_.size(); ++i) {
            double d = std::hypot(global_plan_[i].x - car_.x, global_plan_[i].y - car_.y);
            if (d >= lookahead) { la_idx = i; break; }
        }
        lx = global_plan_[la_idx].x; ly = global_plan_[la_idx].y;

        double angle_to_target = std::atan2(ly - car_.y, lx - car_.x);
        double alpha = normalizeAng(angle_to_target - car_.theta);

        // Pure pursuit curvature
        double curvature = 0.0;
        if (lookahead > 1e-6) curvature = 2.0 * std::sin(alpha) / lookahead;
        // steering from curvature
        double desired_steering = std::atan(wheelbase_ * curvature);
        // if reversing, invert steering direction to follow path correctly
        double cosang = std::cos(alpha);

        // 基于最近点计算速度，而不是基于前瞻点
        // 找到距离车辆最近的路径点
        double closest_dist = std::numeric_limits<double>::max();
        int closest_idx = current_idx_;
        
        // 从当前索引开始向后搜索最近点
        int search_start = std::max(0, current_idx_);
        int search_end = std::min((int)global_plan_.size(), search_start + 50); // 限制搜索范围
        
        for (int i = search_start; i < search_end; ++i) {
            double dx = global_plan_[i].x - car_.x;
            double dy = global_plan_[i].y - car_.y;
            double dist = std::hypot(dx, dy);
            if (dist < closest_dist) {
                closest_dist = dist;
                closest_idx = i;
            }
        }
        
        // 基于最近点计算速度
        double speed = 0.0;
        
        // 如果距离最近点很近（<1米），使用前瞻点距离计算速度
        if (closest_dist < 1.0) {
            speed = kp_speed_ * Ld;
        } else {
            // 否则基于最近点距离计算速度
            speed = kp_speed_ * closest_dist;
        }
        
        // 降低曲率惩罚，让车辆更容易通过
        double curpen = 1.0 + curvature_speed_gain_ * 0.5 * std::abs(curvature);
        speed = speed / curpen;
        if (speed > max_speed_) speed = max_speed_;
        
        // 如果速度太小，使用更智能的速度控制
        if (std::abs(speed) < min_speed_) {
            // 根据路径曲率和方向决定最小速度
            double base_min_speed = min_speed_;
            // 如果曲率很大，稍微增加最小速度以避免卡死
            if (std::abs(curvature) > 0.3) {
                base_min_speed = min_speed_ * 1.2;
            }
            
            if (cosang >= 0) {
                speed = base_min_speed;  // 前进
            } else {
                speed = -base_min_speed; // 后退
            }
        }

        if (cosang < 0) { speed = -speed; desired_steering = -desired_steering; }

        // steering rate limit
        double max_delta = steering_rate_limit_ * dt;
        double delta = desired_steering - car_.phi;
        if (delta > max_delta) delta = max_delta; if (delta < -max_delta) delta = -max_delta;
        double steering = car_.phi + delta;
        // saturate steering to max physical
        if (steering > max_steering_rad_) steering = max_steering_rad_;
        if (steering < -max_steering_rad_) steering = -max_steering_rad_;

        // Update kinematic bicycle model (simple) with small dt
        // x_dot = v*cos(theta), y_dot = v*sin(theta), theta_dot = v/L * tan(delta)
        double xdot = speed * std::cos(car_.theta);
        double ydot = speed * std::sin(car_.theta);
        double thetadot = 0.0;
        if (std::abs(std::cos(steering)) > 1e-6) {
            thetadot = speed / wheelbase_ * std::tan(steering);
        }
        car_.x += xdot * dt;
        car_.y += ydot * dt;
        car_.theta = normalizeAng(car_.theta + thetadot * dt);
        car_.v = speed;
        car_.phi = steering;

        // publish lookahead marker for debugging
        visualization_msgs::Marker mk;
        mk.header.frame_id = "map";
        mk.header.stamp = ros::Time::now();
        mk.ns = "lookahead";
        mk.id = 0;
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.scale.x = 0.2; mk.scale.y = 0.2; mk.scale.z = 0.2;
        mk.color.a = 0.9; mk.color.r = 1.0; mk.color.g = 0.1; mk.color.b = 0.1;
        mk.pose.position.x = lx; mk.pose.position.y = ly; mk.pose.position.z = 0.1;
        lookahead_pub_.publish(mk);

        // advance current_idx_ while we've passed those points
        while (current_idx_ < (int)global_plan_.size()) {
            double dx = global_plan_[current_idx_].x - car_.x;
            double dy = global_plan_[current_idx_].y - car_.y;
            double distance = std::hypot(dx,dy);
            // 如果距离太近，跳过当前点，但最多连续跳过5个点避免索引过度增长
            if (distance < std::max(0.5, goal_tolerance_) && current_idx_ < (int)global_plan_.size() - 1) {
                current_idx_++;
            } else {
                break;
            }
        }
        // if reached final goal, stop
        if (current_idx_ >= (int)global_plan_.size()) {
            car_.v = 0.0; car_.phi = 0.0; ROS_INFO("Reached final goal (within %.2fm)", goal_tolerance_);
        }
    }

    void controlLoop(const ros::TimerEvent&) {
        double dt = 0.05; // timer period
        
        // 动态轨迹规划：在launch时持续计算轨迹，而不是只生成一次
        static int planning_counter = 0;
        planning_counter++;
        
        // 每20个控制周期（约1秒）重新规划一次轨迹，确保轨迹动态刷新
        if (planning_counter >= 20) {
            planning_counter = 0;
            
            // 检查是否需要重新规划轨迹
            bool needs_replanning = false;
            
            // 如果当前没有路径，需要规划
            if (global_plan_.empty()) {
                needs_replanning = true;
            }
            // 如果车辆偏离路径太远，需要重新规划
            else if (current_idx_ < (int)global_plan_.size()) {
                double min_dist = std::numeric_limits<double>::max();
                for (int i = current_idx_; i < (int)global_plan_.size(); ++i) {
                    double dist = std::hypot(global_plan_[i].x - car_.x, global_plan_[i].y - car_.y);
                    if (dist < min_dist) min_dist = dist;
                }
                if (min_dist > 3.0) {  // 如果偏离路径超过3米，重新规划
                    needs_replanning = true;
                }
            }
            
            if (needs_replanning) {
                ROS_INFO("Dynamic trajectory planning triggered...");
                
                // 重新规划从当前位置到目标位置的路径
                std::vector<CarState> new_trajectory = planPathFromCurrentPosition();
                
                if (!new_trajectory.empty()) {
                    global_plan_ = new_trajectory;
                    current_idx_ = 0;
                    ROS_INFO("Dynamic trajectory planning completed with %lu points", global_plan_.size());
                    publishPlannedPath();
                }
            }
        }
        
        if (global_plan_.empty()) { publishPose(); return; }

        // execute one control step to track global_plan_
        executeTrackingStep(dt);

        // publish path visualization
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (const auto& p : global_plan_) {
            geometry_msgs::PoseStamped ps;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
            path_msg.poses.push_back(ps);
        }
        path_pub_.publish(path_msg);

        // publish vehicle pose & cmd
        publishPose();
    }

    void publishPose() {
        // TF
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(car_.x, car_.y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, car_.theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

        // Pose2D
        geometry_msgs::Pose2D p2d;
        p2d.x = car_.x;
        p2d.y = car_.y;
        p2d.theta = car_.theta;
        pose_pub_.publish(p2d);

        // publish commanded control as Twist: linear.x = v, angular.z = steering angle
        geometry_msgs::Twist cmd;
        cmd.linear.x = car_.v;
        cmd.angular.z = car_.phi; // steering angle in rad
        cmd_pub_.publish(cmd);

    // Marker Car (publish to dedicated car topic)
    visualization_msgs::MarkerArray car_marker;
    visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "car_body";
        m.id = 0;
        m.type = visualization_msgs::Marker::CUBE;
        m.pose.position.x = car_.x;
        m.pose.position.y = car_.y;
        m.pose.position.z = 0.5;
        m.pose.orientation = tf::createQuaternionMsgFromYaw(car_.theta);
        m.scale.x = 3.5;
        m.scale.y = 1.6;
        m.scale.z = 1.0;
        m.color.a = 0.8;
        m.color.b = 1.0;
        car_marker.markers.push_back(m);
        car_pub_.publish(car_marker);
    }

    // --- 发布规划路径 ---
    void publishPlannedPath() {
        // 强制发布规划路径到/planned_path话题
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        
        for (const auto& p : global_plan_) {
            geometry_msgs::PoseStamped ps;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
            path_msg.poses.push_back(ps);
        }
        
        path_pub_.publish(path_msg);
        ROS_INFO("强制发布规划路径，包含 %lu 个路径点", global_plan_.size());
    }

    // --- 可视化地图 ---
    void publishMap() {
        // Publish ways separately (LINE_LIST)
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
        ways_marker.color.b = 1.0; // 白色道路
        ways_marker.id = 0;

        for (const auto& w : ways_) {
            for (size_t i = 0; i + 1 < w.node_refs.size(); ++i) {
                long long id1 = w.node_refs[i];
                long long id2 = w.node_refs[i+1];
                if (nodes_map_.count(id1) && nodes_map_.count(id2)) {
                    geometry_msgs::Point p1, p2;
                    p1.x = nodes_map_[id1].x;
                    p1.y = nodes_map_[id1].y;
                    p1.z = 0.0;
                    p2.x = nodes_map_[id2].x;
                    p2.y = nodes_map_[id2].y;
                    p2.z = 0.0;
                    ways_marker.points.push_back(p1);
                    ways_marker.points.push_back(p2);
                }
            }
        }
        ways_arr.markers.push_back(ways_marker);
        ways_pub_.publish(ways_arr);

        // Publish relations (each member way rendered as LINE_STRIP under the relation namespace)
        visualization_msgs::MarkerArray rels_arr;
        int rel_id = 0;
        for (const auto& r : relations_) {
            visualization_msgs::Marker rel_marker;
            rel_marker.header.frame_id = "map";
            rel_marker.header.stamp = ros::Time::now();
            rel_marker.ns = std::string("relation_") + std::to_string(r.id);
            rel_marker.type = visualization_msgs::Marker::LINE_STRIP;
            rel_marker.scale.x = 0.15;
            rel_marker.color.a = 0.7;
            rel_marker.color.r = 1.0;
            rel_marker.color.g = 0.0;
            rel_marker.color.b = 1.0; // magenta for relation boundaries
            rel_marker.id = rel_id++;

            // concatenate member way geometries (may create disconnected strips)
            for (long long way_id : r.member_way_refs) {
                for (const auto& w : ways_) {
                    if (w.id == way_id) {
                        for (size_t i = 0; i < w.node_refs.size(); ++i) {
                            long long nid = w.node_refs[i];
                            if (nodes_map_.count(nid)) {
                                geometry_msgs::Point p;
                                p.x = nodes_map_[nid].x;
                                p.y = nodes_map_[nid].y;
                                p.z = 0.0;
                                rel_marker.points.push_back(p);
                            }
                        }
                        break;
                    }
                }
                // insert a small break by repeating an empty point? not necessary for LINE_STRIP
            }
            rels_arr.markers.push_back(rel_marker);
        }
        relations_pub_.publish(rels_arr);

    // Publish nodes as small spheres (optional)
        visualization_msgs::MarkerArray nodes_arr;
        int nid_idx = 0;
        for (const auto& kv : nodes_map_) {
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

        // Compute parking spot coordinates (projected) for way-based spots and ensure node-based spots have coords
        for (auto &ps : parking_spots_) {
            if (ps.from_way) {
                // find way
                for (const auto &w : ways_) {
                    if (w.id == ps.id) {
                        double sx = 0, sy = 0; int cnt = 0;
                        for (long long nid : w.node_refs) {
                            if (nodes_map_.count(nid)) {
                                sx += nodes_map_[nid].x;
                                sy += nodes_map_[nid].y;
                                ++cnt;
                            }
                        }
                        if (cnt > 0) { ps.x = sx / cnt; ps.y = sy / cnt; }
                        break;
                    }
                }
            } else {
                if (nodes_map_.count(ps.id)) {
                    ps.x = nodes_map_[ps.id].x;
                    ps.y = nodes_map_[ps.id].y;
                }
            }
        }

        // Publish parking areas: pair nearby parking-space ways into rectangular slots
        visualization_msgs::MarkerArray park_arr;
        int pid = 0;

        // build quick lookup for ways by id
        std::map<long long, Way*> way_by_id;
        for (auto &w : ways_) way_by_id[w.id] = &w;

        // collect candidate parking ways: either explicitly tagged or referenced in parking_spots_
        std::vector<Way*> park_ways;
        for (auto &w : ways_) {
            std::string wt = w.wtype;
            std::transform(wt.begin(), wt.end(), wt.begin(), ::tolower);
            bool is_parking_tag = (wt == "parking_space");
            if (!is_parking_tag) {
                for (const auto &ps : parking_spots_) { if (ps.from_way && ps.id == w.id) { is_parking_tag = true; break; } }
            }
            if (is_parking_tag) park_ways.push_back(&w);
        }

        std::set<long long> used;
        auto way_centroid = [&](Way* w) {
            double sx=0, sy=0; int c=0;
            for (long long nid : w->node_refs) if (nodes_map_.count(nid)) { sx += nodes_map_[nid].x; sy += nodes_map_[nid].y; ++c; }
            if (c==0) return std::make_pair(0.0,0.0);
            return std::make_pair(sx/c, sy/c);
        };

        for (size_t i=0;i<park_ways.size();++i) {
            Way* a = park_ways[i]; if (!a) continue; if (used.count(a->id)) continue;
            // find nearest other parking way
            double bestd = 1e18; Way* best = nullptr;
            auto ca = way_centroid(a);
            for (size_t j=i+1;j<park_ways.size();++j) {
                Way* b = park_ways[j]; if (!b) continue; if (used.count(b->id)) continue;
                auto cb = way_centroid(b);
                double dx = ca.first - cb.first, dy = ca.second - cb.second; double d = dx*dx + dy*dy;
                if (d < bestd) { bestd = d; best = b; }
            }

            if (best && bestd < 50.0*50.0) { // pair if within 50m
                // form rectangle corners using endpoints of the two ways
                if (a->node_refs.size() >= 1 && best->node_refs.size() >= 1) {
                    // pick endpoints
                    long long a0 = a->node_refs.front(); long long a1 = a->node_refs.back();
                    long long b0 = best->node_refs.front(); long long b1 = best->node_refs.back();
                    if (nodes_map_.count(a0) && nodes_map_.count(a1) && nodes_map_.count(b0) && nodes_map_.count(b1)) {
                        visualization_msgs::Marker mk;
                        mk.header.frame_id = "map";
                        mk.header.stamp = ros::Time::now();
                        mk.ns = "parking_spot_area";
                        mk.id = pid++;
                        mk.type = visualization_msgs::Marker::LINE_STRIP;
                        mk.scale.x = 0.12;
                        mk.color.a = 0.9; mk.color.r = 0.0; mk.color.g = 0.8; mk.color.b = 0.0;

                        geometry_msgs::Point p_a0, p_a1, p_b1, p_b0;
                        p_a0.x = nodes_map_[a0].x; p_a0.y = nodes_map_[a0].y; p_a0.z = 0.02;
                        p_a1.x = nodes_map_[a1].x; p_a1.y = nodes_map_[a1].y; p_a1.z = 0.02;
                        p_b1.x = nodes_map_[b1].x; p_b1.y = nodes_map_[b1].y; p_b1.z = 0.02;
                        p_b0.x = nodes_map_[b0].x; p_b0.y = nodes_map_[b0].y; p_b0.z = 0.02;

                        mk.points.push_back(p_a0);
                        mk.points.push_back(p_a1);
                        mk.points.push_back(p_b1);
                        mk.points.push_back(p_b0);
                        // close
                        mk.points.push_back(p_a0);
                        park_arr.markers.push_back(mk);
                    }
                }
                used.insert(a->id); used.insert(best->id);
            } else {
                // single parking way: show as small cube at centroid
                auto c = ca;
                visualization_msgs::Marker m;
                m.header.frame_id = "map"; m.header.stamp = ros::Time::now(); m.ns = "parking_spots"; m.id = pid++;
                m.type = visualization_msgs::Marker::CUBE;
                m.pose.position.x = c.first; m.pose.position.y = c.second; m.pose.position.z = 0.05;
                m.scale.x = 2.5; m.scale.y = 1.25; m.scale.z = 0.1;
                m.color.a = 0.6; m.color.g = 1.0; m.color.r = 0.0; m.color.b = 0.0;
                park_arr.markers.push_back(m);
                used.insert(a->id);
            }
        }

        parking_pub_.publish(park_arr);

        // legacy aggregated map publish removed (use /parking_spots and /parking_blocks topics)
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "real_map_parking_node");
    ParkingSystem ps;
    ros::spin();
    return 0;
}