#include "parking_demo/osm_map_loader.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

namespace parking_demo {

OSMMapLoader::OSMMapLoader() {}
OSMMapLoader::~OSMMapLoader() {}

bool OSMMapLoader::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open OSM file: %s", filename.c_str());
        return false;
    }

    nodes_.clear();
    ways_.clear();
    relations_.clear();

    std::string line;
    while (std::getline(file, line)) {
        // Parse nodes
        if (line.find("<node") != std::string::npos) {
            NodePoint n;
            n.id = std::stoll(extractAttribute(line, "id"));
            n.lat = std::stod(extractAttribute(line, "lat"));
            n.lon = std::stod(extractAttribute(line, "lon"));

            // Check for parking_space tag in inline or multi-line format
            if (line.find("/>") == std::string::npos) {
                std::string sub;
                while (std::getline(file, sub) && sub.find("</node>") == std::string::npos) {
                    if (sub.find("<tag") != std::string::npos) {
                        std::string v = extractAttribute(sub, "v");
                        if (v == "parking_space") {
                            ParkingSpot ps;
                            ps.id = n.id;
                            ps.from_way = false;
                            parking_spots_.push_back(ps);
                        }
                    }
                }
            }
            nodes_[n.id] = n;
        }
        // Parse ways
        else if (line.find("<way") != std::string::npos) {
            Way w;
            w.id = std::stoll(extractAttribute(line, "id"));

            while (std::getline(file, line) && line.find("</way>") == std::string::npos) {
                if (line.find("<nd") != std::string::npos) {
                    w.node_refs.push_back(std::stoll(extractAttribute(line, "ref")));
                }
                if (line.find("<tag") != std::string::npos) {
                    std::string k = extractAttribute(line, "k");
                    std::string v = extractAttribute(line, "v");
                    if (k == "relation") w.relation = v;
                    if (k == "subtype") w.subtype = v;
                    if (k == "type") w.wtype = v;
                    if (v == "parking_space") {
                        ParkingSpot ps;
                        ps.id = w.id;
                        ps.from_way = true;
                        parking_spots_.push_back(ps);
                    }
                }
            }
            ways_.push_back(w);
        }
        // Parse relations
        else if (line.find("<relation") != std::string::npos) {
            Relation r;
            r.id = std::stoll(extractAttribute(line, "id"));

            while (std::getline(file, line) && line.find("</relation>") == std::string::npos) {
                if (line.find("<member") != std::string::npos) {
                    std::string type = extractAttribute(line, "type");
                    if (type == "way") {
                        std::string ref = extractAttribute(line, "ref");
                        if (!ref.empty()) {
                            r.member_way_refs.push_back(std::stoll(ref));
                        }
                    }
                }
                if (line.find("<tag") != std::string::npos) {
                    std::string k = extractAttribute(line, "k");
                    std::string v = extractAttribute(line, "v");
                    if (k == "type") r.type = v;
                }
            }
            relations_.push_back(r);
        }
    }

    ROS_INFO("✅ Loaded %lu nodes, %lu ways, %lu relations",
             nodes_.size(), ways_.size(), relations_.size());
    return true;
}

void OSMMapLoader::projectToLocal() {
    if (nodes_.empty()) {
        ROS_WARN("No nodes to project");
        return;
    }

    // Use first node as origin
    auto it = nodes_.begin();
    origin_lat_ = it->second.lat;
    origin_lon_ = it->second.lon;

    const double R = 6378137.0;  // Earth radius in meters
    const double lat_rad = origin_lat_ * M_PI / 180.0;

    for (auto& pair : nodes_) {
        double dlat = pair.second.lat - origin_lat_;
        double dlon = pair.second.lon - origin_lon_;

        // Simple mercator projection
        pair.second.x = R * (dlon * M_PI / 180.0) * std::cos(lat_rad);
        pair.second.y = R * (dlat * M_PI / 180.0);
    }

    ROS_INFO("✅ Projected to local coordinates (origin: lat=%.6f, lon=%.6f)",
             origin_lat_, origin_lon_);
}

void OSMMapLoader::extractParkingSpots() {
    // Compute (x,y) for parking spots
    for (auto& ps : parking_spots_) {
        if (ps.from_way) {
            // Find way and compute centroid
            for (const auto& w : ways_) {
                if (w.id == ps.id) {
                    double sx = 0, sy = 0;
                    int count = 0;
                    for (long long nid : w.node_refs) {
                        if (nodes_.count(nid)) {
                            sx += nodes_[nid].x;
                            sy += nodes_[nid].y;
                            ++count;
                        }
                    }
                    if (count > 0) {
                        ps.x = sx / count;
                        ps.y = sy / count;
                    }
                    break;
                }
            }
        } else {
            // Node-based parking spot
            if (nodes_.count(ps.id)) {
                ps.x = nodes_[ps.id].x;
                ps.y = nodes_[ps.id].y;
            }
        }
    }

    ROS_INFO("✅ Extracted %lu parking spots", parking_spots_.size());
}

std::string OSMMapLoader::extractAttribute(const std::string& line, const std::string& key) {
    std::string search = key + "=\"";
    size_t start = line.find(search);

    if (start == std::string::npos) {
        // Try single quotes
        search = key + "='";
        start = line.find(search);
        if (start == std::string::npos) return "";
    }

    start += search.length();
    size_t end = line.find_first_of("\"'", start);
    return line.substr(start, end - start);
}

} // namespace parking_demo
