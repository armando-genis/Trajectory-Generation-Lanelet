#ifndef GLOBAL_PLANNER_HPP
#define GLOBAL_PLANNER_HPP

#include <opencv2/opencv.hpp>

// lanelet libraries
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <boost/optional/optional_io.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <set>


using namespace lanelet;
using namespace std;

struct point_struct { double x, y, heading; 
                    int priority;};

// ---- Render containers (no ROS types) ----
struct CrosswalkRender {
  std::vector<std::vector<cv::Point2d>> base;     // base fill polygons
  std::vector<std::vector<cv::Point2d>> stripes;  // zebra stripe quads
};

struct LaneletRender {
  std::vector<std::vector<cv::Point2d>> lane_bounds; // every left/right bound polyline
};

// ---- small geometry helpers ----
static inline double segLen2d(const lanelet::ConstPoint3d& a, const lanelet::ConstPoint3d& b) {
  const double dx = b.x() - a.x();
  const double dy = b.y() - a.y();
  return std::hypot(dx, dy);
}

static double polylineLength2d(const lanelet::ConstLineString3d& ls) {
  double L = 0.0;
  for (size_t i = 0; i + 1 < ls.size(); ++i) L += segLen2d(ls[i], ls[i+1]);
  return L;
}

// Return 2D point at arclength s along a 3D LineString (in XY)
static cv::Point2d pointAtS2d(const lanelet::ConstLineString3d& ls, double s) {
  if (ls.empty()) return cv::Point2d();
  if (s <= 0.0)   return cv::Point2d(ls.front().x(), ls.front().y());

  double acc = 0.0;
  for (size_t i = 0; i + 1 < ls.size(); ++i) {
    double seg = segLen2d(ls[i], ls[i+1]);
    if (acc + seg >= s) {
      double r = (s - acc) / (seg > 1e-9 ? seg : 1.0);
      double x = ls[i].x() + r * (ls[i+1].x() - ls[i].x());
      double y = ls[i].y() + r * (ls[i+1].y() - ls[i].y());
      return {x, y};
    }
    acc += seg;
  }
  return cv::Point2d(ls.back().x(), ls.back().y());
}

// Convert a Lanelet boundary to cv polyline
static std::vector<cv::Point2d> toPolyline2d(const lanelet::ConstLineString3d& ls) {
  std::vector<cv::Point2d> out; out.reserve(ls.size());
  for (const auto& p : ls) out.emplace_back(p.x(), p.y());
  return out;
}

class GlobalPlanner
{
private:

    // colors for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    //global planner parameters
    double waypoint_interval = 0.5;
    int start_lanelet_id_ = 0;
    int end_lanelet_id_ = 0;
    double x_offset_ = 0.0;
    double y_offset_ = 0.0;
    std::string map_path_ = "";

    CrosswalkRender crosswalkR_;
    LaneletRender  laneletR_;

    void map_routing(lanelet::LaneletMapPtr &map);

    std::vector<std::vector<point_struct>> neighbor_points_;  // point if the neighbor lanelet
    std::vector<point_struct> all_waypoints_;  // point if the neighbor lanelet

    // get paths and neighbors
    void generateNeighborWaypoints(lanelet::LaneletMapPtr &map, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath);
    bool isBeyondTarget(const lanelet::ConstLanelet &lanelet, const routing::LaneletPath &shortestPath);
    bool isBranchingLanelet(const lanelet::ConstLanelet &path_lanelet, const lanelet::ConstLanelet &candidate_lanelet);
    bool isCompatibleTrajectory(const lanelet::ConstLanelet &path_lanelet, const lanelet::ConstLanelet &candidate_lanelet, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath, lanelet::LaneletMapPtr &map);
    int countMeaningfulConnections(const lanelet::ConstLanelet &candidate_lanelet, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath, int current_path_index, lanelet::LaneletMapPtr &map);
    std::pair<double, double> getEndDirection(const std::vector<lanelet::ConstPoint3d> &points);
    std::pair<double, double> getStartDirection(const std::vector<lanelet::ConstPoint3d> &points);
    double calculatePathLength(const routing::LaneletPath &path);
    double calculateRemainingPathLength(const routing::LaneletPath &path, int start_index);
    std::vector<point_struct> getAllWaypointsStruct() const;

public:
    GlobalPlanner(double x_offset, double y_offset, std::string map_path, int start_lanelet_id, int end_lanelet_id);
    ~GlobalPlanner();
    std::vector<point_struct> getAllAllWaypointsStruct();
    void build_lanelet_renderables( const lanelet::LaneletMapPtr& map, CrosswalkRender& cw, LaneletRender& lr);
    CrosswalkRender getCrosswalkRender() const;
    LaneletRender getLaneletRender() const;
};

#endif // GLOBAL_PLANNER_HPP