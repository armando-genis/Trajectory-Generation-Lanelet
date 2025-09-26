#include <opencv2/opencv.hpp>
#include <cmath>
#include <stdexcept>

#include "trajectory_frenet/frenet_planner.hpp"
#include "trajectory_frenet/planner_params.hpp"

#include "trajectory_frenet/GlobalPlanner.hpp"

using namespace trajectory_frenet;

cv::Point2i offset(float x, float y, int image_width=1400, int image_height=800){
  cv::Point2i output;
  // Adjusted scaling and offset for smaller image size
  output.x = image_width - (int(x*10) + 50);  // Reduced scaling from 100 to 10
  output.y = image_height - int(y * 10) - image_height/4;  // Adjusted vertical offset
  return output;
}

Eigen::Vector2d rotation(double theta, double x, double y) {
	Eigen::Matrix2d rotationZ;
	rotationZ << std::cos(theta), -1*std::sin(theta),
							 std::sin(theta), std::cos(theta);
	Eigen::Vector2d point;
	point << x, y;
	return rotationZ*point;
}

int main(int argc, char** argv) {

    // lanelet map 
    std::shared_ptr<GlobalPlanner> global_planner_;
    std::string map_path_ = "/workspace/navpilot_ws/src/hdmap_stack/hdmap_visualizer/osms/1_new66.osm";
    double x_offset_ = 0.0;
    double y_offset_ = 25.0;
    int start_lanelet_id_ = 3848;
    int end_lanelet_id_ = 31;

    global_planner_ = std::make_shared<GlobalPlanner>(x_offset_, y_offset_, map_path_, start_lanelet_id_, end_lanelet_id_);

    std::vector<point_struct> all_waypoints_from_global_planner_;
    all_waypoints_from_global_planner_ = global_planner_->getAllAllWaypointsStruct();

    CrosswalkRender crosswalkR = global_planner_->getCrosswalkRender();
    LaneletRender laneletR = global_planner_->getLaneletRender();
  
    // 1) Construct the planner
    TrajectoryPlanner planner;

    // 2) Build center lane 
    std::vector<std::vector<double>> center_lane;
    const double a = 0.051,    
                b = 3.45,     
                c = 2.05;     
    double distance_traced = 0.0, prevX = 0.0;
    double prevY = 4.2*(std::sin(std::pow((prevX*a - b),2) + (prevX*a - b) + c) + 1);  


    for (double x = 0; x < 150; x += 0.01) {  
        
      double y   = 4.2*(std::sin(std::pow((x*a - b),2) + (x*a - b) + c) + 1);
      double dy  = 4.2*std::cos(std::pow(x*a - b,2) + (x*a - b) + c)*((2*a*(x*a - b)) + a);
      double ddy = 4.2*((-std::sin(std::pow(x*a - b,2) + (x*a - b) + c)*(std::pow((2*a*(x*a - b) + a),2)))
                + (std::cos(std::pow(x*a - b,2) + (x*a - b) + c)*(2*a*a)));
      double curvature = std::abs(ddy) / std::pow(1 + dy*dy, 1.5);
      double yaw = dy;
      distance_traced += std::hypot(x - prevX, y - prevY);
      center_lane.push_back({ x, y, yaw, curvature, distance_traced });
      prevX = x;  prevY = y;
    }

    cv::namedWindow("Trajectory Planner", cv::WINDOW_NORMAL);

    cv::VideoWriter writer(
      "trajectory.mp4",
      cv::VideoWriter::fourcc('m','p','4','v'),
      30.0,
      cv::Size(1400, 800)          // ← match new image size
    );
    if (!writer.isOpened()) {
      std::cerr << "ERROR: cannot open video writer\n";
      return -1;
    }

    auto y_base = [&](double x){
        return 4.2 * (std::sin(std::pow((x*a - b), 2)
                          + (x*a - b)
                          + c) + 1);
    };

    // Initialize moving obstacles (traffic vehicles)
    struct MovingObstacle {
        double s_position;     // longitudinal position along the road
        double lane_id;        // which lane (0=center, -1=left, +1=right, etc.)
        double velocity;       // speed of the obstacle
        double initial_s;      // starting position
    };
    
    std::vector<MovingObstacle> moving_obstacles = {
        // Slower traffic in different lanes
        {20.0, -1, 6.0/3.6, 20.0},    // Left lane, slower
        {40.0,  0, 8.0/3.6, 40.0},    // Center lane  
        {60.0,  1, 7.0/3.6, 60.0},    // Right lane
        {80.0, -2, 5.0/3.6, 80.0},    // Far left lane, slowest
        {100.0, 2, 9.0/3.6, 100.0},   // Far right lane
        {35.0,  1, 6.5/3.6, 35.0},    // Right lane, slower
        {90.0, -1, 7.5/3.6, 90.0},    // Left lane
    };
    
    std::vector<std::vector<double>> obstacles;

    // 4) Initial Frenet state - start in center lane
    SamplingParams sp;
    double d0 = 0.0;  // Start in center lane (reference path)
    double dv0 = 0.0; 
    double da0 = 0.0;
    double s0 = 0.0; 
    double sv0 = 10.0/3.6;
    
    LateralState  lat{d0, dv0, da0};
    LongitudinalState lon{s0, sv0};
    
    double x = 0.0;
	  double y = 0.0;
    double simulation_time = 0.0;
    const double dt = 0.1;  // time step for simulation
    
    // Cache parameters outside the loop for performance
    SamplingParams sp_cached;
    std::vector<double> lane_positions_cached = sp_cached.getLanePositions();

    // 5) Main planning loop
    while (true) {
        // Update moving obstacles
        obstacles.clear();
        
        for (auto& obs : moving_obstacles) {
            // Update obstacle position
            obs.s_position += obs.velocity * dt;
            
            // Reset obstacle if it goes too far ahead
            if (obs.s_position > 150.0) {
                obs.s_position = obs.initial_s - 20.0;
            }
            
            // Convert to world coordinates
            // Find the corresponding point on the center lane
            for (int i = 0; i < center_lane.size(); ++i) {
                if (center_lane[i][4] >= obs.s_position) {
                    double obs_x = center_lane[i][0];
                    double obs_y = center_lane[i][1];
                    
                    // Apply lane offset
                    int lane_index = obs.lane_id + 2; // Convert from -2,-1,0,1,2 to 0,1,2,3,4
                    if (lane_index >= 0 && lane_index < lane_positions_cached.size()) {
                        double lane_offset = lane_positions_cached[lane_index];
                        obs_y += lane_offset;
                    }
                    
                    obstacles.push_back({obs_x, obs_y});
                    break;
                }
            }
        }
        
        simulation_time += dt;
        std::vector<FrenetPath> allPaths;
        FrenetPath p = planner.plan(lat, lon, center_lane, obstacles, allPaths);

        // validate planner output
        if (p.d.size() < 2 || p.s.size() < 2 || p.world.size() < 2) {
          std::cerr << "Planner returned insufficient data\n";
          break;
        }
        // advance along the chosen path
        d0  = p.d[1][0];  
        dv0 = p.d[1][1];  
        da0 = p.d[1][2];
        s0  = p.s[1][0];  
        sv0 = p.s[1][1];

        lat = {d0, dv0, da0};
        lon = {s0, sv0};
        
        x = p.world[1][0];
        y = p.world[1][1];

        // 2) early-stop based on planner’s s
        double cur_s    = p.s[1][0];
        double total_s  = center_lane.back()[4];
        double early_stop = 5.0;
        if (cur_s >= total_s - early_stop) {
          break;
        }

        // visualize - much smaller image for better performance
        cv::Mat lane(800, 1400, CV_8UC3, cv::Scalar(255, 255, 255));



        // (A) Draw crosswalk bases (light gray)
        for (const auto& poly : crosswalkR.base) {
          std::vector<cv::Point> screen;
          screen.reserve(poly.size());
          for (const auto& pt : poly) {
            screen.push_back(offset(pt.x, pt.y, lane.cols, lane.rows)); // your existing helper
          }
          const std::vector<std::vector<cv::Point>> pts{screen};
          cv::fillPoly(lane, pts, cv::Scalar(200, 200, 200)); // BGR
        }

        // (B) Draw zebra stripes (darker gray)
        for (const auto& quad : crosswalkR.stripes) {
          std::vector<cv::Point> screen;
          screen.reserve(quad.size());
          for (const auto& pt : quad) {
            screen.push_back(offset(pt.x, pt.y, lane.cols, lane.rows));
          }
          const std::vector<std::vector<cv::Point>> pts{screen};
          cv::fillPoly(lane, pts, cv::Scalar(150, 150, 150));
        }

        // (C) Draw every lane boundary (thin white lines)
        for (const auto& bound : laneletR.lane_bounds) {
          std::vector<cv::Point> screen;
          screen.reserve(bound.size());
          for (const auto& pt : bound) {
            screen.push_back(offset(pt.x, pt.y, lane.cols, lane.rows));
          }
          if (screen.size() >= 2) {
            const std::vector<std::vector<cv::Point>> pts{screen};
            cv::polylines(lane, pts, /*isClosed*/false, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
          }
        }



        
        // Highway lanes (draw all 5 lanes with lane markings)
        for (double lane_offset : lane_positions_cached) {
            for(int i{1}; i<center_lane.size(); i++){
                double x1 = center_lane[i-1][0];
                double y1 = center_lane[i-1][1] + lane_offset;
                double x2 = center_lane[i][0];
                double y2 = center_lane[i][1] + lane_offset;
                
                // Draw solid lines for outer lanes, dashed for inner lanes
                if (lane_offset == lane_positions_cached.front() || lane_offset == lane_positions_cached.back()) {
                    // Outer highway boundaries - solid white lines
                    cv::line(lane, offset(x1, y1, lane.cols, lane.rows), 
                            offset(x2, y2, lane.cols, lane.rows), cv::Scalar(200, 200, 200), 2);
                } else if (lane_offset == 0.0) {
                    // Center reference line - solid yellow
                    cv::line(lane, offset(x1, y1, lane.cols, lane.rows), 
                            offset(x2, y2, lane.cols, lane.rows), cv::Scalar(0, 255, 255), 2);
                } else {
                    // Lane dividers - dashed white lines
                    if (i % 8 < 4) { // Create dashed effect
                        cv::line(lane, offset(x1, y1, lane.cols, lane.rows), 
                                offset(x2, y2, lane.cols, lane.rows), cv::Scalar(150, 150, 150), 1);
                    }
                }
            }
        }

        // Traffic vehicles (obstacles)
        for(int i{0}; i<obstacles.size(); i++){
          auto obs_center = offset(obstacles[i][0], obstacles[i][1], lane.cols, lane.rows);
          
          // Draw vehicle as a rectangle (simulating car shape) - scaled down
          cv::Rect vehicle_rect(obs_center.x - 12, obs_center.y - 5, 24, 10);
          cv::rectangle(lane, vehicle_rect, cv::Scalar(50, 50, 200), -1);  // Dark red filled
          cv::rectangle(lane, vehicle_rect, cv::Scalar(0, 0, 0), 1);       // Black border
          
          // Add simple headlight/taillight indication
          cv::circle(lane, cv::Point(obs_center.x + 9, obs_center.y - 2), 2, cv::Scalar(255, 255, 255), -1);
          cv::circle(lane, cv::Point(obs_center.x + 9, obs_center.y + 2), 2, cv::Scalar(255, 255, 255), -1);
        }

        // Robot (ego vehicle) - scaled down
        cv::line(lane, 
                 offset(p.world[0][0]+rotation(p.world[0][2],-1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,0.75)(1), 
                        lane.cols, lane.rows),
                 offset(p.world[0][0]+rotation(p.world[0][2],1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,0.75)(1), 
                        lane.cols, lane.rows), 
                 cv::Scalar(0, 0, 0), 3);
        cv::line(lane, 
                 offset(p.world[0][0]+rotation(p.world[0][2],-1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,-0.75)(1), 
                        lane.cols, lane.rows),     
                 offset(p.world[0][0]+rotation(p.world[0][2],1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,-0.75)(1), 
                        lane.cols, lane.rows), 
                 cv::Scalar(0, 0, 0), 3);
        cv::line(lane, 
                 offset(p.world[0][0]+rotation(p.world[0][2],1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,0.75)(1), 
                        lane.cols, lane.rows),
                 offset(p.world[0][0]+rotation(p.world[0][2],1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,-0.75)(1), 
                        lane.cols, lane.rows), 
                 cv::Scalar(0, 0, 0), 3);
        cv::line(lane, 
                 offset(p.world[0][0]+rotation(p.world[0][2],-1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,0.75)(1), 
                        lane.cols, lane.rows),
                 offset(p.world[0][0]+rotation(p.world[0][2],-1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,-0.75)(1), 
                        lane.cols, lane.rows), 
                 cv::Scalar(0, 0, 0), 3);

        // All Trajectories (draw every 3rd trajectory to reduce rendering load)
        for(int traj_idx = 0; traj_idx < allPaths.size(); traj_idx += 3){
          FrenetPath &t = allPaths[traj_idx];
          for(int i{2}; i<t.world.size(); i+=2){ // Skip every other point for performance
            cv::line(lane, offset(t.world[i-2][0], t.world[i-2][1], lane.cols, lane.rows), 
                    offset(t.world[i][0], t.world[i][1], lane.cols, lane.rows), cv::Scalar(255, 0, 0), 1);
          }
        }

        // Trajectory (selected path)
        for(int i{1}; i<p.world.size(); i++){
          cv::line(lane, offset(p.world[i-1][0], p.world[i-1][1], lane.cols, lane.rows), offset(p.world[i][0], p.world[i][1], lane.cols, lane.rows), cv::Scalar(0, 255, 0), 3);
        }
        
        writer.write(lane);  // No need to resize since we're already using optimal size

        cv::resizeWindow("Trajectory Planner", 1400, 800);
        cv::imshow("Trajectory Planner", lane);
        cv::waitKey(33);  // ~30 FPS instead of unlimited frame rate
    }
    writer.release();
    return 0;
}
