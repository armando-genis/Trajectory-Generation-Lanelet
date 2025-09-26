#include "trajectory_frenet/frenet_planner.hpp"
#include "trajectory_frenet/planner_params.hpp"
#include "trajectory_frenet/frenet_path.hpp"
#include "trajectory_frenet/polynomial.hpp"

#include <cmath>
#include <iostream>


namespace trajectory_frenet {
TrajectoryPlanner::TrajectoryPlanner() {
  // Initialize cached parameters once
  lane_positions_ = sp_.getLanePositions();
}

TrajectoryPlanner::~TrajectoryPlanner() {}

FrenetPath TrajectoryPlanner::plan(const LateralState& lateral_state,
                                  const LongitudinalState& longitudinal_state,
                                  std::vector<std::vector<double>>& center_lane,
                                  std::vector<std::vector<double>>& obstacles,
                                  std::vector<FrenetPath>& all_paths) {

  std::vector<FrenetPath> candidate_paths;
  generateCandidatePaths(lateral_state, longitudinal_state, candidate_paths);

  convertToWorldCoordinates(candidate_paths, center_lane);

  // First, check validity of FULL paths (before truncation)
  std::vector<FrenetPath> feasible_paths;
  feasible_paths = getValidPaths(candidate_paths, obstacles);
  
  // Then truncate only the feasible paths at obstacles (for traffic following)
  for (auto& path : feasible_paths) {
    truncatePathAtObstacles(path, obstacles);
  }

  // get the best path out of all feasible paths based on cost
  FrenetPath best_path;
  
  // If no feasible paths found, return the best candidate path (before collision/constraint filtering)
  if (feasible_paths.empty()) {
    if (!candidate_paths.empty()) {
      // Find the path with minimum cost among all candidates
      double min_cost = std::numeric_limits<double>::max();
      for (const auto& path : candidate_paths) {
        if (path.total_cost < min_cost) {
          min_cost = path.total_cost;
          best_path = path;
        }
      }
    }
    all_paths = std::move(candidate_paths);
    return best_path;
  }

  all_paths = std::move(candidate_paths);
  
  double total_cost = std::numeric_limits<double>::max();
  for (auto& path : feasible_paths) {
    if (total_cost >= path.total_cost) {
      total_cost = path.total_cost;
      best_path = path;
    }
  }
  return best_path;
}

void TrajectoryPlanner::generateCandidatePaths(const LateralState& lateral_state,
                                               const LongitudinalState& longitudinal_state,
                                               std::vector<FrenetPath>& candidate_paths) {
  
  candidate_paths.clear();
  candidate_paths.reserve(75); // Pre-allocate for expected number of paths (5 lanes × 3 velocities × 5 time horizons)
  
  // Iterate over prediction horizon range 
  for (double T = sp_.min_prediction_limit; T < sp_.max_prediction_limit; T += sp_.horizon_step) {

    // Iterate over discrete highway lanes
    for (double lat_offset : lane_positions_) {
      double lat_speed = 0.0, lat_acceleration = 0.0;
      
      std::vector<std::vector<double>> lat_traj;
      Eigen::Matrix<double, 6, 1> lat_boundaries;
      lat_boundaries << 
        lateral_state.offset,             // Initial lateral offset
        lateral_state.speed,              // Initial lateral speed
        lateral_state.acceleration,       // Initial lateral acceleration
        lat_offset,                       // Target lateral offset
        lat_speed,                        // Target lateral speed
        lat_acceleration;                 // Target lateral acceleration
      Polynomial lat_traj_poly(lat_boundaries, T);
      
      // Iterate over time steps for a given lane and a given time horizon 
      // to generate lateral trajectory points 
      double lateral_jerk_cost = 0.0;
      lat_traj.reserve(int(T / sp_.time_step) + 1); // Pre-allocate
      for (double t = 0; t <= T; t += sp_.time_step) {
        auto p = lat_traj_poly.evaluate_position(t);
        auto v = lat_traj_poly.evaluate_velocity(t);
        auto a = lat_traj_poly.evaluate_acceleration(t);
        auto j = lat_traj_poly.evaluate_jerk(t);
        lateral_jerk_cost += j * j;
        lat_traj.push_back({p, v, a, j, t});
      }
      
      // Iterate over longitudinal velocity range for a given lane and a given time horizon
      for (double lon_speed = sp_.target_velocity - sp_.velocity_step;
                  lon_speed <= sp_.target_velocity + sp_.velocity_step;
                  lon_speed += sp_.velocity_step) {
        
        FrenetPath candidate;
        candidate.T = T;
        candidate.d = lat_traj;
        candidate.lateral_jerk_cost = lateral_jerk_cost;

        std::vector<std::vector<double>> lon_traj;
        Eigen::Matrix<double, 5, 1> lon_boundaries;
        
        double lon_acceleration = 0.0;
        lon_boundaries <<
          longitudinal_state.distance,      // Initial longitudinal distance
          longitudinal_state.speed,         // Initial longitudinal speed
          0.0,                              // Initial longitudinal acceleration  
          lon_speed,                        // Target longitudinal speed      
          lon_acceleration;                 // Target longitudinal acceleration
        
        Polynomial lon_traj_poly(lon_boundaries, T);

        // Iterate over time steps for a given longitudinal velocity and a lateral trajectory
        // to generate longitudinal trajectory points
        double longitudinal_jerk_cost = 0.0;
        lon_traj.reserve(int(T / sp_.time_step) + 1); // Pre-allocate
        for (double t = 0; t <= T; t += sp_.time_step) {
          auto p = lon_traj_poly.evaluate_position(t);
          auto v = lon_traj_poly.evaluate_velocity(t);
          auto a = lon_traj_poly.evaluate_acceleration(t);
          auto j = lon_traj_poly.evaluate_jerk(t);
          longitudinal_jerk_cost += j * j;
          if (v > candidate.max_velocity) {
            candidate.max_velocity = v;
          }
          if (a > candidate.max_acceleration) {
            candidate.max_acceleration = a;
          }
          lon_traj.push_back({p, v, a, j, t});
        }
        
        candidate.s = lon_traj;
        candidate.longitudinal_jerk_cost = longitudinal_jerk_cost;
        
        computeCost(candidate);
        candidate_paths.push_back(candidate);
      }
    }
  }
}

void TrajectoryPlanner::convertToWorldCoordinates(std::vector<FrenetPath>& candidate_paths,
                                                  const std::vector<std::vector<double>>& center_lane) {

  // calculate x,y in world frame
  for (auto& candidate : candidate_paths) {
    int j = 0;
    for (int i = 0; i < candidate.s.size(); ++i) {
      double x, y, yaw;
      for (; j < center_lane.size(); ++j) {
        if (std::abs(candidate.s[i][0] - center_lane[j][4]) <= 0.1) {
          x = center_lane[j][0] + candidate.d[i][0] * std::cos(center_lane[j][2] + 1.57);
          y = center_lane[j][1] + candidate.d[i][0] * std::sin(center_lane[j][2] + 1.57);
          break;
        }
      }
      candidate.world.push_back({x,y,0,0});
    }
    
    // calculate yaw in world frame
    for (int i = 0; i < candidate.world.size()-1; ++i) {
      candidate.world[i][2] = std::atan2((candidate.world[i+1][1] - candidate.world[i][1]),
                                         (candidate.world[i+1][0] - candidate.world[i][0]));

      candidate.world[i][3] = std::hypot((candidate.world[i+1][0] - candidate.world[i][0]),
                                         (candidate.world[i+1][1] - candidate.world[i][1]));
    }
    candidate.world[candidate.world.size()-1][2] = candidate.world[candidate.world.size()-2][2];
    candidate.world[candidate.world.size()-1][3] = candidate.world[candidate.world.size()-2][3];

    // calculate maximum curvature for the trajectory
    double curvature = std::numeric_limits<double>::min();
    for (int i = 0; i < candidate.world.size()-1; ++i) {
      double temp = std::abs(candidate.world[i+1][2] - candidate.world[i][2]) / (candidate.world[i][3]);
      if (curvature < temp) {
        curvature = temp;
      }
    }
    candidate.max_curvature = curvature;
  }
}

std::vector<FrenetPath> TrajectoryPlanner::getValidPaths(const std::vector<FrenetPath>& candidate_paths, 
                                                         const std::vector<std::vector<double>>& obstacles) {
  std::vector<FrenetPath> feasible_paths;
  int collision_failures = 0;
  int kinematic_failures = 0;
  
  for (auto& candidate : candidate_paths) {
    bool has_collision = checkFullPathCollision(candidate, obstacles);
    bool violates_constraints = !checkKinematicConstraints(candidate);
    
    if (has_collision) collision_failures++;
    if (violates_constraints) kinematic_failures++;
    
    if (!has_collision && !violates_constraints) {
      feasible_paths.push_back(candidate);
    }
  }
  
  // Debug information (reduced frequency)
  static int debug_counter = 0;
  if (feasible_paths.empty() && (debug_counter++ % 30 == 0)) {
    std::cout << "No feasible paths found! Total candidates: " << candidate_paths.size() 
              << ", Collision failures: " << collision_failures 
              << ", Kinematic failures: " << kinematic_failures << std::endl;
  }
  
  return feasible_paths;
}

bool TrajectoryPlanner::checkCollision(const FrenetPath& candidate_path, 
                                       const std::vector<std::vector<double>>& obstacles) {
  // Simple collision check for truncated paths (used after truncation for final validation)
  const double r2 = (vp_.robot_footprint * 0.8) * (vp_.robot_footprint * 0.8);

  // Early exit if no obstacles
  if (obstacles.empty()) return false;
  
  // Check only the actual path points (no lookahead needed since paths are already filtered)
  for (const auto& wp : candidate_path.world) {
      double x = wp[0], y = wp[1];

      for (const auto& obs : obstacles) {
          double dx = x - obs[0];
          double dy = y - obs[1];

          if (dx*dx + dy*dy <= r2) {
              return true;
          }
      }
  }

  return false;
}

bool TrajectoryPlanner::checkFullPathCollision(const FrenetPath& candidate_path, 
                                               const std::vector<std::vector<double>>& obstacles) {
  // This checks if the FULL intended path has collisions (used for initial filtering)
  const double r2 = vp_.robot_footprint * vp_.robot_footprint;

  // Early exit if no obstacles
  if (obstacles.empty()) return false;
  
  // Check each point along the full trajectory
  for (const auto& wp : candidate_path.world) {
      double x = wp[0], y = wp[1];

      for (const auto& obs : obstacles) {
          double dx = x - obs[0];
          double dy = y - obs[1];

          if (dx*dx + dy*dy <= r2) {
              return true; // Collision found
          }
      }
  }

  return false; // No collisions on the full path
}

bool TrajectoryPlanner::checkKinematicConstraints(const FrenetPath& candidate_path) const {
  bool vel_ok = candidate_path.max_velocity <= vp_.max_velocity;
  bool acc_ok = candidate_path.max_acceleration <= vp_.max_acceleration;
  bool curv_ok = candidate_path.max_curvature <= vp_.max_curvature;
  
  // Debug output for failed constraints (reduced frequency)
  static int constraint_debug_counter = 0;
  if ((!vel_ok || !acc_ok || !curv_ok) && (constraint_debug_counter++ % 100 == 0)) {
    std::cout << "Kinematic constraint violation: "
              << "vel=" << candidate_path.max_velocity << "/" << vp_.max_velocity << " "
              << "acc=" << candidate_path.max_acceleration << "/" << vp_.max_acceleration << " "
              << "curv=" << candidate_path.max_curvature << "/" << vp_.max_curvature << std::endl;
  }
  
  return vel_ok && acc_ok && curv_ok;
}

void TrajectoryPlanner::truncatePathAtObstacles(FrenetPath& path, const std::vector<std::vector<double>>& obstacles) {
  if (obstacles.empty() || path.world.empty()) return;
  
  const double safety_distance = vp_.robot_footprint + 1.0; // Increased safety margin
  const double r2 = safety_distance * safety_distance;
  
  // Find the first collision point
  size_t collision_index = path.world.size(); // Default: no collision
  
  for (size_t i = 0; i < path.world.size(); ++i) {
    double x = path.world[i][0];
    double y = path.world[i][1];
    
    for (const auto& obs : obstacles) {
      double dx = x - obs[0];
      double dy = y - obs[1];
      
      if (dx*dx + dy*dy <= r2) {
        collision_index = i;
        break;
      }
    }
    
    if (collision_index < path.world.size()) break;
  }
  
  // If collision found, truncate the path before the collision point
  if (collision_index < path.world.size() && collision_index > 3) { // Keep at least 3 points for direction
    // Truncate with some buffer before the obstacle
    size_t truncate_at = std::max(3UL, collision_index - 2); // 2-point buffer before collision
    
    // Truncate all trajectory components
    path.world.resize(truncate_at);
    path.s.resize(truncate_at);
    path.d.resize(truncate_at);
    
    // Update the time horizon to match truncated path
    if (!path.s.empty()) {
      path.T = path.s.back()[4]; // time is stored in the 5th element
    }
    
    // Recompute cost for the truncated path
    computeCost(path);
  }
}

void TrajectoryPlanner::computeCost(FrenetPath& path) {
  
  // Lateral cost terms
  const double terminal_lateral_offset = path.d.back()[0];
  const double lateral_jerk_penalty     = cw_.kjd * path.lateral_jerk_cost;
  const double lateral_time_penalty     = cw_.ktd * path.T;
  
  // Find the closest lane center for lane-keeping preference
  double min_distance_to_lane = std::numeric_limits<double>::max();
  for (double lane_pos : lane_positions_) {
    double distance = std::abs(terminal_lateral_offset - lane_pos);
    if (distance < min_distance_to_lane) {
      min_distance_to_lane = distance;
    }
  }
  
  // Penalize deviation from lane centers more heavily
  const double lateral_deviation_penalty = cw_.ksd * (min_distance_to_lane * min_distance_to_lane);

  const double lateral_cost =
        lateral_jerk_penalty
      + lateral_time_penalty
      + lateral_deviation_penalty;

  // Longitudinal cost terms
  const double initial_longitudinal_pos = path.s.front()[0];
  const double final_longitudinal_pos   = path.s.back()[0];
  const double longitudinal_jerk_penalty= cw_.kjs * path.longitudinal_jerk_cost;
  const double longitudinal_time_penalty= cw_.kts * path.T;
  const double longitudinal_error_penalty = cw_.kss * ((initial_longitudinal_pos - final_longitudinal_pos) 
                                                   * (initial_longitudinal_pos - final_longitudinal_pos));

  const double longitudinal_cost = longitudinal_jerk_penalty
                                 + longitudinal_time_penalty
                                 + longitudinal_error_penalty;

  // Total cost = weighted sum
  path.total_cost = cw_.klat * lateral_cost + cw_.klon * longitudinal_cost;
}

} // namespace trajectory_frenet