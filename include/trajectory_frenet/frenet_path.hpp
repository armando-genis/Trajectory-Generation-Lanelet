#pragma once

#include <vector>
#include <climits>

namespace trajectory_frenet {
class FrenetPath
{
  public:
    std::vector<std::vector<double>> s;   // longitudinal data
    std::vector<std::vector<double>> d;   // lateral data
    std::vector<std::vector<double>> world;  // world coordinates {x,y,yaw, length of tiny segment}
    
    double T;
    double total_cost;                        
    double lateral_jerk_cost; 
    double longitudinal_jerk_cost;            
    double max_velocity = INT_MIN;
    double max_acceleration = INT_MIN; 
    double max_curvature = INT_MIN; 
};
} // namespace trajectory_frenet