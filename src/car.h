#ifndef CAR_H
#define CAR_H

#include <vector>
#include "json.hpp"

using std::vector;
using std::pair;

class Car {
    
    public:
  
    // Constructors
    Car();
    Car(int lane, double velocity);

    // Destructor
    virtual ~Car();

    // Functions
    double get_d();
   
    void update_predictions(const nlohmann::basic_json<>::value_type& data);
    
    vector<pair<int, double>> generate_successor_trajectories();
    
    pair<int, double> get_lowest_cost_trajectory(vector<pair<int, double>> trajectories);
    
    vector<vector<double>> create_trajectory(const nlohmann::basic_json<>::value_type& data, const vector<double> map_waypoints_x,         const vector<double> map_waypoints_y, const vector<double> map_waypoints_s);
  
  
    // Cost functions
    double acceleration_cost(double new_velocity);
    double speed_cost(double new_velocity);
    double lane_cost(double new_lane);
  
  
    // Variables
    int lane;
    bool car_in_front, car_to_left, car_to_right;
    double velocity, car_front_speed;
 
  
    private:
 
    static constexpr double lane_width = 4.0;
    static constexpr double collision_tol = 45.0;
    
    static constexpr double max_velocity = 49;
    static constexpr double max_acceleration = 9.5;
    static constexpr double max_jerk = 9.5;
  
    static constexpr double speed_cost_weight = 30.0;
    static constexpr double acc_cost_weight = 50.0;
    static constexpr double lane_cost_weight = 15.0;
};

#endif  // CAR_H