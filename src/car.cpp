#include <iostream>
#include <string>
#include "math.h"
#include "spline.h"
#include "helpers.h"
#include "car.h"

using std::pair;
using std::vector;
using nlohmann::json;

// Useful template for getting argmin of a vector
template <typename T, typename A>
int arg_min(vector<T, A> const& vec) {
    return static_cast<int>(std::distance(vec.begin(), min_element(vec.begin(), vec.end())));
}


// Initializes Vehicle
Car::Car(){}

Car::Car(int lane, double velocity) 
{
    this->lane = lane;
    this->velocity = velocity;
    
    car_front_speed = 0.0;
   
    // Initialise state variables
    // Assume ego wil not start directly behind another car
    // Left and right indicators updated from sensor as ego proceeeds along the track  
    car_in_front = false;
    car_to_left = false;
    car_to_right = false;
}

// Destructor
Car::~Car() {}


double Car::get_d()
{
    // Return d value halfway along lane width
    return (0.5 + lane) * lane_width;
}


void Car::update_predictions(const nlohmann::basic_json<>::value_type& data)
{         
     // Contains info on all cars around the track
     auto sensor_fusion = data["sensor_fusion"];
     double current_s = data["s"];   
     auto prev_path_size = data["previous_path_x"].size();
  
     // Set indicators to default
     car_in_front = false;
     car_to_left = false;
     car_to_right = false;
  
     for (auto vehicle_data : sensor_fusion)
     {
         double vehicle_s = vehicle_data[5];       
         
         // Focus on vehicles currently nearby
         if(fabs(vehicle_s - current_s) < 2 * collision_tol)
         {
             double vehicle_d = vehicle_data[6]; 
             int vehicle_lane = static_cast<int>(vehicle_d / lane_width);
             
             double vehicle_vx = vehicle_data[3];
             double vehicle_vy = vehicle_data[4];
             double vehicle_speed = hypot(vehicle_vx, vehicle_vy);
           
             // Update to get the predicted location of where the vehicle will be
             vehicle_s += vehicle_speed * 0.02 * prev_path_size;
           
             // Check car in front
             if((vehicle_lane == lane) && (vehicle_s > current_s) && ((vehicle_s - current_s) < collision_tol))
             { 
                 car_in_front = true;
                 car_front_speed = vehicle_speed;
             }
             
             // Check car to left
             if((vehicle_lane == lane - 1) && (fabs(current_s < vehicle_s) < collision_tol))
             {
                 car_to_left = true;
             }
             
             // Check car to right
             if((vehicle_lane == lane + 1) && (fabs(current_s < vehicle_s) < collision_tol))
             {
                 car_to_right = true;
             }
         }
     }
}


vector<pair<int, double>> Car::generate_successor_trajectories()
{
  // Successor tragectory container
  vector<pair<int, double>> successors;
  
  // Keeping same lane and same / higher speed if nothing ahead
  if(!car_in_front)
      successors.push_back(std::make_pair(lane, fmin(velocity + 0.224, max_velocity)));
  
  // If car in front can stay in current lane and slow down to speed of car in front with 20% buffer
  if(car_in_front)
      successors.push_back(std::make_pair(lane, fmax(velocity - 0.224, 0.8 * car_front_speed)));
      
  // Change lane left, avoid changing lanes at very slow speed or top speed
  if((velocity > 0.2 * max_velocity) && !car_to_left && (lane != 0))
      successors.push_back(std::make_pair(lane - 1, fmin(velocity, max_velocity - 0.056)));
       
  // Change lane right, avoid changing lanes at very slow speed or top speed
  if((velocity > 0.2 * max_velocity) && !car_to_right && (lane != 2))
      successors.push_back(std::make_pair(lane + 1, fmin(velocity, max_velocity - 0.056)));
  
  return successors;
}


std::pair<int, double> Car::get_lowest_cost_trajectory(vector<pair<int, double>> trajectories)
{
    vector<double> cost_vector;
    double cost;
  
    for (auto trajectory : trajectories)
    {
        // New lane and associated velocity
        int lane = trajectory.first;
        double vel = trajectory.second;
        cost = 0.0;
        cost += acc_cost_weight * acceleration_cost(vel);
        cost += speed_cost_weight * speed_cost(vel);
        cost += lane_cost_weight * lane_cost(lane);
        cost_vector.push_back(cost);
    }        
    return trajectories[arg_min(cost_vector)];    
}


// Cost against exceeding max acceleration
double Car::acceleration_cost(double new_velocity)
{
    double acceleration = (new_velocity - velocity) / 0.02;
    return std::abs(acceleration) > max_acceleration ? 1.0 : std::abs(acceleration) / max_acceleration;
}


// Cost against exceeding speed limit or driving too slowly
double Car::speed_cost(double new_velocity)
{
    return new_velocity > max_velocity ? 1.0 : (max_velocity - new_velocity) / max_velocity;
}


// Cost to prefer getting into middle lane
double Car::lane_cost(double new_lane)
{
    return static_cast<double>(fabs(new_lane - 1)); 
}


vector<vector<double>> Car::create_trajectory(const nlohmann::basic_json<>::value_type& data, 
    const vector<double> map_waypoints_x, const vector<double> map_waypoints_y, const vector<double> map_waypoints_s)
{
    double car_x = data["x"];
    double car_y = data["y"];
    double car_s = data["s"];
    double car_d = data["d"];
    double car_yaw = data["yaw"];
    double car_speed = data["speed"];

    // Previous path data given to the Planner
    auto previous_path_x = data["previous_path_x"];
    auto previous_path_y = data["previous_path_y"];  
    auto prev_path_size = previous_path_x.size();
  
    // Previous path's end s and d values 
    double end_path_s = data["end_path_s"];
    double end_path_d = data["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion = data["sensor_fusion"];
  
    // List of widely spaced points
    vector<double> ptsx, ptsy;
          
    // Reference points
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);
          
    if(prev_path_size < 2)
    {    
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);
        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }    
    // Redefine reference state as previous path and state
    else 
    {  
        ref_x = previous_path_x[prev_path_size - 1];
        ref_y = previous_path_y[prev_path_size - 1];
        double ref_x_prev = previous_path_x[prev_path_size - 2];
        double ref_y_prev = previous_path_y[prev_path_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);  
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }
    
    int segment_length = static_cast<int>(collision_tol);
      
    for (int i = segment_length; i <= 3 * segment_length; i += segment_length)
    {   
        vector<double> next_wp = getXY(car_s + i, this->get_d(), map_waypoints_s, map_waypoints_x, map_waypoints_y);    
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }
          
    // Convert to car co-ordinates
    for (auto i = 0; i < ptsx.size(); i++)
    {    
        double shift_x = ptsx[i] - ref_x; 
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw);
        ptsy[i] = shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw);
    }
          
    // Create spline
    tk::spline s;
        
    s.set_points(ptsx, ptsy);
          
    // Vectors that will be used in planner
    vector<double> next_x_vals, next_y_vals;
          
    for (auto i=0; i < prev_path_size; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
          
    double target_x = static_cast<double>(segment_length);
    double target_y = s(target_x);
    double target_dist = hypot(target_x, target_y);
    double x_add_on = 0;
          
    // Add new points to path planner
    for (int i = 1; i <= 50 - prev_path_size; i++)
    {
        double N = target_dist / (0.02 * velocity / 2.24);
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);
            
        x_add_on = x_point;            
        double x_ref = x_point;
        double y_ref = y_point;
            
        // Rotate back to lane coordinates
        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            
        x_point += ref_x;
        y_point += ref_y;
            
        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point); 
    }
    return {next_x_vals, next_y_vals};
}
