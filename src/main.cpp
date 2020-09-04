#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "math.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "car.h"

// For convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::pair;


// Checks if the SocketIO event has JSON data
// If there is data the JSON object in string format will be returned, else the empty string will be returned
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  
  // Initialise car state
  // (lane, velocity) 
  Car ego(1, 0.0);
  
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  
  while (getline(in_map_, line)) 
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  h.onMessage([&ego, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") 
      {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") 
        {  
          // Use sensor data to update the state         
          ego.update_predictions(j[1]);
          
          // Generate possible trajectories
          vector<pair<int, double>> successor_trajectories = ego.generate_successor_trajectories();
          
          // Get lowest cost trajectory
          pair<int, double> successor = ego.get_lowest_cost_trajectory(successor_trajectories);
          
          // Update ego lane and velocity
          ego.lane = successor.first;
          ego.velocity = successor.second;
         
          // Build trajectory and feed back to simulator
          vector<vector<double>> next_vals = ego.create_trajectory(j[1], map_waypoints_x, map_waypoints_y, map_waypoints_s);
          
          json msgJson;
         
          msgJson["next_x"] = next_vals[0];
          msgJson["next_y"] = next_vals[1];

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}