#include <assert.h> // #include <assert> does not work, why?

#include <iomanip>

#include <fstream>

#include <iostream>

#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "spline.h"
#include "parameters.h"
#include "utils.h"
#include "data_model.h"
#include "kinematics.h"
#include "maneuvers.h"
#include "trajectory.h"

using namespace std;

// for convenience
using json = nlohmann::json;


int main() {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
  
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    ifstream in_map_(map_file_.c_str(), ifstream::in);
  
    string line;
    while (getline(in_map_, line)) {
    	istringstream iss(line);
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
  // double ref_val = MAX_VELOCITY_DELTA_PRE_PLANNING_INTERVAL; // initial
  Car my_car;
  my_car.a = 0;
  my_car.jerk = 0;
  

  int update_count = 0; // used to debug to capture the first trace
  uWS::Hub h;
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx,
               &map_waypoints_dy, &my_car, &update_count]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
          {
            // j[1] is the data JSON object
            // My car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"]; // in degree
            double car_speed = j[1]["speed"]; // in mile per hour
        
            // Previous path data given to the Planner
            // actually they are the remaining points of trajectory not yet visited by the car
            // they are issued by the path planner to the car in the previous control time
            auto remaining_path_x = j[1]["previous_path_x"];
            auto remaining_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            double remaining_path_end_s = j[1]["end_path_s"];
            // not yet used, keep for documentation purpose
            double remaining_path_end_d = j[1]["end_path_d"];
            // not yet used, keep might be needed
        
            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];
        
            // Assemble information to call trajectory_f:
            my_car.id = -1; // hopefully impossible id of the other cars
            my_car.x  = car_x;
            my_car.y  = car_y;
            my_car.yaw = deg2rad(car_yaw);
            
            double old_v = my_car.v;
            my_car.v  = mph_2_meterps(car_speed);
            my_car.s  = wrap_around(car_s);
            my_car.d  = car_d;
            my_car.lane_index = d_to_lane_index(car_d);
            
            double old_a = my_car.a;
            my_car.a = (my_car.v - old_v)/UPDATE_INTERVAL;
            
            my_car.jerk = (my_car.a - old_a)/UPDATE_INTERVAL;
            
            my_car.remaining_path_end_s = wrap_around(remaining_path_end_s);
            my_car.remaining_path_end_d = remaining_path_end_d;
            
            ios::fmtflags old_settings = cout.flags();
            cout.precision(5);
            
            cout << "car_s|d|v: " << setw(7) << car_s << "|" << setw(7) << car_d
            << "|"<< setw(5) << my_car.v << "; ";
            
            // << " car_x|y: " << setw(7)<< car_x << " | " << setw(7)<< car_y << " remaining_path_end_s|d: "<< setw(7)
            // << remaining_path_end_s << " | " << setw(7)<< remaining_path_end_d << " car_speed (meters/s) " << mph_2_meterps(car_speed)
            // << endl;
            
            // cout << "car_s: " << car_s << ", car_{x, y}: " << car_x << ", " << car_y << " remaining_path_end_{s, d}: "
            //      << remaining_path_end_s << ", " << remaining_path_end_d << " car_speed (meters/s) " << mph_2_meterps(car_speed)
            //      << endl;
            TRAJECTORY remaining_trajectory;
            // cout << "rem. p_{x, y}_len: " << remaining_path_x.size() << ", " << remaining_path_y.size() << ", ";
            // transfer to the remaining trajectory from auto type to pair of double<vector>, otherwise, the compiler reject
            // the vector assginment.
            // cout << endl;
            // cout << "remaining x: ";
            for (auto x:remaining_path_x) {
              remaining_trajectory.x_vals.push_back(x);
              // cout << setw(6) << x << ", ";
             }
            
            //cout << endl;
            //cout << "remaining y: ";
            for (auto y:remaining_path_y) {
              remaining_trajectory.y_vals.push_back(y);
              // cout << setw(6) << y << ", ";
             }
            
            // cout << endl;
            
            // Fix and refine the waypoint maps to improve the resolution of computing
            
            WAYPOINTS_MAP refined_maps = refine_maps_f(my_car,
                                                       map_waypoints_x, map_waypoints_y, map_waypoints_s,
                                                       map_waypoints_dx, map_waypoints_dy);
            TRAJECTORY trajectory
            = trajectory_f(my_car, sensor_fusion, remaining_trajectory, refined_maps);
        
            json msgJson;
            msgJson["next_x"] = trajectory.x_vals;
            msgJson["next_y"] = trajectory.y_vals;
        
            auto msg = "42[\"control\","+ msgJson.dump()+"]";
        
            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
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
