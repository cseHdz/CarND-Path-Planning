#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h" 
#include <math.h>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
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
  
  // Start in the middle lane
  int lane = 1;
  
  // Reference Velocity to target
  double ref_vel = 0.0; // (mph)

  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Number of remaining points in the path
          int prev_size = previous_path_x.size();
          
           // Check if there are points left on our path
           if (prev_size > 0) {
             car_s = end_path_s;
           }
          
          /**********************************************************
            Identify whether there are vehicles in the vicinity and 
            if they are blocking our path, pass them if safe
          **********************************************************/
          
          // Flag to identify if there are vehicles too close to the ego vehicle
          bool too_close = false;
          bool safe_pass_right = true;
          bool safe_pass_left = true;
          
          // Determine if there are cars within the immediate vicinity of the ego car.
          // If straight ahead - determine if it's safe to pass on the right or on the left.
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
            
            float d = sensor_fusion[i][6];
            int car_lane = floor(d/4);
             
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            // Get the expected location of the ego car in 2 ms
            check_car_s += ((double)prev_size*0.02*check_speed);
            
            double check_s_dist = check_car_s - car_s;
            
            // Check whethere a nearby vehicle is within 20m the ego vechicle
            if (fabs(check_s_dist) < 20){
              
              // There is a car within 30 min in our lane
              if ((car_lane == lane) && check_car_s > car_s){
                too_close = true;   
              } 
              else if (car_lane == lane - 1){
                // PASS LEFT
                safe_pass_left = false;
              } 
              else if (car_lane == lane + 1){
                // PASS RIGHT
                safe_pass_right = false;
              }
              
            }
          }
                 
          double MAX_VEL = 49.4;
          double MAX_ACC = 0.224; // Avoid Jerk
          
          // Attempt to pass the car
          // If yes, and no car right, pass right
          // If yes, and car right, and no car left, pass left
          // If yes, and car right, and car left, slow down
          if (too_close){
            
            // Always prefer to pass on the left
            if (safe_pass_left && lane > 0){
              lane--;
            }
            else if (safe_pass_right && lane < 2){
              lane++;
            }
            else{
              ref_vel -= MAX_ACC;
            }
          } else if (ref_vel < MAX_VEL){
           
            ref_vel += MAX_ACC;         
            ref_vel = fmin(ref_vel, MAX_VEL);
          }
            
          
         /**********************************************************
         	Add new points to the trajectory
         **********************************************************/
                
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Reference state of the car
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Leverage existing points in the trajectory for continuity
          if ( prev_size < 2 ) {
            
            // If there are only a few points left - just use the current reference          
            // Calculate a hypothetical point tangent to the path (same yaw)
            
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
            
          } else {
            
            // Use the next two points from the existing list as reference
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            
            // Calculate the direction of travel implied between both points
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Add a desired number of points to the patj using Frenet coordinates
          int n_points = 3;
          int dist = 30;
          
          for (int i = 0; i < n_points; ++i){       
            
            vector<double> next_wp = getXY(car_s + dist * (i + 1), 2 + 4*lane, 
                                           map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            ptsx.push_back(next_wp[0]);
            ptsy.push_back(next_wp[1]);
          }
          
          /**********************************************************
          	Interpolate points to ensure a smooth trajectory.
            The implementation below uses a spline between points with zero angle.              
          **********************************************************/
          
          
          for ( int i = 0; i < ptsx.size(); i++ ) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
            
              // Rotate the points to align with the car's frame of reference
              // Instead of dealing with yea, angle becomes 0.

              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }
          
          // Initialize the spline with the desired points
          tk:: spline s;
          s.set_points(ptsx, ptsy);
                   
          // Points in (x,y) that will be used for trajectory planning
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Initiliaze the vectors with points remaining from the previous path
          for ( int i = 0; i < prev_size; i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Split the points so that they are spread with the desired distance
          // and travelling at the desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0;
         
          // Ensure there is always 50 points on the trajectory
          // Add as many points as needed to reach 50
          for( int i = 1; i < 50 - prev_size; i++ ) {
            
            // Determine the number of points based on distance divided by velocity
            // Note: Velociy has to be converted to km/h
            double N = target_dist/(0.02 * ref_vel / 2.24);
            
            // Add points with distance spread evenly
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Convert points back to global coordinates - Inverse rotation from before
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            // Shift points back from (0,0) to the car's current coordinates
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          /**********************************************************
            Add next (x,y) points to the trajectory
           ***********************************************************/
           
          json msgJson;
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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