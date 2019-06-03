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

  // Define the order of lanes
  int lane = 1; 

  // Define the reference of velocity 
  double ref_v = 0;  // mph

  h.onMessage([&ref_v, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int prev_size = previous_path_x.size(); 

          /*
          * -------------------------------SENSOR FUSION--------------------------------------------- 
          * 1. we will look around the ego vehicle, and check the status of other vehicles on the road
          * 2. specifically, we look at if the car's in front of us, left to us or right to us. 
          */

          // re-assign value to car_s if previous path was not empty 
          if (prev_size > 0 ){
            car_s = end_path_s; 
          }
          // define a flag for closeness check 
          bool too_close = false; 

          // define a flag for the relative postion of the car
          // car_front: the car is in front of us, and it is too close 
          // car_left: the car is to the left of us, and a lane change can not be excuted. 
          // car_right: the car is to the right of us, and a lane change can not be excuted. 
          bool car_front = false;  
          bool car_left = false; 
          bool car_right = false; 

          // loop over the sensor function result to check other vehicle's behavior 
          for (int i = 0; i < sensor_fusion.size(); i++){
            // check if the car is current in my lane, initialize car_lane as "-1"
            // | --- | --- | --- | (each | represets a lane, in our case, we use the range of d to determine the lane position)
            int car_lane = -1; 
            
            // read in the lateral position of the car "d"
            float d = sensor_fusion[i][6]; 

            // determine the car_lane using logic 
            if (d > 0 && d < 4){
              car_lane = 0; 
            } else if (d > 4 && d < 8){
              car_lane = 1; 
            } else if (d > 8 && d < 12){
              car_lane = 2; 
            }
            
            // note that if the car's d is not with the range of (0, 12), we don't need to think about it
            if (car_lane < 0) continue; 

            // next we need to compute the speed of the car "v" & and its longitudinal displacement "s"
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy); 
            double check_car_s = sensor_fusion[i][5];   

            // predict the future car's position after executing previous trajecotry 
            check_car_s += (double)prev_size*0.02*check_speed;

            // update the flags of the relative position 

            if (car_lane == lane){
              // the vehicle is in the same lane as the EGO vehicle 
              car_front |= check_car_s > car_s && check_car_s - car_s < 30; 
            } else if (car_lane - lane == -1){
              // the vehicle is to the left of the EGO vehicle
              car_left |=  check_car_s > car_s - 30 && check_car_s < car_s + 30; 
            } else if (car_lane - lane == 1){
              // the vehicle is to the right of the EGO vehicle
              car_right |= check_car_s > car_s - 30 && check_car_s < car_s + 30; 
            }
          }

          // ---------------------------------- PART II: Behavior Planning ------------------------------------------
          // case A: there is a slow vehicle in front of us
          if (car_front){
            // case A-1: we are not on the leftmost lane and the left lane is empty 
            if (!car_left && lane > 0){
              lane --; // we do a left lane chang since it is usually faster 
            } else if (!car_right && lane < 2){
            // case A-2: we are not on the rightmost lane and the right lane is empty  
              lane ++; 
            } else {
            // case A-3: if both left and right lanes are occupied by vehicles, we have to slow down
              ref_v -= 0.224;   
            }
          // case B: there is no vehicle in front of us, we just need to drive normally  
          } else {
            // the EGO car is not on the center lane and its good to do a lane change. 
            if (lane != 1 && ((lane == 0 && !car_right) || (lane == 2 && !car_left))){
              lane = 1; 
            }
            if (ref_v < 49.5){
              ref_v += 0.224; 
            }
          }

          // ---------------------------------- PART III: Trajectory Generation --------------------------------------
          // we first created some widely spread (x,y) waypoints to generate an outline, and later
          // using a spline to interpolate the waypoints in between. 
          vector<double> ptsx; 
          vector<double> ptsy; 

          // define the reference of the vehicle for all states, x, y and yaw
          // the reference values could either be the starting point of the vehicle, or the end point of last path. 
          double ref_x = car_x;
          double ref_y = car_y; 
          double ref_yaw = deg2rad(car_yaw); 

          // if the previous path is almost empty, we use the car's starting point as reference
          if (prev_size < 2){
            // assume the two consecutive points have the same yaw angle. 
            double prev_car_x = car_x - cos(ref_yaw); 
            double prev_car_y = car_y - sin(ref_yaw); 

            // push back both current and starting trajectory point 
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y); 
          }else{ // using the previous trajectory's end point as reference
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1]; 

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2]; 
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev); 

            // push the points back to the waypoints list 
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y); 
          }

          // In frenet frame, add points to the path roughly 30 meters to have a previw horizon of the trajectory
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Then, for each point in the trajectory list, we convert it into local frame. 
          for (int i = 0; i<ptsx.size(); i++){
            // shift the cars location from global coordinates to local coordinates 
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y; 

            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);  
          }

          // Then, we create a spline to interpolate the waypoints between each sparse waypoints 
          tk::spline s; 

          // Set waypoints created to the spline
          s.set_points(ptsx,ptsy);
          
          // Start with all the previous waypoint from previous published trajectory
          for (int i=0; i < prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // we then calculated how we split our waypoints in order to traveral at desired speed
          double target_x = 30;  
          double target_y = s(target_x); 
          double target_dist = sqrt(pow(target_x,2) + pow(target_y,2)); 

          // define an x value incrementer 
          double x_add_on = 0; 

          // Fill the rest of the waypoints
          for (int i=0; i < 50-prev_size; i++){
            double N = (target_dist/(0.02*ref_v/2.24)); 
            double x_point = x_add_on + target_dist/N; 
            double y_point = s(x_point);

            x_add_on = x_point; 

            double x_ref = x_point; 
            double y_ref = y_point; 

            // rotate the coordinates back to the global frame
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw)); 

            // add on the distance between the vehicle and the origin of the world map
            x_point += ref_x; 
            y_point += ref_y; 

            // push back the interpolated waypoints 
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point); 
          }
          

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