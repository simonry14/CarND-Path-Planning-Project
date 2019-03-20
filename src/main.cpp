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

// Defined here for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Declare variables to bind to waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Read waypoints from highway_map.csv
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

  //Our car's lane. 0 left lane, 1 middle lane, 2 right lane.
  int lane = 1;
  //Reference velocity in mph.
  double ref_vel = 0.0;

  h.onMessage([&ref_vel, &lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          // Provided previous path point size.
            int prev_size = previous_path_x.size();

           // Preventing collitions.
            if (prev_size > 0) {
              car_s = end_path_s;
            }

          // PREDICTION : Analysing the positions of other cars on the road.
            bool car_ahead = false; //Is there a car ahead
            bool car_left = false; // Is there a car to the left
            bool car_right = false; //Is there a car to the right
            for ( int i = 0; i < sensor_fusion.size(); i++ ) { // loop through all other cars on the road
                float d = sensor_fusion[i][6]; //get ith Car's d coordinate in frenet
                int car_lane = -1;
                // is it on the same lane we are
                if ( d > 0 && d < 4 ) {
                  car_lane = 0; //ith car is in the left lane
                } else if ( d > 4 && d < 8 ) {
                  car_lane = 1; //ith car is in the middle lane
                } else if ( d > 8 && d < 12 ) {
                  car_lane = 2; //ith car is in the right lane
                }
                if (car_lane < 0) {
                  continue;
                }
                // Find speed of the ith car.
                double vx = sensor_fusion[i][3]; //get ith Car's speed in the x component
                double vy = sensor_fusion[i][4]; //get ith Car's speed in the y component
                double check_speed = sqrt(vx*vx + vy*vy); //get ith Car's real speed. mangnitude of the 2 components
                double check_car_s = sensor_fusion[i][5]; //get ith Car's s coordinate in frenet
                // Estimate ith car's s position after executing previous trajectory.
                check_car_s += ((double)prev_size*0.02*check_speed); // D = time * speed

                if ( car_lane == lane ) {
                  // If the ith Car is in our car lane.
                  car_ahead |= check_car_s > car_s && check_car_s - car_s < 25; // Car is ahead of our car if its within 25m
                } else if ( car_lane - lane == -1 ) {
                  // If the ith Car is in a lane left of ego car.
                  car_left |= car_s - 25 < check_car_s && car_s + 25 > check_car_s; //Car is within 25m of our car in left lane
                } else if ( car_lane - lane == 1 ) {
                  //  If the ith Car is in a lane right of ego car.
                  car_right |= car_s - 25 < check_car_s && car_s + 25 > check_car_s; //Car is within 25m of our car in right lane
                }
            }

            // BEHAVIOUR PLANNING : Changing behaviour of our Car depending on positions of other cars on the road.
            double speed_difference = 0;
            const double MAX_SPEED = 49.9; // Maximum allowed speed. Speed limit is 50
            const double MAX_ACC = .224; // Maximum acceleration allowed.
            if ( car_ahead ) { // If there's a Car ahead of us in our lane
              if ( !car_left && lane > 0 ) { // If theres no car to our left and we are in the middle or right lane
                // if there is no car left and there is a left lane.
                lane--; // Change lane left.
              } else if ( !car_right && lane != 2 ){ //If theres no car to our right and we aren't in the right lane (2)
                // if there is no car right and there is a right lane.
                lane++; // Change to a lane on our right.
              } else { // else there's a car ahead and to the left or right so we cannot change lane. then reduce speed
                speed_difference -= MAX_ACC;
              }
            } else { //Theres no car ahead of us
              if ( lane != 1 ) { // if we are not on the middle lane.
                //If we are in the left lane and theres no car in the right lane or in the right lane and theres no car to the left
                if ( ( lane == 0 && !car_right ) || ( lane == 2 && !car_left ) ) {
                  lane = 1; // Back to middle lane.
                }
              }
              if ( ref_vel < MAX_SPEED ) { //Increase speed if our vel is less that the maximum speed
                speed_difference += MAX_ACC;
              }
            }


            //TRAJECTORY CALCULATION
          	vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // Are there any previous points?
            if ( prev_size < 2 ) {
                // If there are less than 2 previous points available, use the car's current position and its heading to get determine point
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            } else {
                // Use the last two points only.
                ref_x = previous_path_x[prev_size - 1]; //last x point
                ref_y = previous_path_y[prev_size - 1]; //last y point

                double ref_x_prev = previous_path_x[prev_size - 2]; //2nd last x point
                double ref_y_prev = previous_path_y[prev_size - 2]; //2nd last y point
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }

            // Setting up target points in the future.
            vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // Making coordinates to local car coordinates.
            for ( int i = 0; i < ptsx.size(); i++ ) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            // Create the spline.
            tk::spline s;
            s.set_points(ptsx, ptsy);

            // Output path points from previous path for continuity.
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            for ( int i = 0; i < prev_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate distance y position on 30 m ahead.
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;

            for( int i = 1; i < 50 - prev_size; i++ ) {
              ref_vel += speed_difference;
              if ( ref_vel > MAX_SPEED ) {
                ref_vel = MAX_SPEED;
              } else if ( ref_vel < MAX_ACC ) {
                ref_vel = MAX_ACC;
              }
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

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
