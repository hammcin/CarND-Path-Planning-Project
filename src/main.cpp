#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "constants.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

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

  // start in lane 1
  // int lane = 1;

  // Have a reference velocity to target
  double ref_vel = 0.0; // mph

  // Keep track of state of vehicle
  // string state = "KL";

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,
               &map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
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

          // Length of previous path
          int prev_size = previous_path_x.size();

          // Total size of the path
          int path_size = 50;

          // Number of points to add to the previous path
          int project_size = path_size - prev_size;

          // closest distance to another vehicle
          double buffer = 30; // meters?

          // Max change in reference velocity
          // double delta_v = MAX_ACCELERATION * DT * CONVERT_FACTOR; // MPH

          // Target speed
          double target_speed = (SPEED_LIMIT - 5.0) / CONVERT_FACTOR; // m/s

          // Use car position after it finishes its current path
          // instead of current car position

          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          vector<Vehicle> vehicles;
          for (int i=0; i<sensor_fusion.size(); ++i)
          {
            Vehicle other_v;

            double d = sensor_fusion[i][6];

            double pos = sensor_fusion[i][5];

            double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];

            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];

            double v = sqrt(vx*vx + vy*vy);

            double accel = 0;

            pos += prev_size*DT*v;

            other_v = Vehicle(pos, v, accel, d, "CS");

            vehicles.push_back(other_v);
          }

          double d = end_path_d;
          double pos = end_path_s;
          double v;
          double accel;
          if (prev_size < 3)
          {
            double x;
            double y;
            double x_prev;
            double y_prev;
            double theta_prev;
            if (prev_size > 1)
            {
              x = previous_path_x[prev_size-1];
              y = previous_path_y[prev_size-1];
              x_prev = previous_path_x[prev_size-2];
              y_prev = previous_path_y[prev_size-2];

              double vx = (x - x_prev)/DT;
              double vy = (y - y_prev)/DT;
              v = sqrt(vx*vx + vy*vy);
            }
            else
            {

              v = car_speed;
            }

            accel = 0;
          }
          else
          {
            double x = previous_path_x[prev_size-1];
            double y = previous_path_y[prev_size-1];
            double x_prev = previous_path_x[prev_size-2];
            double y_prev = previous_path_y[prev_size-2];
            double x_prev2 = previous_path_x[prev_size-3];
            double y_prev2 = previous_path_y[prev_size-3];

            double vx = (x - x_prev)/DT;
            double vy = (y - y_prev)/DT;
            v = sqrt(vx*vx + vy*vy);

            double vx_prev = (x_prev - x_prev2)/DT;
            double vy_prev = (y_prev - y_prev2)/DT;

            double d_vx_dt = (vx - vx_prev)/DT;
            double d_vy_dt = (vy - vy_prev)/DT;

            accel = sqrt(d_vx_dt*d_vx_dt + d_vy_dt*d_vy_dt);
          }


          // Keep track of state of vehicle
          string state = "KL";

          Vehicle ego = Vehicle(pos, v, accel, d, state);

          vector<double> vehicle_data;
          vehicle_data.push_back(buffer);
          vehicle_data.push_back(target_speed);
          vehicle_data.push_back(project_size);
          ego.configure(vehicle_data);

          // cout << "Initial v: " << ref_vel << endl;

          Vehicle trajectory = ego.choose_next_state(vehicles);
          double proposed_speed = trajectory.start_state.s[1] * CONVERT_FACTOR; // MPH
          lane = trajectory.lane;
          double delta_v = trajectory.start_state.s[2] * DT * CONVERT_FACTOR; // MPH

          // cout << "Chosen state: " << trajectory.state << endl;

          /*
          cout << "Proposed speed: " << proposed_speed << endl;
          cout << "Delta velocity: " << delta_v_test << endl;
          if (ref_vel > proposed_speed)
          {
            cout << "Delta velocity max: " << -1*delta_v << endl;
          }
          else if (ref_vel <= (proposed_speed - delta_v))
          {
            cout << "Delta velocity max: " << delta_v << endl;
          }
          else
          {
            cout << "Delta velocity max: " << 0 << endl;
          }

          cout << "Final v: " << ref_vel << endl;
          cout << endl;
          */


          vector<double> start_s(3);
          start_s[0] = pos;
          start_s[1] = v;
          start_s[2] = accel;

          vector<double> start_d(3);
          start_d[0] = end_path_d;
          start_d[1] = 0.0;
          start_d[2] = 0.0;

          vector<double> goal_state_s(3);
          goal_state_s[0] = trajectory.s;
          goal_state_s[1] = trajectory.s_dot;
          goal_state_s[2] = trajectory.s_ddot;

          vector<double> goal_state_d(3);
          goal_state_d[0] = LANE_WIDTH/2.0 + LANE_WIDTH*trajectory.lane;
          goal_state_d[1] = 0.0;
          goal_state_d[2] = 0.0;


          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline and fill it
          // in with more points that control speed

          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y, yaw states
          // either we will reference the starting point as where the car is or
          // at the previous path's end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          if (prev_size < 2)
          {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path's end point as starting reference
          else
          {

            // Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points that make the path tangent to the previous path's
            // end points
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          // In Frenet add evenly 30m spaced points ahead of the starting
          // reference
          vector<double> next_wp0 = getXY(car_s+30,
                                          (LANE_WIDTH/2+LANE_WIDTH*lane),
                                          map_waypoints_s,map_waypoints_x,
                                          map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,
                                          (LANE_WIDTH/2+LANE_WIDTH*lane),
                                          map_waypoints_s,map_waypoints_x,
                                          map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,
                                          (LANE_WIDTH/2+LANE_WIDTH*lane),
                                          map_waypoints_s,map_waypoints_x,
                                          map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i=0; i<ptsx.size(); ++i)
          {

            // shift car reference angle to 0 degrees
            double new_angle = 0;
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(new_angle-ref_yaw)
                       -shift_y*sin(new_angle-ref_yaw));
            ptsy[i] = (shift_x*sin(new_angle-ref_yaw)
                       +shift_y*cos(new_angle-ref_yaw));

          }

          // create a spline
          tk::spline s;

          // set (x,y) points to the spline
          s.set_points(ptsx,ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for (int i=0; i<prev_size; ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our
          // desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous
          // points, here we will always output 50 points
          for (int i=1; i<= path_size-prev_size; ++i) {

            ref_vel += delta_v;

            double N = (target_dist/(DT*ref_vel/CONVERT_FACTOR));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

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
