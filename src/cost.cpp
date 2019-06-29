#include "cost.h"
#include "vehicle.h"
#include "constants.h"

#include <vector>
#include <functional>
#include <iostream>
#include <cmath>

using std::vector;
using std::cout;
using std::endl;

const double REACH_GOAL_LANE = 0.15;
const double EFFICIENCY = 0.85;

double goal_lane_cost(Vehicle &vehicle, vector<Vehicle> &vehicles,
                      Vehicle &trajectory)
{

  if (VERBOSE)
  {
    cout << "Intended lane: " << get_intended_lane(trajectory) << endl;
    cout << "Final lane: " << trajectory.lane << endl;
  }

  double cost = (fabs(vehicle.goal_lane - get_intended_lane(trajectory))
                 + fabs(vehicle.goal_lane - trajectory.lane))/(LANES_AVAILABLE - 1);

  if (VERBOSE)
  {
    cout << "Goal lane cost: " << cost << endl;
  }

  return cost;
}

double inefficiency_cost(Vehicle &vehicle, vector<Vehicle> &vehicles,
                         Vehicle &trajectory)
{

  if (VERBOSE)
  {
    cout << "Intended lane: " << get_intended_lane(trajectory) << endl;
  }

  double proposed_speed_intended = lane_speed(vehicle, vehicles,
                                              get_intended_lane(trajectory));

  if (VERBOSE)
  {
    cout << "Intended lane speed: " << proposed_speed_intended << endl;

    cout << "Final lane: " << trajectory.lane << endl;
  }

  double proposed_speed_final = lane_speed(vehicle, vehicles, trajectory.lane);

  if (VERBOSE)
  {
    cout << "Final lane speed: " << proposed_speed_final << endl;
  }

  double cost = (2.0*vehicle.target_speed - proposed_speed_intended
                 - proposed_speed_final) / vehicle.target_speed;

  if (VERBOSE)
  {
    cout << "Inefficiency cost: " << cost << endl;
  }

  return cost;
}

int get_intended_lane(Vehicle &trajectory)
{
  double intended_lane;
  if (trajectory.state.compare("PLCL") == 0)
  {
    intended_lane = trajectory.lane - 1;
  }
  else if (trajectory.state.compare("PLCR") == 0)
  {
    intended_lane = trajectory.lane + 1;
  }
  else {
    intended_lane = trajectory.lane;
  }
  return intended_lane;
}

double lane_speed(Vehicle &vehicle, vector<Vehicle> &vehicles, int new_lane)
{
  int min_s = MAX_S;
  bool found_vehicle = false;
  double vehicle_speed;
  Vehicle temp_vehicle;
  for (int i=0; i<vehicles.size(); ++i)
  {
    temp_vehicle = vehicles[i];
    if (temp_vehicle.lane == new_lane
        && temp_vehicle.start_state.s[0] > vehicle.start_state.s[0]
        && temp_vehicle.start_state.s[0] < min_s)
    {
      min_s = temp_vehicle.start_state.s[0];
      vehicle_speed = temp_vehicle.start_state.s[1];
      found_vehicle = true;
    }
  }

  double delta_s = min_s - vehicle.start_state.s[0];
  if (found_vehicle && vehicle_speed <= vehicle.target_speed
      && delta_s < 2*vehicle.buffer)
  {
    return vehicle_speed;
  }

  return vehicle.target_speed;
}

double behavior_cost(Vehicle &vehicle, vector<Vehicle> &vehicles,
                      Vehicle &trajectory)
{
  double cost = 0.0;

  vector <std::function <double (Vehicle &, vector<Vehicle> &,
                                Vehicle &) > >
    cf_list = {goal_lane_cost, inefficiency_cost};
  vector<double> weight_list = {REACH_GOAL_LANE, EFFICIENCY};

  for (int i=0; i<cf_list.size(); ++i)
  {
    double new_cost = weight_list[i]*cf_list[i](vehicle, vehicles, trajectory);
    cost += new_cost;
  }

  return cost;
}

vector<double> calc_spline_points(double target_x, double target_dist,
                                  int n_pts, double ref_vel, double delta_v)
{
  vector<double> x_points;

  double x_add_on = 0; // meters

  for (int i=1; i<= n_pts; ++i)
  {
    ref_vel += delta_v;

    double N = (target_dist/(DT*ref_vel/CONVERT_FACTOR));
    double x_point = x_add_on + (target_x)/N; // meters

    x_add_on = x_point;

    x_points.push_back(x_point);
  }

  return x_points;
}

void shift_ref_frame(vector<double> &ptsx, vector<double> &ptsy,
                     double ref_x, double ref_y, double ref_yaw)
{
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
}

void inverse_shift_ref_frame(vector<double> &ptsx, vector<double> &ptsy,
                             double ref_x, double ref_y, double ref_yaw)
{
  for (int i=0; i<ptsx.size(); ++i)
  {
    double x_ref = ptsx[i];
    double y_ref = ptsy[i];

    // rotate back to normal after rotating it earlier
    ptsx[i] = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
    ptsy[i] = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

    ptsx[i] += ref_x;
    ptsy[i] += ref_y;
  }
}
