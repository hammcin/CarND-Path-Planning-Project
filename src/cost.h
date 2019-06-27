#ifndef COST_H
#define COST_H

#include "vehicle.h"

#include <vector>

using std::vector;

// Sum weighted cost functions to get total cost for trajectory.
double behavior_cost(Vehicle &vehicle, vector<Vehicle> &vehicles,
                      Vehicle &trajectory);

// Cost increases based on distance of final lane of trajectory.
double goal_lane_cost(Vehicle &vehicle, vector<Vehicle> &vehicles,
                      Vehicle &trajectory);

// Cost becomes higher for trajectories with final lane that has traffic slower
// than vehicle's target speed.
double inefficiency_cost(Vehicle &vehicle, vector<Vehicle> &vehicles,
                         Vehicle &trajectory);

// Generate helper data to use in cost functions:
// intended_lane: the current lane +/- 1 if vehicle is planning or
//   execuitng a lane change.
// Note that intended_lane helps to differentiate between planning and executing
//   a lane change in the cost functions.
int get_intended_lane(Vehicle &trajectory);

// Since there is no vehicle next to the ego vehicle during a lane change, the
//   speed limit for a lane is equal to the speed of the vehicle in front of the
//   ego vehicle.
double lane_speed(Vehicle &vehicle, vector<Vehicle> &vehicles, int lane);

// Calculate how to break up spline points so that we travel at our
// desired reference velocity
vector<double> calc_spline_points(double target_x, double target_dist,
                                  int n_pts, double ref_vel, double delta_v);

void shift_ref_frame(vector<double> &ptsx, vector<double> &ptsy,
                     double ref_x, double ref_y, double ref_yaw);

void inverse_shift_ref_frame(vector<double> &ptsx, vector<double> &ptsy,
                             double ref_x, double ref_y, double ref_yaw);

#endif  // COST_H
