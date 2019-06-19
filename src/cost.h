#ifndef COST_H
#define COST_H

#include "vehicle.h"

#include <vector>

using std::vector;

// Sum weighted cost functions to get total cost for trajectory.
double calculate_cost(Vehicle &vehicle, vector<Vehicle> &vehicles,
                      Vehicle &trajectory);

// Cost becomes higher for trajectories with intended lane and final lane that
//   have traffic slower than vehicle's target speed.
double inefficiency_cost(Vehicle &vehicle, vector<Vehicle> &vehicles,
                         Vehicle &trajectory);

// Since there is no vehicle next to the ego vehicle during a lane change, the
//   speed limit for a lane is equal to the speed of the vehicle in front of the
//   ego vehicle.
double lane_speed(Vehicle &vehicle, vector<Vehicle> &vehicles, int lane);

#endif  // COST_H
