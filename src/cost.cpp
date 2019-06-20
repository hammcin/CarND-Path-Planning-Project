#include "cost.h"
#include "vehicle.h"
#include "constants.h"

#include <vector>
#include <functional>
#include <iostream>

using std::vector;
using std::cout;
using std::endl;

const double EFFICIENCY = 1;

double inefficiency_cost(Vehicle &vehicle, vector<Vehicle> &vehicles,
                         Vehicle &trajectory)
{

  // cout << "Lane: " << trajectory.lane << endl;

  double proposed_speed = lane_speed(vehicle, vehicles, trajectory.lane);

  // cout << "Lane Speed: " << proposed_speed << endl;

  double cost = (vehicle.target_speed - proposed_speed) / vehicle.target_speed;

  return cost;
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

  if (found_vehicle && vehicle_speed <= vehicle.target_speed)
  {
    return vehicle_speed;
  }

  return vehicle.target_speed;
}

double calculate_cost(Vehicle &vehicle, vector<Vehicle> &vehicles,
                      Vehicle &trajectory)
{
  double cost = 0.0;

  vector <std::function <double (Vehicle &, vector<Vehicle> &,
                                Vehicle &) > >
    cf_list = {inefficiency_cost};
  vector<double> weight_list = {EFFICIENCY};

  for (int i=0; i<cf_list.size(); ++i)
  {
    double new_cost = weight_list[i]*cf_list[i](vehicle, vehicles, trajectory);
    cost += new_cost;
  }

  return cost;
}
