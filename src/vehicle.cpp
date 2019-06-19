#include "vehicle.h"
#include "constants.h"
#include "cost.h"

#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <iostream>

using std::vector;
using std::string;
using std::cout;
using std::endl;

Vehicle::Vehicle(){}

Vehicle::Vehicle(double s, double s_dot, double s_ddot, int lane, string state)
{
  this->s = s;
  this->s_dot = s_dot;
  this->s_ddot = s_ddot;
  this->lane = lane;
  this->state = state;
}

Vehicle::~Vehicle(){}

Vehicle Vehicle::choose_next_state(vector<Vehicle> &vehicles)
{
  vector<string> states = successor_states();
  double cost;
  vector<double> costs;
  vector<Vehicle> final_trajectories;


  // cout << "Choose Next State" << endl;
  // cout << "=================" << endl;
  // cout << endl;


  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
  {
    vector<Vehicle> trajectory = generate_trajectory(*it, vehicles);
    if (trajectory.size() != 0)
    {

      // cout << "State: " << *it << endl;

      cost = calculate_cost(*this, vehicles, trajectory[0]);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory[0]);

      // cout << "Cost: " << cost << endl;
    }
  }

  // cout << endl;

  vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  // cout << "Best cost: " << costs[best_idx] << endl;
  cout << endl;

  return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states()
{
  vector<string> states;

  states.push_back("KL");

  if (lane != 0)
  {
    states.push_back("LCL");
  }

  if (lane != LANES_AVAILABLE - 1)
  {
    states.push_back("LCR");
  }

  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
                                             vector<Vehicle> &vehicles)
{
  vector<Vehicle> trajectory;
  if (state.compare("KL") == 0)
  {
    trajectory = keep_lane_trajectory(vehicles);
  }
  else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
  {
    trajectory = lane_change_trajectory(state, vehicles);
  }

  return trajectory;
}

vector<double> Vehicle::get_kinematics(vector<Vehicle> &vehicles, int new_lane)
{
  double delta_t = this->project_size * DT;
  double max_velocity_accel_limit = this->s_dot + MAX_ACCELERATION * delta_t;
  double min_velocity_accel_limit = this->s_dot - MAX_ACCELERATION * delta_t;
  double new_position;
  double new_velocity;
  double new_accel;
  Vehicle vehicle_ahead;

  if (get_vehicle_ahead(vehicles, new_lane, vehicle_ahead))
  {

    // cout << "Vehicle in front" << endl;

    if ((vehicle_ahead.s - this->s) <= this->buffer)
    {

      // cout << "Vehicle in buffer region" << endl;

      /*
      cout << "Max acceleration v: " << max_velocity_accel_limit << endl;
      cout << "Max deceleration v: " << min_velocity_accel_limit << endl;
      cout << "Vehicle in front v: " << vehicle_ahead.s_dot << endl;
      cout << "Speed limit v: " << this->target_speed << endl;
      */

      double vehicle_ahead_v = vehicle_ahead.s_dot;
      if (vehicle_ahead_v > max_velocity_accel_limit)
      {
        vehicle_ahead_v = max_velocity_accel_limit;
      }
      else if (vehicle_ahead_v < min_velocity_accel_limit)
      {
        vehicle_ahead_v = min_velocity_accel_limit;
      }

      double speed_limit_v = this->target_speed;
      if (speed_limit_v > max_velocity_accel_limit)
      {
        speed_limit_v = max_velocity_accel_limit;
      }
      else if (speed_limit_v < min_velocity_accel_limit)
      {
        speed_limit_v = min_velocity_accel_limit;
      }

      new_velocity = std::min(vehicle_ahead_v, speed_limit_v);

      // cout << "Chosen v: " << new_velocity << endl;

    }
    else
    {

      // cout << "No vehicle in buffer region" << endl;

      double max_accel_in_front = 2 * (vehicle_ahead.s
                                     + vehicle_ahead.s_dot * delta_t
                                     + vehicle_ahead.s_ddot * delta_t * delta_t / 2.0
                                     - this->buffer - this->s
                                     - this->s_dot * delta_t)
                                     / (delta_t * delta_t);
      double max_velocity_in_front = this->s_dot + max_accel_in_front * delta_t;

      /*
      cout << "Max acceleration v: " << max_velocity_accel_limit << endl;
      cout << "Max deceleration v: " << min_velocity_accel_limit << endl;
      cout << "Max in front v: " << max_velocity_in_front << endl;
      cout << "Speed limit v: " << this->target_speed << endl;
      */

      if (max_velocity_in_front > max_velocity_accel_limit)
      {
        max_velocity_in_front = max_velocity_accel_limit;
      }
      else if (max_velocity_in_front < min_velocity_accel_limit)
      {
        max_velocity_in_front = min_velocity_accel_limit;
      }

      double speed_limit_v = this->target_speed;
      if (speed_limit_v > max_velocity_accel_limit)
      {
        speed_limit_v = max_velocity_accel_limit;
      }
      else if (speed_limit_v < min_velocity_accel_limit)
      {
        speed_limit_v = min_velocity_accel_limit;
      }

      new_velocity = std::min(max_velocity_in_front, speed_limit_v);

      // cout << "Chosen v: " << new_velocity << endl;

    }
  }
  else
  {

    // cout << "No vehicle in front" << endl;

    // cout << "Max acceleration v: " << max_velocity_accel_limit << endl;
    // cout << "Speed limit v: " << this->target_speed << endl;

    double speed_limit_v = this->target_speed;
    if (speed_limit_v > max_velocity_accel_limit)
    {
      speed_limit_v = max_velocity_accel_limit;
    }
    else if (speed_limit_v < min_velocity_accel_limit)
    {
      speed_limit_v = min_velocity_accel_limit;
    }

    new_velocity = speed_limit_v;

    // cout << "Chosen v: " << new_velocity << endl;
  }

  new_accel = (new_velocity - this->s_dot) / delta_t;
  new_position = this->s + this->s_dot * delta_t
                 + new_accel * delta_t * delta_t / 2.0;

  return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::keep_lane_trajectory(vector<Vehicle> &vehicles)
{

// cout << "State: " << "KL" << endl;

  vector<Vehicle> trajectory;
  vector<double> kinematics = get_kinematics(vehicles, this->lane);
  double new_s = kinematics[0];
  double new_v = kinematics[1];
  double new_a = kinematics[2];
  trajectory.push_back(Vehicle(new_s, new_v, new_a, this->lane, "KL"));

  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string lane_change_state,
                                               vector<Vehicle> &vehicles)
{
  int new_lane = this->lane + lane_direction[lane_change_state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  for (int i=0; i<vehicles.size(); ++i)
  {
    next_lane_vehicle = vehicles[i];
    if (next_lane_vehicle.s >= (this->s - buffer)
        && next_lane_vehicle.s <= (this->s + buffer)
        && next_lane_vehicle.lane == new_lane)
        {

      /*
      cout << "State: " << lane_change_state << endl;
      cout << "Lane change unavailable" << endl;
      */

      return trajectory;
    }
  }

  /*
  cout << "State: " << lane_change_state << endl;
  cout << "Lane change available" << endl;
  */

  vector<double> kinematics = get_kinematics(vehicles, new_lane);
  trajectory.push_back(Vehicle(kinematics[0], kinematics[1], kinematics[2],
                               new_lane, lane_change_state));

  return trajectory;
}

bool Vehicle::get_vehicle_ahead(vector<Vehicle> &vehicles, int new_lane,
                                Vehicle &rVehicle)
{
  bool found_vehicle = false;
  double min_s = MAX_S;
  Vehicle temp_vehicle;

  for (int i=0; i<vehicles.size(); ++i)
  {
    temp_vehicle = vehicles[i];
    if (temp_vehicle.lane == new_lane)
    {
      double check_car_s = temp_vehicle.s;

      if ((check_car_s>this->s) && (check_car_s<min_s))
      {
        min_s = check_car_s;
        rVehicle = temp_vehicle;
        found_vehicle = true;
      }
    }
  }

  return found_vehicle;
}

bool Vehicle::get_vehicle_behind(vector<Vehicle> &vehicles, int new_lane,
                        Vehicle &rVehicle)
{
  double max_s_behind = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (int i=0; i<vehicles.size(); ++i)
  {
    temp_vehicle = vehicles[i];
    if (temp_vehicle.lane == new_lane)
    {
      double check_car_s = temp_vehicle.s;

      if ((check_car_s<this->s) && (check_car_s>max_s_behind))
      {
        max_s_behind = check_car_s;
        rVehicle = temp_vehicle;
        found_vehicle = true;
      }
    }
  }

  return found_vehicle;
}

void Vehicle::configure(vector<double> &data)
{
  buffer = data[0];
  target_speed = data[1];
  project_size = data[2];
}
