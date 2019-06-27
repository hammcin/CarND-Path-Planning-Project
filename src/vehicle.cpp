#include "vehicle.h"
#include "constants.h"
#include "cost.h"

#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <iostream>
#include <map>

using std::vector;
using std::string;
using std::map;
using std::cout;
using std::endl;

Vehicle::Vehicle(){}

Vehicle::Vehicle(double s, double s_dot, double s_ddot, double d, int lane,
                 string state)
{
  vector<double> s_start;
  s_start.push_back(s);
  s_start.push_back(s_dot);
  s_start.push_back(s_ddot);
  this->start_state.s = s_start;

  vector<double> d_start;
  d_start.push_back(d);
  d_start.push_back(0.0);
  d_start.push_back(0.0);
  this->start_state.d = d_start;

  this->lane = lane;

  this->state = state;
}

Vehicle::~Vehicle(){}

int Vehicle::frenet_to_lane(double d)
{
  int lane;
  if (d >= 0 && d <= LANE_WIDTH)
  {
    lane = 0;
  }
  else if (d > LANE_WIDTH && d <= 2*LANE_WIDTH)
  {
    lane = 1;
  }
  else if (d > 2*LANE_WIDTH && d <= 3*LANE_WIDTH)
  {
    lane = 2;
  }
  return lane;
}

double Vehicle::lane_to_frenet(int lane)
{
  return LANE_WIDTH/2.0 + lane*LANE_WIDTH;
}

Vehicle Vehicle::choose_next_state(vector<Vehicle> &vehicles)
{
  cout << "=================" << endl;
  cout << "Choose Next State" << endl;
  cout << "=================" << endl;

  vector<string> states = successor_states(vehicles);
  double cost;
  vector<double> costs;
  vector<Vehicle> final_trajectories;

  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
  {
    vector<Vehicle> trajectory = generate_trajectory(*it, vehicles);
    if (trajectory.size() != 0)
    {

      cout << "State: " << *it << endl;

      cost = behavior_cost(*this, vehicles, trajectory[0]);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory[0]);

      cout << "Total cost: " << cost << endl;
    }
  }

  vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  cout << "Best cost: " << costs[best_idx] << endl;

  return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states(vector<Vehicle> &vehicles)
{
  Vehicle vehicle_ahead;
  bool is_vehicle_ahead = get_vehicle_ahead(vehicles, this->lane, vehicle_ahead);
  double curr_lane_ahead_v = this->target_speed;
  if (is_vehicle_ahead)
  {
    curr_lane_ahead_v = vehicle_ahead.start_state.s[1];

    cout << "Vehicle ahead" << endl;
    cout << "Vehicle ahead velocity: " << curr_lane_ahead_v << endl;
  }

  Vehicle vehicle_behind;
  bool is_vehicle_behind = get_vehicle_behind(vehicles, this->lane, vehicle_behind);
  double curr_lane_behind_v = this->target_speed;
  if (is_vehicle_behind)
  {
    curr_lane_behind_v = vehicle_behind.start_state.s[1];

    cout << "Vehicle behind" << endl;
    cout << "Vehicle behind velocity: " << curr_lane_behind_v << endl;
  }

  int new_lane;

  vector<string> states;
  string state = this->state;

  double d_diff = fabs(this->start_state.d[0] - lane_to_frenet(this->lane));

  cout << "Distance from center of lane: " << d_diff << endl;
  bool d_diff_test = d_diff < 0.2;
  cout << "Distance from center of lane less than: " << d_diff_test << endl;

  if (state.compare("LCL") == 0)
  {
    if (d_diff < 0.2)
    {
      states.push_back("KL");
    }
    else
    {
      this->lane -= lane_direction[state];
      states.push_back("LCL");
    }
  }
  else if (state.compare("LCR") == 0)
  {
    if (d_diff < 0.2)
    {
      states.push_back("KL");
    }
    else
    {
      this->lane -= lane_direction[state];
      states.push_back("LCR");
    }
  }
  else
  {
    states.push_back("KL");
  }

  if (state.compare("KL") == 0)
  {
    if (this->lane != 0)
    {
      states.push_back("PLCL");
    }
    if (this->lane != LANES_AVAILABLE - 1)
    {
      states.push_back("PLCR");
    }
  }
  else if (state.compare("PLCL") == 0)
  {
    new_lane = this->lane + lane_direction[state];

    Vehicle closest_vehicle;
    bool found_vehicle = get_vehicle_ahead(vehicles, new_lane, closest_vehicle);
    double next_lane_v = this->target_speed;
    if(found_vehicle)
    {
      next_lane_v = closest_vehicle.start_state.s[1];

      cout << "Found vehicle in next lane" << endl;
      cout << "Next lane vehicle speed: " << next_lane_v << endl;
    }

    if (this->lane != 0)
    {
      states.push_back("PLCL");
      if ((this->start_state.s[1] >= (next_lane_v - (5.0/CONVERT_FACTOR)))
          && (this->start_state.s[1] <= (next_lane_v + (5.0/CONVERT_FACTOR))))
      {
        states.push_back("LCL");
      }
      else if (this->start_state.s[1] > next_lane_v)
      {
        if ((this->start_state.s[1] >= (curr_lane_behind_v - (5.0/CONVERT_FACTOR)))
            && (this->start_state.s[1] <= (curr_lane_behind_v + (5.0/CONVERT_FACTOR))))
        {
          states.push_back("LCL");
        }
      }
      else if (this->start_state.s[1] < next_lane_v)
      {
        if ((this->start_state.s[1] >= (curr_lane_ahead_v - (5.0/CONVERT_FACTOR)))
            && (this->start_state.s[1] <= (curr_lane_ahead_v + (5.0/CONVERT_FACTOR))))
        {
          states.push_back("LCL");
        }
      }
    }
  }
  else if (state.compare("PLCR") == 0)
  {
    new_lane = this->lane + lane_direction[state];

    Vehicle closest_vehicle;
    bool found_vehicle = get_vehicle_ahead(vehicles, new_lane, closest_vehicle);
    double next_lane_v = this->target_speed;
    if(found_vehicle)
    {
      next_lane_v = closest_vehicle.start_state.s[1];

      cout << "Found vehicle in next lane" << endl;
      cout << "Next lane vehicle speed: " << next_lane_v << endl;
    }

    if (this->lane != LANES_AVAILABLE - 1)
    {
      states.push_back("PLCR");
      if ((this->start_state.s[1] >= (next_lane_v - (5.0/CONVERT_FACTOR)))
          && (this->start_state.s[1] <= (next_lane_v + (5.0/CONVERT_FACTOR))))
      {
        states.push_back("LCR");
      }
      else if (this->start_state.s[1] > next_lane_v)
      {
        if ((this->start_state.s[1] >= (curr_lane_behind_v - (5.0/CONVERT_FACTOR)))
            && (this->start_state.s[1] <= (curr_lane_behind_v + (5.0/CONVERT_FACTOR))))
        {
          states.push_back("LCR");
        }
      }
      else if (this->start_state.s[1] < next_lane_v)
      {
        if ((this->start_state.s[1] >= (curr_lane_ahead_v - (5.0/CONVERT_FACTOR)))
            && (this->start_state.s[1] <= (curr_lane_ahead_v + (5.0/CONVERT_FACTOR))))
        {
          states.push_back("LCR");
        }
      }
    }
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
  else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
  {
    trajectory = prep_lane_change_trajectory(state, vehicles);
  }

  return trajectory;
}

vector<double> Vehicle::get_kinematics(vector<Vehicle> &vehicles, int new_lane)
{
  double delta_t = this->project_size * DT;
  double max_velocity_accel_limit = this->start_state.s[1]
                                    + MAX_ACCELERATION * delta_t;
  double min_velocity_accel_limit = this->start_state.s[1]
                                    - MAX_ACCELERATION * delta_t;
  double new_position;
  double new_velocity;
  double new_accel;
  Vehicle vehicle_ahead;

  if (get_vehicle_ahead(vehicles, new_lane, vehicle_ahead))
  {

    cout << "Vehicle in front" << endl;

    if ((vehicle_ahead.start_state.s[0] - this->start_state.s[0]) <= this->buffer)
    {

      cout << "Vehicle in buffer region" << endl;


      cout << "Max acceleration v: " << max_velocity_accel_limit << endl;
      cout << "Max deceleration v: " << min_velocity_accel_limit << endl;
      cout << "Vehicle in front v: " << vehicle_ahead.start_state.s[1] << endl;
      cout << "Speed limit v: " << this->target_speed << endl;


      double vehicle_ahead_v = vehicle_ahead.start_state.s[1];
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

      cout << "Chosen v: " << new_velocity << endl;

    }
    else
    {

      cout << "No vehicle in buffer region" << endl;

      double max_accel_in_front = 2 * (vehicle_ahead.start_state.s[0]
                                     + vehicle_ahead.start_state.s[1] * delta_t
                                     + vehicle_ahead.start_state.s[2] * delta_t * delta_t / 2.0
                                     - this->buffer - this->start_state.s[0]
                                     - this->start_state.s[1] * delta_t)
                                     / (delta_t * delta_t);
      double max_velocity_in_front = this->start_state.s[1] + max_accel_in_front * delta_t;


      cout << "Max acceleration v: " << max_velocity_accel_limit << endl;
      cout << "Max deceleration v: " << min_velocity_accel_limit << endl;
      cout << "Max in front v: " << max_velocity_in_front << endl;
      cout << "Speed limit v: " << this->target_speed << endl;


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

      cout << "Chosen v: " << new_velocity << endl;

    }
  }
  else
  {

    cout << "No vehicle in front" << endl;

    cout << "Max acceleration v: " << max_velocity_accel_limit << endl;
    cout << "Speed limit v: " << this->target_speed << endl;

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

    cout << "Chosen v: " << new_velocity << endl;
  }

  new_accel = (new_velocity - this->start_state.s[1]) / delta_t;
  new_position = this->start_state.s[0] + this->start_state.s[1] * delta_t
                 + new_accel * delta_t * delta_t / 2.0;

  return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::keep_lane_trajectory(vector<Vehicle> &vehicles)
{

cout << "State: " << "KL" << endl;

  vector<Vehicle> trajectory;
  vector<double> kinematics = get_kinematics(vehicles, this->lane);
  double new_s = kinematics[0];
  double new_v = kinematics[1];
  double new_a = kinematics[2];
  trajectory.push_back(Vehicle(new_s, new_v, new_a, lane_to_frenet(this->lane),
                       this->lane, "KL"));

  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string prep_state,
                                                     vector<Vehicle> &vehicles)
{

  cout << "State: " << prep_state << endl;

  double new_position;
  double new_velocity;
  double new_accel;
  vector<Vehicle> trajectory;

  int new_lane = this->lane + lane_direction[prep_state];

  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  for (int i=0; i<vehicles.size(); ++i)
  {
    next_lane_vehicle = vehicles[i];
    if (next_lane_vehicle.start_state.s[0] >= (this->start_state.s[0] - buffer)
        && next_lane_vehicle.start_state.s[0] <= (this->start_state.s[0] + buffer)
        && next_lane_vehicle.lane == new_lane)
      {
      cout << "Prep lane change unavailable" << endl;

      return trajectory;
    }
  }

  Vehicle vehicle_ahead;
  bool is_vehicle_ahead = get_vehicle_ahead(vehicles, this->lane, vehicle_ahead);
  double curr_lane_v = this->target_speed;
  if (is_vehicle_ahead)
  {
    curr_lane_v = vehicle_ahead.start_state.s[1];

    cout << "Vehicle ahead" << endl;
    cout << "Vehicle ahead speed: " << curr_lane_v << endl;
  }

  Vehicle closest_vehicle;
  bool found_vehicle = get_vehicle_ahead(vehicles, new_lane, closest_vehicle);
  double next_lane_v = this->target_speed;
  if(found_vehicle)
  {
    next_lane_v = closest_vehicle.start_state.s[1];

    cout << "Found vehicle in next lane" << endl;
    cout << "Next lane vehicle speed: " << next_lane_v << endl;
  }

  if (next_lane_v >= curr_lane_v)
  {
    cout << "Next lane vehicle faster or same speed" << endl;

    vector<double> curr_lane_new_kinematics = get_kinematics(vehicles, this->lane);
    new_position = curr_lane_new_kinematics[0];
    new_velocity = curr_lane_new_kinematics[1];
    new_accel = curr_lane_new_kinematics[2];
    trajectory.push_back(Vehicle(new_position, new_velocity, new_accel,
                                 lane_to_frenet(this->lane),
                                 this->lane, prep_state));
  }
  else if (next_lane_v < curr_lane_v)
  {
    cout << "Next lane vehicle slower" << endl;

    double delta_t = this->project_size * DT;
    double max_velocity_accel_limit = this->start_state.s[1]
                                      + MAX_ACCELERATION * delta_t;
    double min_velocity_accel_limit = this->start_state.s[1]
                                      - MAX_ACCELERATION * delta_t;

    cout << "Max acceleration velocity: " << max_velocity_accel_limit << endl;
    cout << "Min acceleration velocity: " << min_velocity_accel_limit << endl;

    if (next_lane_v > max_velocity_accel_limit)
    {
      next_lane_v = max_velocity_accel_limit;
    }
    else if (next_lane_v < min_velocity_accel_limit)
    {
      next_lane_v = min_velocity_accel_limit;
    }

    cout << "Speed limit: " << this->target_speed << endl;

    double speed_limit_v = this->target_speed;
    if (speed_limit_v > max_velocity_accel_limit)
    {
      speed_limit_v = max_velocity_accel_limit;
    }
    else if (speed_limit_v < min_velocity_accel_limit)
    {
      speed_limit_v = min_velocity_accel_limit;
    }

    Vehicle vehicle_behind;
    if (get_vehicle_behind(vehicles, this->lane, vehicle_behind))
    {
      cout << "Vehicle behind" << endl;

      cout << "Vehicle behind position: " << vehicle_behind.start_state.s[0] << endl;
      cout << "Vehicle behind velocity: " << vehicle_behind.start_state.s[1] << endl;
      cout << "Vehicle behind acceleration: " << vehicle_behind.start_state.s[2] << endl;

      cout << "Ego vehicle position: " << this->start_state.s[0] << endl;
      cout << "Ego vehicle velocity: " << this->start_state.s[1] << endl;
      cout << "Ego vehicle acceleration: " << this->start_state.s[2] << endl;

      cout << "Buffer: " << this->buffer << endl;
      cout << "Delta t: " << delta_t << endl;

      double min_accel_behind = 2 * (vehicle_behind.start_state.s[0]
                                     + vehicle_behind.start_state.s[1] * delta_t
                                     + vehicle_behind.start_state.s[2] * delta_t * delta_t / 2.0
                                     + this->buffer - this->start_state.s[0]
                                     - this->start_state.s[1] * delta_t)
                                     / (delta_t * delta_t);
      double min_velocity_behind = this->start_state.s[1] + min_accel_behind * delta_t;

      cout << "Min velocity behind: " << min_velocity_behind << endl;

      if (min_velocity_behind > max_velocity_accel_limit)
      {
        min_velocity_behind = max_velocity_accel_limit;
      }
      else if (min_velocity_behind < min_velocity_accel_limit)
      {
        min_velocity_behind = min_velocity_accel_limit;
      }

      new_velocity = std::min(std::max(min_velocity_behind, next_lane_v),
                              speed_limit_v);

      cout << "Chosen velocity: " << new_velocity << endl;
    }
    else
    {
      cout << "No vehicle behind" << endl;

      new_velocity = std::min(next_lane_v, speed_limit_v);

      cout << "Chosen velocity: " << new_velocity << endl;
    }
    new_accel = (new_velocity - this->start_state.s[1]) / delta_t;
    new_position = this->start_state.s[0] + this->start_state.s[1] * delta_t
                   + new_accel * delta_t * delta_t / 2.0;
    trajectory.push_back(Vehicle(new_position, new_velocity, new_accel,
                                 lane_to_frenet(this->lane),
                                 this->lane, prep_state));
  }

  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string lane_change_state,
                                               vector<Vehicle> &vehicles)
{

  cout << "State: " << lane_change_state << endl;

  int new_lane = this->lane + lane_direction[lane_change_state];
  vector<Vehicle> trajectory;
  if (!(lane_change_state.compare(this->state) == 0))
  {
    Vehicle next_lane_vehicle;
    // Check if a lane change is possible (check if another vehicle occupies
    //   that spot).
    for (int i=0; i<vehicles.size(); ++i)
    {
      next_lane_vehicle = vehicles[i];

      double d_diff = fabs(next_lane_vehicle.start_state.d[0]
                           - this->start_state.d[0]);
      double comp_d = 2*(LANE_WIDTH/2.0 - 0.2);
      int lane_diff = fabs(this->lane - next_lane_vehicle.lane);
      if (next_lane_vehicle.lane != new_lane && next_lane_vehicle.lane != this->lane
          && next_lane_vehicle.start_state.s[0] >= (this->start_state.s[0] - buffer)
          && next_lane_vehicle.start_state.s[0] <= (this->start_state.s[0] + buffer)
          && d_diff <= comp_d + (lane_diff - 1)*LANE_WIDTH)
      {

        cout << "Vehicle in buffer region in non-intended lane" << endl;
        cout << "Vehicle preparing lane change to intended lane" << endl;
        cout << "Lane change unavailable" << endl;

        return trajectory;
      }

      if (next_lane_vehicle.start_state.s[0] >= (this->start_state.s[0] - buffer)
          && next_lane_vehicle.start_state.s[0] <= (this->start_state.s[0] + buffer)
          && next_lane_vehicle.lane == new_lane)
      {

        cout << "Vehicle in buffer region in intended lane" << endl;
        cout << "Lane change unavailable" << endl;

        return trajectory;
      }
    }
  }

  cout << "Lane change available" << endl;

  vector<double> kinematics = get_kinematics(vehicles, new_lane);
  trajectory.push_back(Vehicle(kinematics[0], kinematics[1], kinematics[2],
                               lane_to_frenet(new_lane), new_lane,
                               lane_change_state));

  return trajectory;
}

Vehicle Vehicle::state_in(double t)
{
  vector<double> s = this->start_state.s;
  vector<double> d = this->start_state.d;

  vector<double> new_s(3);
  new_s[0] = s[0] + s[1]*t + s[2]*t*t/2.0;
  new_s[1] = s[1] + s[2]*t;
  new_s[2] = s[2];

  vector<double> new_d(3);
  new_d[0] = d[0] + d[1]*t + d[2]*t*t/2.0;
  new_d[1] = d[1] + d[2]*t;
  new_d[2] = d[2];

  return Vehicle(new_s[0], new_s[1], new_s[2], new_d[0], this->lane,
                 this->state);
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
      double check_car_s = temp_vehicle.start_state.s[0];

      if ((check_car_s>this->start_state.s[0]) && (check_car_s<min_s))
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
      double check_car_s = temp_vehicle.start_state.s[0];

      if ((check_car_s<this->start_state.s[0]) && (check_car_s>max_s_behind))
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
  goal_lane = data[3];
}
