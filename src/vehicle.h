#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <string>
#include <map>

using std::vector;
using std::string;
using std::map;

class Vehicle
{
public:
  // Constructor
  Vehicle();

  Vehicle(double s, double s_dot, double s_ddot, int lane, string state);

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  Vehicle choose_next_state(vector<Vehicle> &vehicles);

  // Provides the possible next states given the current state for the FSM.
  vector<string> successor_states();

  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> generate_trajectory(string state,
                                        vector<Vehicle> &vehicles);

  // Gets next timestep kinematics (position, velocity, acceleration) for a
  //   given lane.  Tries to choose the maximum velocity and acceleration, given
  //   other vehicle positions and accel/velocity constraints.
  vector<double> get_kinematics(vector<Vehicle> &vehicles, int lane);

  // Generate a keep lane trajectory.
  vector<Vehicle> keep_lane_trajectory(vector<Vehicle> &vehicles);

  // Generate a lane change trajectory.
  vector<Vehicle> lane_change_trajectory(string state,
                                           vector<Vehicle> &vehicles);

  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise.  The passed reference rVehicle is updated if a vehicle is
  //   found.
  bool get_vehicle_ahead(vector<Vehicle> &vehicles, int lane,
                         Vehicle &rVehicle);

  // Returns a true if a vehicle is found behind the current vehicle, false
  //   otherwise.  The passed reference rVehicle is updated if a vehicle is
  //   found.
  bool get_vehicle_behind(vector<Vehicle> &vehicles, int lane,
                          Vehicle &rVehicle);

  void configure(vector<double> &data);

  // public Vehicle variables
  map<string, int> lane_direction = {{"LCL", -1}, {"LCR", 1}};

  double s, s_dot, s_ddot;

  int lane;

  string state;

  // Ego vehicle variables
  double buffer;
  double target_speed;
  int project_size;
};

#endif  // VEHICLE_H
