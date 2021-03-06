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

  Vehicle(double s, double s_dot, double s_ddot, double d, int lane,
          string state);

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  int frenet_to_lane(double d);

  double lane_to_frenet(int lane);

  Vehicle choose_next_state(vector<Vehicle> &vehicles);

  // Provides the possible next states given the current state for the FSM.
  vector<string> successor_states(vector<Vehicle> &vehicles);

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

  // Generate a trajectory preparing for a lane change.
  vector<Vehicle> prep_lane_change_trajectory(string prep_state,
                                              vector<Vehicle> &vehicles);

  // Generate a lane change trajectory.
  vector<Vehicle> lane_change_trajectory(string lane_change_state,
                                         vector<Vehicle> &vehicles);

  Vehicle state_in(double t);

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
  struct frenet_coord
  {
    vector<double> s;
    vector<double> d;
  };

  frenet_coord start_state;

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1},
                                     {"LCR", 1}, {"PLCR", 1}};

  int lane;

  string state;

  // Ego vehicle variables
  double buffer;
  double target_speed;
  int project_size, goal_lane;
};

#endif  // VEHICLE_H
