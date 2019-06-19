#ifndef CONSTANTS_H
#define CONSTANTS_H

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554; // meters?

// Lane parameters
const double LANE_WIDTH = 4.0; // meters
const int LANES_AVAILABLE = 3;

// Speed limit
const double SPEED_LIMIT = 50.0; // MPH

// Max acceleration
const double MAX_ACCELERATION = 5.0; // m/s/s

// Convert between MPH and m/s
const double CONVERT_FACTOR = 2.24; // units (MPH divided by m/s)

// Time interval between points
const double DT = 0.02; // seconds

#endif  // CONSTANTS_H
