#ifndef PARAMETERS
#define PARAMETERS

/*
  parameters.h
  The parameters for path planning design.

*/
const double METERS_PER_SECOND_IN_MPH = 1609.344/3600;
double mph_2_meterps(double mph) {
  double meter_per_seconds = mph*METERS_PER_SECOND_IN_MPH;
  return meter_per_seconds;
}
const double SPEED_LIMIT = mph_2_meterps(49.0); // mph the top speed allowed
// const double MINIMUM_SPEED = mph_2_meterps(5.0); // the minimum speed to get moving
const int NUM_LANES = 3;
// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;

const double VEHICLE_LENGTH = 3.0; // meters, 23 meters is the maximum vehicle length, according to California highway standard
// const double BUFFER_ZONE = 10*VEHICLE_LENGTH;
const double NEARBY = 1*VEHICLE_LENGTH; // metres, very near to my_car

const double UPDATE_INTERVAL = 0.02; // seconds, the interval to update maneuver decision

const int PLANNED_TRAJECTORY_LENGTH = 50; // 3; // the length of the planned trajectory fed to the simulator
// In the current implementation, PLANNED_TRAJECTORY_LENGTH cannot be larger than 10. It might be a bug in the implementation.
const int NUM_ADOPTED_REMAINING_TRAJECTORY_POINTS = 50; // 3, 30;
// the length of the first portion of the remaining trajectory (previous_path)
// from experiment, it seems 25 might be too few when the CPU is busy.

const double VELOCITY_INCREMENT_LIMIT = 0.07; // 0.125;

const double MAX_ACCELERATION_METERS_PER_SECOND_SQUARE = 10; // meter/s^2
const double MAX_VELOCITY_DELTA_PRE_UPDATE_INTERVAL
= MAX_ACCELERATION_METERS_PER_SECOND_SQUARE * UPDATE_INTERVAL;
// const double MAX_VELOCITY_DELTA_PRE_UPDATE_INTERVAL = 0.015; // The above seems too big still

const double MAX_JERK_METERS_PER_SECOND_CUBIC = 10; // meter/s^3
const double MAX_ACCELERATION_DELTA_METERS_PER_UPDATE_INTERVAL
= MAX_JERK_METERS_PER_SECOND_CUBIC * UPDATE_INTERVAL;
const double COLLISION_C  = .1E6f;
const double DANGER_C     = .1E7f;
const double EFFICIENCY_C = .1E3f;
const double NOT_MIDDLE_C = .1E1f;
const double LANE_CHANGE_C= .1E3f;
const double NEAR_ZERO = .1E-1f;
const double DESIRED_TIME_BUFFER = 10; // seconds, according to http://copradar.com/redlight/factors/ ; change from 30 to 10 for better differentiation
const double SAFE_DISTANCE = 90.0; // meters, huge number for indefinite futrue time

#endif
