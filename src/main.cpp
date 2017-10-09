#include <assert.h> // #include <assert> does not work, why?

#include <iomanip>

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double METERS_PER_SECOND_IN_MPH = 1609.344/3600;
double mph_2_meterps(double mph) {
  double meter_per_seconds = mph*METERS_PER_SECOND_IN_MPH;
  return meter_per_seconds;
}
const double SPEED_LIMIT = mph_2_meterps(49.5); // mph the top speed allowed
const int NUM_LANES = 3;
const double VEHICLE_LENGTH = 23.0; // meters, 23 meters is the maximum vehicle length, according to California highway standard
const double BUFFER_ZONE = 0.5*VEHICLE_LENGTH;
const double UPDATE_INTERVAL = 0.02; // seconds, the interval to update maneuver decision
const double MAX_ACCELERATION_METERS_PER_SECOND_SQUARE = 10; // meter/s^2
const double MAX_VELOCITY_DELTA_PRE_UPDATE_INTERVAL
= MAX_ACCELERATION_METERS_PER_SECOND_SQUARE*UPDATE_INTERVAL;
const double MAX_JERK_METERS_PER_SECOND_CUBIC = 10; // meter/s^3
const double MAX_ACCELERATION_DELTA_METERS_PER_UPDATE_INTERVAL
= MAX_JERK_METERS_PER_SECOND_CUBIC*UPDATE_INTERVAL;
// const int JERK_SAMPLES_NUM = int(1/UPDATE_INTERVAL); // sample times in one second
int initial_steps = 0; // 200, the intial steps without considering lane changing to stablize
const double COLLISION_C  = .1E7f;
const double DANGER_C     = .1E6f;
const double EFFICIENCY_C = .1E3f;
const double NEAR_ZERO = .1E-3f;
const double DESIRED_TIME_BUFFER = 3; // seconds, according to http://copradar.com/redlight/factors/
const double INDEFINIT_FUTURE = .1E4f; // huge number for indefinite futrue time
enum DIRECTION {LEFT = 1, RIGHT = 2};

enum MANEUVER_STATE {KL=1, LCL=2, LCR=3, PLCL=4, PLCR=5};

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {
  /* maps_x, and maps_y are the {x, y}-coordinates of the waypoints.
     Returns the index of the waypoint that is closest to the point (x, y)
   */
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(size_t i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
  /*
    maps_x, and maps_y are the {x, y}-coordinates of the waypoints.
    returns the next waypoint relative to the point (x, y) in terms of the index of waypoints.
    */
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  double heading = atan2( (map_y-y),(map_x-x) );
  double angle = abs(theta-heading);
  if(angle > pi()/4) {          // The closest waypoint has been passed by the point (x, y)
    closestWaypoint++;
  }
  return closestWaypoint;
}

// Transform from Cartesian x, y coordinates to Frenet s, d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
  /*

   */
  int next_wp = NextWaypoint(x, y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1; // circular path
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp]; // offset relative to previous waypoint
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s, d coordinates to Cartesian x, y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
  /*

   */
  int prev_wp = -1;
  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x, y};
}

int lane_width = 4;
// starting from 0, from the left most to the right most
int lane_center_d(int lane_index) {
  return (lane_index + 0.5)*lane_width;
}

bool within_lane(int lane, double d) {
  return (lane*lane_width < d) && (d < (lane+1)*lane_width);
 }

int d_to_lane_index(double d) {
  assert(0 <= d);
  assert(d <= lane_width*NUM_LANES);
  int lane_index = (int)floor(d/lane_width);
  if (lane_index < 0) {
    cout << "Negative lane index: " << lane_index << ", d: " << d << endl;
  }
  if (NUM_LANES < lane_index) {
    cout << "lane index beyond NUM_LANES: " << NUM_LANES << ", lane_index: " << lane_index << ", d: " << d << endl;
  }
  //assert(0 <= lane_index);
  //assert(lane_index < NUM_LANES);
  return lane_index;
}
// Parse the sensor_fusion data
string state_str(MANEUVER_STATE state) {
  switch(int(state)) {
  case int(KL):
    return "KL";
  case int(LCL):
    return "LCL";
  case int(LCR):
    return "LCR";
  case int(PLCL):
    return "PLCL";
  case int(PLCR):
    return "PLCR";
  default:
    return "Invalid";
  }
}

struct Decision {
  int    lane_index_changed_to; // note, for prepare to change lane, it's not changed actually
  MANEUVER_STATE maneuver;
  // double velocity_delta;
  double cost;
  double projected_acceleration;
};
struct CarDescription {
  double id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  double v;
  double a;
  double jerk;
  int    lane_index;
  bool   empty;
};

struct LaneData {
  CarDescription nearest_front;
  CarDescription nearest_back;
  // double         car_density_front;
  double gap_front;
  double gap_behind;
};

typedef map<int, LaneData> DATA_LANES;
typedef vector< vector<double> > SENSOR_FUSION;
double projected_gap(CarDescription front, CarDescription behind, double delta_t = UPDATE_INTERVAL) {
  // ignore accelerations, assuming they are 0, to simplify
  return front.s - behind.s + (front.v - behind.v)*delta_t - VEHICLE_LENGTH;
}

DATA_LANES parse_sensor_data(CarDescription my_car, SENSOR_FUSION sensor_fusion) {
  DATA_LANES data_lanes;
  for (int i = 0; i < NUM_LANES; i++) {
    LaneData lane_data;
    data_lanes[i] = lane_data; // assume copy semantics
    data_lanes[i].nearest_back.empty = true;
    data_lanes[i].nearest_front.empty = true;
  }
  // cout << "parse_sensor_data once" << endl;

  CarDescription a_car;
  for (auto data:sensor_fusion) {
    // cout << "fusion_element: ";
    // for (auto element:data) {
    //   cout << element << ", ";
    // }
    // cout << endl;
    a_car.d  = data[6];
    if ((a_car.d < 0) || (lane_width*NUM_LANES < a_car.d)) {
      continue;
    }
    a_car.id = data[0];
    a_car.x  = data[1];
    a_car.y  = data[2];
    a_car.vx = data[3];
    a_car.vy = data[4];
    a_car.s  = data[5];

    a_car.lane_index = d_to_lane_index(a_car.d);
    a_car.v = sqrt(a_car.x*a_car.x + a_car.y*a_car.y);
    a_car.empty = false;

    // cout << "a car at lane: " << a_car.lane_index;
    if (a_car.s <= my_car.s) {// there is a car behind
      if (data_lanes[a_car.lane_index].nearest_back.empty) {
        // cout << ", first registration for nearest_back ";
        data_lanes[a_car.lane_index].nearest_back       = a_car;
        // data_lanes[a_car.lane_index].nearest_back.empty = false;
      } else {
        if (data_lanes[a_car.lane_index].nearest_back.s < a_car.s) {
          data_lanes[a_car.lane_index].nearest_back = a_car;
          // cout << ", update for nearest_back ";
        }}}
    if (my_car.s <= a_car.s) { // there is a car in front
      if (data_lanes[a_car.lane_index].nearest_front.empty) {
        // cout << ", first registration for nearest_front ";
        data_lanes[a_car.lane_index].nearest_front       = a_car;
        // data_lanes[a_car.lane_index].nearest_front.empty = false;
      } else {
        if (a_car.s < data_lanes[a_car.lane_index].nearest_front.s) {
          // cout << ", update for nearest_back ";
          data_lanes[a_car.lane_index].nearest_front = a_car;
        }}}}
  // For only the legal lanes adjacent to my_car.lane_index,
  // compute the gap with the nearest front and behind respectively
  int left_lane  = my_car.lane_index -1;
  int right_lane = my_car.lane_index +1;
  vector<int> lanes_interested = {my_car.lane_index};
  if (0 <= left_lane)         lanes_interested.push_back(left_lane);
  if (right_lane < NUM_LANES) lanes_interested.push_back(right_lane);

  for (auto lane:lanes_interested) {
    if (data_lanes[lane].nearest_back.empty) {
      data_lanes[lane].gap_behind = INDEFINIT_FUTURE; // extremely large
    } else {
      data_lanes[lane].gap_behind =
        projected_gap(my_car, data_lanes[lane].nearest_back, UPDATE_INTERVAL);
    }
    if (data_lanes[lane].nearest_front.empty) {
      data_lanes[lane].gap_front = INDEFINIT_FUTURE; // extremely large
    } else {
      data_lanes[lane].gap_front =
        projected_gap(data_lanes[lane].nearest_front, my_car, UPDATE_INTERVAL);
    }
  }
  return data_lanes;
}
template <typename T>
void vector_remove(vector<T> & a_vector, T value) {
  a_vector.erase(std::remove(a_vector.begin(), a_vector.end(), value), a_vector.end());
}

template <typename T>
typename T::iterator min_map_element(T& m) {
  return min_element(m.begin(), m.end(),
                     [](typename T::value_type& l,
                        typename T::value_type& r) -> bool { return l.second.cost < r.second.cost; });
}

// constexpr unsigned int str2int(const char* str, int h = 0)
// {
//   return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
// }

double const ONE_OVER_INTERVAL_SQUARE =
  1/(UPDATE_INTERVAL*UPDATE_INTERVAL);
double const ONE_OVER_INTERVAL =
  1/UPDATE_INTERVAL;
double const ACCELERATION_ALLOW_PER_JERK_LIMIT =
  MAX_JERK_METERS_PER_SECOND_CUBIC*UPDATE_INTERVAL;


// Decision lane_keep_decision(CarDescription my_car, DATA_LANES data_lanes, int lane_changed_to) {
//   Decision decision;
//   decision.projected_acceleration =
//     acceleration_required_in_front(my_car, data_lanes, lane_changed_to);
//   decision.lane_index_changed_to = lane_changed_to;
//   return decision;
// }

// Junk yard:
// cout << "gap_front: " << setw(8) << gap_front << ", ";

// double minimum_gap = min(gap_behind, gap_front);
// decision.cost = exp(-minimum_gap - BUFFER_ZONE);
// decision.cost = exp(-gap_front);
// decision.lane_index_changed_to = lane_changed_to;

// decision.velocity_delta = gap_front <= 0 ?
//   -MAX_VELOCITY_DELTA_PER_UPDATE_INTERVAL : MAX_VELOCITY_DELTA_PER_UPDATE_INTERVAL;
// double lane_change_decision(CarDescription my_car, DATA_LANES data_lanes, int lane_changed_to) {

// }
double acceleration_required_in_front
(CarDescription my_car, DATA_LANES data_lanes, int lane_changed_to) {
  double extra_speed_allowed = SPEED_LIMIT - my_car.v;
  double speed_limit_allowed_acceleration =
    extra_speed_allowed*ONE_OVER_INTERVAL;
  double feasible_acceleration;
  if (data_lanes[lane_changed_to].nearest_front.empty) {
    feasible_acceleration = speed_limit_allowed_acceleration;
    // effective no consideration of the car in frontfs
  } else {
    double gap_front = data_lanes[lane_changed_to].gap_front;
    double available_room = gap_front - BUFFER_ZONE;
    feasible_acceleration =
      available_room*ONE_OVER_INTERVAL_SQUARE - my_car.v*ONE_OVER_INTERVAL;
  }
  double max_acceleration_allowed_by_jerk_limit;
  double acceleration;
  if (0 <= feasible_acceleration) {
    max_acceleration_allowed_by_jerk_limit =
      my_car.a + ACCELERATION_ALLOW_PER_JERK_LIMIT;
    vector<double> values
      = {speed_limit_allowed_acceleration,
         feasible_acceleration,
         max_acceleration_allowed_by_jerk_limit};
    acceleration = *min_element(values.begin(), values.end());
  }
  else {
    max_acceleration_allowed_by_jerk_limit =
      my_car.a - ACCELERATION_ALLOW_PER_JERK_LIMIT;
    acceleration = max(feasible_acceleration,
                       max_acceleration_allowed_by_jerk_limit);
  }
  return acceleration;
}
double acceleration_required_behind
(CarDescription my_car, DATA_LANES data_lanes, int lane_index) {
  if (data_lanes[lane_index].nearest_back.empty) {
    return my_car.a; // no need to change
  } else {
    double gap_behind = data_lanes[lane_index].gap_behind;
    double delta_v =
      my_car.v - data_lanes[lane_index].nearest_back.v;
    double min_acceleration_pushed_by_nearest_back =
      (delta_v*delta_v)/(2*gap_behind);
    double acceleration =
      min(min_acceleration_pushed_by_nearest_back,
          my_car.a + ACCELERATION_ALLOW_PER_JERK_LIMIT);
    return acceleration;
  }
  return my_car.a; // will not be executed, just to fool the compiler
  }

Decision project_maneuver(MANEUVER_STATE proposed_state, CarDescription my_car, DATA_LANES data_lanes) {
  Decision decision;
  // double acceleration = 0;
  switch(int(proposed_state)) {
  case int(KL):
    decision.projected_acceleration = acceleration_required_in_front(my_car, data_lanes, my_car.lane_index);
    decision.lane_index_changed_to = my_car.lane_index;
    break;
  case int(LCL):
    decision.projected_acceleration = acceleration_required_in_front(my_car, data_lanes, my_car.lane_index-1);
    decision.lane_index_changed_to = my_car.lane_index-1;
    break;
  case int(LCR):
    decision.projected_acceleration = acceleration_required_in_front(my_car, data_lanes, my_car.lane_index+1);
    decision.lane_index_changed_to = my_car.lane_index+1;
    break;
  case int(PLCL):
    decision.lane_index_changed_to = my_car.lane_index;
    decision.projected_acceleration = acceleration_required_behind(my_car, data_lanes, my_car.lane_index -1);
    break;
  case int(PLCR):
    decision.lane_index_changed_to = my_car.lane_index;
    decision.projected_acceleration = acceleration_required_behind(my_car, data_lanes, my_car.lane_index +1);
    break;
  default:
    cout << "Not supported proposed state: " << proposed_state << endl;
    break;
  };
  decision.maneuver = proposed_state;
  return decision;              // this decision's state needs to be evaluated
}
double positive_minimum_solution(double a, double b, double c) {
  double d = b*b -4*a*c;
  double s1 = (-b + d)/(2*a);
  double s2 = (-b - d)/(2*a);
  vector<double> buf;
  if (0 <= s1) buf.push_back(s1);
  if (0 <= s2) buf.push_back(s2);

  if (0 < buf.size()) {
    return *min_element(buf.begin(), buf.end());
  } else {
    return INDEFINIT_FUTURE; // no solution
  }
}

double collision_cost_f(Decision decision, CarDescription my_car, DATA_LANES data_lanes) {
  double front_collision_cost = 0;
  double behind_collision_cost = 0;

  if (!data_lanes[decision.lane_index_changed_to].nearest_front.empty) {
    double a_f = 0.5*decision.projected_acceleration;
    double b_f = my_car.v - data_lanes[decision.lane_index_changed_to].nearest_front.v;
    double c_f = my_car.s
      - data_lanes[decision.lane_index_changed_to].nearest_front.s + VEHICLE_LENGTH;
    double front_collision_time = positive_minimum_solution(a_f, b_f, c_f);
    if (INDEFINIT_FUTURE <= front_collision_time) {
      front_collision_cost = 0;
    } else {
      front_collision_cost = exp(-pow(front_collision_time, 2));
    }} else {
    front_collision_cost = 0;
  }

  if (!data_lanes[decision.lane_index_changed_to].nearest_back.empty) {
    double a_b = 0.5*decision.projected_acceleration;
    double b_b = my_car.v - data_lanes[decision.lane_index_changed_to].nearest_back.v;
    double c_b = my_car.s
      - data_lanes[decision.lane_index_changed_to].nearest_back.s - VEHICLE_LENGTH;
    double behind_collision_time = positive_minimum_solution(a_b, b_b, c_b);
    if (INDEFINIT_FUTURE <= behind_collision_time) {
      behind_collision_cost = 0;
    } else {
      behind_collision_cost = exp(-pow(behind_collision_time, 2));
    }} else {
    behind_collision_cost = 0;
  }
  double cost = front_collision_cost + behind_collision_cost;
  return cost;
}
double buffer_cost_f(Decision decision, CarDescription my_car, DATA_LANES data_lanes) {
  // double cost = 0;
  double projected_v = (my_car.v + decision.projected_acceleration*UPDATE_INTERVAL);
  if ((projected_v <= NEAR_ZERO) ||
      (!data_lanes[decision.lane_index_changed_to].nearest_front.empty &&
       (data_lanes[decision.lane_index_changed_to].gap_front <= NEAR_ZERO)) ||
      (!data_lanes[decision.lane_index_changed_to].nearest_back.empty &&
       (data_lanes[decision.lane_index_changed_to].gap_behind <= NEAR_ZERO))) {
    return 10; // collision already
  } else {
    if (!data_lanes[decision.lane_index_changed_to].nearest_front.empty) {
      double time_away = data_lanes[decision.lane_index_changed_to].gap_front/projected_v;
      if (DESIRED_TIME_BUFFER < time_away) {
        return 0.0;
      } else {
        return (1.0 - pow(DESIRED_TIME_BUFFER - time_away, 2));
    }} else {
      return 0.0;
    }}
  return 0.0;
}
double inefficiency_cost_f(Decision decision, CarDescription my_car,DATA_LANES data_lanes) {
  double projected_v = (my_car.v + decision.projected_acceleration*UPDATE_INTERVAL);
  double cost = pow((SPEED_LIMIT - projected_v)/SPEED_LIMIT, 2);
  return cost;
}
double calculate_cost(Decision decision, CarDescription my_car, DATA_LANES data_lanes) {
  double collision_cost = collision_cost_f(decision, my_car, data_lanes);
  double inefficiency_cost = inefficiency_cost_f(decision, my_car, data_lanes);
  double buffer_cost = buffer_cost_f(decision, my_car, data_lanes);
  double cost = COLLISION_C*collision_cost + DANGER_C*buffer_cost + EFFICIENCY_C*inefficiency_cost;
  return cost;
}

Decision evaluate_decision(MANEUVER_STATE proposed_state, CarDescription my_car, DATA_LANES data_lanes) {
  Decision decision = project_maneuver(proposed_state, my_car, data_lanes);
  decision.cost = calculate_cost(decision, my_car, data_lanes);
  return decision;
}
Decision maneuver(CarDescription my_car, DATA_LANES data_lanes) {
  vector<MANEUVER_STATE> states = {KL, LCL, LCR, PLCL, PLCR};

  // starting from 0, from the left most to the right most
  if (my_car.lane_index == 0) {// no more left turn
    vector_remove(states, LCL);
    vector_remove(states, PLCL);
  }
  if (my_car.lane_index == NUM_LANES-1) {// no more right turn
    vector_remove(states, LCR);
    vector_remove(states, PLCR);
  }
  map<MANEUVER_STATE, Decision> decisions;
  for (auto proposed_state:states) {
    Decision a_decision = evaluate_decision(proposed_state, my_car, data_lanes);
    cout << state_str(proposed_state) << ", cost: " << setw(7) <<  a_decision.cost << ", ";
    decisions[proposed_state] = a_decision;
  }
  Decision decision = min_map_element(decisions)->second;
  cout << "Selected maneuver: " << setw(4) << state_str(decision.maneuver) << ", cost: " << setw(7) << decision.cost << " ";
  return decision;
}

// for (size_t i = 0; i < sensor_fusion.size(); i++) {
//   // car in in my lane
//   float d = sensor_fusion[i][6];
//   // the format of sensor_fusion data: vector of vector of id, x, y, vx, vy, s, d
//   if (within_lane(lane_index, d)) {
//     double vx = sensor_fusion[i][3];
//     double vy = sensor_fusion[i][4];
//     double another_car_speed = sqrt(vx*vx + vy*vy);
//     double another_car_projected_s =
//       (double)sensor_fusion[i][5] + ((double)prev_size*UPDATE_INTERVAL*another_car_speed);
//     // the position of the other car in the slight future
//     if ((car_s < another_car_projected_s) &&
//         ((another_car_projected_s - car_s) < 30)) {
//       // the other car is in front, and too close, within 30 meters distance
//       // lower reference velocity so my car dosen't crash into the car in front
//       // could flag to try to change lane
//       // ref_val = 29.5; // mph
//       too_close = true;

//       if (lane_index > 0) {
//         lane_index = 0;
//       }
//     }
//   }
//  }
// // end of rear collision

// // incremental acceleration/deacceleration

// if (too_close && (0 < ref_val)) {
//   ref_val -= 0.224; // roughly equivalent to deacceleration 5m/s^2
//   // cout << "ref_val: " << ref_val << endl;
//  } else if (ref_val < SPEED_LIMIT) {
//   ref_val += 0.224; // roughly equivalent to acceleration 5m/s^2
//   // cout << "ref_val: " << ref_val << endl;
//  }
// // end of acceleration/deacceleration

int main() {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
  
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;
  
    ifstream in_map_(map_file_.c_str(), ifstream::in);
  
    string line;
    while (getline(in_map_, line)) {
    	istringstream iss(line);
    	double x;
    	double y;
    	float s;
    	float d_x;
    	float d_y;
    	iss >> x;
    	iss >> y;
    	iss >> s;
    	iss >> d_x;
    	iss >> d_y;
    	map_waypoints_x.push_back(x);
    	map_waypoints_y.push_back(y);
    	map_waypoints_s.push_back(s);
    	map_waypoints_dx.push_back(d_x);
    	map_waypoints_dy.push_back(d_y);
    }
  double ref_val = MAX_VELOCITY_DELTA_PRE_UPDATE_INTERVAL; // initial
  CarDescription my_car;
  my_car.a = 0;
  my_car.jerk = 0;
  
  uWS::Hub h;
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
               &my_car, &ref_val]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"]; // in degree
            double car_speed = j[1]["speed"]; // in mile per hour

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            // double end_path_d = j[1]["end_path_d"]; // not yet used, keep might be needed

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            // int lane_index = d_to_lane_index(car_d);
            // starting from 0, from the left most to the right most
            // cout << "lane_index: " << lane_index << endl;
            
            // Need to work on to avoid rear collision
            // bool too_close = false;
            // find ref_v to use
            
            // CarDescription my_car;
            my_car.id = -1; // hopefully impossible id of the other cars
            my_car.x  = car_x;
            my_car.y  = car_y;
            
            double old_v = my_car.v;
            
            my_car.v  = mph_2_meterps(car_speed);
            my_car.vx = data[3];
            my_car.vy = data[4];
            my_car.s  = car_s;
            my_car.d  = car_d;
            assert(0 < car_d);
            my_car.lane_index = d_to_lane_index(car_d);
            
            double old_a = my_car.a;
            
            my_car.a = (my_car.v - old_v)/UPDATE_INTERVAL;
            
            my_car.jerk = (my_car.a - old_a)/UPDATE_INTERVAL;
            
            DATA_LANES data_lanes = parse_sensor_data(my_car, sensor_fusion);
            vector<double> next_d = {car_d, car_d, car_d}; // the d coordinates for the seeding points for spline
            
            ios::fmtflags old_settings = cout.flags();
            Decision decision;
            decision.maneuver = KL; // default
            if (initial_steps == 0) {
              cout.precision(2);
              cout << "fr col: " << (!data_lanes[my_car.lane_index].nearest_front.empty &&
                                     (data_lanes[my_car.lane_index].gap_front <= 0)) << ", ";
              cout << "rr col: " << (!data_lanes[my_car.lane_index].nearest_back.empty &&
                                     (data_lanes[my_car.lane_index].gap_behind <= 0)) << ", ";
              decision = maneuver(my_car, data_lanes);
              // next_d = lane_center_d(decision.lane_index_changed_to);
              int shift_direction = 0;
              if (decision.maneuver == LCL) {
                shift_direction = -1;
              } else if (decision.maneuver == LCR) {
                shift_direction = 1;
              } else {
                shift_direction = 0;
              }
              double lane_gap = abs(car_d - lane_center_d(decision.lane_index_changed_to));
              next_d = {car_d + shift_direction * lane_gap * 0.2,
                        car_d + shift_direction * lane_gap * 0.3,
                        car_d + shift_direction * lane_gap * 0.5
              };
            
              cout << ", next lane: " << decision.lane_index_changed_to;
              cout << " prev lane: " << my_car.lane_index; // << endl;
            
              ref_val = decision.projected_acceleration*UPDATE_INTERVAL + my_car.v;
              cout << "new speed: " << ref_val;
            
              // if (((0 < decision.velocity_delta) && (ref_val < SPEED_LIMIT)) ||
              //     ((decision.velocity_delta < 0) && (0 < ref_val)))
              //   //ref_val += decision.velocity_delta;
              //   // The above needs to be changed
              //   ref_val = decision.projected_acceleration + my_car.v; // ref_val might need to be converted back to mph
            
              //data_lanes[decision.lane_index_changed_to].nearest_front.v;
             } else {
              if (ref_val < SPEED_LIMIT) {
              // need to review the following
                ref_val += MAX_VELOCITY_DELTA_PRE_UPDATE_INTERVAL;
              }
              initial_steps -= 1;
             }
            
            // double dist_inc = 0.5;
            // double next_s = 0;
            
            // seeding points to generate smooth trajectory through spline
            vector<double> ptsx;
            vector<double> ptsy;
            
            // reference x, y, yaw
            // either we will reference the starting point as where the car is
            // or at the previous paths and points
            double ref_x   = car_x;
            double ref_y   = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            int prev_size  = previous_path_x.size();
            // cout << ", prev_size: " << prev_size;
            cout << endl;
            
            if (prev_size < 2) {
              // If the previous state is almost empty,
              // use car's current position as starting reference
              // use two points to make the path tangent to the car
              double prev_car_x = car_x - cos(ref_yaw); // converted from car_yaw deg2rad
              double prev_car_y = car_y - sin(ref_yaw);
            
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
            
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
             } else { // use the previous path's end point as starting reference
              // Redefine reference state by previous path and point
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
            
              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev); // in radians unit
            
              // use two points to make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
            
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
              cout << "ref_x/y_prev: (" << ref_x_prev << ", " << ref_y_prev << "), ref_x/y: (" <<
                ref_x << ", " << ref_y << ")" << endl;
             }
            // In Frenet add evenly 30m spaced points ahead of the state reference
            // For smooth path generation, replace car_s by end_path_s, for continuity
            if (0 < prev_size) { // replace car_s
              car_s = end_path_s;
             }
            
            vector<double> next_wp0 = getXY(car_s + 30, next_d[0],
                                            map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, next_d[1],
                                            map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, next_d[2],
                                            map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);
            
            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);
            
            // Convert to local coordinates with ref_x, ref_y as the origin,
            // and -ref_yaw as the angle relative to the global x-axis
            for (size_t i = 0; i < ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
            
              // shift car reference angle to 0 degree
              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
             }
            
            cout << "x: ";
            for (auto x:ptsx) {
              cout << setw(9) << x << ", ";
             }
            cout << endl;
            
            cout << "y: ";
            for (auto y:ptsy) {
              cout << setw(9) << y << ", ";
             }
            cout << endl;
            
            cout.flags(old_settings);
            
            // Create a spline
            tk::spline spline;
            // set (x, y) points to the spline
            spline.set_points(ptsx, ptsy);
            
            // Define the actual (x, y) points in the planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            // Start with all of the previous path points from last time
            size_t start_using_previous_path = 0;
            if (decision.maneuver != KL) {
              start_using_previous_path = previous_path_x.size() - 10; // use the last part of the previous path
             }
            for (size_t i = start_using_previous_path; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            
            // Calculate how to break up spline points
            // so that we travel at desired reference velocity
            double target_x = 30.0;
            double target_y = spline(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            
            double x_add_on = 0;
            
            // Fill up the rest of the points for the planner
            const int plan_length = 50;
            for (size_t i = 1; i <= plan_length - (previous_path_x.size() - start_using_previous_path); i++) {
              // double num_equal_segments = (target_dist/(UPDATE_INTERVAL*ref_val/2.24));
              double num_equal_segments = (target_dist/(UPDATE_INTERVAL*ref_val));
              // conversion between mile and kilometer
              double x_point = x_add_on+(target_x)/num_equal_segments;
              double y_point = spline(x_point);
              x_add_on = x_point;
            
              double x_ref = x_point;
              double y_ref = y_point;
            
              // rotate and shift back to global coordinates
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
              x_point += ref_x;
              y_point += ref_y;
            
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
             }

            json msgJson;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

     h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
         std::cout << "Connected!!!" << std::endl;
       });
     
       h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                              char *message, size_t length) {
         ws.close();
         std::cout << "Disconnected" << std::endl;
       });

     int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
