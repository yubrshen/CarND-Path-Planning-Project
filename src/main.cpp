#include <assert.h> // #include <assert> does not work, why?

#include <iomanip>

#include <fstream>

#include <iostream>

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
#include "parameters.h"
#include "utils.h"

using namespace std;

// for convenience
using json = nlohmann::json;

enum DIRECTION {LEFT = 1, RIGHT = 2};

enum MANEUVER_STATE {KL=1, LCL=2, LCR=3, PLCL=4, PLCR=5};

vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y,
                                  vector<double> eval_at_x) {
  // uses the spline library to interpolate points connecting a series of x and y values
  // output is spline evaluated at each eval_at_x point

  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
    return { 0 };
  }

  tk::spline s;
  s.set_points(pts_x,pts_y);    // currently it is required that X is already sorted
  vector<double> output;
  for (double x: eval_at_x) {
    output.push_back(s(x));
  }
  return output;
}

vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y,
                                  double interval, int output_size) {
  // uses the spline library to interpolate points connecting a series of x and y values
  // output is output_size number of y values beginning at y[0] with specified fixed interval

  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
    return { 0 };
  }

  tk::spline s;
  s.set_points(pts_x,pts_y);    // currently it is required that X is already sorted
  vector<double> output;
  for (int i = 0; i < output_size; i++) {
    output.push_back(s(pts_x[0] + i * interval));
  }
  return output;
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
struct KINEMATIC_DATA {
  double a;
  double v;
  double gap_front;
  double gap_behind;
  double horizon; // evaluation horizon
};

struct Decision {
  int    lane_index_changed_to; // note, for prepare to change lane, it's not changed actually
  MANEUVER_STATE maneuver;
  // double velocity_delta;
  double cost;
  KINEMATIC_DATA projected_kinematics; // for key: "velocity", and "acceleration"
};

struct CarDescription {
  double id;
  double x;
  double y;
  double yaw;
  double vx;
  double vy;
  double s;
  double d;
  double v;
  double remaining_path_end_s;
  double remaining_path_end_d;
  double a;
  double jerk;
  int    lane_index;
  bool   empty;
};

struct LaneData {
  CarDescription nearest_front;
  CarDescription nearest_back;
  // double         car_density_front;
  //double gap_front;
  //double gap_behind;
};

struct DATA_LANES {
  map<int, LaneData> lanes;
  //double projected_duration;
  bool car_to_left = false;
  bool car_to_right = false;
  bool car_crashing_front_or_behind = false;
};
struct TRAJECTORY {
  vector<double> x_vals;
  vector<double> y_vals;
};

typedef vector< vector<double> > SENSOR_FUSION;
// double projected_gap(CarDescription front, CarDescription behind, double delta_t = UPDATE_INTERVAL) {
//     // ignore accelerations, assuming they are 0, to simplify
//     return front.s - behind.s + (front.v - behind.v)*delta_t - VEHICLE_LENGTH;
//   }

double projected_gap_front(double front_s, double front_v,
                           double behind_s, double behind_v, double behind_a,
                           double delta_t) {
  double gap = front_s - behind_s + (front_v - behind_v)*delta_t +
    - 0.5*behind_a*(delta_t*delta_t) - VEHICLE_LENGTH;
  return gap;
}

double projected_gap_behind(double behind_s, double behind_v,
                            double front_s, double front_v, double front_a,
                            double delta_t) {
  double gap = front_s - behind_s + (front_v - behind_v)*delta_t +
    + 0.5*front_a*(delta_t*delta_t) - VEHICLE_LENGTH;
  return gap;
}
void update_surronding(CarDescription my_car, double s_diff, int lane, DATA_LANES *data_lanes) {
  data_lanes-> car_crashing_front_or_behind = false;
  data_lanes-> car_to_left    = false;
  data_lanes-> car_to_right   = false;
  if ((0 <= s_diff) && (s_diff < NEARBY)) {
    switch (my_car.lane_index - lane) {
    case 0:
      data_lanes->car_crashing_front_or_behind = true;
      break;
    case 1:
      data_lanes->car_to_left = true;
      break;
    case -1:
      data_lanes->car_to_right = true;
    default:
      break;
    }} else {
    // cout <<"car_{right, left, ahead}: " << data_lanes->car_to_right << ", " << data_lanes->car_to_left << ", " << data_lanes->car_crashing_front_or_behind;
  }
}

DATA_LANES parse_sensor_data(CarDescription my_car, SENSOR_FUSION sensor_fusion) {
  DATA_LANES data_lanes;
  for (int i = 0; i < NUM_LANES; i++) {
    LaneData lane_data;
    data_lanes.lanes[i] = lane_data; // assume copy semantics
    data_lanes.lanes[i].nearest_back.empty = true;
    data_lanes.lanes[i].nearest_front.empty = true;
  }

  CarDescription a_car;
  for (auto data:sensor_fusion) {
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
      if (data_lanes.lanes[a_car.lane_index].nearest_back.empty) {
        // cout << ", first registration for nearest_back ";
        data_lanes.lanes[a_car.lane_index].nearest_back       = a_car;
      } else {
        if (data_lanes.lanes[a_car.lane_index].nearest_back.s < a_car.s) {
          data_lanes.lanes[a_car.lane_index].nearest_back = a_car;
          // cout << ", update for nearest_back ";
        }}}
    if (my_car.s <= a_car.s) { // there is a car in front
      if (data_lanes.lanes[a_car.lane_index].nearest_front.empty) {
        // cout << ", first registration for nearest_front ";
        data_lanes.lanes[a_car.lane_index].nearest_front       = a_car;
      } else {
        if (a_car.s < data_lanes.lanes[a_car.lane_index].nearest_front.s) {
          // cout << ", update for nearest_back ";
          data_lanes.lanes[a_car.lane_index].nearest_front = a_car;
        }}}}
  // For only the legal lanes adjacent to my_car.lane_index,
  int left_lane  = my_car.lane_index -1;
  int right_lane = my_car.lane_index +1;
  vector<int> lanes_interested = {my_car.lane_index};
  if (0 <= left_lane)         lanes_interested.push_back(left_lane);
  if (right_lane < NUM_LANES) lanes_interested.push_back(right_lane);
  for (auto lane:lanes_interested) {
    if (!data_lanes.lanes[lane].nearest_back.empty) {
      double s_diff = my_car.s - data_lanes.lanes[lane].nearest_back.s;
      update_surronding(my_car, s_diff, lane, &data_lanes);
    }
    if (!data_lanes.lanes[lane].nearest_front.empty) {
      double s_diff = data_lanes.lanes[lane].nearest_front.s - my_car.s;
      update_surronding(my_car, s_diff, lane, &data_lanes);
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



void update_gaps_in_kinematic(CarDescription front, CarDescription my_car, CarDescription behind,
                              double horizon, KINEMATIC_DATA *kinematic) {
  kinematic->horizon = horizon;
  if (behind.empty) {
    kinematic->gap_behind = SAFE_DISTANCE; // extremely large
  } else {
    kinematic->gap_behind =
      projected_gap_behind(behind.s, behind.v, my_car.s, kinematic->v, kinematic->a, kinematic->horizon);
  }
  if (front.empty) {
    kinematic->gap_front = SAFE_DISTANCE; // extremely large
  } else {
    kinematic->gap_front =
      projected_gap_front(front.s, front.v, my_car.s, kinematic->v, kinematic->a, kinematic->horizon);
  }
}

const double SPEED_ADJUSTMENT_PERIOD = 30; // seconds
double const ONE_OVER_ADJUSTMENT_INTERVAL_SQUARE =
  1/(SPEED_ADJUSTMENT_PERIOD * SPEED_ADJUSTMENT_PERIOD);
double const ONE_OVER_ADJUSTMENT_INTERVAL = 1/SPEED_ADJUSTMENT_PERIOD;

// KINEMATIC_DATA kinematic_required_in_front
// (CarDescription my_car, DATA_LANES data_lanes, int lane_changed_to) {
//   KINEMATIC_DATA kinematic;
//   // double acceleration;
//   double extra_speed_allowed = SPEED_LIMIT - my_car.v;
//   double speed_limit_allowed_acceleration =
//     extra_speed_allowed * ONE_OVER_ADJUSTMENT_INTERVAL;
//   double feasible_acceleration;
//   // double target_v = SPEED_LIMIT;

//   if (data_lanes.lanes[lane_changed_to].nearest_front.empty) {
//     feasible_acceleration = speed_limit_allowed_acceleration;
//     // effective no consideration of the car in frontfs
//   } else {
//     double gap_front = data_lanes.lanes[lane_changed_to].nearest_front.s - my_car.s;
//       // data_lanes.lanes[lane_changed_to].gap_front;
//     double available_room = gap_front - BUFFER_ZONE;
//     feasible_acceleration =
//       available_room * ONE_OVER_ADJUSTMENT_INTERVAL_SQUARE - my_car.v * ONE_OVER_ADJUSTMENT_INTERVAL;
//   }
//   if (feasible_acceleration <= 0) {
//     kinematic.a = feasible_acceleration;
//     // collision is happening at the time, deceleration immediately
//     // The time is at the end of the adopted remaining trajectory, and
//     // the start of new planned trajectory
//   } else { // 0 < feasible_acceleration
//     if (speed_limit_allowed_acceleration < 0) { // speeding over limit
//       double acceleration_delta
//         = min(fabs(my_car.a - speed_limit_allowed_acceleration),
//               MAX_ACCELERATION_DELTA_METERS_PER_UPDATE_INTERVAL);
//       kinematic.a = my_car.a - acceleration_delta;
//     } else { // 0 <= speed_limit_allowed_acceleration and 0 < feasible_acceleration
//       kinematic.a
//         = min(min(speed_limit_allowed_acceleration,
//                              feasible_acceleration),
//               my_car.a + MAX_ACCELERATION_DELTA_METERS_PER_UPDATE_INTERVAL);
//     }}
//   // Consider the sensed car in front may not be far away
//   // The Ego's speed shall be at most than that of the car in front
//   kinematic.v = min(data_lanes.lanes[lane_changed_to].nearest_front.v,
//                  my_car.v + kinematic.a * UPDATE_INTERVAL); // kinematic.v is used per UPDATE_INTERVAL

//   update_gaps_in_kinematic(data_lanes.lanes[lane_changed_to].nearest_front,
//                            my_car,
//                            data_lanes.lanes[lane_changed_to].nearest_back,
//                            10*UPDATE_INTERVAL, &kinematic);
//   return kinematic;
// }
KINEMATIC_DATA kinematic_required_in_front
(CarDescription my_car, DATA_LANES data_lanes, int lane_changed_to) {
  KINEMATIC_DATA kinematic;
  kinematic.v = SPEED_LIMIT; // assuming there is no car in front.
  kinematic.horizon = 200*UPDATE_INTERVAL; // 4 seconds
  double projected_my_car_s    = my_car.s + kinematic.horizon*(my_car.v + kinematic.v)/2;
  double projected_front_car_s
    = data_lanes.lanes[lane_changed_to].nearest_front.s
    + kinematic.horizon*data_lanes.lanes[lane_changed_to].nearest_front.v;
  double gap_front = projected_front_car_s - projected_my_car_s;
  if (gap_front < SAFE_DISTANCE) {
    kinematic.v = data_lanes.lanes[lane_changed_to].nearest_front.v;
  }
  if (data_lanes.car_crashing_front_or_behind) {
    kinematic.v = 0.0;
  }
  kinematic.a = (kinematic.v - my_car.v)/kinematic.horizon;
  return kinematic;
}
//map<string, double>
KINEMATIC_DATA kinematic_required_behind
(CarDescription my_car, DATA_LANES data_lanes, int lane_index) {
  KINEMATIC_DATA kinematic;
  if (data_lanes.lanes[lane_index].nearest_back.empty) {
    kinematic.a = my_car.a;
    kinematic.v = my_car.v;
  } else {
    double gap_behind = my_car.s - data_lanes.lanes[lane_index].nearest_back.s;
    if (gap_behind <= 0) { // invalid with assumption that the other car is behind
      kinematic.a = my_car.a;
      kinematic.v = my_car.v;
    } else {
      double delta_v =
        my_car.v - data_lanes.lanes[lane_index].nearest_back.v;
      double min_acceleration_pushed_by_nearest_back =
        (delta_v*delta_v)/(2*gap_behind);
      kinematic.a =
        min(min_acceleration_pushed_by_nearest_back,
            my_car.a + MAX_ACCELERATION_DELTA_METERS_PER_UPDATE_INTERVAL);
      kinematic.v = min(data_lanes.lanes[lane_index].nearest_front.v,
                        my_car.v + kinematic.a * UPDATE_INTERVAL); // kinematic.v is used per UPDATE_INTERVAL
    }}
  update_gaps_in_kinematic(data_lanes.lanes[lane_index].nearest_front,
                           my_car,
                           data_lanes.lanes[lane_index].nearest_back,
                           10*UPDATE_INTERVAL, &kinematic);
  return kinematic;
}
Decision project_maneuver(MANEUVER_STATE proposed_state, CarDescription my_car, DATA_LANES data_lanes) {
  Decision decision;
  int changed_lane = my_car.lane_index;

  switch(int(proposed_state)) {
  case int(KL):
    decision.projected_kinematics = kinematic_required_in_front(my_car, data_lanes, my_car.lane_index);
    decision.lane_index_changed_to = my_car.lane_index;
    break;
  case int(LCL):
    changed_lane = my_car.lane_index-1;
    decision.projected_kinematics = kinematic_required_in_front(my_car, data_lanes, changed_lane);
    decision.lane_index_changed_to = changed_lane;
    break;
  case int(LCR):
    changed_lane = my_car.lane_index+1;
    decision.projected_kinematics = kinematic_required_in_front(my_car, data_lanes, changed_lane);
    decision.lane_index_changed_to = changed_lane;
    break;
  case int(PLCL):
    decision.lane_index_changed_to = my_car.lane_index;
    // no lane change yet, but evaluate with the proposed change
    decision.projected_kinematics = kinematic_required_behind(my_car, data_lanes, my_car.lane_index -1);
    break;
  case int(PLCR):
    decision.lane_index_changed_to = my_car.lane_index;
    // no lane change yet, but evaluate with the proposed change
    decision.projected_kinematics = kinematic_required_behind(my_car, data_lanes, my_car.lane_index +1);
    break;
  default:
    cout << "Not supported proposed state: " << proposed_state << endl;
    break;
  };
  decision.maneuver = proposed_state;
  cout // <<  "prop. man.: "
       << state_str(decision.maneuver) << ", " << " lane to: " << decision.lane_index_changed_to << ", ";
  return decision;              // this decision's state needs to be evaluated
}
double logistic(double x) {
  // returns a value between 0 and 1 for x in the range[0, infinity] and
  // - 1 to 0 for x in the range[-infinity, infinity].
  // Useful for cost functions.
  return 2.0 / (1 + exp(-x)) - 1.0;
  }
vector<double> solv_2nd_degree_poly(double a, double b, double c) {
  double d  = sqrt(b*b -4*a*c);
  double s1 = (-b + d)/(2*a);
  double s2 = (-b - d)/(2*a);
  return {s1, s2};
}

double collision_time_in_future(double a, double b, double c, double horizon) {
  vector<double> candidates = solv_2nd_degree_poly(a, b, c);
  double s0 = candidates[0] + horizon;
  double s1 = candidates[1] + horizon;
  double s  = SAFE_DISTANCE;
  if (0 <= s0) {
    s = s0;
  }
  if ((0 <= s1) && (s1 < s)) {
    s = s1;
  }
  return s;
}

double collision_cost_f(Decision decision, CarDescription my_car, DATA_LANES data_lanes) {
  double front_collision_cost  = 0;
  double behind_collision_cost = 0;
  double gap_front_0  = SAFE_DISTANCE;
  double gap_behind_0 = SAFE_DISTANCE;

  if (!data_lanes.lanes[decision.lane_index_changed_to].nearest_front.empty) {
    gap_front_0 = (data_lanes.lanes[decision.lane_index_changed_to].nearest_front.s - my_car.s);
  }

  if (!data_lanes.lanes[decision.lane_index_changed_to].nearest_back.empty) {
    gap_behind_0 = (my_car.s - data_lanes.lanes[decision.lane_index_changed_to].nearest_back.s);
  }
  // cout << " lane studied: " << decision.lane_index_changed_to << ", ";

  // if ((SAFE_DISTANCE <= decision.projected_kinematics.gap_front) &&
  //     (SAFE_DISTANCE <= decision.projected_kinematics.gap_behind)) {
  //   // for the case, when there is no car in front or behind
  //   return 0;
  // }
  if ((SAFE_DISTANCE <= gap_front_0) &&
      (SAFE_DISTANCE <= gap_behind_0)) {
    // for the case, when there is no car in front or behind
    return 0;
  }

  if (gap_front_0  < BUFFER_ZONE ||
      gap_behind_0 < BUFFER_ZONE) {
    cout << " too close, ";
    return 2.0;
  }

  // evaluate collision risk with the projected accelerate and speed
  // over a period of horizon
  // double a_f = 0.5*decision.projected_kinematics.a;
  double a_f   = 0.5*my_car.a;
  // double b_f = decision.projected_kinematics.v
  double b_f   = my_car.v
    - data_lanes.lanes[decision.lane_index_changed_to].nearest_front.v;
  double c_f   = my_car.s
    - data_lanes.lanes[decision.lane_index_changed_to].nearest_front.s + VEHICLE_LENGTH;
  double front_collision_time
    //  = collision_time_in_future(a_f, b_f, c_f, decision.projected_kinematics.horizon);
    = collision_time_in_future(a_f, b_f, c_f, 0.0);

  front_collision_cost = exp(-pow(front_collision_time, 2));
  cout << " coll. in front in " << front_collision_time << " sec. ";

  // double a_b = 0.5*decision.projected_kinematics.a;
  double a_b = 0.5*my_car.a;
  // double b_b = decision.projected_kinematics.v
  double b_b = my_car.v
    - data_lanes.lanes[decision.lane_index_changed_to].nearest_back.v;
  double c_b = my_car.s
    - data_lanes.lanes[decision.lane_index_changed_to].nearest_back.s - VEHICLE_LENGTH;
  double behind_collision_time
    //  = collision_time_in_future(a_b, b_b, c_b, decision.projected_kinematics.horizon);
    = collision_time_in_future(a_b, b_b, c_b, 0.0);
  behind_collision_cost = exp(-pow(behind_collision_time, 2));
  // cout << " coll. behind in " << behind_collision_time << " sec. ";

  double cost = front_collision_cost + 1.0*behind_collision_cost; // rear collision is less risky
  return cost;
}

// I'm confused with case of PLCL, and PLCR, on which lane, the collision risk is accessed?
// It should be on the current lane, not the contemplating lane.
// Need to double check.
double buffer_cost_f(Decision decision, CarDescription my_car, DATA_LANES data_lanes) {
  if (SAFE_DISTANCE <= decision.projected_kinematics.gap_front) {
    return 0;
  }
  double projected_v = decision.projected_kinematics.v;
  double time_away = decision.projected_kinematics.gap_front/projected_v;
  if (DESIRED_TIME_BUFFER < time_away) {
    return 0.0;
  } else {
    return exp(-fabs(time_away));
  }}
double inefficiency_cost_f(Decision decision, CarDescription my_car, DATA_LANES data_lanes) {
  double projected_v = decision.projected_kinematics.v;
  // expect the speed can match SPEED_LIMIT in 1 UPDATE_INTERVAL seconds
  // just relatively compare
  double cost = pow((SPEED_LIMIT - projected_v)/SPEED_LIMIT, 2);
  return cost;
}
double not_middle_cost_f(Decision decision, CarDescription my_car, DATA_LANES data_lanes) {
  // favor the middle lane, to have more options to change lane when needed
  return logistic(fabs(decision.lane_index_changed_to - 2));
}
double lane_change_extra_cost_f(Decision decision) {
  if ((decision.maneuver == LCL) || (decision.maneuver == LCR))
    return 1;
  else
    return 0;
}
double calculate_cost(Decision decision, CarDescription my_car, DATA_LANES data_lanes) {
  // cout << " lane: " << decision.lane_index_changed_to;
  double collision_cost         = COLLISION_C*collision_cost_f(decision, my_car, data_lanes);
  double inefficiency_cost      = EFFICIENCY_C*inefficiency_cost_f(decision, my_car, data_lanes);
  double buffer_cost            = DANGER_C*buffer_cost_f(decision, my_car, data_lanes);
  double not_middle_cost        = NOT_MIDDLE_C*not_middle_cost_f(decision, my_car, data_lanes);
  double lane_change_extra_cost = LANE_CHANGE_C*lane_change_extra_cost_f(decision);
  double cost
    = collision_cost + buffer_cost + inefficiency_cost + not_middle_cost + lane_change_extra_cost;
  cout << "coll. c: " << setw(3) << collision_cost << " buf. c: " << setw(3) << buffer_cost
       << " ineff. c: " << setw(3) << inefficiency_cost << ", ";
  return cost;
}

Decision evaluate_decision(MANEUVER_STATE proposed_state, CarDescription my_car, DATA_LANES data_lanes) {
  Decision decision = project_maneuver(proposed_state, my_car, data_lanes);
  decision.cost = calculate_cost(decision, my_car, data_lanes);
  return decision;
}
Decision maneuver(CarDescription my_car, DATA_LANES data_lanes) {
  vector<MANEUVER_STATE> states;
  if (!data_lanes.car_crashing_front_or_behind) {
    states.push_back(KL);
  }
  // starting from 0, from the left most to the right most
  if (0 < my_car.lane_index) {// change to left lane possible
    if (!data_lanes.car_to_left) {
      states.push_back(LCL);
    }
    //states.push_back(PLCL); // put PLCx after LCx in favor of LCx when the cost is equal
  }
  if (my_car.lane_index < NUM_LANES-1) { // change to right lane possible
    if (!data_lanes.car_to_right) {
      states.push_back(LCR);
    }
    //states.push_back(PLCR); // put PLCx after LCx in favor of LCx when the cost is equal
  }
  map<MANEUVER_STATE, Decision> decisions;
  for (auto proposed_state:states) {
    Decision a_decision = evaluate_decision(proposed_state, my_car, data_lanes);
    cout << state_str(proposed_state) << ", cost: " << setw(5) <<  a_decision.cost << " | ";
    decisions[proposed_state] = a_decision;
  }

  Decision decision = min_map_element(decisions)->second;
  // if ((decision.maneuver != KL) && decisions[KL].cost == decisions[decision.maneuver].cost) {
  //   decision = decisions[KL];
  // }
  cout << "Sel. man.: "  << state_str(decision.maneuver); // << ", cost: " << setw(7) << decision.cost << " ";
  cout << endl; // end of displaying cost evaluations
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
//       (double)sensor_fusion[i][5] + ((double)remaining_path_adopted_size*UPDATE_INTERVAL*another_car_speed);
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

struct WAYPOINTS_MAP {
  vector<double> _x;
  vector<double> _y;
  vector<double> _s;
  vector<double> _dx;
  vector<double> _dy;
};

// vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y,
//                                   double interval, int output_size) {
//   // uses the spline library to interpolate points connecting a series of x and y values
//   // output is output_size number of y values beginning at y[0] with specified fixed interval

//   if (pts_x.size() != pts_y.size()) {
//     cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
//     return { 0 };
//   }

//   tk::spline s;
//   s.set_points(pts_x,pts_y);    // currently it is required that X is already sorted
//   vector<double> output;
//   for (int i = 0; i < output_size; i++) {
//     output.push_back(s(pts_x[0] + i * interval));
//   }
//   return output;
// }

int NUM_WAYPOINTS_BEHIND = 5;
int NUM_WAYPOINTS_AHEAD  = 5;

WAYPOINTS_MAP refine_maps_f(CarDescription my_car, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                            vector<double> map_waypoints_dx, vector<double> map_waypoints_dy) {
  // ********************* CONSTRUCT INTERPOLATED WAYPOINTS OF NEARBY AREA **********************
  int num_waypoints = map_waypoints_x.size();
  int next_waypoint_index = NextWaypoint(my_car.x, my_car.y, my_car.yaw,
                                         map_waypoints_x, map_waypoints_y);
  vector<double> coarse_waypoints_s, coarse_waypoints_x, coarse_waypoints_y,
  coarse_waypoints_dx, coarse_waypoints_dy;
  for (int i = -NUM_WAYPOINTS_BEHIND; i < NUM_WAYPOINTS_AHEAD; i++) {
    // for smooting, take so many previous and so many subsequent waypoints
    int idx = (next_waypoint_index+i) % num_waypoints;
    if (idx < 0) {
      // correct for wrap
      idx += num_waypoints;
    }
    // correct for wrap in s for spline interpolation (must be continuous)
    double current_s = map_waypoints_s[idx];
    double base_s    = map_waypoints_s[next_waypoint_index];
    if ((i < 0) && (base_s < current_s)) {
      current_s -= MAX_S;
    }
    if (i > 0 && current_s < base_s) {
      current_s += MAX_S;
    }
    coarse_waypoints_s.push_back(current_s);
    coarse_waypoints_x.push_back(map_waypoints_x[idx]);
    coarse_waypoints_y.push_back(map_waypoints_y[idx]);
    coarse_waypoints_dx.push_back(map_waypoints_dx[idx]);
    coarse_waypoints_dy.push_back(map_waypoints_dy[idx]);
  }

  // extrapolate to higher resolution

  double dist_inc = 0.5; // interpolated parameters, 0.5 meters
  int num_interpolation_points = (coarse_waypoints_s[coarse_waypoints_s.size()-1] - coarse_waypoints_s[0]) / dist_inc;
  // The last s minus the first s, divided by dist_inc, so it's the number of segments of dist_inc, between the beginning and the end.

  WAYPOINTS_MAP refined_maps;
  refined_maps._s.push_back(coarse_waypoints_s[0]);
  for (int i = 1; i < num_interpolation_points; i++) {
    refined_maps._s.push_back(coarse_waypoints_s[0] + i * dist_inc);
  }

  refined_maps._x  = interpolate_points(coarse_waypoints_s, coarse_waypoints_x,  dist_inc, num_interpolation_points);
  refined_maps._y  = interpolate_points(coarse_waypoints_s, coarse_waypoints_y,  dist_inc, num_interpolation_points);
  refined_maps._dx = interpolate_points(coarse_waypoints_s, coarse_waypoints_dx, dist_inc, num_interpolation_points);
  refined_maps._dy = interpolate_points(coarse_waypoints_s, coarse_waypoints_dy, dist_inc, num_interpolation_points);

  // refined_maps._s  = map_waypoints_s;
  // refined_maps._x  = map_waypoints_x;
  // refined_maps._y  = map_waypoints_y;
  // refined_maps._dx = map_waypoints_dx;
  // refined_maps._dy = map_waypoints_dy;

  return refined_maps;
}

// Next to resolve the compilation dependency.

TRAJECTORY trajectory_f(CarDescription my_car, SENSOR_FUSION sensor_fusion, TRAJECTORY remaining_trajectory,
                        WAYPOINTS_MAP waypoints_maps) {
  TRAJECTORY trajectory; // the output

  DATA_LANES data_lanes = parse_sensor_data(my_car, sensor_fusion);

  Decision decision = maneuver(my_car, data_lanes);

  // default values of for the start of the new trajectory, applicable when there is not enough remaining_trajectory
  double start_s   = my_car.s;
  double start_x   = my_car.x;
  double start_y   = my_car.y;
  double start_yaw = my_car.yaw;
  double start_v   = my_car.v;
  double start_d   = my_car.d;
  double start_d_dot = 0;

  int remaining_path_adopted_size =
    min((int)remaining_trajectory.x_vals.size(), NUM_ADOPTED_REMAINING_TRAJECTORY_POINTS);

  // double traj_start_time = remaining_path_adopted_size * UPDATE_INTERVAL;

  int new_traj_size = PLANNED_TRAJECTORY_LENGTH - remaining_path_adopted_size;

  // modulate the start values of trajectory by the remaining trajectory:
  if (2 <= remaining_path_adopted_size) {
    // consider current position to be last point of previous path to be kept
    start_x          = remaining_trajectory.x_vals[remaining_path_adopted_size-1];
    start_y          = remaining_trajectory.y_vals[remaining_path_adopted_size-1];
    double start_x2  = remaining_trajectory.x_vals[remaining_path_adopted_size-2];
    double start_y2  = remaining_trajectory.y_vals[remaining_path_adopted_size-2];
    double start_yaw = atan2(start_y-start_y2,
                      start_x-start_x2);
    vector<double> frenet = getFrenet(start_x, start_y, start_yaw, waypoints_maps._x, waypoints_maps._y, waypoints_maps._s);
    start_s = frenet[0];
    start_d = frenet[1];

    // determine dx, dy vector from set of interpoated waypoints, with start_x, start_y as reference point;
    // since interpolated waypoints are ~1m apart and path points tend to be <0.5m apart, these
    // values can be reused for previous two points (and using the previous waypoint data may be
    // more accurate) to calculate vel_s (start_v), vel_d (start_d_dot), acc_s (s_ddot), and acc_d (d_ddot)
    int next_interp_waypoint_index = NextWaypoint(start_x, start_y, start_yaw,
                                                  waypoints_maps._x, waypoints_maps._y);
    double dx = waypoints_maps._dx[next_interp_waypoint_index - 1];
    double dy = waypoints_maps._dy[next_interp_waypoint_index - 1];
    // sx,sy vector is perpendicular to dx,dy
    double sx = -dy;
    double sy = dx;

    // calculate start_v & start_d_dot
    double vel_x1 = (start_x - start_x2) / UPDATE_INTERVAL;
    double vel_y1 = (start_y - start_y2) / UPDATE_INTERVAL;
    // want projection of xy velocity vector (V) onto S (sx,sy) and D (dx,dy) vectors, and since S
    // and D are unit vectors this is simply the dot products of V with S and V with D
    start_v = vel_x1 * sx + vel_y1 * sy;
    start_d_dot = vel_x1 * dx + vel_y1 * dy;

    // have to get another point to calculate s_ddot, d_ddot from xy acceleration
    // start_x3 = remaining_trajectory.x_vals[remaining_path_adopted_size-3];
    // start_y3 = remaining_trajectory.y_vals[remaining_path_adopted_size-3];
    // vel_x2 = (start_x2 - start_x3) / UPDATE_INTERVAL;
    // vel_y2 = (start_y2 - start_y3) / UPDATE_INTERVAL;
    // acc_x = (vel_x1 - vel_x2) / UPDATE_INTERVAL;
    // acc_y = (vel_y1 - vel_y2) / UPDATE_INTERVAL;
    // s_ddot = acc_x * sx + acc_y * sy;
    // d_ddot = acc_x * dx + acc_y * dy;
  }

  // ********************* PRODUCE NEW PATH ***********************
  // begin by pushing the last and next-to-last point from the previous path for setting the
  // spline the last point should be the first point in the returned trajectory, but because of
  // imprecision, also add that point manually

  vector<double>
    coarse_s_traj, coarse_x_traj, coarse_y_traj,
    interpolated_s_traj, interpolated_x_traj, interpolated_y_traj;

  double prev_s = start_s - start_v * UPDATE_INTERVAL;

  // first two points of coarse trajectory, to ensure spline begins smoothly
  if (2 <= remaining_path_adopted_size) {
    coarse_s_traj.push_back(prev_s);
    coarse_x_traj.push_back(remaining_trajectory.x_vals[remaining_path_adopted_size-2]);
    coarse_y_traj.push_back(remaining_trajectory.y_vals[remaining_path_adopted_size-2]);

    coarse_s_traj.push_back(start_s);
    coarse_x_traj.push_back(remaining_trajectory.x_vals[remaining_path_adopted_size-1]);
    coarse_y_traj.push_back(remaining_trajectory.y_vals[remaining_path_adopted_size-1]);
  } else {
    double prev_s = start_s - 1;
    double prev_x = start_x - cos(start_yaw);
    double prev_y = start_y - sin(start_yaw);

    coarse_s_traj.push_back(prev_s);
    coarse_x_traj.push_back(prev_x);
    coarse_y_traj.push_back(prev_y);

    coarse_s_traj.push_back(start_s);
    coarse_x_traj.push_back(start_x);
    coarse_y_traj.push_back(start_y);
  }

  // last two points of coarse trajectory, use target_d and current s + 30,60
  double target_s1 = start_s + 30;
  double target_d1 = decision.lane_index_changed_to;
  vector<double> target_xy1 = getXY(target_s1, target_d1, waypoints_maps._s, waypoints_maps._x, waypoints_maps._y);
  double target_x1 = target_xy1[0];
  double target_y1 = target_xy1[1];
  coarse_s_traj.push_back(target_s1);
  coarse_x_traj.push_back(target_x1);
  coarse_y_traj.push_back(target_y1);

  double target_s2 = target_s1 + 30;
  double target_d2 = target_d1;
  vector<double> target_xy2 = getXY(target_s2, target_d2, waypoints_maps._s, waypoints_maps._x, waypoints_maps._y);
  double target_x2 = target_xy2[0];
  double target_y2 = target_xy2[1];
  coarse_s_traj.push_back(target_s2);
  coarse_x_traj.push_back(target_x2);
  coarse_y_traj.push_back(target_y2);

  // next s values
  double target_v = decision.projected_kinematics.v; // best_target[0][1];
  double next_s = start_s;
  double next_v = start_v;
  // double next_a = s_ddot;
  const double VELOCITY_INCREMENT_LIMIT = 0.125;
  for (int i = 0; i < new_traj_size; i++) {
    double v_incr; //, a_incr;
    if (fabs(target_v - next_v) < 2 * VELOCITY_INCREMENT_LIMIT) {
      v_incr = 0;
    } else {
      // arrived at VELOCITY_INCREMENT_LIMIT value empirically
      v_incr = (target_v - next_v)/(fabs(target_v - next_v)) * VELOCITY_INCREMENT_LIMIT;
    }
    next_v += v_incr;
    next_s += next_v * UPDATE_INTERVAL;
    interpolated_s_traj.push_back(next_s);
  }

  interpolated_x_traj = interpolate_points(coarse_s_traj, coarse_x_traj, interpolated_s_traj);
  interpolated_y_traj = interpolate_points(coarse_s_traj, coarse_y_traj, interpolated_s_traj);

  // add previous path, if any, to next path
  // Start with all of the previous path points from last time
  for (int i = 0; i < remaining_path_adopted_size; i++) {
    trajectory.x_vals.push_back(remaining_trajectory.x_vals[i]);
    trajectory.y_vals.push_back(remaining_trajectory.y_vals[i]);
  }

  // add xy points from newly generated path
  // Fill up the rest of the points for the planner
  for (int i = 0; i < interpolated_s_traj.size(); i++) {
    trajectory.x_vals.push_back(interpolated_x_traj[i]);
    trajectory.y_vals.push_back(interpolated_y_traj[i]);
  }
  return trajectory;
}

/* Code junk yard
// decision.maneuver = KL; // default
// cout << "fr col: " << (!data_lanes.lanes[my_car.lane_index].nearest_front.empty &&
//                        (data_lanes.lanes[my_car.lane_index].gap_front <= 0)) << ", ";
// cout << "rr col: " << (!data_lanes.lanes[my_car.lane_index].nearest_back.empty &&
//                        (data_lanes.lanes[my_car.lane_index].gap_behind <= 0)) << ", ";

// int shift_direction = 0;
// if (decision.maneuver == LCL) {
//   shift_direction = -1;
//  } else if (decision.maneuver == LCR) {
//   shift_direction = 1;
//  } else {
//   shift_direction = 0;
//  }
// double lane_gap = abs(my_car.d - lane_center_d(decision.lane_index_changed_to));
// next_d = {my_car.d + shift_direction * lane_gap * 0.2,
//           my_car.d + shift_direction * lane_gap * 0.3,
//           my_car.d + shift_direction * lane_gap * 0.5
// };

// cout << ", next lane: " << decision.lane_index_changed_to;
// cout << " prev lane: " << my_car.lane_index; // << endl;

--------------- old trajactory generation:
// seeding points to generate smooth trajectory through spline

    vector<double> next_d = {my_car.d, my_car.d, my_car.d}; // the d coordinates for the seeding points for spline
    vector<double> ptsx;
    vector<double> ptsy;
    vector<double> ptss;

    // reference x, y, yaw
    // either we will reference the starting point as where the car is
    // or at the end of the adopted remaining paths and points
    double ref_x   = my_car.x;
    double ref_y   = my_car.y;
    double ref_s   = my_car.s;
    cout << "from my_car.s, updating ref_s: " << ref_s << ", ";

    double ref_yaw = my_car.yaw;
    double ref_v   = my_car.v;

    int remaining_path_adopted_size  =
      min((int)remaining_trajectory.x_vals.size(), NUM_ADOPTED_REMAINING_TRAJECTORY_POINTS);
    // only the first portion of the remaining_path is used in the new trajectory planning.
    // This is a trade-off of considering continuity, and the delay in path planner in providing new
    // trajectory.
    // cout << ", remaining_path_adopted_size: " << remaining_path_adopted_size;
    // cout << endl;

    // For smooth path generation, replace car_s by
    // the end of the s at the adopted remaining path

    if (remaining_path_adopted_size < 2) {
      // If the previous trajectory is almost empty,
      // use car's current position as starting reference
      // use two points to make the path tangent to the car
      double prev_car_x = my_car.x - cos(my_car.yaw);
      double prev_car_y = my_car.y - sin(my_car.yaw);
      double prev_car_s = my_car.s - 1;

      ptsx.push_back(prev_car_x);
      ptsx.push_back(my_car.x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(my_car.y);

      ptss.push_back(prev_car_s);
      ptss.push_back(my_car.s);

     } else { // use the previous path's end point as starting reference
      // Redefine reference state by previous path and point
      ref_x = remaining_trajectory.x_vals[remaining_path_adopted_size-1];
      ref_y = remaining_trajectory.y_vals[remaining_path_adopted_size-1];

      double ref_x_prev = remaining_trajectory.x_vals[remaining_path_adopted_size-2];
      double ref_y_prev = remaining_trajectory.y_vals[remaining_path_adopted_size-2];
      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev); // in radians unit
      double prev_car_s = ref_s - my_car.v * UPDATE_INTERVAL;

      // use two points to make the path tangent to the previous path's end point
      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);

      ptss.push_back(prev_car_s);
      ptss.push_back(ref_s);

      // cout << "ref_x/y_prev: (" << ref_x_prev << ", " << ref_y_prev << "), ref_x/y: (" <<
      //   ref_x << ", " << ref_y << ")" << endl;
     }
    // The computation of ref_v may need higher accuracy.

    if (0 < remaining_path_adopted_size) { // replace car_s
      vector<double> frenet_s_d
        = getFrenet(remaining_trajectory.x_vals[remaining_path_adopted_size-1],
                    remaining_trajectory.y_vals[remaining_path_adopted_size-1],
                    ref_yaw,
                    waypoints_maps._x, waypoints_maps._y, waypoints_maps._s);
      ref_s = frenet_s_d[0];
      cout << "from remaining trajectory, updating ref_s: " << ref_s << ", ";
    }
    if (1 < remaining_path_adopted_size) {
      vector<double> frenet_s_d_prev
        = getFrenet(remaining_trajectory.x_vals[remaining_path_adopted_size-2],
                    remaining_trajectory.y_vals[remaining_path_adopted_size-2],
                    ref_yaw,
                    waypoints_maps._x, waypoints_maps._y, waypoints_maps._s);

      double ref_s_prev = frenet_s_d_prev[0];
      double ref_s_delta = (ref_s - ref_s_prev);
      //ref_v = ref_s_delta/UPDATE_INTERVAL;
      // ref_v = max(ref_v, 0.02); // estimate the minimum speed, not working,
      // causing non-increasing s sequence, probably due to lack of resolution.
      // cout << "ref_s: " << ref_s << ", ref_s_prev: " << ref_s_prev << ", ref_s_delta: " << ref_s_delta << ", ref_v: " << ref_v << ", ";
     }
    // In Frenet add evenly 30m spaced points ahead of the state reference
    vector<double> next_wp0 = getXY(wrap_around(ref_s + 30),
                                    lane_center_d(decision.lane_index_changed_to), //next_d[0],
                                    waypoints_maps._s, waypoints_maps._x, waypoints_maps._y);
    vector<double> next_wp1 = getXY(wrap_around(ref_s + 60),
                                    lane_center_d(decision.lane_index_changed_to), //next_d[1],
                                    waypoints_maps._s, waypoints_maps._x, waypoints_maps._y);
    vector<double> next_wp2 = getXY(wrap_around(ref_s + 90),
                                    lane_center_d(decision.lane_index_changed_to), //next_d[2],
                                    waypoints_maps._s, waypoints_maps._x, waypoints_maps._y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    ptss.push_back(ref_s + 30);
    ptss.push_back(ref_s + 60);
    ptss.push_back(ref_s + 90);

    vector<double> velocity_spaced_s_traj;
    vector<double> volocity_spaced_x_traj;
    vector<double> volocity_spaced_y_traj;
    double updated_s = ref_s;
    int new_traj_size = PLANNED_TRAJECTORY_LENGTH - remaining_path_adopted_size;
    double updated_v = ref_v;
      //my_car.v + my_car.a * remaining_path_adopted_size;
    // double target_a = decision.projected_kinematics.a;
    double target_v = decision.projected_kinematics.v;
      // max(my_car.v + target_a * UPDATE_INTERVAL, 10.0); // make the minimum speed to be 10 meters/second; new_traj_size
    // target_v = min(SPEED_LIMIT, target_v);

    // cout << "target_v: " << target_v;
    const double VELOCITY_INCREMENT_LIMIT = 0.125;
    double v_increment = 0;
    double prev_updated_s = -MAX_S; // impossibly small

    cout <<"target_v | updated_v: " << target_v << " | " << updated_v << " v_increment | updated_s: ";
    for (int i = 0; i < new_traj_size ; i++) {
      if (fabs(target_v - updated_v) < 2*VELOCITY_INCREMENT_LIMIT) {
        v_increment = 0;
      } else {
        v_increment = copysign(VELOCITY_INCREMENT_LIMIT, target_v - updated_v);
      }
      updated_v += v_increment;
      updated_s += updated_v * UPDATE_INTERVAL;
      updated_s = wrap_around(updated_s);
      if (updated_s <= prev_updated_s) {
        break; // this updated_s would violate the assertion of spline of increasing values.
      }
      prev_updated_s = updated_s;

      velocity_spaced_s_traj.push_back(updated_s);
      cout << setw(5) << v_increment << " | " << updated_s << ", ";
     }
     cout << endl;
-------------------------- end of old trajectory generation
// DEBUG
// if (true // update_count == 0
//     ) {
//   update_count += 1;
//   ofstream log;
//   log.open("trajactory_xys.csv");
//   log << "x,y,s" << endl;
//   for (int i = 0; i < new_traj_size; i++) {
//     log << volocity_spaced_x_traj[i] << ", " << volocity_spaced_y_traj[i] <<
//       ", " << velocity_spaced_s_traj[i] << endl;
//   }
//   log.close();
//  }

// END of DEBUG

// DEBUG
// if (true // update_count == 1
//     ) {
//   update_count += 1;
//   ofstream log;
//   log.open("trajactory_xy.csv");
//   log << "x,y" << endl;
//   for (int i = remaining_path_adopted_size; i < trajectory.y_vals.size(); i++) {
//     log << trajectory.x_vals[i] << ", " << trajectory.y_vals[i] << endl;
//   }
//   log.close();
//  }
//  cout.flags(old_settings);
// END of DEBUG


 */


TRAJECTORY trajectory_f_old(CarDescription my_car, SENSOR_FUSION sensor_fusion, TRAJECTORY remaining_trajectory,
                        WAYPOINTS_MAP waypoints_maps) {
  // vector<double> waypoints_maps._x, vector<double> waypoints_maps._y, vector<double> waypoints_maps._s) {
    TRAJECTORY trajectory; // the output
    DATA_LANES data_lanes = parse_sensor_data(my_car, sensor_fusion); // , 50*UPDATE_INTERVAL look ahead by a few intervals for potential collision
    vector<double> next_d = {my_car.d, my_car.d, my_car.d}; // the d coordinates for the seeding points for spline

    Decision decision;
    // decision.maneuver = KL; // default
    // cout << "fr col: " << (!data_lanes.lanes[my_car.lane_index].nearest_front.empty &&
    //                        (data_lanes.lanes[my_car.lane_index].gap_front <= 0)) << ", ";
    // cout << "rr col: " << (!data_lanes.lanes[my_car.lane_index].nearest_back.empty &&
    //                        (data_lanes.lanes[my_car.lane_index].gap_behind <= 0)) << ", ";
    decision = maneuver(my_car, data_lanes);
    // int shift_direction = 0;
    // if (decision.maneuver == LCL) {
    //   shift_direction = -1;
    //  } else if (decision.maneuver == LCR) {
    //   shift_direction = 1;
    //  } else {
    //   shift_direction = 0;
    //  }
    // double lane_gap = abs(my_car.d - lane_center_d(decision.lane_index_changed_to));
    // next_d = {my_car.d + shift_direction * lane_gap * 0.2,
    //           my_car.d + shift_direction * lane_gap * 0.3,
    //           my_car.d + shift_direction * lane_gap * 0.5
    // };

    // cout << ", next lane: " << decision.lane_index_changed_to;
    // cout << " prev lane: " << my_car.lane_index; // << endl;

    // seeding points to generate smooth trajectory through spline
    vector<double> ptsx;
    vector<double> ptsy;
    vector<double> ptss;

    // reference x, y, yaw
    // either we will reference the starting point as where the car is
    // or at the end of the adopted remaining paths and points
    double ref_x   = my_car.x;
    double ref_y   = my_car.y;
    double ref_s   = my_car.s;
    cout << "from my_car.s, updating ref_s: " << ref_s << ", ";

    double ref_yaw = my_car.yaw;
    double ref_v   = my_car.v;

    int remaining_path_adopted_size  =
      min((int)remaining_trajectory.x_vals.size(), NUM_ADOPTED_REMAINING_TRAJECTORY_POINTS);
    // only the first portion of the remaining_path is used in the new trajectory planning.
    // This is a trade-off of considering continuity, and the delay in path planner in providing new
    // trajectory.
    // cout << ", remaining_path_adopted_size: " << remaining_path_adopted_size;
    // cout << endl;

    // For smooth path generation, replace car_s by
    // the end of the s at the adopted remaining path

    if (remaining_path_adopted_size < 2) {
      // If the previous trajectory is almost empty,
      // use car's current position as starting reference
      // use two points to make the path tangent to the car
      double prev_car_x = my_car.x - cos(my_car.yaw);
      double prev_car_y = my_car.y - sin(my_car.yaw);
      double prev_car_s = my_car.s - 1;

      ptsx.push_back(prev_car_x);
      ptsx.push_back(my_car.x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(my_car.y);

      ptss.push_back(prev_car_s);
      ptss.push_back(my_car.s);

     } else { // use the previous path's end point as starting reference
      // Redefine reference state by previous path and point
      ref_x = remaining_trajectory.x_vals[remaining_path_adopted_size-1];
      ref_y = remaining_trajectory.y_vals[remaining_path_adopted_size-1];

      double ref_x_prev = remaining_trajectory.x_vals[remaining_path_adopted_size-2];
      double ref_y_prev = remaining_trajectory.y_vals[remaining_path_adopted_size-2];
      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev); // in radians unit
      double prev_car_s = ref_s - my_car.v * UPDATE_INTERVAL;

      // use two points to make the path tangent to the previous path's end point
      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);

      ptss.push_back(prev_car_s);
      ptss.push_back(ref_s);

      // cout << "ref_x/y_prev: (" << ref_x_prev << ", " << ref_y_prev << "), ref_x/y: (" <<
      //   ref_x << ", " << ref_y << ")" << endl;
     }
    // The computation of ref_v may need higher accuracy.

    if (0 < remaining_path_adopted_size) { // replace car_s
      vector<double> frenet_s_d
        = getFrenet(remaining_trajectory.x_vals[remaining_path_adopted_size-1],
                    remaining_trajectory.y_vals[remaining_path_adopted_size-1],
                    ref_yaw,
                    waypoints_maps._x, waypoints_maps._y, waypoints_maps._s);
      ref_s = frenet_s_d[0];
      cout << "from remaining trajectory, updating ref_s: " << ref_s << ", ";
    }
    if (1 < remaining_path_adopted_size) {
      vector<double> frenet_s_d_prev
        = getFrenet(remaining_trajectory.x_vals[remaining_path_adopted_size-2],
                    remaining_trajectory.y_vals[remaining_path_adopted_size-2],
                    ref_yaw,
                    waypoints_maps._x, waypoints_maps._y, waypoints_maps._s);

      double ref_s_prev = frenet_s_d_prev[0];
      double ref_s_delta = (ref_s - ref_s_prev);
      //ref_v = ref_s_delta/UPDATE_INTERVAL;
      // ref_v = max(ref_v, 0.02); // estimate the minimum speed, not working,
      // causing non-increasing s sequence, probably due to lack of resolution.
      // cout << "ref_s: " << ref_s << ", ref_s_prev: " << ref_s_prev << ", ref_s_delta: " << ref_s_delta << ", ref_v: " << ref_v << ", ";
     }
    // In Frenet add evenly 30m spaced points ahead of the state reference
    vector<double> next_wp0 = getXY(wrap_around(ref_s + 30),
                                    lane_center_d(decision.lane_index_changed_to), //next_d[0],
                                    waypoints_maps._s, waypoints_maps._x, waypoints_maps._y);
    vector<double> next_wp1 = getXY(wrap_around(ref_s + 60),
                                    lane_center_d(decision.lane_index_changed_to), //next_d[1],
                                    waypoints_maps._s, waypoints_maps._x, waypoints_maps._y);
    vector<double> next_wp2 = getXY(wrap_around(ref_s + 90),
                                    lane_center_d(decision.lane_index_changed_to), //next_d[2],
                                    waypoints_maps._s, waypoints_maps._x, waypoints_maps._y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    ptss.push_back(ref_s + 30);
    ptss.push_back(ref_s + 60);
    ptss.push_back(ref_s + 90);

    vector<double> velocity_spaced_s_traj;
    vector<double> volocity_spaced_x_traj;
    vector<double> volocity_spaced_y_traj;
    double updated_s = ref_s;
    int new_traj_size = PLANNED_TRAJECTORY_LENGTH - remaining_path_adopted_size;
    double updated_v = ref_v;
      //my_car.v + my_car.a * remaining_path_adopted_size;
    // double target_a = decision.projected_kinematics.a;
    double target_v = decision.projected_kinematics.v;
      // max(my_car.v + target_a * UPDATE_INTERVAL, 10.0); // make the minimum speed to be 10 meters/second; new_traj_size
    // target_v = min(SPEED_LIMIT, target_v);

    // cout << "target_v: " << target_v;
    const double VELOCITY_INCREMENT_LIMIT = 0.125;
    double v_increment = 0;
    double prev_updated_s = -MAX_S; // impossibly small

    cout <<"target_v | updated_v: " << target_v << " | " << updated_v << " v_increment | updated_s: ";
    for (int i = 0; i < new_traj_size ; i++) {
      if (fabs(target_v - updated_v) < 2*VELOCITY_INCREMENT_LIMIT) {
        v_increment = 0;
      } else {
        v_increment = copysign(VELOCITY_INCREMENT_LIMIT, target_v - updated_v);
      }
      updated_v += v_increment;
      updated_s += updated_v * UPDATE_INTERVAL;
      updated_s = wrap_around(updated_s);
      if (updated_s <= prev_updated_s) {
        break; // this updated_s would violate the assertion of spline of increasing values.
      }
      prev_updated_s = updated_s;

      velocity_spaced_s_traj.push_back(updated_s);
      cout << setw(5) << v_increment << " | " << updated_s << ", ";
     }
     cout << endl;

    volocity_spaced_x_traj = interpolate_points(ptss, ptsx, velocity_spaced_s_traj);
    volocity_spaced_y_traj = interpolate_points(ptss, ptsy, velocity_spaced_s_traj);
    // Start with all of the previous path points from last time
    for (int i = 0; i < remaining_path_adopted_size; i++) {
      trajectory.x_vals.push_back(remaining_trajectory.x_vals[i]);
      trajectory.y_vals.push_back(remaining_trajectory.y_vals[i]);
    }

    // Fill up the rest of the points for the planner
    for (int i = 0; i < velocity_spaced_s_traj.size(); i++) {
      trajectory.x_vals.push_back(volocity_spaced_x_traj[i]);
      trajectory.y_vals.push_back(volocity_spaced_y_traj[i]);
    }
      // DEBUG
    // if (true // update_count == 0
    //     ) {
    //   update_count += 1;
    //   ofstream log;
    //   log.open("trajactory_xys.csv");
    //   log << "x,y,s" << endl;
    //   for (int i = 0; i < new_traj_size; i++) {
    //     log << volocity_spaced_x_traj[i] << ", " << volocity_spaced_y_traj[i] <<
    //       ", " << velocity_spaced_s_traj[i] << endl;
    //   }
    //   log.close();
    //  }

    // END of DEBUG

    // DEBUG
    // if (true // update_count == 1
    //     ) {
    //   update_count += 1;
    //   ofstream log;
    //   log.open("trajactory_xy.csv");
    //   log << "x,y" << endl;
    //   for (int i = remaining_path_adopted_size; i < trajectory.y_vals.size(); i++) {
    //     log << trajectory.x_vals[i] << ", " << trajectory.y_vals[i] << endl;
    //   }
    //   log.close();
    //  }
    //  cout.flags(old_settings);
    // END of DEBUG

    return trajectory;
  }

int main() {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
  
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
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
  // double ref_val = MAX_VELOCITY_DELTA_PRE_PLANNING_INTERVAL; // initial
  CarDescription my_car;
  my_car.a = 0;
  my_car.jerk = 0;
  

  int update_count = 0; // used to debug to capture the first trace
  uWS::Hub h;
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
               &my_car,
               &update_count]
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
            // actually they are the remaining points of trajectory not yet visited by the car
            // they are issued by the path planner to the car in the previous control time
            auto remaining_path_x = j[1]["previous_path_x"];
            auto remaining_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            double remaining_path_end_s = j[1]["end_path_s"]; // not yet used, keep for documentation purpose
            double remaining_path_end_d = j[1]["end_path_d"]; // not yet used, keep might be needed

            cout << "car_s: " << car_s << ", car_{x, y}: " << car_x << ", " << car_y << " remaining_path_end_{s, d}: "
                 << remaining_path_end_s << ", " << remaining_path_end_d << " car_speed (meters/s) " << mph_2_meterps(car_speed)
                 << endl;

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            // Assemble information to call trajectory_f:
            my_car.id = -1; // hopefully impossible id of the other cars
            my_car.x  = car_x;
            my_car.y  = car_y;
            my_car.yaw = deg2rad(car_yaw);
            
            double old_v = my_car.v;
            my_car.v  = mph_2_meterps(car_speed);
            my_car.s  = wrap_around(car_s);
            my_car.d  = car_d;
            my_car.lane_index = d_to_lane_index(car_d);
            
            double old_a = my_car.a;
            my_car.a = (my_car.v - old_v)/UPDATE_INTERVAL;
            
            my_car.jerk = (my_car.a - old_a)/UPDATE_INTERVAL;
            
            my_car.remaining_path_end_s = wrap_around(remaining_path_end_s);
            my_car.remaining_path_end_d = remaining_path_end_d;
            
            ios::fmtflags old_settings = cout.flags();
            cout.precision(5);
            
            TRAJECTORY remaining_trajectory;
            cout << "rem. p_{x, y}_len: " << remaining_path_x.size() << ", " << remaining_path_y.size() << ", ";
            // transfer to the remaining trajectory from auto type to pair of double<vector>, otherwise, the compiler reject
            // the vector assginment.
            cout << endl;
            cout << "remaining x: ";
            for (auto x:remaining_path_x) {
              remaining_trajectory.x_vals.push_back(x);
              cout << setw(6) << x << ", ";
             }
            
            cout << endl;
            cout << "remaining y: ";
            for (auto y:remaining_path_y) {
              remaining_trajectory.y_vals.push_back(y);
              cout << setw(6) << y << ", ";
             }
            
            cout << endl;
            
            // remaining_trajectory.x_vals = remaining_path_x;
            // remaining_trajectory.y_vals = remaining_path_y;
            
            // Fix and refine the waypoint maps to improve the resolution of computing
            // ref_v based on the remaining trajectory, for smoother trajectory planning.
            
            WAYPOINTS_MAP refined_maps = refine_maps_f(my_car,
                                                     map_waypoints_x, map_waypoints_y, map_waypoints_s,
                                                     map_waypoints_dx, map_waypoints_dy);
            TRAJECTORY trajectory
            = trajectory_f(my_car, sensor_fusion, remaining_trajectory, refined_maps);
            
            // // Define the actual (x, y) points in the planner
            // vector<double> next_x_vals;
            // vector<double> next_y_vals;

            cout << endl;
            cout << "planned traj. sent to simulator" << endl;
            cout << "x: ";
            for (auto x:trajectory.x_vals) {
              cout << setw(6) << x << ", ";
            }
            cout << endl;
            cout << "y: ";
            for (auto y:trajectory.y_vals) {
              cout << setw(6) << y << ", ";
            }
            cout << endl;

            json msgJson;
            msgJson["next_x"] = trajectory.x_vals;
            msgJson["next_y"] = trajectory.y_vals;

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
