enum DIRECTION {LEFT = 1, RIGHT = 2};

enum MANEUVER {KL=1, LCL=2, LCR=3, PLCL=4, PLCR=5};

// Parse the sensor_fusion data
string state_str(MANEUVER state) {
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
  MANEUVER maneuver;
  // double velocity_delta;
  double cost;
  KINEMATIC_DATA projected_kinematics; // for key: "velocity", and "acceleration"
};

struct Car {
  double id;
  double x;
  double y;
  double yaw;
  double v_x;
  double v_y;
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
  Car nearest_front;
  Car nearest_back;
  // double         car_density_front;
  double gap_front; // the projected smallest distance with the car in front, depreciated
  double gap_behind; // the projected smallest distance with the car behind, depreciated
  double congestion_front;      // the congestion with the car in front
  double congestion_behind;     // the congestion with the car behind
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
