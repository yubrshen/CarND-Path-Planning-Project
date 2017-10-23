struct WAYPOINTS_MAP {
  vector<double> _x;
  vector<double> _y;
  vector<double> _s;
  vector<double> _dx;
  vector<double> _dy;
};

int NUM_WAYPOINTS_BEHIND = 5;
int NUM_WAYPOINTS_AHEAD  = 5;

WAYPOINTS_MAP refine_maps_f(Car my_car, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
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

  return refined_maps;
}

TRAJECTORY trajectory_f(Car my_car, SENSOR_FUSION sensor_fusion,
                        TRAJECTORY remaining_trajectory,
                        WAYPOINTS_MAP waypoints_maps)
{
  TRAJECTORY trajectory; // the return value

  int remaining_path_adopted_size = min((int)remaining_trajectory.x_vals.size(),
                                        NUM_ADOPTED_REMAINING_TRAJECTORY_POINTS);

  int new_traj_size = PLANNED_TRAJECTORY_LENGTH - remaining_path_adopted_size;
  // cout << " new_traj_size: " << new_traj_size << "; ";

  double start_time = remaining_path_adopted_size * UPDATE_INTERVAL;
  double end_time   = start_time + new_traj_size  * UPDATE_INTERVAL;

  DATA_LANES data_lanes = parse_sensor_data(my_car, sensor_fusion,
                                            start_time, end_time);

  Decision decision = maneuver_f(my_car, data_lanes);

  // default values for the start of the new trajectory,
  // applicable when there is not enough remaining_trajectory
  double start_s   = my_car.s;
  double start_x   = my_car.x;
  double start_y   = my_car.y;
  double start_yaw = my_car.yaw;
  double start_v   = my_car.v;
  double start_d   = my_car.d;

  // modulate the start values of trajectory by the remaining trajectory:
  if (2 <= remaining_path_adopted_size) {
    // consider current position to be last point of previous path to be kept
    start_x          = remaining_trajectory.x_vals[remaining_path_adopted_size-1];
    start_y          = remaining_trajectory.y_vals[remaining_path_adopted_size-1];
    double start_x2  = remaining_trajectory.x_vals[remaining_path_adopted_size-2];
    double start_y2  = remaining_trajectory.y_vals[remaining_path_adopted_size-2];
    double start_yaw = atan2(start_y-start_y2,
                             start_x-start_x2);
    vector<double> frenet = getFrenet(start_x, start_y, start_yaw,
                                      waypoints_maps._x, waypoints_maps._y,
                                      waypoints_maps._s);
    start_s = frenet[0];
    start_s = wrap_around(start_s); // maybe needed
    start_d = frenet[1];

    // determine dx, dy vector from set of interpoated waypoints,
    // with start_x, start_y as reference point;
    // since interpolated waypoints are ~1m apart and
    // path points tend to be <0.5m apart,
    // these values can be reused for previous two points
    // (and using the previous waypoint data may be more accurate)
    // to calculate vel_s (start_v), vel_d (start_d_dot),
    // acc_s (s_ddot), and acc_d (d_ddot)
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
    // want projection of xy velocity vector (V) onto S (sx,sy) and D (dx,dy) vectors,
    // and since S and D are unit vectors this is simply the dot products
    // of V with S and V with D
    start_v = vel_x1 * sx + vel_y1 * sy;
  }

  // ********************* PRODUCE NEW PATH ***********************
  // begin by pushing the last and next-to-last point
  // from the previous path for setting the
  // spline the last point should be the first point in the returned trajectory,
  // but because of imprecision, also add that point manually

  double prev_s = wrap_around(start_s - start_v * UPDATE_INTERVAL);
  int smallest_start_index = 0; // default 0
  if (start_s < prev_s)
    {
      smallest_start_index = 1;
      cout << "start_s <= prev_s start_s | prev_s: "
           << start_s << "|" << prev_s << "; ";
    }
  double prev_x, prev_y;

  // first two points of coarse trajectory, to ensure spline begins smoothly
  if (2 <= remaining_path_adopted_size) {
    prev_x = (remaining_trajectory.x_vals[remaining_path_adopted_size-2]);
    prev_y = (remaining_trajectory.y_vals[remaining_path_adopted_size-2]);
  } else {
    prev_s = wrap_around(start_s - 1);
    prev_x = start_x - cos(start_yaw);
    prev_y = start_y - sin(start_yaw);
  }

  // last two points of coarse trajectory, use target_d and current s + 30,60
  double target_1_s = (start_s + 30);
  if (MAX_S <= target_1_s)
    {
      smallest_start_index = 2;
      target_1_s -= MAX_S;
    }
  double target_d1 = lane_center_d(decision.lane_index_changed_to);
  vector<double> target_xy1 = getXY(target_1_s, target_d1,
                                    waypoints_maps._s,
                                    waypoints_maps._x,
                                    waypoints_maps._y);
  double target_1_x = target_xy1[0];
  double target_1_y = target_xy1[1];
  double target_2_s = (target_1_s + 30);
  if (MAX_S <= target_2_s)
    {
      smallest_start_index = 3;
      target_2_s -= MAX_S;
    }
  double target_d2 = target_d1;
  vector<double> target_xy2 = getXY(target_2_s, target_d2,
                                    waypoints_maps._s,
                                    waypoints_maps._x,
                                    waypoints_maps._y);
  double target_2_x = target_xy2[0];
  double target_2_y = target_xy2[1];
  vector<double> coarse_s_traj, coarse_x_traj, coarse_y_traj;

  // arrange the seeding trajectory points to ensure coarse_s_traj has
  // increasing order
  map<string, map<string, double> > seeds =
    {
      {"prev",     {{"s", prev_s},     {"x", prev_x},     {"y", prev_y}}},
      {"start",    {{"s", start_s},    {"x", start_x},    {"y", start_y}}},
      {"target_1", {{"s", target_1_s}, {"x", target_1_x}, {"y", target_1_y}}},
      {"target_2", {{"s", target_2_s}, {"x", target_2_x}, {"y", target_2_y}}}
    };
  map<string, vector<double>* > trajs =
    {
      {"s", &coarse_s_traj},
      {"x", &coarse_x_traj},
      {"y", &coarse_y_traj}
    };

  for (string sxy: {"s", "x", "y"})
    {
      // cout << "case : " << smallest_start_index << "; ";

      // cout << "re-arranged: ";
      switch (smallest_start_index)
        {
        case 0:
          for (string p: {"prev", "start", "target_1", "target_2"})
            {
              trajs[sxy]->push_back(seeds[p][sxy]);
            }
          break;
        case 1:
          for (string p: {"start", "target_1", "target_2", "prev"})
            {
              // cout << seeds[p][sxy] << ", ";
              trajs[sxy]->push_back(seeds[p][sxy]);
            }
          break;
        case 2:
          for (string p: {"target_1", "target_2", "prev", "start"})
            {
              trajs[sxy]->push_back(seeds[p][sxy]);
            }
          break;
        case 3:
          for (string p: {"target_2", "prev", "start", "target_1"})
            {
              trajs[sxy]->push_back(seeds[p][sxy]);
            }
          break;
        default:
          cout << "Illegal index of the smallest s value. ";
        }
    }
  // cout << " coarse_s_traj.size(): " << coarse_s_traj.size() << "; " << endl;

  // next s values
  vector<double> interpolated_s_traj, interpolated_x_traj, interpolated_y_traj;
  double target_v = decision.projected_kinematics.v; // best_target[0][1];
  double next_s = start_s;
  // double prev_updated_s = -MAX_S; // impossibly small

  double next_v = start_v;
  // cout << " next_v: ";
  for (int i = 0; i < new_traj_size; i++) {
    double v_incr = 0;
    next_s += next_v * UPDATE_INTERVAL;
    // prevent non-increasing s values:
    next_s = wrap_around(next_s);
    // if (next_s <= prev_updated_s)
    //   break;
    // prev_updated_s = next_s;
    // cout << setw(5) << next_v << ", ";
    interpolated_s_traj.push_back(next_s);
    if (fabs(target_v - next_v) < 2 * VELOCITY_INCREMENT_LIMIT) {
      v_incr = 0;
    } else {
      // arrived at VELOCITY_INCREMENT_LIMIT value empirically
      v_incr =
        (target_v - next_v)/(fabs(target_v - next_v)) * VELOCITY_INCREMENT_LIMIT;
    }
    next_v += v_incr;
  }
  // cout << " coarse_s_traj: ";
  // for (auto s: coarse_s_traj)
  //   {
  //     cout << s << ", ";
  //   }
  // cout << endl;

  interpolated_x_traj =
    interpolate_points(coarse_s_traj, coarse_x_traj, interpolated_s_traj);
  interpolated_y_traj =
    interpolate_points(coarse_s_traj, coarse_y_traj, interpolated_s_traj);

  // add previous path, if any, to next path
  // Start with the adopted portion of the previous path points from last time
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
