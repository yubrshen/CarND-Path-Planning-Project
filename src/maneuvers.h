Decision project_maneuver(MANEUVER proposed_maneuver, Car my_car,
                          DATA_LANES data_lanes) {
  Decision decision;
  int changed_lane = my_car.lane_index;

  switch(int(proposed_maneuver)) {
  case int(KL):
    decision.projected_kinematics =
      kinematic_required_in_front(my_car, data_lanes, my_car.lane_index);
    decision.lane_index_changed_to = my_car.lane_index;
    break;
  case int(LCL):
    changed_lane = my_car.lane_index-1;
    decision.projected_kinematics =
      kinematic_required_in_front(my_car, data_lanes, changed_lane);
    decision.lane_index_changed_to = changed_lane;
    break;
  case int(LCR):
    changed_lane = my_car.lane_index+1;
    decision.projected_kinematics =
      kinematic_required_in_front(my_car, data_lanes, changed_lane);
    decision.lane_index_changed_to = changed_lane;
    break;
  case int(PLCL):
    decision.lane_index_changed_to = my_car.lane_index;
    // no lane change yet, but evaluate with the proposed change
    decision.projected_kinematics =
      kinematic_required_behind(my_car, data_lanes, my_car.lane_index -1);
    break;
  case int(PLCR):
    decision.lane_index_changed_to = my_car.lane_index;
    // no lane change yet, but evaluate with the proposed change
    decision.projected_kinematics =
      kinematic_required_behind(my_car, data_lanes, my_car.lane_index +1);
    break;
  default:
    cout << "Not supported proposed state: " << proposed_maneuver << endl;
    break;
  };
  decision.maneuver = proposed_maneuver;
  cout // <<  "prop. man.: "
       << setw(5) << state_str(decision.maneuver) << ", " << " to: "
       << decision.lane_index_changed_to << ", ";
  return decision;
}
double collision_cost_f(Decision decision, Car my_car, DATA_LANES data_lanes)
{
  if (data_lanes.car_crashing_front_or_behind)
    {
      return 1.0;
    } else
    {
      return 0.0;
    }
}
double buffer_cost_f(Decision decision, Car my_car, DATA_LANES data_lanes)
{ // express the requirements that both the gap_front and gap_behind should be
  // larger or equal to SAFE_DISTANCE.

  double cost_front  = data_lanes.lanes[decision.lane_index_changed_to].congestion_front;
  double cost_behind = data_lanes.lanes[decision.lane_index_changed_to].congestion_behind;
  return cost_front + 1.0 * cost_behind;
}
double inefficiency_cost_f(Decision decision, Car my_car, DATA_LANES data_lanes) {
  double projected_v = decision.projected_kinematics.v;
  // expect the speed can match SPEED_LIMIT in 1 UPDATE_INTERVAL seconds
  // just relatively compare
  double cost = pow((SPEED_LIMIT - projected_v)/SPEED_LIMIT, 2);
  return cost;
}
double not_middle_cost_f(Decision decision, Car my_car, DATA_LANES data_lanes) {
  // favor the middle lane, to have more options to change lane when needed
  return logistic(fabs(decision.lane_index_changed_to - 2));
}
double lane_change_extra_cost_f(Car my_car, Decision decision) {
  if ((decision.maneuver == LCL) || (decision.maneuver == LCR))
    return exp(-fabs(my_car.v));
  else
    return 0;
}
double calculate_cost(Decision decision, Car my_car, DATA_LANES data_lanes) {
  // cout << " lane: " << decision.lane_index_changed_to;
  double collision_cost
    = COLLISION_C *   collision_cost_f(decision, my_car, data_lanes);
  double inefficiency_cost
    = EFFICIENCY_C *  inefficiency_cost_f(decision, my_car, data_lanes);
  double buffer_cost
    = DANGER_C *      buffer_cost_f(decision, my_car, data_lanes);
  double not_middle_cost
    = NOT_MIDDLE_C *  not_middle_cost_f(decision, my_car, data_lanes);
  double lane_change_extra_cost
    = LANE_CHANGE_C * lane_change_extra_cost_f(my_car, decision);
  double cost = collision_cost + buffer_cost + inefficiency_cost
    + not_middle_cost + lane_change_extra_cost;
  cout << "coll. c: " << setw(3) << collision_cost << " buf. c: " << setw(3) << buffer_cost
       << " ineff. c: " << setw(3) << inefficiency_cost << ", ";
  return cost;
}
Decision maneuver_f(Car my_car, DATA_LANES data_lanes) {
  vector<MANEUVER> states;
  if (!data_lanes.car_crashing_front_or_behind) {
    states.push_back(KL);
  }
  // starting from 0, from the left most to the right most
  if (0 < my_car.lane_index) {// change to left lane possible
    if (!data_lanes.car_to_left) {
      states.push_back(LCL);
    }
  }
  if (my_car.lane_index < NUM_LANES-1) { // change to right lane possible
    if (!data_lanes.car_to_right) {
      states.push_back(LCR);
    }
  }
  map<MANEUVER, Decision> decisions;
  for (auto proposed_maneuver:states) {
    // Decision a_decision = evaluate_decision(proposed_maneuver, my_car, data_lanes);
    Decision decision = project_maneuver(proposed_maneuver, my_car, data_lanes);
    decision.cost = calculate_cost(decision, my_car, data_lanes);

    cout << setw(5) << state_str(proposed_maneuver) << ", cost: "
         << setw(5) <<  decision.cost << " | ";
    decisions[proposed_maneuver] = decision;
  }

  Decision decision = min_map_element(decisions)->second;
  cout << "Sel. man.: "  << setw(5) << state_str(decision.maneuver);
  // << ", cost: " << setw(7) << decision.cost << " ";
  cout << endl; // end of displaying cost evaluations
  return decision;
}
