double projected_gap_front(double front_s, double front_v,
                          double behind_s, double behind_v, double behind_a,
                          double delta_t)
{
  double gap = front_s - behind_s + (front_v - behind_v)*delta_t +
    - 0.5*behind_a*(delta_t * delta_t) - VEHICLE_LENGTH;
  return gap;
}

double projected_gap_behind(double behind_s, double behind_v,
                            double front_s, double front_v, double front_a,
                            double delta_t)
{
  double gap = front_s - behind_s + (front_v - behind_v)*delta_t +
    + 0.5*front_a*(delta_t * delta_t) - VEHICLE_LENGTH;
  return gap;
}
void update_gaps_in_kinematic(Car front, Car my_car, Car behind,
                              double horizon, KINEMATIC_DATA *kinematic)
{
  kinematic->horizon = horizon;
  if (behind.empty) {
    kinematic->gap_behind = SAFE_DISTANCE; // extremely large
  } else {
    kinematic->gap_behind = projected_gap_behind
      (behind.s, behind.v, my_car.s,
       kinematic->v, kinematic->a, kinematic->horizon);
  }
  if (front.empty) {
    kinematic->gap_front = SAFE_DISTANCE; // extremely large
  } else {
    kinematic->gap_front = projected_gap_front
      (front.s, front.v, my_car.s,
       kinematic->v, kinematic->a, kinematic->horizon);
  }
}

double start_distance_congestion(double dist_start)
{
  return exp(-max(dist_start/SAFE_DISTANCE, 0.0) );
}

const double SAFE_DISTANCE_CONGESTION = start_distance_congestion(SAFE_DISTANCE);
double threshold_congestion(double time_threshold, double start_time)
{
  double damper = SAFE_DISTANCE_CONGESTION/exp(-start_time);
  // adjust the congestion for this case,
  // to be comparable with that computed by start_distance_congestion
  // if time_threshold == start_time,
  // then the congestion would be equal to start_distance_congestion(SAFE_DISTANCE)
  double c = damper * exp(-time_threshold);
  return c;
}

double congestion_f(Car front, Car behind, double start_time, double end_time)
{ // returns the congestion coefficient between the two cars.
  // To simplify, assume they have zero acceleration
  double c = 0.0;
  double dist_start = (front.s - behind.s) + (front.v - behind.v)*start_time;
  if (behind.v <= front.v)
    {
      c = start_distance_congestion(dist_start);
      cout << " start_time: " << setw(5) << start_time
           << ", front faster, dist_start: "
           << setw(7) << dist_start << " c: " << setw(7) << c << "; ";
    } else
    { // behind.v > front.v
      if (dist_start <= SAFE_DISTANCE)
        {
          double punish_weight = 1.01; // punish further this case

          c = punish_weight * start_distance_congestion(dist_start);
          cout <<  " start_time: " << setw(5) << start_time
               << ", front slower and start with less safe distance, dist_start: "
               << setw(7) << dist_start <<  " c: " << setw(7) << c <<"; ";
        } else
        { // dist_start > SAFE_DISTANCE
          // with equation:
          // dist = (front.s - behind.s) + (front.v - behind.v)* t = SAFE_DISTANCE
          // time_threshold should be when the projected distance
          // between the front and the behind would equal to SAFE_DISTANCE
          double time_threshold =
            (SAFE_DISTANCE - (front.s - behind.s)) / (front.v - behind.v);
          cout <<
            "front slower, and start wtih more than safe distance, time_threshold: "
               << setw(7) << time_threshold << " c: " << setw(7) << c <<"; ";
          assert(start_time <= time_threshold); // by the model's reasoning
          c = threshold_congestion(time_threshold, start_time);
        }
    }
  return c;
}
void update_surronding(Car my_car, double congestion, int lane,
                       DATA_LANES *data_lanes)
{
  /*
    Based on the distance between the car in front, and that behind,
    congestion to determine the car's status,
    represented in the fields of DATA_LANES:
    car_crashing_front_or_behind, car_to_left, car_to_right.
   */
  data_lanes-> car_crashing_front_or_behind = false;
  data_lanes-> car_to_left                  = false;
  data_lanes-> car_to_right                 = false;
  if (0.899 < congestion)
    {
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
    }} else
    {
      // cout <<"car_{right, left, ahead}: " << data_lanes->car_to_right << ", "
      // << data_lanes->car_to_left << ", "
      // << data_lanes->car_crashing_front_or_behind;
    }
}

DATA_LANES parse_sensor_data(Car my_car, SENSOR_FUSION sensor_fusion,
                             double start_time, double end_time)
{ /* find the nearest car in front, and behind, and
     find the congestion conditions in front of my_car, and behind
     for the time period of start_time and end_time.
  */

  DATA_LANES data_lanes;
  for (int i = 0; i < NUM_LANES; i++)
    { // initialize the data structure with default values
    LaneData lane_data;
    data_lanes.lanes[i] = lane_data; // assume copy semantics
    data_lanes.lanes[i].nearest_back.empty = true;
    data_lanes.lanes[i].nearest_front.empty = true;
    data_lanes.lanes[i].gap_front  = SAFE_DISTANCE;
    data_lanes.lanes[i].gap_behind = SAFE_DISTANCE;
    data_lanes.lanes[i].congestion_front  = 0.0;
    data_lanes.lanes[i].congestion_behind = 0.0;
    }

  Car a_car;
  for (auto data:sensor_fusion)
    { // find the nearest in front and behind
    a_car.d     = data[6];
    if ((a_car.d < 0) || (lane_width*NUM_LANES < a_car.d))
      {
      continue;                 // ignore invalid record
      }
    a_car.id    = data[0];
    a_car.x     = data[1];
    a_car.y     = data[2];
    a_car.v_x   = data[3];
    a_car.v_y   = data[4];
    a_car.s     = data[5];

    a_car.lane_index = d_to_lane_index(a_car.d);
    a_car.v     = sqrt(pow(a_car.v_x, 2) +
                       pow(a_car.v_y, 2));
    a_car.empty = false;

    // cout << "a car at lane: " << a_car.lane_index;
    // Find the nearest cars in front of my_car, and behind:
    if (a_car.s <= my_car.s) {// there is a car behind
      if (data_lanes.lanes[a_car.lane_index].nearest_back.empty) {
        // cout << ", first registration for nearest_back ";
        data_lanes.lanes[a_car.lane_index].nearest_back        = a_car;
      } else {
        if (data_lanes.lanes[a_car.lane_index].nearest_back.s < a_car.s) {
          data_lanes.lanes[a_car.lane_index].nearest_back      = a_car;
          // cout << ", update for nearest_back ";
        }}}
    if (my_car.s <= a_car.s) { // there is a car in front
      if (data_lanes.lanes[a_car.lane_index].nearest_front.empty) {
        // cout << ", first registration for nearest_front ";
        data_lanes.lanes[a_car.lane_index].nearest_front       = a_car;
      } else {
        if (a_car.s < data_lanes.lanes[a_car.lane_index].nearest_front.s) {
          // cout << ", update for nearest_back ";
          data_lanes.lanes[a_car.lane_index].nearest_front     = a_car;
        }}}}

  // For only the legal lanes adjacent to my_car.lane_index,
  int left_lane  = my_car.lane_index -1;
  int right_lane = my_car.lane_index +1;
  // cout << "candidates_{left | right}_lane: " << left_lane << " | "
  // << right_lane << "; ";
  vector<int> lanes_interested = {my_car.lane_index};
  if (0 <= left_lane)         lanes_interested.push_back(left_lane);
  if (right_lane < NUM_LANES) lanes_interested.push_back(right_lane);
  for (auto lane:lanes_interested) {
    cout << "interested lane: " << lane << "; ";
    if (!data_lanes.lanes[lane].nearest_back.empty)
      {
        cout << " back congestion: ";
        double congestion = congestion_f(my_car, data_lanes.lanes[lane].nearest_back,
                                         start_time, end_time);
        data_lanes.lanes[lane].congestion_behind = congestion;
        update_surronding(my_car, congestion, lane, &data_lanes);
      }
    if (!data_lanes.lanes[lane].nearest_front.empty)
      {
        cout << " front congestion: ";
        double congestion = congestion_f(data_lanes.lanes[lane].nearest_front, my_car,
                                         start_time, end_time);
        data_lanes.lanes[lane].congestion_front = congestion;
        update_surronding(my_car, congestion, lane, &data_lanes);
      }
  }
  return data_lanes;
}


KINEMATIC_DATA kinematic_required_in_front
(Car my_car, DATA_LANES data_lanes, int lane_changed_to) {
  KINEMATIC_DATA kinematic;
  kinematic.v = SPEED_LIMIT; // assuming there is no car in front.
  kinematic.horizon = 200*UPDATE_INTERVAL; // 4 seconds
  double projected_my_car_s    = my_car.s + kinematic.horizon*(my_car.v + kinematic.v)/2;
  // assuming an average speed, an approximation in order to estimate my_car_s position
  // at the end of the horizon
  double projected_front_car_s
    = data_lanes.lanes[lane_changed_to].nearest_front.s
    + kinematic.horizon*data_lanes.lanes[lane_changed_to].nearest_front.v;
  double gap_front = projected_front_car_s - projected_my_car_s;
  if (!data_lanes.lanes[lane_changed_to].nearest_front.empty && (gap_front < SAFE_DISTANCE))
  //if (0.3 < data_lanes.lanes[lane_changed_to].congestion_front)
    {
      kinematic.v = data_lanes.lanes[lane_changed_to].nearest_front.v;
    }
  kinematic.a = (kinematic.v - my_car.v)/kinematic.horizon;
  return kinematic;
}
//map<string, double>
KINEMATIC_DATA kinematic_required_behind
(Car my_car, DATA_LANES data_lanes, int lane_index) {
  KINEMATIC_DATA kinematic;
  if (data_lanes.lanes[lane_index].nearest_back.empty) {
    kinematic.a = my_car.a;
    kinematic.v = my_car.v;
  } else {
    double gap_behind =
      my_car.s - data_lanes.lanes[lane_index].nearest_back.s;
    if (gap_behind <= 0)
      { // invalid with assumption that the other car is behind
      kinematic.a = my_car.a;
      kinematic.v = my_car.v;
    } else {
      double delta_v =
        my_car.v - data_lanes.lanes[lane_index].nearest_back.v;
      double min_acceleration_pushed_by_nearest_back =
        (delta_v*delta_v)/(2*gap_behind);
      kinematic.a =
        min(min_acceleration_pushed_by_nearest_back,
            my_car.a +
            MAX_ACCELERATION_DELTA_METERS_PER_UPDATE_INTERVAL);
      kinematic.v = min(data_lanes.lanes[lane_index].nearest_front.v,
                        my_car.v + kinematic.a * UPDATE_INTERVAL);
      // kinematic.v is used per UPDATE_INTERVAL
    }}
  update_gaps_in_kinematic(data_lanes.lanes[lane_index].nearest_front,
                           my_car,
                           data_lanes.lanes[lane_index].nearest_back,
                           10*UPDATE_INTERVAL, &kinematic);
  return kinematic;
}
