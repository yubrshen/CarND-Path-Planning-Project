#ifndef UTILS
#define UTILS
/*
utils.h

Here are utilities for path planning.

 */

#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "parameters.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
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
  // assert(0 <= d);
  // assert(d <= lane_width*NUM_LANES);
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

double wrap_around(double s) {
  double new_s = 0;
  if (s < 0)
    new_s = remainder(s, MAX_S);
  // new_s = s - q*MAX_S, where q = cell(s/MAX_S), so when s < 0, q*MAX_S <= s, thus 0 <= s - q*MAX_S
  else
    new_s = fmod(s, MAX_S);
  // new_s = s - q*MAX_S, where q = floor(s/MAX_S), so when 0 < s, q*MAX_S <= s, 0 <= s - q*MAX_S
  assert(0 <= new_s);
  assert(new_s < MAX_S);
  return new_s;
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  double heading = atan2( (map_y-y),(map_x-x) );
  double angle = abs(theta-heading);
  if(angle > pi()/4)
  {
    closestWaypoint++;
  }
  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_s)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }
  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
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
  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }
  // calculate s value
  double frenet_s = maps_s[0];
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }
  frenet_s += distance(0,0,proj_x,proj_y);
  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;
  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
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
  return {x,y};
}

#endif
