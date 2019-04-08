#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
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

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

void print_vector(std::vector<double> const &input) {
  for(int i = 0; i < input.size(); i++) {
    std::cout << input.at(i) << ' ' ;
  }
}

void print_vector(std::vector<string> const &input) {
  for(int i = 0; i < input.size(); i++) {
    std::cout << input.at(i) << ' ' ;
  }
}

vector<string> get_possible_actions(int lane) {
  vector<string> possible_actions;
  if(lane == 0) {
    possible_actions.push_back("Right");
  } else if(lane == 1) {
    possible_actions.push_back("Left");
    possible_actions.push_back("Right");
  } else {
    possible_actions.push_back("Left");
  }

  //std::cout << "\n\n########## possible_actions ############" << "\n";
  //print_vector(possible_actions);
  //std::cout << "\n########## possible_actions ############" << "\n";

  return possible_actions;
}

int get_next_lane(int current_lane, bool &switch_lane, vector<string> possible_actions, double car_s, vector<vector<double>> sensor_fusion) {

  // For each possible state, from sensor fusion data loop through all the cars from the possible next lane
  int next_lane;
  int safe_front_buffer = 30;
  int safe_back_buffer = 20;
  bool switch_left_lane = false;
  bool switch_right_lane = false;

  std::cout << "\n@@@@@@@@@@@@@@ current_lane: " << current_lane << " @@@@@@@@@@@@@@@@@" << "\n";

  for(int s = 0; s < possible_actions.size(); s++) {
    string current_action = possible_actions[s];
    std::cout << "\n########## possible_action: " << current_action << " ############" << "\n";
    if(current_action == "Left") {
      next_lane = current_lane - 1; 
    } else { 
      next_lane = current_lane + 1; 
    } 

    for(int j = 0; j < sensor_fusion.size(); j++) {
      // verify is a car is in the current lane
      float df = sensor_fusion[j][6];
      if(df < (2 + 4*next_lane +2) && df > (2 + 4*next_lane -2)) {
        double vfx = sensor_fusion[j][3];
        double vfy = sensor_fusion[j][4];
        double check_speed_f = sqrt(vfx*vfx+vfy*vfy);
        double check_car_s_f = sensor_fusion[j][5];

        // 0.02 since the car will visit a point every 0.02 seconds 
        check_car_s_f += (check_speed_f / 2.24); // predict next s for the given car
        bool front_check_failed = ((check_car_s_f > car_s) && ((check_car_s_f - car_s) < safe_front_buffer));
        bool rear_check_failed = ((check_car_s_f < car_s) && ((car_s - check_car_s_f) < safe_back_buffer));
        if(front_check_failed || rear_check_failed) {
          if(current_action == "Left") {
            switch_left_lane = false;
          } else {
            switch_right_lane = false;
          } 

          if(front_check_failed) {
            std::cout << "Front safety check failed ########## \n" ;
          } else {
            std::cout << "Rear safety check failed ########## \n" ;
          }
          std::cout << "######## cannot change to " << current_action << " lane.\n" ;
          break;
        //} else {
        //  std::cout << "Front and Rear safety check did not fail ########## \n" ;
        //  std::cout << "j: " << j << "\n";
        //  std::cout << "sensor_fusion size: " << sensor_fusion.size() << "\n";
        }
      }

      // Made it to last sensor fusion data for a possible lane without failing safety checks
      if(j == (sensor_fusion.size() - 1)) {
        //std::cout << "Made it to last sensor fusion data for a possible lane without failing safety checks ########## \n" ;
        if(current_action == "Left") {
          switch_left_lane = true;
        } else {
          switch_right_lane = true;
        }
      }
    }

    // favoring left lane switches
    if(switch_left_lane) {
      break;
    }
  }

  if((switch_left_lane) || (switch_right_lane)) {
    std::cout << "!!!!!!!! switching to " << next_lane << " lane !!!!!!!!!!!! by taking\n";
    switch_lane = true;
  }

  return next_lane;
}

#endif  // HELPERS_H
