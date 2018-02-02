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
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

int main() {
  uWS::Hub h;

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

#ifdef REPO_ORIGINAL
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
#else

  // start in lane 1.
  int lane = 1;

  // target velocity
  double ref_vel = 0; //49.5; // mph

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {

    const double MIN_ACCEL = 0.010;
    const double MAX_ACCEL = 0.224;
    const double MIN_SPEED = MAX_ACCEL * 5;
    const double MAX_SPEED = 49.7;
    const double SAFETY_RANGE_FRONT = 10;
    const double SAFETY_RANGE_REAR = 10;
    const double SAFETY_TIME_RANGE0 = 1.7; // sec
    const double SAFETY_TIME_RANGE1 = 2.5; // sec
    const double SAFETY_TIME_RANGE2 = 3.0; // sec

#endif
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
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

#ifndef REPO_ORIGINAL

                int prev_size = previous_path_x.size();
                if (prev_size > 0) {
                   car_s = end_path_s;
                }




                //
                // Prediction
                //

                bool vacant_ahead  = true;
                bool vacant_left2  = (lane == 2 ? true : false);
                bool vacant_left1  = (lane >= 1 ? true : false);
                bool vacant_right1 = (lane <= 1 ? true : false);
                bool vacant_right2 = (lane == 0 ? true : false);

                double ahead_car_speed  = ref_vel;
                double vacant_ahead_time  = 100.;
                double vacant_left_time  = 100.;
                double vacant_right_time = 100.;

                for ( int i = 0; i < sensor_fusion.size(); i++ ) {
                   // data format for each car is: [id, x, y, vx, vy, s, d]. 
                   double s = sensor_fusion[i][5];
                   float  d = sensor_fusion[i][6];

                   // lane 0: 0 < d < 4
                   // lane 1: 4 < d < 8
                   // lane 2: 8 < d < 12
                   int car_lane = d / 4;
                   if (car_lane < 0 || 3 <= car_lane) {
                      continue;
                   }
                   bool ahead = (car_s < s ? true : false);

                   // car speed in global map
                   double vx = sensor_fusion[i][3];
                   double vy = sensor_fusion[i][4];
                   double speed = sqrt(vx*vx + vy*vy);

                   // correct exact car position with lapse from previous trajectory.
                   s += (double)prev_size * 0.02 * speed;

                   double distance = s - car_s;
                   if (std::abs(distance) <= 0.0000001) distance = 0.0000001; // prevent Zero division
                   double relative_speed = speed - car_speed;

                   // its estimation assumes all cars drive along the road
                   // distance > 0 : the car is ahead
                   // distance < 0 : the car is behind
                   // relative_speed < 0: the car is slower
                   // relative_speed > 0: the car is faster
                   //
                   // collision condition
                   // distance > 0 and relative_speed < 0
                   // distance < 0 and relative_speed > 0
                   double time_to_collision = std::abs(distance / relative_speed);
                   if ((distance > 0 && relative_speed > 0) ||
                       (distance < 0 && relative_speed < 0)) {
                      time_to_collision = SAFETY_TIME_RANGE2 + 10; // practically maximum value
                   }


#ifdef DEBUF_MSG
                   if (car_lane == lane && -20 < distance && distance < 50) {
                      std::cout << "sensor fusion: "
                                << "id: " << i
                                << "  lane: " << car_lane
                                // << "  speed: " << car_speed
                                // << "  speed: " << speed
                                << "  distance: " << static_cast<int>(distance)
                                << "  relative speed: " << static_cast<int>(relative_speed)
                                << "  collision time: " << time_to_collision
                                << "  s: " << s
                                << "  d: " << d
                                << std::endl;
                   }
#endif // DEBUF_MSG

                   // time-base prediction
                   if (time_to_collision < SAFETY_TIME_RANGE0) {
                      if (car_lane == lane && ahead) {
                         vacant_ahead = false;
                         if (time_to_collision < vacant_ahead_time) {
                            vacant_ahead_time = time_to_collision;
                            ahead_car_speed  = speed;
                         }
                      }
                   }

                   if (time_to_collision < SAFETY_TIME_RANGE1) {
                      if (car_lane == lane - 1) {
                         vacant_left1 = false;
                         if (time_to_collision < vacant_left_time)
                            vacant_left_time = time_to_collision;
                      }
                      if (car_lane == lane + 1) {
                         vacant_right1 = false;
                         if (time_to_collision < vacant_right_time)
                            vacant_right_time = time_to_collision;
                      }
                   }

                   if (time_to_collision < SAFETY_TIME_RANGE2) {
                      if (car_lane == lane - 2) {
                         vacant_left2 = false;
                      }
                      if (car_lane == lane + 2) {
                         vacant_right2 = false;
                      }
                   }

                   // distance-base prediction
                   bool vicinity_s = (((s > car_s && s - car_s < SAFETY_RANGE_FRONT) ||
                                       (s < car_s && car_s - s < SAFETY_RANGE_REAR)) ? true : false);

                   if (car_lane == lane && ahead && vicinity_s) {
                      vacant_ahead = false;
                   }
                   if (car_lane == lane - 1 && vicinity_s) {
                      vacant_left1 = false;
                   }
                   if (car_lane == lane + 1 && vicinity_s) {
                      vacant_right1 = false;
                   }
                   if (car_lane == lane - 2 && vicinity_s) {
                      vacant_left2 = false;
                   }
                   if (car_lane == lane + 2 && vicinity_s) {
                      vacant_right2 = false;
                   }
                }




                //
                // Behavior
                //

                if (vacant_ahead) {
                   // if no car ahead, speed up to MAX_SPEED
                   double diff_speed = (MAX_SPEED - ref_vel) * 0.02;
                   if (diff_speed < MIN_ACCEL) diff_speed = 0;
                   ref_vel += std::min(diff_speed, MAX_ACCEL);

                   // return to the center lane when the road is clear
                   if (vacant_left1 && vacant_left2) {
                      std::cout << "move to center lane :" << vacant_left_time << std::endl;
                      lane --;
                   } else if (vacant_right1 && vacant_right2) {
                      std::cout << "move to center lane :" << vacant_right_time << std::endl;
                      lane ++;
                   }

                } else {
                   // 
                   std::cout << "time to collision: " << vacant_ahead_time << " ";
                   if (vacant_left1 && vacant_right1) {
                      // select side
                      if (vacant_right_time < vacant_left_time) {
                         std::cout << "-> left lane :" << vacant_left_time;
                         lane --;
                      } else {
                         std::cout << "-> right lane: " << vacant_right_time;
                         lane ++;
                      }

                   } else if (vacant_left1) {
                      std::cout << "-> left lane :" << vacant_left_time;
                      lane --;
                   } else if (vacant_right1) {
                      std::cout << "-> right lane: " << vacant_right_time;
                      lane ++;
                   } else if (MIN_SPEED < ref_vel) {
                      // speed control when no lane to escape to
                      std::cout << "-> slow down";

                      // Speed Cost
                      double diff_speed = std::abs((ref_vel - ahead_car_speed) * 0.02);
                      if (diff_speed < MIN_ACCEL) diff_speed = 0;
                      ref_vel -= std::min(diff_speed, MAX_ACCEL);
                   }

                   std::cout << std::endl;
                }





                //
                // Trajectory from Q&A video
                //

                // (x, y) space waypoints
                vector<double> ptsx;
                vector<double> ptsy;

                double ref_x = car_x;
                double ref_y = car_y;
                double ref_yaw = deg2rad(car_yaw);

                if ( prev_size < 2 ) {
                   // in the case with not enough data
                   double prev_car_x = car_x - cos(car_yaw);
                   double prev_car_y = car_y - sin(car_yaw);

                   ptsx.push_back(prev_car_x);
                   ptsx.push_back(car_x);

                   ptsy.push_back(prev_car_y);
                   ptsy.push_back(car_y);
                } else {
                   // Use the latest two points.
                   ref_x = previous_path_x[prev_size - 1];
                   ref_y = previous_path_y[prev_size - 1];

                   double ref_x_prev = previous_path_x[prev_size - 2];
                   double ref_y_prev = previous_path_y[prev_size - 2];
                   ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                   ptsx.push_back(ref_x_prev);
                   ptsx.push_back(ref_x);

                   ptsy.push_back(ref_y_prev);
                   ptsy.push_back(ref_y);
                }

                // 30m spaced 3 points ahead of the starting reference in Frenet
                vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                ptsx.push_back(next_wp0[0]);
                ptsx.push_back(next_wp1[0]);
                ptsx.push_back(next_wp2[0]);

                ptsy.push_back(next_wp0[1]);
                ptsy.push_back(next_wp1[1]);
                ptsy.push_back(next_wp2[1]);

                // Shift car reference angle to 0 degree
                for ( int i = 0; i < ptsx.size(); i++ ) {
                   double shift_x = ptsx[i] - ref_x;
                   double shift_y = ptsy[i] - ref_y;

                   ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                   ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                }

                // Create the spline
                tk::spline s;
                s.set_points(ptsx, ptsy);

                // Actual (x, y) points to use for the path planner
                for ( int i = 0; i < prev_size; i++ ) {
                   next_x_vals.push_back(previous_path_x[i]);
                   next_y_vals.push_back(previous_path_y[i]);
                }

                // Calculate distance y position on 30 m ahead.
                double target_x = 30.0;
                double target_y = s(target_x);
                double target_dist = sqrt(target_x * target_x + target_y*target_y);

                double x_add_on = 0;

                // fill up 50 till points
                for( int i = 1; i < 50 - prev_size; i++ ) {

                   double N = target_dist / (0.02 * ref_vel / 2.24);
                   double x_point = x_add_on + target_x / N;
                   double y_point = s(x_point);

                   x_add_on = x_point;

                   double x_ref = x_point;
                   double y_ref = y_point;

                   x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                   y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
                   x_point += ref_x;
                   y_point += ref_y;

                   next_x_vals.push_back(x_point);
                   next_y_vals.push_back(y_point);
                }
#endif

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
