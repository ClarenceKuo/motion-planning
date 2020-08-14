#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#define NUM_PREV_POINTS 2
#define MAX_A 0.224
#define MAX_V 50
#define DEBUG 1

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
double ref_v = MAX_A;
vector<vector<double>> getTraj(Car, Metadata, Waypoint);
int getCurrentLane(Car);
bool canSwitch(int, Metadata, Car);
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          //Store data into self defined data structure
          Car car{
            .x = j[1]["x"],
            .y = j[1]["y"],
            .s = j[1]["s"],
            .d = j[1]["d"],
            .yaw = j[1]["yaw"],
            .speed = j[1]["speed"]
          };
          
          Metadata metadata{
            // Previous path data given to the Planner
            .previous_path_x = j[1]["previous_path_x"],
            .previous_path_y = j[1]["previous_path_y"],
            
            // Previous path's end s and d values 
            .end_path_s = j[1]["end_path_s"],
            .end_path_d = j[1]["end_path_d"],
            
            // a list of all other cars on the road.
            .sensor_fusion = j[1]["sensor_fusion"]
          };
          
          Waypoint waypoint{
            .map_x = map_waypoints_x,
            .map_y = map_waypoints_y,
            .map_s = map_waypoints_s,
            .map_dx = map_waypoints_dx,
            .map_dy = map_waypoints_dy
          };

          json msgJson;
          
          // Generate next_x_vals and next_y_vals to form trajectory.
          vector<vector<double>> traj = getTraj(car, metadata, waypoint);
          vector<double> next_x_vals = traj[0];
          vector<double> next_y_vals = traj[1];

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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

int getCurrentLane(Car car){
  for( int i = 0; i < 3; i ++){
    if (car.d < (2 + 4 * i + 2) && car.d > (2 + 4 * i - 2))
    	return i;
  }
  if(DEBUG)
    std::cout << "Err in getCurrentLane: Cannot get lane info!!" << std::endl;
}

bool canSwitch(int targetLane, Metadata metadata, Car car){
  for (int i = 0; i < metadata.sensor_fusion.size(); i ++){
    float d = metadata.sensor_fusion[i][6];
    if (d < (2 + 4 * targetLane + 2) && d > (2 + 4 * targetLane - 2)){
      double s = metadata.sensor_fusion[i][5];
      if (abs(s - car.s) < 30){
        if(DEBUG)
          std::cout << "Failed! " << std::endl;
        return false;
      }
    }
  }
  if(DEBUG)
    std::cout << "Success! " << std::endl;
  return true;
}

vector<vector<double>> getTraj(Car car, Metadata metadata, Waypoint waypoint){
  unsigned int prev_size = metadata.previous_path_x.size();
  if(prev_size > 0){
    car.s = metadata.end_path_s;
  }
  
  //flags
  bool tooClose = false;
  bool shouldChangeLane = false;
  int currentLane = getCurrentLane(car);
  
  for (int i = 0; i < metadata.sensor_fusion.size(); i++){
    //iter each car to check if car in same lane
    float d = metadata.sensor_fusion[i][6];
    float d_lane = 2 + 4 * currentLane;
    if ( d < (d_lane + 2) && d > (d_lane - 2)){
      double vx = metadata.sensor_fusion[i][3];
      double vy = metadata.sensor_fusion[i][4];
      double v = sqrt(vx * vx + vy * vy);
      double s = metadata.sensor_fusion[i][5];
      
      //Check distance
      if((s > car.s) && (s - car.s < 30)){
        tooClose = true;
        
        //check if change lane is safe
        if(s - car.s > 15){
          shouldChangeLane = true;
        }
        break;
      }
    }
  }
  //reaction to previous detection
  if(tooClose){
    ref_v -= 0.9 * MAX_A;
    if(shouldChangeLane){
      if(((currentLane == 0) || (currentLane == 2)) && canSwitch(1, metadata, car)){
        if(DEBUG)
          std::cout << "Changed to lane 1" << std::endl;
        currentLane = 1;
      }
      else if(currentLane == 1){
        if(canSwitch(0, metadata, car)){
          currentLane = 0;
          if(DEBUG)
            std::cout << "Changed to lane 0" << std::endl;
        }
        else if(canSwitch(2, metadata, car)){
          currentLane = 2;
          if(DEBUG)
            std::cout << "Changed to lane 2" << std::endl;
        }
      }
    }
  }
  //no blocking, check if can speed up 
  else if(ref_v < (MAX_V - 0.5))
    ref_v += MAX_A;
  
  // generate path
  // fit points
  vector<double> fitx;
  vector<double> fity;
  double ref_x = car.x;
  double ref_y = car.y;
  double ref_yaw = deg2rad(car.yaw);
  
  //check previous data
  if(prev_size < NUM_PREV_POINTS){
    double prev_car_x = car.x - cos(car.yaw);
    double prev_car_y = car.y - sin(car.yaw);
    
    fitx.push_back(prev_car_x);
    fitx.push_back(car.x);
    
    fity.push_back(prev_car_y);
    fity.push_back(car.y);
  }
  else{
    ref_x = metadata.previous_path_x[prev_size - 1];
    ref_y = metadata.previous_path_y[prev_size - 1];
    double ref_x_prev = metadata.previous_path_x[prev_size - 2];
    double ref_y_prev = metadata.previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    
    fitx.push_back(ref_x_prev);
    fitx.push_back(ref_x);
    fity.push_back(ref_y_prev);
    fity.push_back(ref_y);
  }
  
  // Create Anchor points for interpolation
  for(int i = 1; i < 4; i++){
    vector<double> next_wp = getXY(car.s + i * 30, 2 + 4 * currentLane, waypoint.map_s,waypoint.map_x,waypoint.map_y);
    fitx.push_back(next_wp[0]);
    fity.push_back(next_wp[1]);
  }
  
  // Shift ref angle to 0 degree
  for (int i = 0; i < fitx.size(); i++){
    // Shift car reference angle to 0 degrees
    double shift_x = fitx[i] - ref_x;
    double shift_y = fity[i] - ref_y;
    fitx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    fity[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }
  
  // Create spline instance
  tk::spline s;
  s.set_points(fitx,fity);
  
  // Trajectory
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  // Fill in history points
  for (int i = 0; i < prev_size; i++){
    next_x_vals.push_back(metadata.previous_path_x[i]);
    next_y_vals.push_back(metadata.previous_path_y[i]);
  }
  
  //fill rest point until reach 50 points
  unsigned int cur_size = next_x_vals.size();
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y + target_y);
  double prev_node_loc = 0;// ???
  while(cur_size < 50){
    double N = (target_dist / (0.02 * ref_v / 2.24));
    double x_point = prev_node_loc + (target_x) / N;
    double y_point = s(x_point);

    prev_node_loc = x_point;

    double x_ref = x_point;
    double y_ref = y_point; 

    // Transform back to Cartisian 
    x_point = ref_x + (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = ref_y + (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
    cur_size = next_x_vals.size();
  }
  vector<vector<double>> traj; 
  traj.push_back(next_x_vals);
  traj.push_back(next_y_vals);
  return traj;
}
