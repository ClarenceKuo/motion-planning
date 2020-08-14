# Motion Planning

### Goals
In this project, a trajectory planner that outputs suggestion for autonomous vihicles is presented. The goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
![](https://i.imgur.com/5XqMSDv.jpg)

### Input
1. Sensor Fusion: simulated sensor data that detects other vihicles.
2. Map: simulated sparse map data for lane lines, curvature of the road etc.
3. Constraints: speed, accleration, jerk limit.

### Output
50 location points directing where should the car move in the following second.

## Planner Model Documentation
### Constrain
1. speed limit: 50 MPH
2. acceleration limit: 10 m/s^2 = 0.44 MPH/s
3. jerk limit: 10 m/s^3 = 0.44 MPH/s^2

### Detail
Each time frame (0.2 sec), the ```getTraj``` function is called to generate a new trajectory path.
The function perform same-lane detection and checking from the sensor fusion data to insure the car is still safe to continue the same path. 2 flags will be created in this step:
1. ```tooClose``` flag: indicating the car is too close to the car in front of it.
2. ```shouldSwitchLane``` flag: indicating the car is in the safe distance with the car in front of it and should try to swith lane.

After same-lane check, the planner will react to the flags.
1. if ```tooClsoe``` flag is raised, the car speed is reduced according to the max acceleration ```MAX_A``` until the flag is turned off.
2. if ```shouldChangeLane``` flag is raised, the planner exams(through ```canSwitch``` function) whether the neighboring lane is safe to switch. If yes, it collects target lane.

Dealt with both senarios, the planner then start creating the location dots, one dot per 0.2 seconds. There are 50 of them in total.

First, the planner generates 3 (x,y) locations , spaced evenly at 30m. The starting reference point is generated in tangent to the last two points in the previous path of the vehicle. In case the previous path is empty (the car is just starting), the car position itself is used as reference instead.

To smooth the path, using the  ```spline``` library, the planner fit the previously collected location points to create a smooth path.

Finally, the planner genrerate true location points from the smooth path according to the referenced velocity(distance for 0.2 seconds).

## Usage
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```