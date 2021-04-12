# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Below is the data provided from the Simulator to the C++ Program

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
    ``
* [Udacity's Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Project Instructions and Rubric

## Rubric Points
The sections below will dicsuss the details of my implementation of each requirement in the [rubric points](https://review.udacity.com/#!/rubrics/1971/view).

## Implementation Details
All the code for implementation below can be found [on this file](./src/map.cpp).

There are 

### 1. Determine whether vehicles are in the vicinity and attempt to overtake vehicles in current lane.

The input for this section is the data from sensor fusion.

Based on the current speed of vehicle nearbys, we can assess determine:
- if the are vehicles within too close to the ego vehicle
- if there is a car blocking our path in the current lane
- if it's safe to pass on the right
- if it's safe to pass on the left
- whether it's unsafe to overtake and we need to decelerate

A. For this implementation, a safe distance was considered to be 20m, which is 2/3 of the trajectory we are planning.

B. To determine best course of action, we first determine the lane through which each vehicle is travelling.

C. Next, we determine if they expected to be closer than the safe distance within 2ms.

D. Then, if there is a vehicle in front of the ego vehicle, we determine if any vehicle is on the right (from B) within the safe distance (from C). If there is none, then it is safe to pass on the right. Otherwise, repeat the process for the left.

E. If both, lanes are blocked decelerate with a factor of 0.224 m/s to avoid jerk.

F. If there are no cars, proceed to accelerate with a factor of 0.224 m/s to avoid jerk.

### 2. Add points to the new trajectory

In order to ensure consistency between updates, we must add the remaining points from the previous trajectory to our new trajectory in a couple steps.

If available, the next two points in the trajectory are leveraged to set the immediate transition. Otherwise, add a hypothetical point tangent to the direction of the vehicle as the first point.

Then, proceed to add a desired number of waypoints to the trajectory. For this implementation, we are adding the waypoints that are within 30, 60, and 90m from the current location of the vehicle.

As this points are likely to be fart apart, we must proceed to interpolate them through the use of a spline. However, to faciliate interpolation, we must first rotate our points around our yaw and shift them to the origin of our vehicle. This allows all tranformations to be performed at an angle of 0 degrees from (0,0).

We proceed to interpolate the points using the spline. For this, we first determine the target distance in y to be travelled for a given distance in x (Set at 30m for this implementation). Then we determine the number of points needed from the spline by dividing the target distance over the distance travelled in 2ms at the current speed (adjusted from the previous step), and converted to m/s.

Then, we add enough points to the trajectory until there are 50 points (combined with those remaing from the previous trajectory). For each point, we split the distance evenly and perform an inverse transformation to align back to global coordinates. 

These points represent then the new trajectory



