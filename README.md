# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Background Information
This README covers the higher level logic behind the path planner implemented in 
this repository.
For more comprehensive background information on the backend of this project 
please refer to https://github.com/udacity/CarND-Path-Planning-Project.

## Path Planner Logic
The logic is included in the `main` function in `src/main.cpp`. As was done in the 
base implementation of this project (see section Background Information), this 
path planner works with a `reference_velocity` variable (`main.cpp(229)`)
initialized at 0. When no obstacles have been detected (to be explained later), 
this reference velocity increments at a rate acceptable by the jerk and acceleration
constrains (`preferred_acceleration_rate = 0.6; //main.cpp(280)`) until the maximum 
allowed speed of `49km/h` has been reached. 

Obstacles are considered as vehicles within a bounded region around the ego-vehicle's
 lane, in front of the ego-vehicle and have an forward projected position that is 
 closer than a given threshold parameter.
 
 More precisely, `getFrontCarProximityAndVelocitySCoordinates` approximates the positions
 of all detected vehicles in the next time step, based on their current position and 
 velocity. It returns the distance between the ego-vehicle and the project position
 of the closest vehicle, in front of the ego-vehicle, in the same lane. The second 
 returned variable is this said vehicle's velocity. 
 
 A vehicle in front of the ego-vehicle, in the same lane is considered an 
 obstacle if the distance returned by `getFrontCarProximityAndVelocitySCoordinates`
 is than a threshold parameter `critical_front_car_separation //main.cpp(277)`.
 
 At this point, the ego-vehicle begins deccelerating by the same magnitude 
 `preferred_acceleration_rate` until either one of the following situations:
 1. It matches the velocity of the obstacle vehicle
 2. It escapes to the left lane
 3. It escapes to the right lane
 
 In case 1. matching the velocity of the obstacle vehicle has shown empirically
 to prevent collisions when the ego-vehicle attempts to overtake the obstacle vehicle.
 
 To check the feasibility of the escape the function `checkCanEscapeToLane` is used.
 This function takes in a lane number as a variable and scans whether any vehicle 
 in that lane (in front or behind) is too close to the ego-vehicle (by a threshold
 `critical_overtake_free_space`). Note that the `critical_overtake_free_space` is set
 to a value higher than the `critical_front_car_separation`. This ensures that
 the situation in the new lane is actually better than that in the current lane. 
 In practice, this has empirically shown to avoid cases where the ego-vehicle
 hesitates between two lanes, violating the out-of-lane flag.
 
 Checking both in front and behind the ego-vehicle prevents collisions with incoming
 traffic during the transition to the new lane.
 
 Once the ego-vehicle has transitioned into the new lane, it may either accelerate
 to get to the preferred cruising speed or avoid any further obstacles.
 
 The remainder of the program follows that of the base implementation