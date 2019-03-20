# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program.

Highway Driving Project

[image1]: ./images/img.jpg "Path Planning"

## Project Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Code Compilation
The code compiles correctly without errors using cmake and make. No changes were made to CMakeLists.txt

## Valid Trajectories
All the following valid trajectory conditions have been met:

1. The car was able to drive at least 4.32 miles without incident. as seen in the image below.

![alt text][image1]

2. The car doesn't drive faster than the speed limit and it does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

3. The car does not come into contact with any of the other cars on the road.

4. The car stays in its lane, except for the time between changing lanes. The car is able to change lanes

## Reflection

The execution of the project is divided into 3 parts: Prediction, Behavior Planning and Trajectory Calculation. The code is well commented.

### Prediction
The prediction part of the code is between line 105 and 141.
Here the sensor fusion data is used to determine the positions of all other cars on the road. The d values are used to determine in which lanes the cars are while speed is determined using the the vx and vy components. The cars' s positions are also estimated after execution of the previous trajectories. Lastly the positions of the cars relative to our car are determined. These are stored as boolean values in 'car_ahead', 'car_left' and 'car_right' variables.

### Behavior Planning
Behavior Planning involves changing the behavior of our car depending on the positions of other cars on the road. This is achieved through lines 143 to 167. The conditions below are implemented in the logic for changing lanes:
1. If there's a car ahead of us in our lane, and we are in the middle or right lane, and there's no car in the lane left of us  within 25m of our car then we make a left lane change.

2. If there's a car ahead of us in our lane, and we are in the middle or left lane, and there's no car in the lane right of us within 25m of our car then we make a right lane change.

3. If there's a car ahead of us in our lane, and there are cars either to the right or left of us so we cannot make a lane change. In this case reduce speed and maintain the lane.

4. If there's no car ahead of us and we aren't in the middle lane, then get back to the middle lane. Increase speed if our speed is less than the maximum speed.

### Trajectory Calculation
Trajectory calculation is performed from lines 170 to 270. To begin, 2 points from the previous trajectory are determined (if not available) using the the car's current position and its heading. Next 3 target points are defined 30m, 60m and 90m from our current position. These 5 points are transformed into local car coordinates and are used to initialize the spline calculation. Using a for-loop, the other trajectory points are calculated using a the spline function.
