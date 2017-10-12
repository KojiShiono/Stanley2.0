# Stanley2.0
Udacity Self Driving Car Final Project

### Team Members:
1. Koji Shiono (**Team Lead**) : kshiono@umich.edu  
2. Alessandro Melis: alessandro_melis@rocketmail.com
3. Artem Larionov: larionov.artem@gmail.com
4. Bowen Weng: bxw229@case.edu
5. Piyush Karande: piyush.karande@wustl.edu

### Video examples
https://youtu.be/8p-YpxrCijI

https://youtu.be/jWGAe9wpOvs

https://youtu.be/s2fD1IzT5EE

https://youtu.be/yPPAP1gzW78

## 1. Algorithm Summary
The submitted module consists of three parts, the perception, path planning and control.

### 1.1 Perception
Perception module was implemented in "tl_detector" for red traffic light detection. The algorithm starts from counting possible red pixels from the given image. The red traffic light distance is estimated with reference to the average x position of red pixels relevant to the image frame. State and distance is pubslihed if certain threshold is reached.

### 1.2 Path Planning
The closest way point is located given the comlete list of way_points and the ego vehilce's current position. Planned path is constructed with number of "LOOKAHEAD_WPS" points in the future and "LOOKBACK_WPS" points in the past relevant to the closest way point calculated. 

### 1.3 Control
Control module subscribes to the traffic light state and distance, as well as the planned path from the way_point update module. Velocity (brake and throttle) and steering are controlled seperately with PID controllers.

#### 1.3.1 Steering
With all planned way points transferred to vehicle frame, the cross track error is calculated through the "UnivariateSpline" module in scipy. The steering controller than updates its maneuver with the calculate cross track error.

#### 1.3.2 Velocity
The velocity reference is set to the speed limit if no red traffic light is detected. When red traffic light is spotted, velocity reference is set to zero. The velocity target is introduced as an internal variable when the distance between the velocity reference and the current ego vehicle's velocity is too wide that's beyond the designed acceleration and deceleration capability.

## 2. Validation
The current submission is validated with the driving simulator. The working frequencies of publishers are turned down to the level where most of the team members' hardware specs are sufficient to run the program. A higher working frequency is always welcome when hardware capability permits. 

For real driving, we plan to disable the distance detection and set velocity reference to zero whenver a red light is being spotted. 
