# Path Planning Project
[CarND Term 3 Path Planning Project]
(https://github.com/udacity/CarND-Path-Planning-Project)

---

## Overview
In this project, the aim is to implement a path planning model to safely navigate around a virtual highway with traffic.

### Project video

[![Path Planning](http://img.youtube.com/vi/MVja8Lsb7NI/0.jpg)](http://www.youtube.com/watch?v=MVja8Lsb7NI)

## The Project
In order to implement Path Planning model, the main.cpp file was modified. Vehicle class was added to the file. The spline.h was included in the project. The Vehicle class was responsible for generating trajectories and calculating  costs of the trajectories.

### Required Steps For Path Planning

#### Prediction
Prediction is the first step of path planning. It involves, predicting the behavior of other vehicles, and estimating their location at a time step in the future. Because of the time limitations, predictions only performed by increasing the s values of other cars, and the d value of each other car assumed to be constant in each step. The provided vx and vy values were combined in a single v, and this v value used to calculate the s value in the future.

#### Trajectory Generation
Trajectory Generation is the second step and probably the most challenging step of path planning. During generation step, maximum acceleration and maximum velocity values were set in advance. The interactions with the other cars were calculated, such as closest approach, and lowest time to collide. In addition, other penalties added for not driving at the target speed, changing lanes, driving at side lanes, canceling the previous action, driving in an occupied lane.

#### Jerk Minimization
Jerk Minimization is an important aspect of path planning, however, it was not implemented in this projects due to the highly challenging transitions between Frenet and XY coordinates.

### Data From The Simulator
The simulator provides the localization data, previously generated path data, and the sensor fusion data. At each step, new path data was added to the previously generated path data. In order to predict the behavior of the other cars, their velocity was calculated as the square root of the sum of the squares of vx and vy. The location of another vehicle was calculated by adding the multiplication of that velocity and the time to the current s value of the vehicle. The d value of the car assumed to be constant. This assumption was risky; however, lowering the PLANNING_STEPS variable provide a chance to change the behavior in order to prevent collision.

### Trajectory Generation
Trajectory Generation code was based on the provided Python solution to the "Implement Behavior Planner in C++" quiz in Behavior Planning Lesson. It was quite challenging to switch to continuous space from a discrete space. In addition, the Prepare Lane Change Left and Prepare Lane Change Right states were added to the model. Still, the most challenging part was implementing a recursive state function, since at each step different states can be selected.

The trajectory generation was not complete since it can only assume one acceleration for each state in each step. Adding different acceleration options probably provide better solutions for passing the other vehicles and changing lanes.

The PLANNING_HORIZON parameter sets the number of states that the trajectory generator can plan in each step. Increasing the number to a value greater than 5 results in very slow calculations. In order to decrease the running time, when a collision became more likely, the cost for that trajectory assigned to the MAXCOST, and the following states were not considered. Similarly, when a cost for a trajectory was less than 5, that trajectory was selected directly without calculating the cost for other trajectories. This situation only occurs when the vehicle is in the center lane, and there are no other vehicles in front of it. In other words, the center lane is not occupied and the vehicle is in the center lane.

### Cost Calculation
#### Prepare Without Change Cost
When there is a PLCL or PLCR without a following LCL or LCR, there is a penalty. Because this will lead into a deceleration with no improvement.

#### Inefficiency Cost
The penalty for not driving at the target speed. In order to prevent following another car which drives fast, but not at the target speed, a constant cost is added to the cost as a function of the difference between the average speed of the generated trajectory and the target speed.

#### Near Cost
The penalty for collision, or getting close to a collision. It is based on the minimum distance with the vehicle and the other vehicles.

#### Occupied Cost
The penalty for driving in an occupied lane. It is based on the time to reach to the vehicle in front of the vehicle, assuming the vehicle drives at the target speed.

#### Buffer Cost
Assuming there is no collision, how long will it take to collide is calculated, and if that value is greater than the DESIRED_BUFFER, this cost will be greater than 0.

#### Change Lane Cost
Calculated to minimize the total number of lane changes.

#### At Lane Cost
If the vehicle is in the side lanes, a penalty cost will be added.

### Generating Waypoints
During trajectory generation, the acceleration values were calculated for the vehicle for each step. These values were used during generating waypoints.

The spline function was used to calculate y values as a function of x values. As provided in the walkthrough video, the last point of the vehicle and the point before that were used as the first two points. Three following points were also added to the spline. These three points were obtained by using the target_s and target_d values. In order to prevent exceeding maximum acceleration, the maximum difference between d values in the sequential points was set to 4.0.

Using the constructed spline, x value was incremented step by step, and the corresponding y value was calculated.
