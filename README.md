# Robotics 2018
The goal of this set of components is to control the movement of a robot in a simulation under the Robocomp environment. The control of the robot has been carried out in successive and incremental phases adding new functionalities to the movement of the robot in each iteration. The content of each iteration is described below:


## ITERATION 1
### Goals
Move the robot continuously without colliding with obstacles in a simulation based on a mesh generated with an xml file
### Components required
1. DifferentialRobot
2. Laser
### Edited Files
1. specificWorker.h
2. specificWorker.cpp
### Methods implemented
| Method        | Returns | Parameters  | Explanation  |
| :-------------|:-------:|:-----------| :------------|
| Compute()     | void    | No parameter| It orders the distances of the laser beam to the obstacles from least to greatest and if the smallest of them is smaller than an established threshold it rotates and if it does not continue advancing. The direction of rotation will depend on whether the angle of rotation of the smallest measured distance is positive or negative |
### Execution
1. Open a terminal and write: 

```cd ~/robocomp/files/innermodel/simpleworld.xml```

2. Open another terminal and execute: 

```cd/[ path_where_the_component_is ]```
```cmake .```
```make```
```bin/controller```


## ITERATION 2
### Goals
Move the robot to a target marked with the mouse in the simulation (obstacles have not been taken into account)
### Components required
1. DifferentialRobot
2. Laser
### Subscription required
1. RCISMousePicker
### Edited Files
1. specificWorker.h
2. specificWorker.cpp
### Methods implemented (changed)
| Method        | Returns | Parameters  | Explanation  |
| :-------------|:-------:|:----------- | :------------|
| f1(float d)          | float   | The parameter d is a float that represents the module of the distance from the robot to the target | Limits the speed of the robot depending on the distance to the target with which it enters the function |
| f2(float r, float h, float Vx)          | float   | The parameters r, h and Vx are three floats that represent the angle from the robot to the target and two constants that allow us to calculate the acceleration | Define the evolution of the robot's speed as a normal distribution |
| Compute()     | void    | No parameter| Transforms the reference systems of the coordinates so that the robot and the marked target are in the same system and moves the robot to the target with an acceleration that follows a normal distribution |
| SetPick(const Pick &myPick)     | void    | myPick is a constant reference to the Pick class that stores the coordinates of the points captured in the simulation when clicking on them | Set the coordinates of the objective from the click done in the simulation |
### Execution
1. Open a terminal and write: 

```rcnode```

2. Open another terminal and execute: 

```cd ~/robocomp/files/innermodel/simpleworld.xml```

3. Open another terminal and execute: 

```cd/[ path_where_the_component_is ]```
```cmake .```
```make```
```bin/componente1```

## ITERATION 3
### Goals
Move the robot to a target marked with the mouse in the simulation avoiding the obstacles found using the bug algorithm
### Components required
1. DifferentialRobot
2. Laser
### Subscription required
1. RCISMousePicker
### Edited Files
1. specificWorker.h
2. specificWorker.cpp
### Methods implemented (changed)
| Method        | Returns | Parameters  | Explanation  |
| :-------------|:-------:|:-----------| :------------|
| Compute()     | void    | No parameter| Takes the robot from its position to the target by establishing a coordinate system common to the world and the robot and defining a state machine that allows the robot to move around avoiding obstacles |
| letsmove()    | void    | The bState parameter is a reference to the TBaseState class that stores the position of the robot. The ldata parameter is a reference to the TLaserData class that stores the characteristics of the robot's laser beam| Move the robot to the target unless the laser detects an obstacle in its path at a distance less than the set threshold, in that case changes the state of the state machine |
| startbug()    | void    | The bState parameter is a reference to the TBaseState class that stores the position of the robot. The ldata parameter is a reference to the TLaserData class that stores the characteristics of the robot's laser beam| Spin the robot or change the status of the state machine according to the robot's situation with respect to the target |
| bug()         | void    | The bState parameter is a reference to the TBaseState class that stores the position of the robot. The ldata parameter is a reference to the TLaserData class that stores the characteristics of the robot's laser beam| Surround an obstacle by changing the state machine to continue straight if the distance to the obstacle is contained between the threshold and a defined value and changing the state if it moves away from the obstacle to continue rolling the obstacle until an obstacle is detected in the line that take to the objective |
| endbug()      | void    | The bState parameter is a reference to the TBaseState class that stores the position of the robot| Deprecated |
| inTarget()    | bool    | No parameter| Stops the robot if it is at a distance from the target less than a threshold set returning true in that case and false otherwise |
| distanceToTarget(const TBaseState& bState)| float    | The bState parameter is a reference to the TBaseState class that stores the position of the robot| Returns the distance from the current position of the robot to the target |
### Execution
1. Open a terminal and write: 

```rcnode```

2. Open another terminal and execute: 

```cd ~/robocomp/files/innermodel/simpleworld.xml```

3. Open another terminal and execute: 

```cd/[ path_where_the_component_is ]```
```cmake .```
```make```
```bin/componente1```



## ITERATION 4
### Goals
The robot moves towards the target planning the route to the target once known the environment where it will move.
### Components required
1. DifferentialRobot
2. Laser
### Subscription required
1. RCISMousePicker
### Edited Files
1. specificWorker.h
2. specificWorker.cpp
3. grid.h
### Methods implemented
| Method        | Returns | Parameters  | Explanation  |
| :-------------|:-------:|:----------- | :------------|
| setParams(RoboCompCommonBehavior::ParameterList params)         | boolean   | The parameter d is a float that represents the module of the distance from the robot to the target | Limits the speed of the robot depending on the distance to the target with which it enters the function |
|compute()        | void   | No parameter | Define the evolution of the robot's speed as a normal distribution |
| saveToFile()     | void    | No parameter| Transforms the reference systems of the coordinates so that the robot and the marked target are in the same system and moves the robot to the target with an acceleration that follows a normal distribution |
| readFromFile()     | void    | No parameter | Set the coordinates of the objective from the click done in the simulation |
| updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)     | void    | myPick is a constant reference to the Pick class that stores the coordinates of the points captured in the simulation when clicking on them | Set the coordinates of the objective from the click done in the simulation |
| updateVisitedCells(int x, int z)     | void    | No parameter | Set the coordinates of the objective from the click done in the simulation |
| draw()  | void    | No parameter | Set the coordinates of the objective from the click done in the simulation |
| setPick(const Pick &myPick)  | void    | myPick is a constant reference to the Pick class that stores the coordinates of the points captured in the simulation when clicking on them | Set the coordinates of the objective from the click done in the simulation |
### Execution
1. Open a terminal and write: 

```rcnode```

2. Open another terminal and execute: 

```cd ~/robocomp/files/innermodel/simpleworld.xml```

3. Open another terminal and execute: 

```cd/[ path_where_the_component_is ]```
```cmake .```
```make```
```bin/chocachoca```


## ITERATION 5
### Goals
The targets are established using  a series of marks on the wall that will be detected by the robot's camera and the robot will move towards this marks.
### Components required
1. DifferentialRobot
2. Laser
3. GotoPoint
### Subscription required
1. RCISMousePicker
2. AprilTags
### Implements 
1. GotoPoint
### Edited Files
1. specificWorker.h
2. specificWorker.cpp
### Methods implemented
| Method        | Returns | Parameters  | Explanation  |
| :-------------|:-------:|:----------- | :------------|
| setPick(const Pick &myPick) | void   | myPick is a constant reference to the Pick class that stores the coordinates of the points captured in the simulation when clicking on them | Set the coordinates of the objective from the click done in the simulation |
| go(const string &nodo, const float x, const float y, const float alpha) | void | nodo is a reference to the id of the tag, x is a float containing the coordinates of the x axis, y is a float containing the coordinates of the y axis and alpha is the angle  | go to the point specified with the features given by parameters |
| turn(const float speed);     | void    | speed is a float containing the turning speed | turn the robot with aa angle and speed specified |
| atTarget()     | bool | No parameter | returns true if the robot is in the target,false otherwise |
| stop()    | void | No parameter | Stops the robot |
| newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState);     | void  | tags is a constant reference to tagsList, bState is a reference to TBaseState from RobocompGenericBase and hState is a reference to MototStateMap from RoboComp JointMotor | Not used
| newAprilTag(const tagsList &tags)    | void | tags is a constant reference to tagsList containing the distansce and the id of the tag | Catch the features of the tag visualized and transform its coordinates to the coordinates of the simulation |


## Supervisor
### Goals
Helps the Controller to check that the places are visited.
### Components required
1. GotoPoint
### Edited Files
1. specificWorker.h
2. specificWorker.cpp
### Methods implemented
| Method        | Returns | Parameters  | Explanation  |
| :-------------|:-------:|:----------- | :------------|
| setParams(RoboCompCommonBehavior::ParameterList params)          | bool   | params is  a ParameterList from RobocompCommonBehavior | Define the model of the simulation | 
| newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState)          | void   | tags is a constant reference to tagsList, bState is a reference to TBaseState from RobocompGenericBase and hState is a reference to MototStateMap from RoboComp JointMotor | Not used
| newAprilTag(const tagsList &tags)     | void    | tags is a constant reference to tagsList containing the distansce and the id of the tag | Catch the features of the tag visualized and transform its coordinates to the coordinates of the simulation |


### Execution
1. Open a terminal and write: 

```rcnode```

2. Open another terminal and execute: 

```cd ~/robocomp/files/innermodel/informatica-tercio.xml```

3. Open another terminal and execute ApriltagsComp: 

```cd ~/robocomp/components/robocomp-robolab/components/apriltagsComp```
```bin/apriltagsComp```

4. Open another terminal and execute the iteration5's component: 

```cd/[ path_where_the_iteration5_component_is ]```
```cmake .```
```make```
```bin/chocachoca```
5. Open another terminal and execute the supervisor's component: 

```cd/[ path_where_the_supervisor_component_is ]```
```cmake .```
```make```
```bin/supervisor```
