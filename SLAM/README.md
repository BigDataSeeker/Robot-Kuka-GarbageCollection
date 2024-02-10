
# SLAM subsystem for mobile robot executing area cleanup

**The purpose is to develop the underlying algorithm of the mobile robot executing area cleaning functions for Simultaneous localization and mapping(SLAM).** The need behind SLAM is linked to the fact that maps commonly used for agent navigation mainly reflect the environment state captured at the map construction time, while the state of the environment can be different at the time of maps usage. At the same time, it's very hard to accurately determine the current robot location while building an accurate map due to the limited accuracy of sensors. The SLAM approach fuses these two independent processes into a continuous cycle of sequential calculations, while the results of one process affect the calculations of another process.

**This considers a system for an autonomous mobile robot to:** 

1) build the map of initially unknown working environment along the course of movement 

2) localize itself on the environment map. The environment is flat, static and without obstacles. 

*Here: GIFs from coppelia_exp_snake_trajectory_3obj videos*


### Requirements.
Given no initial knowledge about the working environment and robot location on it, the system should:
1) build the map of the environment along the course of movement adding detected litter objects on it  
2) localize itself on the environment map.

### Methods. 

Mapping - its phases

Lacalization - EKF


*Here: figures ...*

### Results. 

The developed software and algorithms for a mobile robot implement the functions of building the environment map while simultaneously localizing the robot on it. 

In an experiment with 2 litter objects on the environment Kuka Youbot successfully detected them and added to the map while localizing itself and tracking own trajectory. 

*Here: fig. 3.7 from thesis*

**The following is developed:**

1. Models for identification between detected litter objects and already present on the map  

2. Models for building the environment map 

3. Models for localizaion of a mobile robot based on **Extended Kalman Filter** with feedback from visual landmarks
 

## Building the environment map:

The problem of buiding the environment map is to add detected litter objects on the map along the course of robot movement.

**The following tasks are to be solved in order to build the map:**

1. Detect litterings lying in the robot's field of view at current
time and calculate their local coordinates relative to the robot - assumed to be solved by the robot Visual analysis subsystem

2. Determine the global robot coordinates at current time in the reference frame relative to the robot wake-up starting point - assumed to be solved by the self-localization module of the robot SLAM subsystem

3. In accordance with the global robot coordinates, for objects already known and mapped, calculate their local coordinates in the reference frame relative to the robot and determine which of them should lie in the robot's field of view.

4. Solve the problem of identifying detected objects with already known and mapped objects that should lie in the field of view of the robot

5. For those litter objects that are not yet on the map, calculate the global coordinates in the reference frame relative to the robot wake-up starting point and put it on the map

The image below illustrates global reference frame relative to the robot wake-up starting point and local reference frame relative to the robot's current position, where Rm - radius of robot arm reach

*Here: fig. 3.1 from thesis*

Local coordinates of the litter object O can be caculated as follows: 

*Here: eq. 3.1 - 3.2 from thesis*

Global coordinates of the litter object O can be caculated as: 

*Here: eq. 3.3 - 3.4 from thesis*

These solve tasks 3 and 5.

**Objects identification in the robot's field-of-view can be divided into several successive phases:**

-  The phase of calculating the local coordinates of objects present on the map relative to the robot
- The phase of checking if a known object is within robot's field-of-view
- The phase of matching detected objects with known ones that should be in the robot's field of view
- The phase of calculating global coordinates for objects discovered, but not matched to any of the known ones, and adding them to the environment map
- The phase of removing known objects from the map for which no corresponding object was found in the robot's field-of-view

In the phase of checking the location of an object in the robot's field-of-view, the range and bearing angle relative to the robot's camera are calculated from its local coordinates. The range and bearing angle are used to check the clauses of a litter object to be the robot's field-of-view: 

*Here: eq. 3.5 - 3.8 from thesis*

where r и β - range and bearing angle relative to the robot's camera, respectively; 𝑑𝑓, 𝑑𝑟, ɣ - distance from the camera to the closest and farest borders of robot's field-of-view and horizontal camera opening angle, respectively

In the phase of matching detected objects with known ones each detected object is checked for matching with each known object that should lie in the robot's field-of-view until there's a match. The threshold of 0.15 is set considering that there's inaccuracy when local coordinates of known objects are calculated from the global ones.

*Here: eq. 3.9 from thesis*

where x_o_det, y_o_det - local coordinates of the object detected in the field-of-view; x_o_pred, y_o_pred - local coordinates of the object that should be in the field-of-view


## Localization of the robot within the environment:

The problem of robot localization is to localize it on the environment map at each time step.

To solve this problem, **Extended Kalman Filter** is used, since it is an effective recursive filter that evaluates the state vector of a dynamic system using a number of incomplete and noisy measurements. To calculate the current state of the system, the current measurement and previous state of the filter itself are used such that state vector distribution is updated every step. 

In the context of mobile robot localization **Extended Kalman Filter** uses the measurements of robot odometry and the measurements of litter objects local coordinates in the robot's field-of-view as visual landmarks. In this work the coordinates of robot's camera center are estimated as robot's coordinates in order to simplify the localization framework. The material point coordinates to be estimated is the camera center because objects local coordinates are calculated relative to camera and the camera center coinsides with the robot's grasper. 

**The following inputs are needed for the localization module working based on EKF:**

1. The velocity control vector given to the robot at the previous time step provided by the path-planning subsystem 

2. Wheel odometry measurements

3. Measurements of local coordinates of litter objects detected in the field-of-view provided by the Visiual analysis subsystem

4. Information about identification between the objects present on the map and the objects currently detected by the Visiual analysis subsystem

**Localization module workflow can be divided into several successive phases:**

- The phase of estimation the location of the robot based on the velocity control vector

- The phase of fusion of the previously estimated robot location with the measurements from the wheel odometry

- The phase of fusion of the previously estimated robot location with the measurements of local coordinates of litter objects detected in the field-of-view

**In the phase of estimation the location of the robot based on the velocity control vector** in the previous time-step the location at current time-step is estimated as follows:

*Here: eq. 3.10 - 3.12 from thesis*

*Here: fig. 3.3 from thesis*

where 𝑈_𝑡−1, 𝜔_𝑡−1 - velocity control vector(linear and angular velocities, respectively); l - robot length; x, y, 𝛩 - x-axis, y-axis global coordinates and bearing angle, respectively.

The covariance matrix is estimated according to:

*Here: eq. 3.13 from thesis*

where P_t, P_t-1 - covariance matrices of the robot location at the current and previous time-steps, respectively; F_t - process evolution matrix that is 3x3 Identity matrix, Q_t - noise covariance matrix that is 3x3 diagonal with 0.5 on the main diagonal in this context.

**In the phase of fusing the estimated robot location with the measurements from the wheel odometry** the location estimated in the previous phase is averaged with odometry measurements(i.e. robot global coordinates) according to:

*Here: eq. 3.14 - 3.17 from thesis*

where x, y, 𝛩 - x-axis, y-axis global coordinates and bearing angle, respectively; l - robot length; dx, dy, d𝜔 - difference of odometry measurments at current time-step wrt previous time-step.

**In the phase of correction previously estimated robot location from the measurements of local coordinates of litter objects** the module estimates the correction vector of the robot's global coordinates based on the coordinates of currently detected objects identified with the objects already present on the map. This phase works based on Kalman Filter parameters calculation for the measurements of each visual landmark(identified litter objects in the field-of-view).

For each identified object 𝑌_𝑡, 𝐻_𝑡, 𝑆_𝑡 ,𝐾_t, x_o_rel, y_o_rel are calculated:

*Here: eq. 3.18 - 3.21 from thesis*

The corrections are fused with the robot location vector and its covariance matrix as below:

*Here: eq. 3.22 - 3.23 from thesis*

where **R_t** - covariance matrix of the measurement noise, **P_est_t** - the covariance matrix of robot location estimated at previous time-step, **w_t** - measurements inaccuracy vector of the Visual analysis subsystem for objects' local coordinates i.e. (0.05, 0.05) in this case, **Z_t** - the vector of actual measurements of local coordinates for detected and identified object, **Y_t** - the vector of difference between the actual sensors' measurements and the measurements from the sensors' math model, **K_t** - the matrix of Kalman coefficients showing how much each location vector component should be corrected accounting for real sensors' measuremens, **S_t** - the covariance matrix of the math model predicting sensors' measuremens and showing how accurate its measuremens predictions, **H_t** - the matrix of objects' local coordinates that if multiplied by the differnce vector between robot currect estimated location and estimated location at the previous time-step gives the differnce vector between sensor current measuremens and its measuremens at the previous time-step(consists of partial derivatives of sensor measurements math model wrt the components of robot location vector)   


### SOFTWARE STRUCTURE

The subsystem software includes a user interface, modules for receiving and processing wheel odometry, as well as mapping and localization modules for a mobile robot.

*Here: fig. 3.8 from thesis in ENG*



