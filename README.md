
# Kuka Youbot for Autonomous Garbage Collection

**In this repo I present the results of development of my final Bachelor thesis project. The goal is to develop the control system for mobile robot Kuka Youbot that makes it capable of autonomous litter collection.** 

A **dedicated paper** (English) was published upon the development of **Visual litter detection** subsystem and can be found by this [link](https://www.rtj-mirea.ru/jour/article/view/732/489)

**ROS middleware** was used to control Kuka Youbot. The code for the control system and its subsytems is written in Python.

*Here i should insert eye-catching .GIF showing on the same plane simultaneously how the robot visits litter in Lab, SLAM sys draws its trajectory while more detected litter spawn on the map, and then the visiting path is drawn on the map*

### System requirements. The robot should do:

1. Given a monocular RGB camera mounted on the Kuka Youbot arm, the Robot should be able to detect litter objects and map them on the working environment reference frame

2. Given no knowledge about its initial location, the robot should explore unknown working environment detecting and mapping litter objects while simultaneously localizing itself within the environment. The working environment is flat, static and without obstacles.   

3. Given no initial knowledge about the number and locations of litter objects within the flat, static and no-obstacles environment, the robot should plan exploration path for detecting litter. Then, given the environment map it built during exploration phase the robot should find optimal path for visiting detected litter and visit litter objects as if it was meant to collect it. 

*Here: Most impressive images and GIFs of any system and subsystem workflow*

## Gentle introduction:
At present, increasing rates of pollution of vast areas by various types of household waste are becoming
an increasingly serious problem. In this connection, the creation of a robotic complex capable of performing
autonomous litter collection functions becomes an urgent need.

### The robotic control system has the following compotents:

1.  **Computer Vision** module enabling the robot of detecting and classfication litter objects within its monocular camera field-of-view and mapping the objects on the working environment reference system. The detailed descritpion of how the module works can be found in the dedicated folder.  
**Methods:** YOLOv5 for litter detection and Homography for coordinates reconstruction.

2. **SLAM** module: enables the robot to simultaneously build the map of unknown working environment and localize itself on the map.The detailed descritpion of how the module works can be found in the dedicated folder.
**Methods:** Extended Kalman Filter with feedback from odometry and litter objects as visual landmarks

3. **Path-planning** module: enables the robot to build an optimal(or sub-optimal) path for visiting the detected within the working environment litter objects for subsequent collection. The detailed descritpion of how the module works can be found in the dedicated folder.
**Methods:** Genetic algorithm for solving Travelling Salesman Person problem.


### Kuka Youbot specifications:

![Real-world Kuka Youbot](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/61a0076b-4b57-4fb9-81a1-364aabf0e5e6)


**Mobile base:**

* Weight – 24 Kg
* Dimensions (L x W x H) – 531 x 380 x 140 mm
* Degrees of freedom – 3 
* Load Capacity – 20 Kg
* Connection protocol - EtherCad
* Supply voltage – 24 V

![чертеж мобильной платформы](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/ddc8466d-1ad4-4655-a929-3c76510f34b6)


**Robotic arm:**

* Weight – 6.3 Kg
* Height – 655 mm
* Number of axes – 5
* Type of gripping device – gripping with two plates
* Load Capacity – 0.5 Kg
* Positioning accuracy – 1 mm
* Supply voltage – 24 V
* Axial rotation speed – 90 deg/s

![Рабочая область манипулятора](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/3a0446e6-73f7-4eee-a03f-3f27d7b19d17)


**Hardware and sensors:**

* Camera(Visual sensor): Asus Xtion Pro with horizontal field-of-view of 58°, vertical field-of-view of 45°, 1240 х 1080 RGB and RGBD
* Lidar 
* odometry wheel sensors
* On-board PC: CPU Intel Atom D510 1.66 GHz, RAM 2 GB SO-DDR2-667 200PIN, SSD 32 GB, 6 USB 2.0 ports, Ubuntu OS
* Communication protocol - TCP

## Experiments

### CompeliaSim simulation

### Real Kuka Youbot deployment

### Result
The control system for mobile robot for area cleanup was developed. It enables the robot to autonomously explore unknown enviroment, detect litter objects and optimally visit them for collection. One of the system's advantage is that it doesn't require exteral global or local positioning systems that makes able to work inside and within the noisy electromagnetic environments. 

**Further improvements:**

* Development of the subsystem robotic arm control
* Development of the path-planning subsystem that makes the robot able to work within dynamic environment with obstacles

