
# Kuka Youbot for Autonomous Garbage Collection

**In this repo I present the results of development of my final Bachelor thesis project. The goal is to develop the control system for Kuka Youbot that makes it capable of autonomous litter collection.** 

### System requirements. The robot should do:

1. Given a monocular RGB camera mounted on the Kuka Youbot arm, the Robot should be able to detect litter objects and map them on the working environment reference frame

2. Given no knowledge about its initial location, the robot should explore unknown working environment detecting and mapping litter objects while simultaneously localizing itself within the environment. The working environment is flat, static and without obstacles.   

3. Given no initial knowledge about the number and locations of litter objects within the flat, static and no-obstacles environment, the robot should plan exploration path for detecting litter. Then, given the environment map it built during exploration phase the robot should find optimal path for visiting detected litter and visit litter objects as if it was meant to collect it. 


*Here i should insert eye-catching .GIF showing on the same plane simultaneously how the robot visits litter in Lab, SLAM sys draws its trajectory while more detected litter spawn on the map, and then the visiting path is drawn on the map*

## Gentle introduction:
At present, increasing rates of pollution of vast areas by various types of household waste are becoming
an increasingly serious problem. In this connection, the creation of a robotic complex capable of performing
autonomous litter collection functions becomes an urgent need.

The robotic control system has the following compotents:

1.  **Computer Vision** module enabling the robot of detecting litter objects within its monocular camera field-of-view and mapping the objects on the working environment reference system. 
**Methods:** YOLOv5 for litter detection and Homography for coordinates reconstruction.

2. **SLAM** module: enables the robot to simultaneously build the map of unknown working environment and localize itself on the map.
**Methods:** Extended Kalman Filter with feedback from odometry and litter objects as visual landmarks

3. **Path-planning** module: enables the robot to build an optimal(or sub-optimal) path for visiting the detected within the working environment litter objects for subsequent collection. 
**Methods:** Genetic algorithm for solving Travelling Salesman Person problem.

