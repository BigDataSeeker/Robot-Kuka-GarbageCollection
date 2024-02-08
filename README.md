
# Kuka Youbot for Autonomous Garbage Collection

**In this repo I present the results of development of my final Bachelor thesis project. The goal is to develop the control system for Kuka Youbot that makes it capable of autonomous litter collection.** 

## Gentle introduction:
At present, increasing rates of pollution of vast areas by various types of household waste are becoming
an increasingly serious problem. In this connection, the creation of a robotic complex capable of performing
autonomous litter collection functions becomes an urgent need.

The robotic control system has the following compotents:

1.  **Computer Vision** module enabling the robot of detecting litter objects within its monocular camera field-of-view and mapping the objects on the working environment reference system. 
**Methods:** YOLOv5 for litter detection and Homography for coordinates reconstruction.

2. **SLAM** module: enables the robot to simultaneously build the map of unknown working environment and localize itself on the map.
**Methods:** Extended Kalman Filter

3. **Path-planning** module: enables the robot to build an optimal(or sub-optimal) path for visiting the detected within the working environment litter objects for subsequent collection. 
**Methods:** Genetic algorithm for solving Travelling Salesman Person problem.

