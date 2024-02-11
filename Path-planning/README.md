
# Path-planning subsystem for mobile robot executing area cleanup

**The purpose is to develop the underlying path-planning algorithm for mobile robot executing area cleaning functions. This considers a system for an autonomous mobile robot to:**  

1) plan an exploration movement trajectory in a rectangular area of given size to detect all litter objects of initially unknown locations and amount
2) Plan an optimal visiting trajectory of all the detected litterings for subsequent collection. The environment is flat, static and without obstacles. 


![](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/blob/main/Experiments/coppelia_exp_snake_trj_2sections_2obj/Two%20sections%20exploration%20narrated_ENG.gif.gif)

### Requirements.
Given the size and aspect ratio of working enviroment that is flat, static and without obstacles the system should:
1) Plan an exploration movement trajectory in the area to detect all litter objects of initially unknown locations and amount 
2) Plan an optimal visiting trajectory of all the detected litter after exploration for subsequent collection.

### Methods. 
The problem of determining optimal visiting path of the detected litterings is Travelling Salesman problem. Within the framework of the proposed structure of path-planning module, algorithm for determining optimal visiting path have been developed using **Genetic algorithm**. 

The robot should plan an exploration movement trajectory in the area to detect all litter objects of initially unknown locations and amount. To solve this the robot moves through the enviroment as follows: moving in the section along snake-like trajectory, visiting litter objects, moving to an adjacent section.

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/8fc2a6ab-37d1-47fa-8a55-73e8c2ac15ca)

### Results. 

See ![experiments folder](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/tree/main/Experiments) for Real Kuka Youbot deployment and CompeliaSim experiments

The developed software and algorithms for a mobile robot implement the functions of planning optimal movement trajectory for enviroment exploration and visiting litterings for collection.

The correct trajectories of the mobile robot's movement through the environment and litter visiting were obtained while correctly divided into sections. In addition, the system has shown its effectiveness in various geometric configurations of the area to be cleaned.


*Here: GIFs from coppelia_exp_snake_trj_2sections_2obj videos*

## Environment exploration reference trajectory planning:

The task of planning the exploration trajectory is to form a path in which the robot, traveling around the environment, will be able to detect all objects of litter for further collection of these objects. The robot moves around the environment according to a snake-like trajectory without deviation from it. As the robot moves, detected objects are added to the map of the area. The aforementioned behavior is implemented within the sections into which the territory to be cleaned is divided. After the robot completes its exploration trajectory of a section, it plans the optimal route for visiting the litter objects for their subsequent collection within this section. Then, robot moves to the next section. 

The input is the size of the rectangular area to be cleaned. The area is divided into Nh х Nw sections: Nh - the number of vertical sections, Nw - the number of horizontal sections.

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/689bee6c-c6b8-4b10-ab76-c433d55eb265)

where H, W – height and width of the area to be cleaned, set by the user; wr - half the width of the robot's field of view; ℎ𝑠 – height of each section; 𝑛𝑤 – the number of vertical movements within one row of sections; 𝑑𝑤 – the horizontal step of the snake-like trajectory; 𝑤𝑠𝑓 – width of sections; 𝑤𝑠𝑒 – the width of the last section in the course of the robot's movement, within the same row.

![terrain_snake_exploration_planning](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/d62b36f1-e2fd-405c-bfed-4f225d979b1c)

![sector_moving_alg_flowchart_ENG](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/013f2a6d-7d59-4ba1-906b-e57bc728fd0a)


##  Planning of litter objects visiting path (Travelling Salesman problem):

In the context of planning the optimal visiting path of litterings, this task involves searching for the shortest route passing through all detected objects at least once, taking into account the current position of the robot. The optimal path planning is performed once when requested by other modules of the robot software, assuming that litter objects won't change their location during visiting time. Grasping an object is possible only if it is located at the intersection of the field of view and the radius of reach of the manipulator.

Travelling Salesman problem is NP-complete problem that can't be solved by Brute-force when the number of cities is more than 66. Given that it's likely to have more than 10 litter objects within one section, **Genetic algorithm** was chosen to solve the problem. It offers good approximation of the optimal route at a relatively low computational cost. Despite the fact that this algorithm may not find the best solution, it is able to generate a relatively optimal solution for finding a route between a hundred cities in less than a minute.

The input for Genetic algorithm is the location of the robot and the location of litterings at the current time. These locations are taken from SLAM module of the robot software.

**The specifics of TSP determines the implementation of evolutionary algorithm operators:**

1. *Solution encoding*: each population individual is encoded as ordered sequence of litterings' locations. It represents the order how the robot visits them. 
2. *Mutation*: implements random permutation of litterings' locations such that the user can set the max sequence distance coefficient of the swapped objects. 

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/a264d754-9eff-44c8-86ac-c178aa060736)

3. *Crossvoer*: random subsequence A is chosen from the first solution, subsequence B consists of the litterings not present in A in the order how they appear in the second solution. The crossover result is the concatenation of A and B s.t. a subsequence having a location ordered first is put to the result beginning.

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/0746d2fb-4a56-4f89-87b4-07fc349c3fb1)


4. *Fitness function*: the inverse length of visiting path

5. *Selection*: tournament selection with probability of a solution being selected proportional to its fitness

6. *Stoping criterion*: a new more optimal solution hasn't appeared in the poplulation during the last N generations.


### SOFTWARE STRUCTURE

The software includes a subsystem of the user interface, interaction with the Vision analysis and SLAM subsystems  and third-party applications, as well as modules for planning the exploration trajectory through the enviroment and the litter objects visiting path for a mobile robot.

![path_planning_structure_ENG](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/9d22a9ee-ae9f-4b31-99ea-02c87ced3233)



