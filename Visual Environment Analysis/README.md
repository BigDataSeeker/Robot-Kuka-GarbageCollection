
# Computer vision system for mobile robot executing area cleanup

**The purpose is to develop the underlying algorithms for the vision system of robots executing area cleaning functions. This considers a visual environmental analysis system for an autonomous mobile robot to search for and recognize different categories of waste, and localize the litter in a given area for subsequent collection. For such purposes, it is sufficient for the robot to have a single on-board camera.** 

A **dedicated paper** (English) was published upon the development of **Visual litter detection** subsystem and can be found by this [link](https://www.rtj-mirea.ru/jour/article/view/732/489)


![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/dd2ac300-8d13-4cee-8754-7d85eef8d962)


### Requirements.
Given a video frame from monocular RGB on-board camera mounted a mobile robot with known image parameters: resolution, focal length, and known geometric position parameters: camera height and angle, as well as robot's position.  The system should detect litter objects in the field of view of the camera, determine their local coordinates relative to the robot camera and their projection on the terrain map.

### Methods. 
Within the framework of the proposed structure of external environment visual analysis, algorithms for detecting and classifying objects of various appearance have been developed using convolutional neural networks. The CNN detector was trained on the open [TACO](https://arxiv.org/abs/2003.06975) dataset. To determine the geometric parameters of a surface in the field of view of the robot and estimate the
coordinates of objects on the ground, a homography matrix was formed to take into account information about the
characteristics and location of the video camera.

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/2684102a-672f-4647-915e-9a238260b44d)


### Results. 
The developed software and algorithms for a mobile robot equipped with a monocular RGB camera are capable of implementing the functions of detection and classification of litter objects in the frame, as well as projection of found objects on a terrain map for their subsequent collection.

![exp_results](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/a07c26ec-41ae-44cb-9f84-d433e88411ea)

## Detection and classification of litter objects in video frame:
YOLOv4 was trained on open dataset TACO comprising 1500 images with 4784 labeled objects sorted into 28 classes, which are globally divided into paper, glass, plastic, and metal. As a result
of training, the model was optimized to 0.41 mAP (mean average precision) by 4 classes with 38 frames/sec running time on GPU.

![2_objs_det_bbox_cls](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/3681873e-ca91-4008-838f-141f3d3179bf)


## Projection of litter objects video frame coordinates in terrain map coordinates :

It's assumed that the robot environment coordinates are known, so litter coordinates can be calculated relative to the robot in order to determine their environment coordinates. 

This can be done by using the homography matrix. *However, this
transformation is valid only for objects on a flat surface.
Thus, the proposed model of coordinate determination
is valid only for objects lying on a flat surface.*

Perspective distortion can be eliminated by converting the previously determined video frame coordinates of litter objects into coordinates in the top-view of the space in front of the robot on the basis of the tilt angle, height above the surface and camera focal length.

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/86aef5e4-e3c9-42c8-af3b-6d87539d9d64)

The coordinates on the shot plane and spatial plane
are defined by the following relation:

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/3df22e8e-7f1e-443e-bda1-89021f2c7acc)

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/74cbd297-cbf6-4f0d-9b66-d8742817d14f)

According to the geometrical explanation in the image above, the homography matrix H0 can be described as follows:

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/90dfbe23-dba7-4f5c-a307-0172af02e04c)

Provided the camera angle is different from 0° and 90°, there is an inverse to this transformation, which can be used to obtain the top view from an image with a distorted perspective and vice versa.

The frame from the robot’s camera is analyzed by YOLOv4, which determines the pixel coordinates of the object on the frame as four bounding box coordinates p_i = ( x_i, y_i); then, the center of object bbox is taken. This coordinate undergoes the ransformation as described in the expression below to determine the relative object pixel coordinates within the visible space in front of the robot:

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/685a30f1-c713-42a2-91e3-3c01366dc6c0)

To convert the pixel coordinates of objects in the visible space in front of the robot into meter coordinates relative to the robot, it is necessary to calculate the dimensions of the visible space. According to the explanations in the image below, the geometric parameters of the space in front of the robot are unambiguously specified by the horizontal opening angle, vertical opening angle, height and camera tilt angle.

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/2d6337db-df1c-4f25-bab9-2ad80253c1cb)

The distance from the camera to the near edge of the visible space d_f, the distance from the camera to the far edge of the visible space d_r, and the length of the visible space Y are determined by the following relations:

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/2b5898a4-757c-46b8-935c-48ce797666dd)

The half-width of the near edge of the visible space w_f, the half-width of the far edge of the visible space w_r, and the width of the visible space X are determined by the following relations: 

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/6c65bca5-a1ab-464e-a6ec-58361134434b)

According to the aforesaid, the coordinates of objects relative to the robot are defined by the following ratios:

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/9926743f-10a0-406b-9dd6-27a7f486e408)

### SOFTWARE STRUCTURE

The software includes a subsystem of the user interface, libraries of image acquisition and preprocessing, as well as modules of target object recognition and calculation of its coordinates relative to the mobile robot.

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/6f500c73-0508-455f-a049-c4c2dc967dc0)

### EXPERIMENTAL RESEARCH

The conducted experiments confirmed the performance of the software and algorithmic software on the basis of Kuka Youbot. 

Thus, images below demonstrate phases of recognizing and localizing several litter objects in the robot’s field of view, removing perspective distortion, and calculating the coordinates of objects relative to the robot according to a camera of height h = 0.47 m, tilt angle α = 45°, vertical camera opening angle β = 23.75°, and horizontal camera opening angle γ = 30.41°.

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/c59ed59a-852b-4885-b412-0726e9777b3e)

![image](https://github.com/BigDataSeeker/Robot-Kuka-GarbageCollection/assets/92204945/6f8613d1-9d86-4807-a0c6-a0ef8461e4c8)


**Camera(Visual sensor)** - Asus Xtion Pro with horizontal field-of-view of 58°, vertical field-of-view of 45°, 1240 х 1080 RGB and RGBD

**Further improvements:**

* Development of the algorithm that can determine coordinates on environment map of detected objects lying on non-flat surface.


