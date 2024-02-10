#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32, Bool,Float32MultiArray
from geometry_msgs.msg import Twist
import sys
import torch
import numpy as np
import cv2
import time
from math import sin, cos, atan2, tan, radians, sqrt, copysign
import copy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from sys import maxsize
from itertools import permutations

sys.path.append('/home/maxim/Desktop/yolov5')
from models.common import DetectMultiBackend
from utils.augmentations import letterbox
from utils.general import non_max_suppression, scale_coords


class camera():
    def __init__(self, hor_fov, ver_fov, height=None, tilt_a=None):
        self.hor_fov = hor_fov
        self.ver_fov = ver_fov
        self.h = height
        self.tilt_a = tilt_a
        self.d_f = None
        self.d_r = None
        self.w_f = None
        self.w_r = None
        self.area_of_view_size = [None, None]
        self.area_of_view_pxl_size = [None, None]
        self.aov_object_coords = None  # relative to robot

    def rectify_perspective(self, img, h=None, tilt_a=None, detections=[]):
        if h is not None:
            self.h = h
        if tilt_a is not None:
            self.tilt_a = tilt_a
        if img is None:
            print('Given images is None')
            return
        self.d_f = self.h * tan(self.tilt_a - self.ver_fov)
        self.d_r = self.h * tan(self.tilt_a + self.ver_fov)

        self.area_of_view_size[1] = self.d_r - self.d_f

        self.w_f = sqrt(self.d_f ** 2 + self.h ** 2) * tan(self.hor_fov)
        self.w_r = sqrt(self.d_r ** 2 + self.h ** 2) * tan(self.hor_fov)

        self.area_of_view_size[0] = 2 * self.w_r

        w_f_pxl = int((self.w_f / self.w_r) * img.shape[1])
        self.area_of_view_pxl_size[0] = img.shape[1]
        self.area_of_view_pxl_size[1] = int(img.shape[1] * self.area_of_view_size[1] / self.area_of_view_size[0])

        pts_src = np.array([[0, 0], [img.shape[1], 0], [img.shape[1], img.shape[0]], [0, img.shape[0]]], np.float32)
        pts_dst = np.array([[0, 0], [self.area_of_view_pxl_size[0], 0],
                            [int((self.area_of_view_pxl_size[0] + w_f_pxl) / 2), self.area_of_view_pxl_size[1]],
                            [int((self.area_of_view_pxl_size[0] - w_f_pxl) / 2), self.area_of_view_pxl_size[1]]],
                           np.float32)
        h_m, _ = cv2.findHomography(pts_src, pts_dst)

        pts_dst_plane_metric = np.array(
            [[-self.w_r, self.d_r], [self.w_r, self.d_r], [self.w_f, self.d_f], [-self.w_f, self.d_f]],
            np.float32)  ##########
        h_m_metric_plane, _ = cv2.findHomography(pts_src, pts_dst_plane_metric)  ##########

        self.aov_object_coords = []
        if len(detections) != 0:
            for det in detections:
                x11, y11, x22, y22 = det[0], det[1], det[2], det[3]
                x_c = int((x11 + x22) / 2)
                y_c = int(0.3 * y11 + 0.7 * y22)
                # y_c = int(y22)

                frame_coords_pxl = np.array([[x_c], [y_c], [1]])

                img = cv2.circle(img, (x_c, y_c), radius=8, color=(100, 100, 100), thickness=-1)
                aov_object_coords_pxl = np.matmul(h_m, frame_coords_pxl)
                aov_object_coords_pxl[0] /= aov_object_coords_pxl[2]
                aov_object_coords_pxl[1] /= aov_object_coords_pxl[2]
                aov_object_coords_pxl[2] /= aov_object_coords_pxl[2]

                y_o = float(self.area_of_view_size[1] * (
                        1 - aov_object_coords_pxl[1] / self.area_of_view_pxl_size[1]) + self.d_f)
                x_o = float(
                    self.area_of_view_size[0] * aov_object_coords_pxl[0] / self.area_of_view_pxl_size[0] - self.w_r)
                self.aov_object_coords.append([x_o, y_o])

        rectified_img = cv2.warpPerspective(img, h_m, (self.area_of_view_pxl_size[0], self.area_of_view_pxl_size[1]))
        return rectified_img, self.aov_object_coords

    def draw_object_map(self, plot=False):
        # the method plots the objects map of space before robot
        object_pxl_map = np.ones((int(self.area_of_view_pxl_size[1] * (1 + self.d_f / self.area_of_view_size[1])),
                                  self.area_of_view_pxl_size[0], 3)) * 150
        x_r_coords = []
        y_r_coords = []

        for obj in self.aov_object_coords:
            x_r = obj[0]
            x_r_coords.append(x_r)
            y_r = obj[1]
            y_r_coords.append(y_r)
            x_pxl = int(self.area_of_view_pxl_size[0] * (x_r + self.w_r) / self.area_of_view_size[0])
            y_pxl = int(self.area_of_view_pxl_size[1] * (1 - (y_r - self.d_f) / self.area_of_view_size[1]))
            object_pxl_map = cv2.circle(object_pxl_map, (x_pxl, y_pxl), radius=10, color=(100, 0, 200), thickness=10)
        if plot and x_r_coords and y_r_coords:
            ax = plt.gca()
            ax.spines['top'].set_color('none')
            ax.spines['left'].set_position('zero')
            ax.spines['right'].set_color('none')
            ax.spines['bottom'].set_position('zero')

            plt.xlim(-self.w_r, self.w_r)
            plt.ylim(0, self.d_r)
            plt.scatter(x_r_coords, y_r_coords)
            plt.grid(True)
            plt.show()

        return object_pxl_map


class City:
    def __init__(self, x, y, z=0):
        self.x = x
        self.y = y
        self.z = z

    def distance(self, city):
        xDis = abs(self.x - city.x)
        yDis = abs(self.y - city.y)
        distance = np.sqrt((xDis ** 2) + (yDis ** 2))
        return distance

    def __repr__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"

def travellingSalesmanProblem(pointList, s = 0):
    # store all vertex apart from source vertex
    vertex = list(pointList)
    del vertex[s]

    # store minimum weight Hamiltonian Cycle
    min_path_weight = maxsize
    next_permutation = permutations(vertex)
    for route in next_permutation:
        # store current Path weight(cost)
        current_pathweight = 0
        # compute current path weight
        k_p = pointList[0]
        for point in route:
            current_pathweight += sqrt((k_p.x - point.x) ** 2 + (k_p.y - point.y) ** 2)
            k_p = point
        # update minimum
        if current_pathweight < min_path_weight: min_path_weight = current_pathweight;min_path = route

    return min_path_weight, min_path

class position():
    def __init__(self, init_x, init_y, init_theta):
        self.x = init_x
        self.y = init_y
        self.theta = init_theta

    def __repr__(self):
        rep_str = f'x:{self.x},y:{self.y},theta:{self.theta}'
        return rep_str


class Lm_match(Exception):
    pass


class Robot():
    def __init__(self, init_sate, init_vel, wheel_r, robot_width,robot_length, camera, model, device, vel_topic_name):
        self.state_k_minus_1 = init_sate
        self.total_lin_odom_calc = 0
        self.total_rot_odom_calc = 0
        self.last_lin_odom = 0
        self.last_rot_odom = 0
        self.current_total_odom = None
        self.last_total_odom = None

        self.state_k = copy.deepcopy(self.state_k_minus_1)
        self.p_k = np.array([[0.1, 0, 0],
                             [0, 0.1, 0],
                             [0, 0, 0.1]])
        self.states_x = []
        self.states_y = []
        self.for_t_vel = init_vel[0]  # angular velocity
        self.side_t_vel = init_vel[1]  # angular velocity
        self.rot_t_vel = init_vel[2]  # angular velocity
        self.pub_vel = rospy.Publisher(vel_topic_name,Twist, queue_size=10)
        self.odom_data_arr_flag = False
        self.vis_sens_cb_exec_flag = False
        self.wheel_r = wheel_r
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.time_k_minus_1 = time.time()  # time of k-1 step in seconds

        self.obstacle_dead_radius = sqrt(self.robot_length**2 + self.robot_width**2)

        self.robot_camera = camera
        self.device = device
        self.litter_detector = model
        self.cv2_img_aov = None
        self.lm_glob_coord = []

    def upd_odom(self):
        fr_wheel_odom = self.current_total_odom[3] - self.last_total_odom[3]
        rr_wheel_odom = self.current_total_odom[2] - self.last_total_odom[2]
        rl_wheel_odom = self.current_total_odom[1] - self.last_total_odom[1]
        fl_wheel_odom = self.current_total_odom[0] - self.last_total_odom[0]
        self.last_total_odom = self.current_total_odom
        if copysign(1, fr_wheel_odom) == copysign(1, rr_wheel_odom) == copysign(1, rl_wheel_odom) == copysign(1,fl_wheel_odom):
            lin_move = self.wheel_r * -(fr_wheel_odom + rr_wheel_odom + rl_wheel_odom + fl_wheel_odom) / 4
            rot_move = 0
        else:
            lin_move = 0
            lin_move_l_side = self.wheel_r * -(fl_wheel_odom + rl_wheel_odom) / 2
            lin_move_r_side = self.wheel_r * -(fr_wheel_odom + rr_wheel_odom) / 2
            rot_move = (lin_move_r_side - lin_move_l_side) / (self.robot_length + self.robot_width)
        self.last_lin_odom = lin_move
        self.last_rot_odom = rot_move
        self.total_lin_odom_calc += lin_move
        self.total_rot_odom_calc += rot_move
        print(f'Last odometry lin:{lin_move} Rot:{rot_move} arrived at {time.time()}')
        print(f'Total odometry calculated - lin:{self.total_lin_odom_calc} rot:{self.total_rot_odom_calc}')


    def wheel_odom_cb(self,msg):
        self.current_total_odom = msg.data
        if self.last_total_odom is None:
            self.last_total_odom = msg.data
        self.odom_data_arr_flag = True


    def vis_sens_cb(self, msg):
        br = CvBridge()
        img = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = cv2.flip(img, 0)
        self.cv2_img_aov = img
        self.vis_sens_cb_exec_flag = True



    def show_map_cb(self, msg):
        print('Drawing a map ...')
        if msg.data:
            map = self.get_map()
            cv2.imshow('Loctation map', map)
            cv2.waitKey(1)

    def get_map(self):
        # the method plots the objects map of the space near robot
        object_pxl_map = np.ones((int(600), int(600), 3)) * 150
        print('Created empty map')
        for obj in self.lm_glob_coord:
            x_g = obj[0]
            y_g = obj[1]
            x_pxl = int(x_g * 100 + 300)
            y_pxl = int(y_g * 100)
            object_pxl_map = cv2.circle(object_pxl_map, (x_pxl, y_pxl), radius=8, color=(100, 0, 200), thickness=8)
        print('Put objects on the map')
        for pos_idx, _ in enumerate(self.states_x):
            x_g = self.states_x[pos_idx]
            y_g = self.states_y[pos_idx]
            x_pxl = int(x_g * 100 + 300)
            y_pxl = int(y_g * 100)
            object_pxl_map = cv2.circle(object_pxl_map, (x_pxl, y_pxl), radius=3, color=(0, 150, 150), thickness=4)
        print('Put trajectory on the map')
        object_pxl_map = cv2.flip(object_pxl_map, 0)
        return object_pxl_map

    def get_aov_objs_coords(self):
        if not isinstance(self.cv2_img_aov, np.ndarray):
            return None
        img2feed = letterbox(self.cv2_img_aov.copy(), (640, 640), stride=32, auto=True)[0]
        img2feed = img2feed.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img2feed = np.ascontiguousarray(img2feed)
        img2feed = torch.from_numpy(img2feed).to(self.device)
        img2feed = img2feed.float()
        img2feed /= 255
        img2feed = img2feed[None]
        s_time_det = time.time()
        pred = self.litter_detector(img2feed, augment=False, visualize=False)
        f_time_det = time.time()
        print(f'Detection lasted for {f_time_det - s_time_det}')
        pred = non_max_suppression(pred, conf_thres=0.4, iou_thres=0.5, classes=None, agnostic=False,
                                   max_det=100)
        for idx in range(len(pred)):
            pred[idx][:, :4] = scale_coords(img2feed.shape[2:], pred[idx][:, :4], self.cv2_img_aov.shape).round()
            # get detected objects coordinates relative to the vis_sensor
            img_rectified, obj_coords = self.robot_camera.rectify_perspective(self.cv2_img_aov, 0.42, radians(50),
                                                                              reversed(pred[idx]))
        return obj_coords

    def motor_vel_set(self, msg):
        cur_time = time.time()
        time_dk = cur_time - self.time_k_minus_1
        self.calc_state_k(time_dk)
        print(f'Global map:{self.lm_glob_coord}')
        print(f'Calculations of cur_state_k took {time.time() - cur_time}')
        # update target velocity and time k-1
        self.for_t_vel = 0.075 * msg.linear.x  # when forward velocity is set to 1 the robot moves 0.075 m/s
        self.side_t_vel = msg.linear.y
        self.rot_t_vel = -radians(
            13) * msg.angular.z  # when rotational velocity is set to 1 the robot turns 13 degrees/s
        self.time_k_minus_1 = cur_time

    def calc_state_k(self, dk):
        self.state_k.x = self.state_k_minus_1.x + (cos(self.state_k_minus_1.theta) * self.for_t_vel - (self.robot_length * self.rot_t_vel * sin(self.state_k_minus_1.theta)) / 2) * dk
        self.state_k.y = self.state_k_minus_1.y + (sin(self.state_k_minus_1.theta) * self.for_t_vel + (self.robot_length * self.rot_t_vel * cos(self.state_k_minus_1.theta)) / 2) * dk
        self.state_k.theta = self.state_k_minus_1.theta + self.rot_t_vel * dk
        print(f'Robot moved {self.for_t_vel * dk} forward and rotated for {self.rot_t_vel * dk} ')
        rospy.loginfo(f'Predicited robot position before EKF - {repr(self.state_k)}')
        self.slam_ekf()
        rospy.loginfo(f'Predicited robot position after EKF - {repr(self.state_k)}')
        self.state_k_minus_1.x = self.state_k.x
        self.state_k_minus_1.y = self.state_k.y
        self.state_k_minus_1.theta = self.state_k.theta
        self.states_x.append(self.state_k_minus_1.x)
        self.states_y.append(self.state_k_minus_1.y)

    def slam_ekf(self):
        # get relative coordinates of detected objects in AOV

        det_objs_aov_coords = self.get_aov_objs_coords()

        print(f'Detected {len(det_objs_aov_coords)} objs')
        # predict state_k based on odometry and fuse with calculated state_k before
        state_k_odom = position(self.state_k_minus_1.x + cos(self.state_k_minus_1.theta) * self.last_lin_odom - (self.robot_length * self.last_rot_odom * sin(self.state_k_minus_1.theta)) / 2,
                                self.state_k_minus_1.y + sin(self.state_k_minus_1.theta) * self.last_lin_odom + (self.robot_length * self.last_rot_odom * cos(self.state_k_minus_1.theta)) / 2,
                                self.state_k_minus_1.theta + self.last_rot_odom)
        self.state_k.x = 0.2 * self.state_k.x + 0.8 * state_k_odom.x
        self.state_k.y = 0.2 * self.state_k.y + 0.8 * state_k_odom.y
        # calculate heading prediction from motion model and odometry and put in range -pi to pi
        self.state_k.theta = 0.2 * self.state_k.theta + 0.8 * state_k_odom.theta
        self.state_k.theta = atan2(sin(self.state_k.theta),cos(self.state_k.theta))

        pred_cur_aov_objs = {}
        for glob_lm_idx, lm in enumerate(self.lm_glob_coord):
            # calculate AOV objects relative to robot  coordinates from global coordinates
            lm_rel2robot_state_coord = self.cacl_rel_obj_coord_from_glob(lm)
            dist = sqrt(lm_rel2robot_state_coord[0] ** 2 + lm_rel2robot_state_coord[1] ** 2)
            heading = atan2(lm_rel2robot_state_coord[0],
                            lm_rel2robot_state_coord[1])  # ranges from -pi to pi (swapped x with y in atan2 args)
            if (self.robot_camera.d_f < dist < self.robot_camera.d_r) and (abs(heading) < self.robot_camera.hor_fov):
                print(f'Object with glob coords:{lm} is pred to be in AOV')  ######################
                pred_cur_aov_objs[glob_lm_idx] = {'rel': (lm_rel2robot_state_coord), 'glob': (lm[0], lm[1])}
        if len(pred_cur_aov_objs) == 0 and len(det_objs_aov_coords) == 0:
            print('None pred AOV objects and none detected')  ##################################
            return
        if len(pred_cur_aov_objs) == 0 and len(det_objs_aov_coords) > 0:
            # put unknown detected objects on global map
            for rel_obj_coord in det_objs_aov_coords:
                obj_glob_coord = self.cacl_glob_obj_coord_from_rel(rel_obj_coord)
                if not any([sqrt((obj_glob_coord[0] - glob_obj[0]) ** 2 + (obj_glob_coord[1] - glob_obj[1]) ** 2) < 0.07
                            for glob_obj in self.lm_glob_coord]):
                    print(
                        f'There were no pred AOV objects but detected object and put on global map with global coords:{obj_glob_coord}')  ##################################
                    self.lm_glob_coord.append(obj_glob_coord)
            return
        if len(pred_cur_aov_objs) > 0 and len(det_objs_aov_coords) == 0:
            # delete known objects which should lie but undetected in AOV from global map
            for glob_lm_idx in sorted(pred_cur_aov_objs.keys(),reverse=True):
                print(
                    f'No objects detected but predicted object and deleted from global map with global coords:{self.lm_glob_coord[glob_lm_idx]}')  ##################################
                del self.lm_glob_coord[glob_lm_idx]
            return
        # if some objects should lie and detected in AOV then do matching between them and get Kalman corrections
        state_covar_corrections = self.match_pred_with_detected_aov_objs(pred_cur_aov_objs, det_objs_aov_coords)
        # Correct state_k and covariance_k from landmarks
        for _, corrections in enumerate(state_covar_corrections):
            self.state_k.x += corrections['state'][0][0] / len(state_covar_corrections)
            self.state_k.y += corrections['state'][1][0] / len(state_covar_corrections)
            self.state_k.theta += corrections['state'][2][0] / len(state_covar_corrections)
            self.p_k -= corrections['covar'] / len(state_covar_corrections)


    def match_pred_with_detected_aov_objs(self, pred_aov_objs, det_aov_objs):
        corrections_from_lms = []
        unmatched_det_objs = []
        for obj_d in det_aov_objs:
            try:
                for obj_glob_coord_idx, obj_p in pred_aov_objs.items():
                    dist = sqrt((obj_p['rel'][0] - obj_d[0]) ** 2 + (obj_p['rel'][1] - obj_d[1]) ** 2)
                    if dist < 0.15:
                        self.correct_glob_obj_pos(obj_glob_coord_idx, obj_p, obj_d)
                        state_k_correction, covariance_k_correction = self.calc_kalman_param(obj_glob_coord_idx, obj_d)
                        corrections_from_lms.append({'state': state_k_correction, 'covar': covariance_k_correction})
                        print(
                            f"It's a match of objects with rel coords det:{obj_d}, pred:{obj_p['rel']}")  #######################
                        print(
                            f"Global lm coords before {obj_p['glob']} after correction:{self.lm_glob_coord[obj_glob_coord_idx]}")
                        del pred_aov_objs[obj_glob_coord_idx]
                        raise Lm_match
                unmatched_det_objs.append(obj_d)
            except Lm_match:
                continue
        for obj_glob_coord_idx in sorted(pred_aov_objs.keys(), reverse=True):
            # delete unmatched objects which are predicted to lie in AOV from global map
            print(
                f"The pred AOV object with glob coords:{self.lm_glob_coord[obj_glob_coord_idx]} didn't match and is removed from glob map")  #######################
            del self.lm_glob_coord[obj_glob_coord_idx]
        for det_obj in unmatched_det_objs:
            # add unmatched detected objects to global map
            det_obj_glob_coord = self.cacl_glob_obj_coord_from_rel(det_obj)
            if not any([sqrt((det_obj_glob_coord[0] - glob_obj[0]) ** 2 + (
                    det_obj_glob_coord[1] - glob_obj[1]) ** 2) < 0.07 for glob_obj in self.lm_glob_coord]):
                print(
                    f"The detected object with glob coords:{det_obj_glob_coord} didn't match and is put on glob map")  #######################
                self.lm_glob_coord.append(det_obj_glob_coord)

        return corrections_from_lms

    def calc_kalman_param(self, lm_glob_coord_idx, det_obj_rel_coord):
        q_k = np.array([[0.5, 0, 0],
                        [0, 0.5, 0],
                        [0, 0, 0.5]])
        self.p_k = self.p_k + q_k

        # det_obj_glob_coord = self.cacl_glob_obj_coord_from_rel(det_obj_rel_coord)
        det_obj_glob_coord = self.lm_glob_coord[lm_glob_coord_idx]
        h_obj_k = self.calc_h_obj_k(det_obj_glob_coord)

        # measurements are relative to robot (x,y) lm coords
        z_k = np.array([[det_obj_rel_coord[0]],
                        [det_obj_rel_coord[1]]])
        '''
        # measurements are relative to robot (range,heading) lm coords
        z_k = np.array([[sqrt((det_obj_glob_coord[0]-self.state_k.x)**2+(det_obj_glob_coord[1]-self.state_k.y)**2)],
                        [atan2(det_obj_glob_coord[1]-self.state_k.y,det_obj_glob_coord[0]-self.state_k.x)]])
        '''
        x_k = np.array([[self.state_k.x-self.state_k_minus_1.x],
                        [self.state_k.y-self.state_k_minus_1.y],
                        [self.state_k.theta-self.state_k_minus_1.theta]])
        w_k = np.array([[0.05],
                        [0.05]])
        # meas_pred_k = (h_obj_k @ x_k + w_k)
        pred_lm_rel_coords = self.cacl_rel_obj_coord_from_glob(self.lm_glob_coord[lm_glob_coord_idx])
        meas_pred_k = np.array([[pred_lm_rel_coords[0]],
                                [pred_lm_rel_coords[1]]])
        y_k = z_k - meas_pred_k
        print(f'lm actual measurements:{z_k}')
        print(f'lm pred measurements:{meas_pred_k}')
        print(f'Y_k :{y_k}')

        r_k = np.array([[1, 0],
                        [0, 1]])
        s_k = h_obj_k @ self.p_k @ np.transpose(h_obj_k) + r_k

        k_k = self.p_k @ np.transpose(h_obj_k) @ np.linalg.pinv(s_k)
        print(f'K_k:{k_k}')
        state_k_correction = k_k @ y_k
        print(f'state_k_correction:{state_k_correction}')

        covariance_k_correction = k_k @ h_obj_k @ self.p_k
        print(f'covariance_k_correction:{covariance_k_correction}')
        return state_k_correction, covariance_k_correction

    def calc_h_obj_k(self, lm_glob_coord):
        # measurement matrix for measuring relative (x,y) lm coords
        h_k = np.array([
            [-cos(self.state_k.theta - 3.14 / 2), -sin(self.state_k.theta - 3.14 / 2),
             (self.state_k.x - lm_glob_coord[0]) * sin(self.state_k.theta - 3.14 / 2) + (lm_glob_coord[1] - self.state_k.y) * cos(self.state_k.theta - 3.14 / 2)],
            [sin(self.state_k.theta - 3.14 / 2), -cos(self.state_k.theta - 3.14 / 2),
             (self.state_k.x - lm_glob_coord[0]) * cos(self.state_k.theta - 3.14 / 2) + (self.state_k.y - lm_glob_coord[1]) * sin(self.state_k.theta - 3.14 / 2)]])
        '''
        # measurement matrix for measuring relative (range,heading) lm coords
        h_k = np.array([
            [(self.state_k.x - lm_glob_coord[0])/sqrt((lm_glob_coord[0]-self.state_k.x)**2+(lm_glob_coord[1]-self.state_k.y)**2),
             (self.state_k.y - lm_glob_coord[1])/sqrt((lm_glob_coord[0]-self.state_k.x)**2+(lm_glob_coord[1]-self.state_k.y)**2),0],
            [(lm_glob_coord[1] - self.state_k.y)/((lm_glob_coord[0]-self.state_k.x)**2+(lm_glob_coord[1]-self.state_k.y)**2),
             (self.state_k.x - lm_glob_coord[0])/((lm_glob_coord[0]-self.state_k.x)**2+(lm_glob_coord[1]-self.state_k.y)**2),-1]])
        '''
        return h_k

    def correct_glob_obj_pos(self, obj_glob_coord_idx, pred_obj_coord, det_obj_coord):
        # Correct object coordinates on global map by fusing with detected object relative coordinates
        det_obj_glob_coord = (self.cacl_glob_obj_coord_from_rel(det_obj_coord))
        self.lm_glob_coord[obj_glob_coord_idx][0] = (self.lm_glob_coord[obj_glob_coord_idx][0] + det_obj_glob_coord[
            0]) / 2
        self.lm_glob_coord[obj_glob_coord_idx][1] = (self.lm_glob_coord[obj_glob_coord_idx][1] + det_obj_glob_coord[
            1]) / 2

    def cacl_glob_obj_coord_from_rel(self, rel_det_obj_coord):  # should return [x_glob,y_glob] of detected object
        # calculate global object coordinates from relative to robot
        glob_det_obj_x = self.state_k.x + rel_det_obj_coord[0] * cos(3.14 / 2 - self.state_k.theta) + rel_det_obj_coord[
            1] * sin(3.14 / 2 - self.state_k.theta)
        glob_det_obj_y = self.state_k.y - rel_det_obj_coord[0] * sin(3.14 / 2 - self.state_k.theta) + rel_det_obj_coord[
            1] * cos(3.14 / 2 - self.state_k.theta)
        return [glob_det_obj_x, glob_det_obj_y]

    def cacl_rel_obj_coord_from_glob(self, glob_lm_coord):
        lm_rel2robot_state_coord = (
            (glob_lm_coord[0] - self.state_k.x) * cos(self.state_k.theta - 3.14 / 2) + (
                        glob_lm_coord[1] - self.state_k.y) * sin(
                self.state_k.theta - 3.14 / 2),
            -(glob_lm_coord[0] - self.state_k.x) * sin(self.state_k.theta - 3.14 / 2) + (
                        glob_lm_coord[1] - self.state_k.y) * cos(
                self.state_k.theta - 3.14 / 2))
        return lm_rel2robot_state_coord

    def go_around_points(self,points2drive,positioning_precision,check_obstacles = False):
        for t_p in points2drive:
            while (sqrt((t_p.x-self.state_k.x)**2+(t_p.y-self.state_k.y)**2) > positioning_precision):
                print(f'Current point to drive is {t_p}')
                # check if there is an obstacle before th e robot
                if check_obstacles:
                    self.avoid_obstacles()
                else:
                    print(f'Now in avoiding obstacle mode')
                t_vel = get_veloc_t_p(t_p,self.state_k)
                print(f'Velocity to set:lin {t_vel.linear.x} rot {t_vel.angular.z}')
                self.odom_data_arr_flag = False
                self.pub_vel.publish(t_vel)
                while not self.odom_data_arr_flag:
                    pass
                self.upd_odom()
                youbot.motor_vel_set(t_vel)

    def avoid_obstacles(self):
        for obst in self.lm_glob_coord:
            if sqrt((obst[0] - self.state_k.x)**2 + (obst[1] - self.state_k.y)**2) <= self.obstacle_dead_radius:
                print(f'Detected obstacle with coords:{obst} ')
                cv2.waitKey(0)
                avoiding_trajectory_pts = [City(obst[0] - self.obstacle_dead_radius * copysign(1,cos(self.state_k.theta)),obst[1]),
                                                City(obst[0],obst[1] + self.obstacle_dead_radius * copysign(1.5,sin(self.state_k.theta)))]
                self.go_around_points(avoiding_trajectory_pts,0.1,check_obstacles = False)

def get_veloc_t_p(t_point,robot_state_k):
    # the function returns velocities to set to the robot to drive it to the target point
    t_gl_youbot_dir = atan2(t_point.y - robot_state_k.y, t_point.x - robot_state_k.x)
    print(f'Current target global heading is {t_gl_youbot_dir}')
    heading_diff = t_gl_youbot_dir - robot_state_k.theta
    if abs(heading_diff) <= 3.14:
        rot_vel = copysign(0.5, heading_diff)
    else:
        rot_vel = -copysign(0.5, heading_diff)
    for_vel = 0
    if abs(t_gl_youbot_dir - robot_state_k.theta) < 0.1:
        rot_vel = 0
        for_vel = 1 * (sqrt((t_point.x - robot_state_k.x) ** 2 + (t_point.y - robot_state_k.y) ** 2) + 0.4)
    t_vel = Twist()
    t_vel.linear.x, t_vel.angular.z = for_vel, -rot_vel
    return t_vel

if __name__ == '__main__':
    rospy.init_node('auto_youbot_driver')
    rospy.loginfo("auto_youbot_driver has been started")
    device = torch.device('cuda')
    model = DetectMultiBackend(
        '/home/maxim/Desktop/Litter_detector/TACO/yolo_taco_single_class/train_sessions/exp/weights/best.pt',
        device=device, dnn=False,
        data='/home/maxim/Desktop/Litter_detector/TACO/yolo_taco_single_class/data.yaml')
    youbot_cam = camera(radians(29), radians(23))
    velocoties_topic_name = '/velocities'
    init_pos = position(0, 0, 1.57)
    youbot = Robot(init_pos, (0, 0, 0), 0.05, 0.32, 0.456, youbot_cam, model, device,velocoties_topic_name)

    rate = rospy.Rate(50)

    vis_sub = rospy.Subscriber('/image', Image, callback=youbot.vis_sens_cb)
    wheel_odom_sub = rospy.Subscriber('/wheel_odom',Float32MultiArray,callback=youbot.wheel_odom_cb)

    map_trig_sub = rospy.Subscriber('/MapTrigger', Bool, callback=youbot.show_map_cb)
    init_vel = Twist()
    init_vel.linear.x = 0
    init_vel.angular.z = 0
    youbot.pub_vel.publish(init_vel)  # executed to receive first odometry measurements
    time.sleep(1)
    print(f'Velocity to set:lin {init_vel.linear.x} rot {init_vel.angular.z}')
    youbot.pub_vel.publish(init_vel)
    while not (youbot.vis_sens_cb_exec_flag and youbot.odom_data_arr_flag):
        pass
    youbot.upd_odom()
    youbot.motor_vel_set(init_vel)  # executed to execute camera.rectify_image() to set AOV configs insted None
    points2drive = [City(0, youbot.robot_camera.w_r + youbot.robot_length/2), City(-2*youbot.robot_camera.w_r - youbot.robot_length/2, youbot.robot_camera.w_r + youbot.robot_length/2),
                    City(-2*youbot.robot_camera.w_r - youbot.robot_length/2, 3*youbot.robot_camera.w_r + youbot.robot_length/2),
                    City(0, 3*youbot.robot_camera.w_r + youbot.robot_length/2)]
    youbot.go_around_points(points2drive,0.01,check_obstacles = False)
    youbot.pub_vel.publish(init_vel)
    youbot.upd_odom()
    map = youbot.get_map()
    cv2.imshow('Location map', map)
    cv2.waitKey(0)
    youbot.pub_vel.publish(init_vel)
    youbot.upd_odom()
    _, objs_order2drive = travellingSalesmanProblem([City(youbot.state_k.x,youbot.state_k.y)]+[City(obj[0],obj[1]) for obj in youbot.lm_glob_coord])
    youbot.go_around_points(objs_order2drive,0.1,check_obstacles = False)
    youbot.pub_vel.publish(init_vel)
    youbot.upd_odom()
    map = youbot.get_map()
    cv2.imshow('Location map', map)
    cv2.waitKey(0)


