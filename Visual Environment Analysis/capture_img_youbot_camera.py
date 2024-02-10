import cv2
import time
import torch
import numpy as np
from math import radians, tan,atan2, cos, sin, sqrt
import math
import sys
import matplotlib
from matplotlib import pyplot as plt
matplotlib.use('Qt5Agg')
sys.path.append(r'C:\Users\CPD8\Desktop\Beliakov\pycharm_project\yolov5')
from models.common import DetectMultiBackend
from utils.augmentations import letterbox
from utils.general import non_max_suppression,scale_coords


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

                #img = cv2.circle(img, (x_c, y_c), radius=8, color=(100, 100, 100), thickness=-1) visualize where
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


# Optimal manipulator degrees: 168, 67, -92, 177, 170
device = torch.device('cpu')
cap = cv2.VideoCapture('http://192.168.88.22:8080/stream?topic=/camera/rgb/image_raw&width=640&height=480&quality=50')
model = DetectMultiBackend(r'C:\Users\CPD8\Desktop\Beliakov\youbot_python_scripts\yolo_data\best.pt', device=device, dnn=False,
                               data=r'C:\Users\CPD8\Desktop\Beliakov\youbot_python_scripts\yolo_data\data.yaml')
vis_sens = camera(radians(29), radians(23))

objs_cls = ['plastic','metal']

while True:
    ret, img = cap.read()
    since = time.perf_counter()
    img2feed = letterbox(img.copy(), (640, 640), stride=32, auto=True)[0]
    img2feed = img2feed.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    img2feed = np.ascontiguousarray(img2feed)
    img2feed = torch.from_numpy(img2feed).to(device)
    img2feed = img2feed.float()
    img2feed /= 255
    img2feed = img2feed[None]

    pred = model(img2feed, augment=False, visualize=False)
    pred = non_max_suppression(pred, conf_thres=0.5, iou_thres=0.5, classes=None, agnostic=False,
                               max_det=100)
    until = time.perf_counter()
    print(f'Video frame processed for {until-since}')
    for idx in range(len(pred)):
        cls_idx = 0
        pred[idx][:, :4] = scale_coords(img2feed.shape[2:], pred[idx][:, :4], img.shape).round()
        # iterate over each detection in the image
        img_rectified, obj_coords = vis_sens.rectify_perspective(img, 0.47, radians(45),
                                                                          reversed(pred[idx]))

        for *xyxy, conf, cls in reversed(pred[idx]):
            xyxy = [int(c.tolist()) for c in xyxy]
            t_l, b_r = (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3])
            t_r = (b_r[0], t_l[1])
            b_l = (t_l[0], b_r[1])
            points = np.array([t_l, t_r, b_r, b_l])
            # draw detection bbox
            cv2.polylines(img, pts=[points], isClosed=True, color=(0, 200, 100), thickness=4)
            #img = cv2.putText(img, objs_cls[cls_idx],t_l , cv2.FONT_HERSHEY_SIMPLEX, 1,(0,150,150), 2, cv2.LINE_AA, False)
            cls_idx += 1
    print(f'Detected {len(pred)} objs with coords:{pred}')
    cv2.imshow("Capturing",img_rectified)
    cv2.waitKey(1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()