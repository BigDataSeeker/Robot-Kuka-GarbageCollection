import socket
import threading
import cv2
import time
import torch
import numpy as np
from math import atan2, cos, sin
import sys
sys.path.append(r'C:\Users\CPD8\Desktop\Beliakov\pycharm_project\yolov5')
from models.common import DetectMultiBackend
from utils.augmentations import letterbox
from utils.general import non_max_suppression,scale_coords

connectionIsOpen = False

def send_data (data):
    conn.send(data)
    print('Data sent: ' + data.decode())

def odom_data():
    data = ''
    global last_total_odom
    while connectionIsOpen:
        rcvd_data = conn.recv(1)
        if rcvd_data.decode() == '\n':
            if data[:6] == '.odom#':
                data_list = data.split(';')
                current_total_odom = [float(data_list[0][6:]),-float(data_list[1]),-float(data_list[2][:-2])]
                if last_total_odom is None:
                    print('Odom is None')
                    last_total_odom = current_total_odom
                else:
                    odom_values = [current_total_odom[0] - last_total_odom[0],current_total_odom[1] - last_total_odom[1],atan2(sin(current_total_odom[2] - last_total_odom[2]),cos(current_total_odom[2] - last_total_odom[2]))]
                    print(f'Processed odom coords - {odom_values}')
                    last_total_odom = current_total_odom
                    print(f'Odometry coords is {current_total_odom}')
            data = ''
        else:
            data += rcvd_data.decode()

def image_thread():
    while connectionIsOpen:
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
                                   max_det=2)

        until = time.perf_counter()
        print(f'Video frame processed for {until - since}')
        for idx in range(len(pred)):
            pred[idx][:, :4] = scale_coords(img2feed.shape[2:], pred[idx][:, :4], img.shape).round()
            # iterate over each detection in the image
            for *xyxy, conf, cls in reversed(pred[idx]):
                xyxy = [int(c.tolist()) for c in xyxy]
                t_l, b_r = (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3])
                t_r = (b_r[0], t_l[1])
                b_l = (t_l[0], b_r[1])
                points = np.array([t_l, t_r, b_r, b_l])
                # draw detection bbox
                cv2.polylines(img, pts=[points], isClosed=True, color=(0, 200, 100), thickness=4)
        print(f'Detected {len(pred)} objs with coords:{pred}')
        cv2.imshow("Capturing", img)
        cv2.waitKey(1)



if __name__ == "__main__":
    device = torch.device('cpu')
    cap = cv2.VideoCapture(
        'http://192.168.88.21:8080/stream?topic=/camera/rgb/image_raw&width=1280&height=1024&quality=50')
    model = DetectMultiBackend(r'C:\Users\CPD8\Desktop\Beliakov\youbot_python_scripts\yolo_data\best.pt', device=device,
                               dnn=False,
                               data=r'C:\Users\CPD8\Desktop\Beliakov\youbot_python_scripts\yolo_data\data.yaml')
    conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    conn.connect(('192.168.88.21', 7777))
    time.sleep(1)
    connectionIsOpen = True
    last_total_odom = None
    odom_thread = threading.Thread(target=odom_data)
    odom_thread.start()
    image_thread = threading.Thread(target=image_thread)
    image_thread.start()

    manip_str = b'LUA_ManipDeg(0,168,67,-92,177,170)^^^'
    send_data(manip_str)
    time.sleep(5)
    start_time = time.time()
    while time.time() - start_time < 20:
        #lin_vel_str = 'LUA_Base(0.1,0,0)^^^'
        #send_data(str.encode(lin_vel_str))
        #time.sleep(2)
        rot_vel_str = 'LUA_Base(0,0,0.314)^^^'
        send_data(str.encode(rot_vel_str))
        time.sleep(2)

    connectionIsOpen = False
    send_data(b'#end#^^^')
    conn.shutdown(socket.SHUT_RDWR)
    conn.close()


