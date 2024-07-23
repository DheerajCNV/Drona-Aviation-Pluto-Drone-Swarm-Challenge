import numpy as np
import cv2
from cv2 import aruco
import math
from pyzbar import pyzbar

markersize = 4; totalMarkers = 250; draw = True

# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# key = getattr(aruco, f'DICT_{markersize}X{markersize}_{totalMarkers}')
# arucoDict = aruco.Dictionary_get(key)
# arucoParam = aruco.DetectorParameters_create()
# bboxs, ids, rejected = aruco.detectMarkers(gray,arucoDict, parameters = arucoParam)

# # print(ids)
# if draw:
#     aruco.drawDetectedMarkers(img,bboxs)
    
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
def mk_1080p():
    fps = 60
    cap.set(cv2.CAP_PROP_FPS, fps)

    # Set the resolution
    width = 1920
    height = 1080
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

count = 0;
def mk_720p():
    fps = 60;
    cap.set(cv2.CAP_PROP_FPS, fps)

    # Set the resolution
    width = 1280
    height = 720
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

count = 0;
mk_720p();

while(True):
      
    # Capture the video frame by frame
    ret, frame = cap.read()
    img = frame
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markersize}X{markersize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray,arucoDict, parameters = arucoParam)

    # print(ids)
    if draw:
        aruco.drawDetectedMarkers(img,bboxs)
    cv2.imshow("detedted", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()