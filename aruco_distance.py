#!/usr/bin/env python3

import cv2 as cv
from cv2 import aruco
import numpy as np

calib_data_path = "C:\David\IIT Tirupati\Technical Competitions_Workshops\InterIIT\Jan-Feb-2023\Drona Aviation\drona_aviation\calib_data\MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
print(cam_mat)
# cam_mat*=1.5
# cam_mat[0][0]*=1.5
# cam_mat[1][1]*=1.5
# cam_mat[0][2]*=1.5
# cam_mat[1][2]*=1.5
print(cam_mat)
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

# cam_mat = np.identity(3, float)
# print(cam_mat)
# dist_coef = np.zeros((1,5))
# print(dist_coef)
MARKER_SIZE = 5 # centimeters

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

param_markers = aruco.DetectorParameters()

detector = aruco.ArucoDetector(marker_dict, param_markers)

cap = cv.VideoCapture(1,cv.CAP_DSHOW)

fps = 60
cap.set(cv.CAP_PROP_FPS, fps)

# Set the resolution
width = 1920
height = 1080
cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    h,  w = frame.shape[:2]
    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cam_mat, dist_coef, (w,h), 1, (w,h))
    frame = cv.undistort(frame, cam_mat, dist_coef, None, newCameraMatrix)
    # x,y,w,h=roi
    # frame=frame[y:y+h,x:x+w]
    
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = detector.detectMarkers(
        gray_frame
    )
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            # tVec[i][0][0] += 45
            # tVec[i][0][1] += 30
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
            # so I have rectified that mistake, I have test that out it increase the accuracy overall.
            # Calculating the distance
            distance = np.sqrt(
                tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            )
            # Draw the pose of the marker
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            # cv.putText(
            #     frame,
            #     f"id: {ids[0]} Z: {round(tVec[i][0][2], 2)}",
            #     top_left,
            #     cv.FONT_HERSHEY_PLAIN,
            #     1.3,
            #     (0, 0, 255),
            #     2,
            #     cv.LINE_AA,
            # )
            print("Z:",round(tVec[i][0][2]*(5/26), 2),end=" ")
            print("x:",round(tVec[i][0][0],1),end=" ")
            print("y:",round(tVec[i][0][1],1))
            # cv.putText(
            #     frame,
            #     f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
            #     bottom_left,
            #     cv.FONT_HERSHEY_PLAIN,
            #     1.0,
            #     (0, 0, 255),
            #     2,
            #     cv.LINE_AA,
            # )
            # print(ids, "  ", corners)

    cv.imshow("frame", frame)

    # h,  w = frame.shape[:2]
    # newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cam_mat, dist_coef, (w,h), 1, (w,h))
    # frame = cv.undistort(frame, cam_mat, dist_coef, None, newCameraMatrix)
    # x,y,w,h=roi
    # frame=frame[y:y+h,x:x+w]
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()