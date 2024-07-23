# from task_1b import detect_ArUco_details, mark_ArUco_image
import cv2 #as cv
import numpy as np
import  glob

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

# while(True):
      
#     count = count + 1;
#     # Capture the video frame by frame
#     ret, frame = cap.read()
#     k = frame
#     cv2.imwrite( 'k_720' + str(count) + '.jpg', k)
#     cv2.imshow('img',k)
#     cv2.waitKey(5000)
#     if count == 50: 
#         break;
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cv2.destroyAllWindows()
###################################################################

chessboardSize = (9,6)
frameSize = (1280, 720)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 23
objp = objp * size_of_chessboard_squares_mm


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.jpg')
count = 0;

for i in images:
    #img = cv2.imread('k_720' + str(i) + '.jpg');
    img = cv2.imread(i)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    count = count+1;
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        print(count);
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv2.imshow('img', img)
        cv2.imwrite("mod_k_720" + str(count) + ".jpg", img)
        cv2.waitKey(1000)


cv2.destroyAllWindows()

############## CALIBRATION #######################################################

retina, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

print("duming the data into one files using numpy ")
np.savez(
    f"MultiMatrix_1",
    camMatrix=cameraMatrix,
    distCoef=dist,
    rVector=rvecs,
    tVector=tvecs,
)

print("-------------------------------------------")

print("loading data stored using numpy savez function\n \n \n")

data = np.load(f"MultiMatrix_1.npz")

camMatrix = data["camMatrix"]
distCof = data["distCoef"]
rVector = data["rVector"]
tVector = data["tVector"]

print("loaded calibration data successfully")
############## UNDISTORTION #####################################################
print(cameraMatrix)

# cameraMatrix = np.array([[1.0276827077775663e+03, 0, 7.0162275090736443e+02], [0, 9.4181971578725791e+02,  3.4476144885393273e+02], [0, 0, 1]])

img = cv2.imread('k_72036.jpg')
h,  w = img.shape[:2]
newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

print(newCameraMatrix)

# Undistort
dst = cv2.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

# crop the image
x, y, w, h = roi

cv2.imwrite('caliResult1_uncut.png', dst)
cv2.imshow('caliResult_uncut.png', dst)
dst = dst[y:y+h, x:x+w]
cv2.imwrite('caliResult1.png',dst)
cv2.imshow('caliResult1.png',dst)
cv2.waitKey(15000)
# After the loop release the cap object

cap.release()
# Destroy all the windows
cv2.destroyAllWindows()