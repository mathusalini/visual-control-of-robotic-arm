import cv2
import numpy as np
import pickle

# ----------------------------
# Load calibration
# ----------------------------
with open("stereo_calibration.pkl", "rb") as f:
    calib = pickle.load(f)

mtxL, distL = calib["mtxL"], calib["distL"]
mtxR, distR = calib["mtxR"], calib["distR"]
R1, R2 = calib["R1"], calib["R2"]
P1, P2 = calib["P1"], calib["P2"]
Q = calib["Q"]

# ----------------------------
# Load ONE test stereo pair
# ----------------------------
left_path  = "/home/group36/stereo_pairs/left_0.png"
right_path = "/home/group36/stereo_pairs/right_0.png"

imgL = cv2.imread(left_path)
imgR = cv2.imread(right_path)

grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

h, w = grayL.shape[:2]
image_size = (w, h)

# ----------------------------
# Rectification maps
# ----------------------------
map1L, map2L = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, image_size, cv2.CV_16SC2)
map1R, map2R = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, image_size, cv2.CV_16SC2)

rectL = cv2.remap(grayL, map1L, map2L, cv2.INTER_LINEAR)
rectR = cv2.remap(grayR, map1R, map2R, cv2.INTER_LINEAR)

# ----------------------------
# Disparity using StereoSGBM
# ----------------------------
num_disp = 16 * 8   # must be multiple of 16
block_size = 7

stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=num_disp,
    blockSize=block_size,
    P1=8 * 1 * block_size**2,
    P2=32 * 1 * block_size**2,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=2,
    disp12MaxDiff=1
)

disp = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

# Normalize for display
disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
disp_vis = disp_vis.astype(np.uint8)

# ----------------------------
# Depth (3D reprojection)
# ----------------------------
points_3d = cv2.reprojectImageTo3D(disp, Q)  # (x,y,z) for each pixel
Z = points_3d[:, :, 2]  # depth in same units as your calibration (usually meters if baseline is meters)

# Show results
cv2.imshow("Rectified Left", rectL)
cv2.imshow("Rectified Right", rectR)
cv2.imshow("Disparity", disp_vis)
cv2.waitKey(0)
cv2.destroyAllWindows()
