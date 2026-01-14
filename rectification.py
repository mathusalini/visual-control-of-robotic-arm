import cv2
import numpy as np
import pickle
import glob
import os

# ----------------------------
# Paths (EDIT if needed)
# ----------------------------
PAIR_DIR = "/home/group36/Group36/visual-control-of-robotic-arm/stereo_pairs"
CALIB_FILE = "stereo_calibration.pkl"
# If your pkl is in the same folder where you run the script, you can use:
# CALIB_FILE = "stereo_calibration.pkl"

# ----------------------------
# Load calibration
# ----------------------------
if not os.path.exists(CALIB_FILE):
    raise FileNotFoundError(f"Calibration file not found: {CALIB_FILE}")

with open(CALIB_FILE, "rb") as f:
    calib = pickle.load(f)

mtxL, distL = calib["mtxL"], calib["distL"]
mtxR, distR = calib["mtxR"], calib["distR"]
R1, R2 = calib["R1"], calib["R2"]
P1, P2 = calib["P1"], calib["P2"]
Q = calib["Q"]

# ----------------------------
# Load a stereo pair (auto-pick first)
# ----------------------------
left_images = sorted(glob.glob(os.path.join(PAIR_DIR, "left_*.png")))
right_images = sorted(glob.glob(os.path.join(PAIR_DIR, "right_*.png")))

if len(left_images) == 0 or len(right_images) == 0:
    raise RuntimeError(f"No images found in {PAIR_DIR}. Expected left_*.png and right_*.png")

if len(left_images) != len(right_images):
    raise RuntimeError(f"Image count mismatch: left={len(left_images)} right={len(right_images)}")

left_path = left_images[0]   # left_000.png
right_path = right_images[0] # right_000.png

imgL = cv2.imread(left_path)
imgR = cv2.imread(right_path)

if imgL is None or imgR is None:
    raise RuntimeError(f"Failed to load images:\n{left_path}\n{right_path}")

grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

h, w = grayL.shape[:2]
image_size = (w, h)

print("Using pair:")
print("  Left :", left_path)
print("  Right:", right_path)
print("Image size:", image_size)

# ----------------------------
# Rectification maps
# ----------------------------
map1L, map2L = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, image_size, cv2.CV_16SC2)
map1R, map2R = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, image_size, cv2.CV_16SC2)

rectL = cv2.remap(grayL, map1L, map2L, cv2.INTER_LINEAR)
rectR = cv2.remap(grayR, map1R, map2R, cv2.INTER_LINEAR)

# ----------------------------
# Rectification visual check (horizontal lines)
# ----------------------------
check = np.hstack((rectL, rectR))
check_color = cv2.cvtColor(check, cv2.COLOR_GRAY2BGR)

# draw horizontal guide lines every 40 px
for y in range(0, check_color.shape[0], 40):
    cv2.line(check_color, (0, y), (check_color.shape[1], y), (0, 255, 0), 1)

# ----------------------------
# Disparity using StereoSGBM
# ----------------------------
num_disp = 16 * 8      # must be multiple of 16 (try 16*6, 16*8, 16*10)
block_size = 7         # try 5,7,9 if needed

stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=num_disp,
    blockSize=block_size,
    P1=8 * 1 * block_size**2,
    P2=32 * 1 * block_size**2,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=2,
    disp12MaxDiff=1,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

disp = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

# Mask invalid disparities
disp_valid = disp
