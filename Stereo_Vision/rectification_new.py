import cv2
import numpy as np
import pickle
import glob
import os
import sys

# -------------------------------------------------
# Use the folder where THIS script is located
# -------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

PAIR_DIR = os.path.join(SCRIPT_DIR, "Stereo_pairs_1")
CALIB_FILE = os.path.join(SCRIPT_DIR, "stereo_calibration.pkl")

print("SCRIPT_DIR :", SCRIPT_DIR)
print("PAIR_DIR   :", PAIR_DIR)
print("CALIB_FILE :", CALIB_FILE)

# -------------------------------------------------
# Basic checks
# -------------------------------------------------
if not os.path.exists(CALIB_FILE):
    raise FileNotFoundError(f"Calibration file not found: {CALIB_FILE}")

if not os.path.isdir(PAIR_DIR):
    raise FileNotFoundError(f"Stereo pair folder not found: {PAIR_DIR}")

# -------------------------------------------------
# Load calibration
# -------------------------------------------------
with open(CALIB_FILE, "rb") as f:
    calib = pickle.load(f)

mtxL, distL = calib["mtxL"], calib["distL"]
mtxR, distR = calib["mtxR"], calib["distR"]
R1, R2 = calib["R1"], calib["R2"]
P1, P2 = calib["P1"], calib["P2"]
Q = calib["Q"]

# -------------------------------------------------
# Load stereo pair (pick first pair automatically)
# -------------------------------------------------
left_images = sorted(glob.glob(os.path.join(PAIR_DIR, "left_*.*")))
right_images = sorted(glob.glob(os.path.join(PAIR_DIR, "right_*.*")))

if len(left_images) == 0 or len(right_images) == 0:
    raise RuntimeError(
        f"No images found in {PAIR_DIR}\n"
        f"Found left={len(left_images)}, right={len(right_images)}\n"
        f"Expected names like left_000.png and right_000.png"
    )

if len(left_images) != len(right_images):
    raise RuntimeError(f"Image count mismatch: left={len(left_images)} right={len(right_images)}")

left_path = left_images[0]
right_path = right_images[0]

imgL = cv2.imread(left_path)
imgR = cv2.imread(right_path)

if imgL is None or imgR is None:
    raise RuntimeError(f"Failed to read:\n{left_path}\n{right_path}")

print("\nUsing pair:")
print("  Left :", left_path)
print("  Right:", right_path)

grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

h, w = grayL.shape[:2]
image_size = (w, h)
print("Image size:", image_size)

# -------------------------------------------------
# Rectification maps + remap
# -------------------------------------------------
map1L, map2L = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, image_size, cv2.CV_16SC2)
map1R, map2R = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, image_size, cv2.CV_16SC2)

rectL = cv2.remap(grayL, map1L, map2L, cv2.INTER_LINEAR)
rectR = cv2.remap(grayR, map1R, map2R, cv2.INTER_LINEAR)

# -------------------------------------------------
# Rectification check image (horizontal lines)
# -------------------------------------------------
check = np.hstack((rectL, rectR))
check_color = cv2.cvtColor(check, cv2.COLOR_GRAY2BGR)
for y in range(0, check_color.shape[0], 40):
    cv2.line(check_color, (0, y), (check_color.shape[1], y), (0, 255, 0), 1)

# -------------------------------------------------
# Disparity (StereoSGBM)
# -------------------------------------------------
num_disp = 16 * 8      # multiple of 16
block_size = 7

stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=num_disp,
    blockSize=block_size,
    P1=8 * block_size * block_size,
    P2=32 * block_size * block_size,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=2,
    disp12MaxDiff=1,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

disp = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
disp_vis = disp_vis.astype(np.uint8)

# -------------------------------------------------
# Depth sample (optional)
# -------------------------------------------------
points_3d = cv2.reprojectImageTo3D(disp, Q)
Z = points_3d[:, :, 2]
cy, cx = h // 2, w // 2
print("\nCenter pixel sample:")
print("  disparity =", disp[cy, cx])
print("  depth(Z)  =", Z[cy, cx])

# -------------------------------------------------
# ALWAYS save outputs (so you have results even if GUI fails)
# -------------------------------------------------
out_rectL = os.path.join(SCRIPT_DIR, "out_rectL.png")
out_rectR = os.path.join(SCRIPT_DIR, "out_rectR.png")
out_check = os.path.join(SCRIPT_DIR, "out_rect_check.png")
out_disp  = os.path.join(SCRIPT_DIR, "out_disparity.png")

cv2.imwrite(out_rectL, rectL)
cv2.imwrite(out_rectR, rectR)
cv2.imwrite(out_check, check_color)
cv2.imwrite(out_disp, disp_vis)

print("\nSaved outputs:")
print(" ", out_rectL)
print(" ", out_rectR)
print(" ", out_check)
print(" ", out_disp)

# -------------------------------------------------
# Show windows (if display is available)
# -------------------------------------------------
has_display = ("DISPLAY" in os.environ) and (os.environ["DISPLAY"].strip() != "")
if not has_display:
    print("\nNo DISPLAY detected. Skipping cv2.imshow().")
    print("Open the saved output images instead.")
    sys.exit(0)

cv2.namedWindow("Rectified Left", cv2.WINDOW_NORMAL)
cv2.namedWindow("Rectified Right", cv2.WINDOW_NORMAL)
cv2.namedWindow("Rectification Check (Left | Right)", cv2.WINDOW_NORMAL)
cv2.namedWindow("Disparity (normalized)", cv2.WINDOW_NORMAL)

cv2.imshow("Rectified Left", rectL)
cv2.imshow("Rectified Right", rectR)
cv2.imshow("Rectification Check (Left | Right)", check_color)
cv2.imshow("Disparity (normalized)", disp_vis)

print("\nWindows opened. Press 'q' or ESC to quit.")
while True:
    k = cv2.waitKey(30) & 0xFF
    if k == 27 or k == ord('q'):
        break

cv2.destroyAllWindows()
