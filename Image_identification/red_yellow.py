import cv2
import numpy as np
import pickle

# ----------------------------
# Settings (EDIT THESE)
# ----------------------------
CALIB_FILE = "stereo_calibration.pkl"
LEFT_CAM_ID = 0
RIGHT_CAM_ID = 2

# Choose your target color in HSV.
# Example below: RED (note: red wraps around HSV -> two ranges)
LOWER1 = np.array([0, 120, 70])
UPPER1 = np.array([10, 255, 255])
LOWER2 = np.array([170, 120, 70])
UPPER2 = np.array([180, 255, 255])

# Add yellow color range for detection
LOWER_YELLOW = np.array([20, 100, 100])  # Lower bound for yellow
UPPER_YELLOW = np.array([40, 255, 255])  # Upper bound for yellow

MIN_AREA = 800  # ignore tiny blobs

# Stereo matcher parameters (you can tune)
NUM_DISP = 16 * 8   # must be multiple of 16
BLOCK_SIZE = 7

# ----------------------------
# Load calibration
# ----------------------------
with open(CALIB_FILE, "rb") as f:
    calib = pickle.load(f)

mtxL, distL = calib["mtxL"], calib["distL"]
mtxR, distR = calib["mtxR"], calib["distR"]
R1, R2 = calib["R1"], calib["R2"]
P1, P2 = calib["P1"], calib["P2"]
Q = calib["Q"]

# ----------------------------
# Open cameras
# ----------------------------
capL = cv2.VideoCapture(LEFT_CAM_ID)
capR = cv2.VideoCapture(RIGHT_CAM_ID)

if not capL.isOpened() or not capR.isOpened():
    raise RuntimeError("Could not open one or both cameras. Check camera IDs.")

# Grab one frame to get image size
retL, frameL = capL.read()
retR, frameR = capR.read()
if not retL or not retR:
    raise RuntimeError("Failed to read from cameras.")

h, w = frameL.shape[:2]
image_size = (w, h)

# ----------------------------
# Precompute rectification maps (fast)
# ----------------------------
map1L, map2L = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, image_size, cv2.CV_16SC2)
map1R, map2R = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, image_size, cv2.CV_16SC2)

# ----------------------------
# Create stereo matcher
# ----------------------------
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=NUM_DISP,
    blockSize=BLOCK_SIZE,
    P1=8 * 1 * BLOCK_SIZE * BLOCK_SIZE,
    P2=32 * 1 * BLOCK_SIZE * BLOCK_SIZE,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=2,
    disp12MaxDiff=1,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

print("Press 'q' to quit.")

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()
    if not retL or not retR:
        print("Camera read failed")
        break

    # Rectify
    rectL = cv2.remap(frameL, map1L, map2L, cv2.INTER_LINEAR)
    rectR = cv2.remap(frameR, map1R, map2R, cv2.INTER_LINEAR)

    # Convert to grayscale for disparity
    grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)

    # Compute disparity (float, in pixels)
    disp = stereo.compute(grayL, grayR).astype(np.float32) / 16.0

    # --- Color detection on LEFT rectified image ---
    hsv = cv2.cvtColor(rectL, cv2.COLOR_BGR2HSV)

    # Red color detection
    mask1 = cv2.inRange(hsv, LOWER1, UPPER1)
    mask2 = cv2.inRange(hsv, LOWER2, UPPER2)
    mask_red = cv2.bitwise_or(mask1, mask2)

    # Yellow color detection
    mask_yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)

    # Combine both red and yellow masks
    mask = cv2.bitwise_or(mask_red, mask_yellow)

    # Clean mask
    mask = cv2.medianBlur(mask, 5)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    output = rectL.copy()
    text = "No object"

    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > MIN_AREA:
            x, y, ww, hh = cv2.boundingRect(c)
            cx = x + ww // 2
            cy = y + hh // 2

            # Get disparity around center (use a small patch median for stability)
            patch = disp[max(0, cy-2):cy+3, max(0, cx-2):cx+3]
            patch = patch[patch > 0]  # remove invalid
            if patch.size > 0:
                d = np.median(patch)

                # Reproject that pixel to 3D:
                # Method 1: reproject whole image then pick point (simple)
                points_3d = cv2.reprojectImageTo3D(disp, Q)
                X, Y, Z = points_3d[cy, cx]  # in same unit as square_size (meters if you used meters)

                # Basic sanity filter: Z should be positive and not crazy
                if np.isfinite(Z) and Z > 0 and Z < 10:
                    text = f"3D (m): X={X:.3f}, Y={Y:.3f}, Z={Z:.3f} | d={d:.2f}px"
                else:
                    text = "3D invalid (check disparity/Q)"
            else:
                text = "Disparity invalid at object (try better texture/light)"

            # Draw
            cv2.rectangle(output, (x, y), (x+ww, y+hh), (0, 255, 0), 2)
            cv2.circle(output, (cx, cy), 5, (255, 0, 0), -1)

    cv2.putText(output, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # Show results
    cv2.imshow("Rectified Left + 3D", output)
    cv2.imshow("Mask", mask)

    # Optional: view disparity for debugging
    disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
    disp_vis = disp_vis.astype(np.uint8)
    cv2.imshow("Disparity", disp_vis)

    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        break

capL.release()
capR.release()
cv2.destroyAllWindows()