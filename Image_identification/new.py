import cv2
import numpy as np
import pickle
import time

# ==========================
# SETTINGS
# ==========================
CALIB_FILE = "stereo_calibration.pkl"
LEFT_CAM_ID = 0
RIGHT_CAM_ID = 1

# Force same resolution as you calibrated with (IMPORTANT!)
FRAME_W = 640
FRAME_H = 480

# ---- HSV ranges ----
# Red wraps -> 2 ranges
RED_L1 = np.array([0, 120, 70])
RED_U1 = np.array([10, 255, 255])
RED_L2 = np.array([170, 120, 70])
RED_U2 = np.array([180, 255, 255])

# Yellow (tune if needed)
YEL_L = np.array([20, 120, 120])
YEL_U = np.array([40, 255, 255])

MIN_AREA = 500

# ---- Stereo ----
NUM_DISP = 16 * 8      # multiple of 16
BLOCK_SIZE = 7         # 5..11 typical
MIN_DISP = 0

# Disparity patch size for median (stability)
PATCH_R = 3  # radius -> patch = (2*PATCH_R+1)^2

# EMA smoothing factor (0..1). Lower = smoother
EMA_ALPHA = 0.20

# stereoRectify alpha:
# 0 = cropped valid only, 1 = full view with borders
RECTIFY_ALPHA = 1.0

# ==========================
# HELPERS
# ==========================
def make_mask(hsv):
    red1 = cv2.inRange(hsv, RED_L1, RED_U1)
    red2 = cv2.inRange(hsv, RED_L2, RED_U2)
    mask_red = cv2.bitwise_or(red1, red2)

    mask_yel = cv2.inRange(hsv, YEL_L, YEL_U)
    return mask_red, mask_yel

def clean_mask(mask):
    mask = cv2.medianBlur(mask, 5)
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    return mask

def largest_contour(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    return c

def point_in_roi(x, y, roi):
    # roi = (x, y, w, h)
    rx, ry, rw, rh = roi
    return (x >= rx) and (y >= ry) and (x < rx + rw) and (y < ry + rh)

def get_disp_median(disp, cx, cy, r=PATCH_R):
    h, w = disp.shape[:2]
    x1 = max(0, cx - r); x2 = min(w, cx + r + 1)
    y1 = max(0, cy - r); y2 = min(h, cy + r + 1)
    patch = disp[y1:y2, x1:x2]
    patch = patch[np.isfinite(patch)]
    patch = patch[patch > 0]  # valid only
    if patch.size == 0:
        return None
    return float(np.median(patch))

def reproject_one(Q, x, y, d):
    # [X Y Z W]^T = Q * [x y d 1]^T
    v = np.array([x, y, d, 1.0], dtype=np.float64)
    X, Y, Z, W = Q.dot(v)
    if W == 0:
        return None
    return (X / W, Y / W, Z / W)

def ema(prev, curr, a=EMA_ALPHA):
    if prev is None:
        return curr
    return (prev[0] * (1 - a) + curr[0] * a,
            prev[1] * (1 - a) + curr[1] * a,
            prev[2] * (1 - a) + curr[2] * a)

# ==========================
# LOAD CALIB
# ==========================
with open(CALIB_FILE, "rb") as f:
    calib = pickle.load(f)

mtxL, distL = calib["mtxL"], calib["distL"]
mtxR, distR = calib["mtxR"], calib["distR"]

# Some pkl files already store R1,R2,P1,P2,Q.
# If not, they store R,T and we compute rectification now.
R = calib.get("R", None)
T = calib.get("T", None)

# ==========================
# OPEN CAMS
# ==========================
capL = cv2.VideoCapture(LEFT_CAM_ID)
capR = cv2.VideoCapture(RIGHT_CAM_ID)

if not capL.isOpened() or not capR.isOpened():
    raise RuntimeError("Could not open cameras. Check IDs.")

capL.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
capL.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
capR.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
capR.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

retL, frameL = capL.read()
retR, frameR = capR.read()
if not retL or not retR:
    raise RuntimeError("Failed to read from cameras.")

h, w = frameL.shape[:2]
image_size = (w, h)

# ==========================
# RECTIFICATION SETUP
# ==========================
if all(k in calib for k in ["R1", "R2", "P1", "P2", "Q"]) and "roi1" in calib and "roi2" in calib:
    R1, R2 = calib["R1"], calib["R2"]
    P1, P2 = calib["P1"], calib["P2"]
    Q = calib["Q"]
    roi1, roi2 = calib["roi1"], calib["roi2"]
else:
    if R is None or T is None:
        raise RuntimeError("Calibration file must contain either (R1,R2,P1,P2,Q,roi1,roi2) OR (R,T).")

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        mtxL, distL, mtxR, distR,
        image_size, R, T,
        alpha=RECTIFY_ALPHA
    )

# Precompute maps
map1L, map2L = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, image_size, cv2.CV_16SC2)
map1R, map2R = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, image_size, cv2.CV_16SC2)

# ==========================
# STEREO MATCHER
# ==========================
stereo = cv2.StereoSGBM_create(
    minDisparity=MIN_DISP,
    numDisparities=NUM_DISP,
    blockSize=BLOCK_SIZE,
    P1=8 * BLOCK_SIZE * BLOCK_SIZE,
    P2=32 * BLOCK_SIZE * BLOCK_SIZE,
    uniquenessRatio=12,
    speckleWindowSize=80,
    speckleRange=2,
    disp12MaxDiff=1,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

# ==========================
# MAIN LOOP
# ==========================
smooth_red = None
smooth_yel = None

print("Press 'q' to quit.")
while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()
    if not retL or not retR:
        break

    # Rectify (this view may show borders if alpha=1)
    rectL = cv2.remap(frameL, map1L, map2L, cv2.INTER_LINEAR)
    rectR = cv2.remap(frameR, map1R, map2R, cv2.INTER_LINEAR)

    # Disparity
    grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)
    disp = stereo.compute(grayL, grayR).astype(np.float32) / 16.0

    # Color detect on rectL
    hsv = cv2.cvtColor(rectL, cv2.COLOR_BGR2HSV)
    mask_red, mask_yel = make_mask(hsv)
    mask_red = clean_mask(mask_red)
    mask_yel = clean_mask(mask_yel)

    out = rectL.copy()

    # Draw valid ROI (important when alpha=1)
    rx, ry, rw, rh = roi1
    cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), (255, 255, 255), 1)
    cv2.putText(out, "Valid ROI", (rx, max(0, ry - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    results = []

    # ---- RED object ----
    c = largest_contour(mask_red)
    if c is not None:
        x, y, ww, hh = cv2.boundingRect(c)
        cx, cy = x + ww // 2, y + hh // 2

        if point_in_roi(cx, cy, roi1):
            d = get_disp_median(disp, cx, cy)
            if d is not None:
                xyz = reproject_one(Q, cx, cy, d)
                if xyz is not None and np.isfinite(xyz[2]) and 0.05 < xyz[2] < 10:
                    smooth_red = ema(smooth_red, xyz)
                    results.append(("RED", (x, y, ww, hh), (cx, cy), smooth_red))

    # ---- YELLOW object ----
    c = largest_contour(mask_yel)
    if c is not None:
        x, y, ww, hh = cv2.boundingRect(c)
        cx, cy = x + ww // 2, y + hh // 2

        if point_in_roi(cx, cy, roi1):
            d = get_disp_median(disp, cx, cy)
            if d is not None:
                xyz = reproject_one(Q, cx, cy, d)
                if xyz is not None and np.isfinite(xyz[2]) and 0.05 < xyz[2] < 10:
                    smooth_yel = ema(smooth_yel, xyz)
                    results.append(("YELLOW", (x, y, ww, hh), (cx, cy), smooth_yel))

    # Draw results
    for name, (x, y, ww, hh), (cx, cy), (X, Y, Z) in results:
        cv2.rectangle(out, (x, y), (x + ww, y + hh), (0, 255, 0), 2)
        cv2.circle(out, (cx, cy), 4, (255, 0, 0), -1)
        cv2.putText(out, f"{name}: X={X:.2f} Y={Y:.2f} Z={Z:.2f} m",
                    (x, max(0, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    # Show
    cv2.imshow("Rectified Left (alpha=1 ok) + 3D", out)
    cv2.imshow("Mask RED", mask_red)
    cv2.imshow("Mask YELLOW", mask_yel)

    disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    cv2.imshow("Disparity", disp_vis)

    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        break

capL.release()
capR.release()
cv2.destroyAllWindows()