import cv2
import numpy as np
import pickle

# ==========================
# SETTINGS
# ==========================
CALIB_FILE = "stereo_calibration_2.pkl"
LEFT_CAM_ID = 0
RIGHT_CAM_ID = 2

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

# ---- World calibration (NO checkerboard) ----
# We'll record 3 points on the floor using a visible marker:
#   O (world origin), Xpoint (+X direction), Ypoint (+Y direction)
CALIB_SAMPLES = 35     # number of valid frames to collect per point
Z_AXIS_FLIP_IF_DOWN = True  # auto-flip Z axis so it points "up" (towards camera negative?) based on your setup

# ==========================
# HELPERS (color + stereo)
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

def put_lines(img, y0=25, dy=22, lines=()):
    y = y0
    for s in lines:
        cv2.putText(img, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        y += dy

# ==========================
# WORLD FRAME (no checkerboard) math
# ==========================
def normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        return None
    return v / n

def build_world_transform_from_3_points(Oc, Xc, Yc):
    """
    Inputs: Oc, Xc, Yc are 3D points (camera frame) corresponding to:
      - Oc: world origin (0,0,0)
      - Xc: a point on +X axis of world
      - Yc: a point on +Y axis of world

    Returns:
      R_WC, t_WC such that P_W = R_WC * P_C + t_WC
    """
    Oc = Oc.reshape(3)
    Xc = Xc.reshape(3)
    Yc = Yc.reshape(3)

    xhat = normalize(Xc - Oc)
    yhat_p = normalize(Yc - Oc)
    if xhat is None or yhat_p is None:
        return None

    zhat = np.cross(xhat, yhat_p)
    zhat = normalize(zhat)
    if zhat is None:
        return None

    yhat = np.cross(zhat, xhat)
    yhat = normalize(yhat)
    if yhat is None:
        return None

    # Optional: make Z axis point "up" consistently.
    # There isn't a universal "up" without extra info.
    # This simply ensures a right-handed frame; and if you notice Z is inverted,
    # you can flip it here.
    if Z_AXIS_FLIP_IF_DOWN:
        # If zhat roughly points "towards camera" vs "away", you may want to flip.
        # Heuristic: if zhat[2] < 0, flip. (Adjust if your setup differs.)
        if zhat[2] < 0:
            zhat = -zhat
            yhat = -yhat  # keep right-handed with xhat fixed

    # Rotation World->Camera has columns = world axes expressed in camera frame:
    R_CW = np.column_stack([xhat, yhat, zhat])   # 3x3
    t_CW = Oc.reshape(3, 1)                      # world origin in camera coords

    # Invert to get Camera->World:
    R_WC = R_CW.T
    t_WC = -R_CW.T @ t_CW

    return R_WC, t_WC

def cam_to_world(R_WC, t_WC, Pc):
    Pc = np.array(Pc, dtype=np.float64).reshape(3, 1)
    Pw = (R_WC @ Pc) + t_WC
    return Pw.reshape(3)

# ==========================
# LOAD CALIB
# ==========================
with open(CALIB_FILE, "rb") as f:
    calib = pickle.load(f)

mtxL, distL = calib["mtxL"], calib["distL"]
mtxR, distR = calib["mtxR"], calib["distR"]

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
# WORLD CALIBRATION STATE
# ==========================
R_WC, t_WC = None, None
Oc = Xc = Yc = None

collecting = None   # None / 'O' / 'X' / 'Y'
samples = []

def start_collect(which):
    global collecting, samples
    collecting = which
    samples = []
    print(f"[CALIB] Collecting {CALIB_SAMPLES} samples for point {which}...")

def add_sample(xyz_cam):
    global samples
    samples.append(np.array(xyz_cam, dtype=np.float64))
    return len(samples)

def finalize_collect():
    global collecting, samples
    if len(samples) < 5:
        print("[CALIB] Not enough samples.")
        collecting = None
        samples = []
        return None
    # Robust: median of samples
    arr = np.stack(samples, axis=0)  # Nx3
    med = np.median(arr, axis=0)
    which = collecting
    collecting = None
    samples = []
    print(f"[CALIB] Saved point {which} = {med}")
    return which, med

# ==========================
# MAIN LOOP
# ==========================
smooth_red = None
smooth_yel = None

print("Controls:")
print("  c  : choose which color marker to use for calibration (red/yellow shown on screen)")
print("  o  : record world origin (0,0,0) at marker position")
print("  x  : record point on +X direction (marker at that floor point)")
print("  y  : record point on +Y direction (marker at that floor point)")
print("  s  : compute world transform (after O, X, Y recorded)")
print("  r  : reset world transform")
print("  q  : quit")

calib_marker_color = "RED"  # which object we use as the marker for floor points

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()
    if not retL or not retR:
        break

    rectL = cv2.remap(frameL, map1L, map2L, cv2.INTER_LINEAR)
    rectR = cv2.remap(frameR, map1R, map2R, cv2.INTER_LINEAR)

    grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)
    disp = stereo.compute(grayL, grayR).astype(np.float32) / 16.0

    hsv = cv2.cvtColor(rectL, cv2.COLOR_BGR2HSV)
    mask_red, mask_yel = make_mask(hsv)
    mask_red = clean_mask(mask_red)
    mask_yel = clean_mask(mask_yel)

    out = rectL.copy()

    # Valid ROI box
    rx, ry, rw, rh = roi1
    cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), (255, 255, 255), 1)
    cv2.putText(out, "Valid ROI", (rx, max(0, ry - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Detect both objects (as before)
    results = []

    def detect_one(name, mask, smooth_prev):
        c = largest_contour(mask)
        if c is None:
            return None, smooth_prev
        x, y, ww, hh = cv2.boundingRect(c)
        cx, cy = x + ww // 2, y + hh // 2
        if not point_in_roi(cx, cy, roi1):
            return None, smooth_prev
        d = get_disp_median(disp, cx, cy)
        if d is None:
            return None, smooth_prev
        xyz = reproject_one(Q, cx, cy, d)
        if xyz is None:
            return None, smooth_prev
        if (not np.isfinite(xyz[2])) or not (0.05 < xyz[2] < 10.0):
            return None, smooth_prev
        smooth_now = ema(smooth_prev, xyz)
        return (name, (x, y, ww, hh), (cx, cy), smooth_now), smooth_now

    red_det, smooth_red = detect_one("RED", mask_red, smooth_red)
    if red_det is not None:
        results.append(red_det)

    yel_det, smooth_yel = detect_one("YELLOW", mask_yel, smooth_yel)
    if yel_det is not None:
        results.append(yel_det)

    # Choose marker detection for calibration (use one of the detected objects)
    marker_det = None
    for det in results:
        if det[0] == calib_marker_color:
            marker_det = det
            break

    # If collecting samples, add marker 3D samples
    if collecting is not None and marker_det is not None:
        _, _, _, (Xc_now, Yc_now, Zc_now) = marker_det
        n = add_sample((Xc_now, Yc_now, Zc_now))
        cv2.putText(out, f"Collecting {collecting}: {n}/{CALIB_SAMPLES}",
                    (10, out.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if n >= CALIB_SAMPLES:
            done = finalize_collect()
            if done is not None:
                which, med = done
                if which == 'O':
                    Oc = med.reshape(3, 1)
                elif which == 'X':
                    Xc = med.reshape(3, 1)
                elif which == 'Y':
                    Yc = med.reshape(3, 1)

    # Draw results (camera + world if available)
    for name, (x, y, ww, hh), (cx, cy), (X, Y, Z) in results:
        cv2.rectangle(out, (x, y), (x + ww, y + hh), (0, 255, 0), 2)
        cv2.circle(out, (cx, cy), 4, (255, 0, 0), -1)

        text1 = f"{name} CAM: X={X:.2f} Y={Y:.2f} Z={Z:.2f} m"
        cv2.putText(out, text1, (x, max(0, y - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        if R_WC is not None and t_WC is not None:
            wx, wy, wz = cam_to_world(R_WC, t_WC, (X, Y, Z))
            text2 = f"{name} WORLD: x={wx:.2f} y={wy:.2f} z={wz:.2f} m"
            cv2.putText(out, text2, (x, max(0, y - 32)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    # HUD
    lines = [
        f"Marker for floor calibration: {calib_marker_color} (press 'c' to toggle)",
        f"Recorded: O={'OK' if Oc is not None else 'NO'}  X={'OK' if Xc is not None else 'NO'}  Y={'OK' if Yc is not None else 'NO'}",
        f"World transform: {'READY' if (R_WC is not None) else 'NOT SET'}",
        "Keys: o=record origin, x=record +X point, y=record +Y point, s=solve, r=reset, q=quit"
    ]
    put_lines(out, y0=22, dy=22, lines=lines)

    cv2.imshow("Rectified Left + 3D (Camera + World)", out)
    cv2.imshow("Mask RED", mask_red)
    cv2.imshow("Mask YELLOW", mask_yel)
    disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    cv2.imshow("Disparity", disp_vis)

    k = cv2.waitKey(1) & 0xFF

    if k == ord('q'):
        break

    # Toggle which color marker is used for floor calibration captures
    if k == ord('c'):
        calib_marker_color = "YELLOW" if calib_marker_color == "RED" else "RED"
        print(f"[CALIB] Marker color set to: {calib_marker_color}")

    # Start collecting samples for each floor point
    if k == ord('o'):
        start_collect('O')
    if k == ord('x'):
        start_collect('X')
    if k == ord('y'):
        start_collect('Y')

    # Solve world transform after O,X,Y recorded
    if k == ord('s'):
        if Oc is None or Xc is None or Yc is None:
            print("[CALIB] Need all three points: record O, X, Y first.")
        else:
            outT = build_world_transform_from_3_points(Oc, Xc, Yc)
            if outT is None:
                print("[CALIB] Failed to compute transform (points may be collinear / noisy). Try again with bigger spacing.")
            else:
                R_WC, t_WC = outT
                print("[CALIB] World transform READY.")
                print("R_WC=\n", R_WC)
                print("t_WC=\n", t_WC.reshape(3))

    # Reset world transform and recorded points
    if k == ord('r'):
        R_WC, t_WC = None, None
        Oc = Xc = Yc = None
        collecting = None
        samples = []
        print("[CALIB] Reset complete.")

capL.release()
capR.release()
cv2.destroyAllWindows()