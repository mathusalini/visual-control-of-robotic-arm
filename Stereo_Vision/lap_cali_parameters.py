import cv2
import numpy as np
import glob
import pickle
import os

# ----------------------------
# SETTINGS (EDIT THESE)
# ----------------------------
CHESSBOARD_SIZE = (11, 7)   # inner corners (columns, rows)
SQUARE_SIZE = 0.025         # meters

# Path to your stereo pairs folder
# Option A (recommended on Windows): absolute path
DATA_DIR = r"D:\FYP-2\visual-control-of-robotic-arm\Stereo_Vision\Stereo_pairs_1"

# Option B: relative path (only works if CWD is correct)
# DATA_DIR = "Stereo_pairs_1"

LEFT_PATTERN = "left_*.png"
RIGHT_PATTERN = "right_*.png"

SAVE_DEBUG_IMAGES = True
DEBUG_DIR = "debug_chessboard"
CALIB_OUT = "stereo_calibration.pkl"

# ----------------------------
# Helper: make dir
# ----------------------------
def ensure_dir(path: str):
    if not os.path.exists(path):
        os.makedirs(path)

# ----------------------------
# Show environment info
# ----------------------------
print("OpenCV version:", cv2.__version__)
print("Current Working Directory (CWD):", os.getcwd())
print("Data dir:", DATA_DIR)

# ----------------------------
# Prepare object points (0,0,0) at top-left inner corner of the board
# X grows to the right, Y grows downward on the board plane, Z = 0
# ----------------------------
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []   # 3D points in chessboard coordinate system
imgpointsL = []  # 2D points in left image
imgpointsR = []  # 2D points in right image

# ----------------------------
# Load images
# ----------------------------
left_glob = os.path.join(DATA_DIR, LEFT_PATTERN)
right_glob = os.path.join(DATA_DIR, RIGHT_PATTERN)

left_images = sorted(glob.glob(left_glob))
right_images = sorted(glob.glob(right_glob))

print("Left images found:", len(left_images))
print("Right images found:", len(right_images))

if len(left_images) == 0 or len(right_images) == 0:
    raise RuntimeError(
        "No images found. Check DATA_DIR and filename patterns.\n"
        f"Left glob:  {left_glob}\n"
        f"Right glob: {right_glob}"
    )

if len(left_images) != len(right_images):
    raise RuntimeError("Image count mismatch between left and right folders/patterns.")

# Debug folder
if SAVE_DEBUG_IMAGES:
    ensure_dir(DEBUG_DIR)

# ----------------------------
# Find corners
# ----------------------------
criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

img_size = None
valid_pairs = 0

for i, (left_path, right_path) in enumerate(zip(left_images, right_images)):
    imgL = cv2.imread(left_path)
    imgR = cv2.imread(right_path)

    if imgL is None or imgR is None:
        print(f"[WARN] Could not read pair {i}:")
        print("   L:", left_path)
        print("   R:", right_path)
        continue

    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    if img_size is None:
        img_size = grayL.shape[::-1]  # (width, height)

    # Optional: flags can help sometimes
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    retL, cornersL = cv2.findChessboardCorners(grayL, CHESSBOARD_SIZE, flags)
    retR, cornersR = cv2.findChessboardCorners(grayR, CHESSBOARD_SIZE, flags)

    if retL and retR:
        cornersL = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria_subpix)
        cornersR = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria_subpix)

        objpoints.append(objp)
        imgpointsL.append(cornersL)
        imgpointsR.append(cornersR)

        valid_pairs += 1
        print(f"[OK] Pair {i}: corners found.")
    else:
        print(f"[FAIL] Pair {i}: corners not found (L={retL}, R={retR}).")

        if SAVE_DEBUG_IMAGES:
            visL = imgL.copy()
            visR = imgR.copy()
            if retL:
                cv2.drawChessboardCorners(visL, CHESSBOARD_SIZE, cornersL, retL)
            if retR:
                cv2.drawChessboardCorners(visR, CHESSBOARD_SIZE, cornersR, retR)

            baseL = os.path.join(DEBUG_DIR, f"pair_{i:03d}_L.png")
            baseR = os.path.join(DEBUG_DIR, f"pair_{i:03d}_R.png")
            cv2.imwrite(baseL, visL)
            cv2.imwrite(baseR, visR)

print("Valid pairs used:", valid_pairs)

if valid_pairs == 0:
    raise RuntimeError(
        "Valid pairs used: 0\n"
        "No chessboards were detected. Likely causes:\n"
        "- CHESSBOARD_SIZE is wrong (inner corners count)\n"
        "- Board not fully visible / blurry / reflections\n"
        "- Images not actually chessboard calibration images\n"
        f"Debug images (if enabled) saved to: {os.path.abspath(DEBUG_DIR)}"
    )

# ----------------------------
# Calibrate individual cameras
# ----------------------------
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(
    objpoints, imgpointsL, img_size, None, None
)
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(
    objpoints, imgpointsR, img_size, None, None
)

print("Left camera RMS:", retL)
print("Right camera RMS:", retR)

# ----------------------------
# Stereo calibration
# ----------------------------
flags = cv2.CALIB_FIX_INTRINSIC
criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

retS, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
    objpoints,
    imgpointsL,
    imgpointsR,
    mtxL, distL,
    mtxR, distR,
    img_size,
    criteria=criteria_stereo,
    flags=flags
)

print("Stereo RMS error:", retS)
print("R (rotation):\n", R)
print("T (translation, meters):\n", T)

# ----------------------------
# Stereo rectification
# ----------------------------
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
    mtxL, distL,
    mtxR, distR,
    img_size,
    R, T,
    alpha=0
)

# ----------------------------
# Save calibration
# ----------------------------
calib_data = {
    "mtxL": mtxL, "distL": distL,
    "mtxR": mtxR, "distR": distR,
    "R": R, "T": T,
    "R1": R1, "R2": R2,
    "P1": P1, "P2": P2,
    "Q": Q,
    "img_size": img_size
}

with open(CALIB_OUT, "wb") as f:
    pickle.dump(calib_data, f)

print("Calibration saved to:", os.path.abspath(CALIB_OUT))
print("Done.")