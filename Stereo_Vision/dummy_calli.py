import cv2
import numpy as np
import glob
import pickle

# ----------------------------
# Chessboard parameters
# ----------------------------
chessboard_size = (9,7)     # INNER corners
square_size = 0.025           # meters

# ----------------------------
# Prepare object points
# ----------------------------
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0],
                       0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpointsL = []
imgpointsR = []

# ----------------------------
# Load images (FIXED PATH)
# ----------------------------
left_images = sorted(glob.glob("/home/group36/stereo_pairs/left_*.png"))
right_images = sorted(glob.glob("/home/group36/stereo_pairs/right_*.png"))

assert len(left_images) == len(right_images), "Image count mismatch"
assert len(left_images) > 0, "No images found"

# ----------------------------
# Find corners
# ----------------------------
image_size = None

for left_img, right_img in zip(left_images, right_images):

    imgL = cv2.imread(left_img)
    imgR = cv2.imread(right_img)

    if imgL is None or imgR is None:
        continue

    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    if image_size is None:
        image_size = grayL.shape[::-1]

    retL, cornersL = cv2.findChessboardCorners(grayL, chessboard_size, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, chessboard_size, None)

    if retL and retR:
        objpoints.append(objp)

        cornersL = cv2.cornerSubPix(
            grayL, cornersL, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

        cornersR = cv2.cornerSubPix(
            grayR, cornersR, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

        imgpointsL.append(cornersL)
        imgpointsR.append(cornersR)

print("Valid pairs used:", len(objpoints))

# ----------------------------
# Safety check
# ----------------------------
if len(objpoints) == 0:
    raise RuntimeError("No valid stereo pairs found. Check chessboard size and images.")

# ----------------------------
# Calibrate individual cameras
# ----------------------------
retL, mtxL, distL, _, _ = cv2.calibrateCamera(
    objpoints, imgpointsL, image_size, None, None)

retR, mtxR, distR, _, _ = cv2.calibrateCamera(
    objpoints, imgpointsR, image_size, None, None)

# ----------------------------
# Stereo calibration
# ----------------------------
flags = cv2.CALIB_FIX_INTRINSIC
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

retS, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
    objpoints,
    imgpointsL,
    imgpointsR,
    mtxL, distL,
    mtxR, distR,
    image_size,
    criteria=criteria,
    flags=flags
)

print("Stereo RMS error:", retS)

# ----------------------------
# Stereo rectification
# ----------------------------
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    mtxL, distL,
    mtxR, distR,
    image_size,
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
    "Q": Q
}

with open("stereo_calibration.pkl", "wb") as f:
    pickle.dump(calib_data, f)

print("Calibration saved to stereo_calibration.pkl")

