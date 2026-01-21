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
# Load calibration - ONLY intrinsics and R, T
# -------------------------------------------------
with open(CALIB_FILE, "rb") as f:
    calib = pickle.load(f)

# These are what we need from calibration:
mtxL, distL = calib["mtxL"], calib["distL"]
mtxR, distR = calib["mtxR"], calib["distR"]

# Rotation and Translation between cameras (essential!)
R = calib.get("R", None)
T = calib.get("T", None)

if R is None or T is None:
    print("\nWARNING: R and T not found in calibration file.")
    print("Checking for alternative key names...")
    # Try alternative names
    R = calib.get("R1", None) 
    T = calib.get("T1", None)
    
if R is None or T is None:
    raise ValueError(
        "Calibration file missing R (rotation) and T (translation) matrices!\n"
        "These are essential for stereo rectification.\n"
        "Your calibration file should contain:\n"
        "  - mtxL, distL (left camera intrinsics)\n"
        "  - mtxR, distR (right camera intrinsics)\n"
        "  - R (rotation between cameras)\n"
        "  - T (translation between cameras)"
    )

print("\nLoaded calibration parameters:")
print("  Left camera matrix:\n", mtxL)
print("  Right camera matrix:\n", mtxR)
print("  Rotation R shape:", R.shape)
print("  Translation T shape:", T.shape)

# -------------------------------------------------
# Load stereo pair
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

h, w = imgL.shape[:2]
image_size = (w, h)
print("Image size:", image_size)

# -------------------------------------------------
# CRITICAL: Recompute rectification with alpha=1.0
# DO NOT use P1, P2, R1, R2, Q from calibration file
# -------------------------------------------------
print("\n" + "="*60)
print("Computing NEW rectification matrices with alpha=1.0")
print("This ensures full image is preserved (no cropping)")
print("="*60)

# Compute rectification transforms with alpha=1.0 for full image
R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(
    cameraMatrix1=mtxL,
    distCoeffs1=distL,
    cameraMatrix2=mtxR,
    distCoeffs2=distR,
    imageSize=image_size,
    R=R,
    T=T,
    flags=cv2.CALIB_ZERO_DISPARITY,  # Important flag
    alpha=1.0,  # Keep all pixels - FULL IMAGE
    newImageSize=image_size
)

print("\nRectification computed successfully!")
print("ROI Left  (x, y, w, h):", roi_left)
print("ROI Right (x, y, w, h):", roi_right)
print("\nNote: ROI indicates the valid (non-black) region after rectification")

# Calculate baseline from T
baseline = np.linalg.norm(T)
print(f"Baseline distance: {baseline:.6f} units")

# -------------------------------------------------
# Create rectification maps
# -------------------------------------------------
print("\nCreating undistortion and rectification maps...")
map1L, map2L = cv2.initUndistortRectifyMap(
    mtxL, distL, R1, P1, image_size, cv2.CV_16SC2)
map1R, map2R = cv2.initUndistortRectifyMap(
    mtxR, distR, R2, P2, image_size, cv2.CV_16SC2)

# -------------------------------------------------
# Apply rectification
# -------------------------------------------------
print("Applying rectification to images...")
grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

rectL = cv2.remap(grayL, map1L, map2L, cv2.INTER_LINEAR)
rectR = cv2.remap(grayR, map1R, map2R, cv2.INTER_LINEAR)

# Also rectify color images for visualization
rectL_color = cv2.remap(imgL, map1L, map2L, cv2.INTER_LINEAR)
rectR_color = cv2.remap(imgR, map1R, map2R, cv2.INTER_LINEAR)

print(f"Rectified image size: {rectL.shape}")

# -------------------------------------------------
# Rectification check image (horizontal lines)
# -------------------------------------------------
check = np.hstack((rectL_color, rectR_color))

# Draw horizontal lines to verify rectification
for y in range(0, check.shape[0], 30):
    cv2.line(check, (0, y), (check.shape[1], y), (0, 255, 0), 1)

# Draw vertical separator
cv2.line(check, (w, 0), (w, h), (255, 0, 0), 2)

# Draw ROI rectangles
if roi_left[2] > 0 and roi_left[3] > 0:
    x, y, rw, rh = roi_left
    cv2.rectangle(check, (x, y), (x + rw, y + rh), (255, 255, 0), 2)
    cv2.putText(check, "Valid ROI", (x + 10, y + 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

if roi_right[2] > 0 and roi_right[3] > 0:
    x, y, rw, rh = roi_right
    cv2.rectangle(check, (w + x, y), (w + x + rw, y + rh), (255, 255, 0), 2)

# -------------------------------------------------
# Disparity computation
# -------------------------------------------------
print("\nComputing disparity map...")

# Adjust parameters based on image size and expected depth range
num_disp = 16 * 8      # Multiple of 16, adjust based on your scene depth
block_size = 5         # Odd number, smaller = more detail but more noise

stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=num_disp,
    blockSize=block_size,
    P1=8 * 3 * block_size * block_size,
    P2=32 * 3 * block_size * block_size,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=2,
    disp12MaxDiff=1,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

disp = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

# Filter invalid disparities
disp[disp <= 0] = 0.01
disp[disp >= num_disp] = 0.01

# Visualize disparity
disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
disp_vis = disp_vis.astype(np.uint8)
disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

# -------------------------------------------------
# Depth computation
# -------------------------------------------------
print("Computing 3D depth map...")
points_3d = cv2.reprojectImageTo3D(disp, Q)
depth = points_3d[:, :, 2]

# Filter invalid depths
depth[depth <= 0] = 0
depth[depth > 50000] = 0  # Filter extreme values

# Sample center pixel
cy, cx = h // 2, w // 2
print(f"\nCenter pixel (x={cx}, y={cy}):")
print(f"  Disparity: {disp[cy, cx]:.2f} pixels")
print(f"  Depth:     {depth[cy, cx]:.2f} units")

# Visualize depth
depth_valid = depth.copy()
depth_valid[depth_valid <= 0] = np.nan

if np.any(~np.isnan(depth_valid)):
    p5 = np.nanpercentile(depth_valid, 5)
    p95 = np.nanpercentile(depth_valid, 95)
    depth_vis = np.clip(depth_valid, p5, p95)
    depth_vis = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX)
    depth_vis = np.nan_to_num(depth_vis, 0).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
else:
    depth_color = np.zeros((h, w, 3), dtype=np.uint8)
    print("WARNING: No valid depth computed!")

# -------------------------------------------------
# Save outputs
# -------------------------------------------------
print("\nSaving output files...")
out_dir = SCRIPT_DIR

cv2.imwrite(os.path.join(out_dir, "1_original_left.png"), imgL)
cv2.imwrite(os.path.join(out_dir, "1_original_right.png"), imgR)
cv2.imwrite(os.path.join(out_dir, "2_rectified_left.png"), rectL_color)
cv2.imwrite(os.path.join(out_dir, "2_rectified_right.png"), rectR_color)
cv2.imwrite(os.path.join(out_dir, "3_rectification_check.png"), check)
cv2.imwrite(os.path.join(out_dir, "4_disparity.png"), disp_color)
cv2.imwrite(os.path.join(out_dir, "5_depth_map.png"), depth_color)

# Save depth data
np.save(os.path.join(out_dir, "depth_data.npy"), depth)
np.save(os.path.join(out_dir, "disparity_data.npy"), disp)

print("\nSaved files:")
print("  1_original_left.png, 1_original_right.png")
print("  2_rectified_left.png, 2_rectified_right.png")
print("  3_rectification_check.png (with horizontal lines)")
print("  4_disparity.png")
print("  5_depth_map.png")
print("  depth_data.npy, disparity_data.npy")

# -------------------------------------------------
# Display windows
# -------------------------------------------------
has_display = ("DISPLAY" in os.environ) and (os.environ["DISPLAY"].strip() != "")
if not has_display:
    print("\nNo DISPLAY detected. Check saved images.")
    sys.exit(0)

cv2.namedWindow("Original Left", cv2.WINDOW_NORMAL)
cv2.namedWindow("Original Right", cv2.WINDOW_NORMAL)
cv2.namedWindow("Rectified Check (with horizontal lines)", cv2.WINDOW_NORMAL)
cv2.namedWindow("Disparity Map", cv2.WINDOW_NORMAL)
cv2.namedWindow("Depth Map", cv2.WINDOW_NORMAL)

cv2.imshow("Original Left", imgL)
cv2.imshow("Original Right", imgR)
cv2.imshow("Rectified Check (with horizontal lines)", check)
cv2.imshow("Disparity Map", disp_color)
cv2.imshow("Depth Map", depth_color)

print("\n" + "="*60)
print("Windows opened successfully!")
print("The rectified images should now show FULL checkerboards")
print("Press 'q' or ESC to quit")
print("="*60)

while True:
    k = cv2.waitKey(30) & 0xFF
    if k == 27 or k == ord('q'):
        break

cv2.destroyAllWindows()
print("\nDone!")