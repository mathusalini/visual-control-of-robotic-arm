import cv2
import numpy as np
import pickle
import os
import sys
from datetime import datetime

# -------------------------------------------------
# Configuration
# -------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CALIB_FILE = os.path.join(SCRIPT_DIR, "stereo_calibration.pkl")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "captured_images")

# Create output directory if it doesn't exist
os.makedirs(OUTPUT_DIR, exist_ok=True)

print("="*60)
print("STEREO IMAGE CAPTURE AND RECTIFICATION CHECKER")
print("="*60)
print("SCRIPT_DIR :", SCRIPT_DIR)
print("CALIB_FILE :", CALIB_FILE)
print("OUTPUT_DIR :", OUTPUT_DIR)

# -------------------------------------------------
# Load calibration
# -------------------------------------------------
if not os.path.exists(CALIB_FILE):
    raise FileNotFoundError(f"Calibration file not found: {CALIB_FILE}")

with open(CALIB_FILE, "rb") as f:
    calib = pickle.load(f)

mtxL, distL = calib["mtxL"], calib["distL"]
mtxR, distR = calib["mtxR"], calib["distR"]
R = calib.get("R", None)
T = calib.get("T", None)

if R is None or T is None:
    raise ValueError("Calibration file missing R and T matrices!")

print("\nCalibration loaded successfully!")

# -------------------------------------------------
# Setup cameras
# -------------------------------------------------
print("\n" + "="*60)
print("Setting up cameras...")
print("="*60)

# Try to open cameras (adjust camera indices if needed)
# Camera indices: 0, 1, 2, etc. depending on your system
LEFT_CAMERA_INDEX = 0
RIGHT_CAMERA_INDEX = 1

cap_left = cv2.VideoCapture(LEFT_CAMERA_INDEX)
cap_right = cv2.VideoCapture(RIGHT_CAMERA_INDEX)

if not cap_left.isOpened():
    raise RuntimeError(f"Cannot open left camera (index {LEFT_CAMERA_INDEX})")
if not cap_right.isOpened():
    raise RuntimeError(f"Cannot open right camera (index {RIGHT_CAMERA_INDEX})")

# Set camera resolution to match calibration
cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print(f"Left camera opened:  Index {LEFT_CAMERA_INDEX}")
print(f"Right camera opened: Index {RIGHT_CAMERA_INDEX}")

# Get actual resolution
ret_l, test_l = cap_left.read()
ret_r, test_r = cap_right.read()

if not ret_l or not ret_r:
    raise RuntimeError("Failed to read from cameras")

h, w = test_l.shape[:2]
image_size = (w, h)
print(f"Camera resolution: {w} x {h}")

# -------------------------------------------------
# Compute rectification parameters with alpha=1.0
# -------------------------------------------------
print("\nComputing rectification parameters...")
R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(
    cameraMatrix1=mtxL,
    distCoeffs1=distL,
    cameraMatrix2=mtxR,
    distCoeffs2=distR,
    imageSize=image_size,
    R=R,
    T=T,
    flags=cv2.CALIB_ZERO_DISPARITY,
    alpha=1.0,  # Full image, no cropping
    newImageSize=image_size
)

# Create rectification maps
map1L, map2L = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, image_size, cv2.CV_16SC2)
map1R, map2R = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, image_size, cv2.CV_16SC2)

print("Rectification parameters computed successfully!")

# -------------------------------------------------
# Function to rectify and display images
# -------------------------------------------------
def process_and_display(imgL, imgR, save_images=False, timestamp=""):
    """Process stereo pair and display rectified result"""
    
    # Convert to grayscale for disparity
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    
    # Apply rectification
    rectL_color = cv2.remap(imgL, map1L, map2L, cv2.INTER_LINEAR)
    rectR_color = cv2.remap(imgR, map1R, map2R, cv2.INTER_LINEAR)
    
    rectL_gray = cv2.remap(grayL, map1L, map2L, cv2.INTER_LINEAR)
    rectR_gray = cv2.remap(grayR, map1R, map2R, cv2.INTER_LINEAR)
    
    # Create side-by-side comparison
    original_pair = np.hstack((imgL, imgR))
    rectified_pair = np.hstack((rectL_color, rectR_color))
    
    # Create rectification check with horizontal lines
    check = rectified_pair.copy()
    for y in range(0, check.shape[0], 30):
        cv2.line(check, (0, y), (check.shape[1], y), (0, 255, 0), 1)
    
    # Draw vertical separator
    cv2.line(check, (w, 0), (w, h), (255, 0, 0), 2)
    
    # Draw ROI rectangles
    if roi_left[2] > 0 and roi_left[3] > 0:
        x, y, rw, rh = roi_left
        cv2.rectangle(check, (x, y), (x + rw, y + rh), (255, 255, 0), 2)
    
    if roi_right[2] > 0 and roi_right[3] > 0:
        x, y, rw, rh = roi_right
        cv2.rectangle(check, (w + x, y), (w + x + rw, y + rh), (255, 255, 0), 2)
    
    # Compute disparity
    num_disp = 16 * 8
    block_size = 5
    
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
    
    disp = stereo.compute(rectL_gray, rectR_gray).astype(np.float32) / 16.0
    disp[disp <= 0] = 0.01
    
    # Visualize disparity
    disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
    disp_vis = disp_vis.astype(np.uint8)
    disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
    
    # Compute depth
    points_3d = cv2.reprojectImageTo3D(disp, Q)
    depth = points_3d[:, :, 2]
    depth[depth <= 0] = 0
    
    # Add text overlay
    info_text = [
        "RECTIFICATION CHECK",
        "Green lines should align across left and right images",
        "Yellow box shows valid ROI",
        f"Image size: {w}x{h}"
    ]
    
    check_with_text = check.copy()
    y_offset = 30
    for text in info_text:
        cv2.putText(check_with_text, text, (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 25
    
    # Save images if requested
    if save_images and timestamp:
        save_dir = os.path.join(OUTPUT_DIR, timestamp)
        os.makedirs(save_dir, exist_ok=True)
        
        cv2.imwrite(os.path.join(save_dir, "1_original_left.png"), imgL)
        cv2.imwrite(os.path.join(save_dir, "1_original_right.png"), imgR)
        cv2.imwrite(os.path.join(save_dir, "2_original_pair.png"), original_pair)
        cv2.imwrite(os.path.join(save_dir, "3_rectified_left.png"), rectL_color)
        cv2.imwrite(os.path.join(save_dir, "3_rectified_right.png"), rectR_color)
        cv2.imwrite(os.path.join(save_dir, "4_rectified_pair.png"), rectified_pair)
        cv2.imwrite(os.path.join(save_dir, "5_rectification_check.png"), check_with_text)
        cv2.imwrite(os.path.join(save_dir, "6_disparity.png"), disp_color)
        
        np.save(os.path.join(save_dir, "disparity_data.npy"), disp)
        np.save(os.path.join(save_dir, "depth_data.npy"), depth)
        
        print(f"\n✓ Images saved to: {save_dir}")
        print("  - Original images (left, right, pair)")
        print("  - Rectified images (left, right, pair)")
        print("  - Rectification check (with lines)")
        print("  - Disparity map")
        print("  - Depth data (.npy)")
    
    return original_pair, rectified_pair, check_with_text, disp_color

# -------------------------------------------------
# Main capture loop
# -------------------------------------------------
print("\n" + "="*60)
print("LIVE CAMERA VIEW - CONTROLS:")
print("="*60)
print("  SPACE   - Capture and save stereo pair")
print("  'c'     - Capture (same as SPACE)")
print("  'r'     - Toggle rectification view")
print("  'q/ESC' - Quit")
print("="*60)

show_rectified = False
capture_count = 0

cv2.namedWindow("Stereo View", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Stereo View", 1280, 480)

while True:
    # Read frames from both cameras
    ret_l, frame_l = cap_left.read()
    ret_r, frame_r = cap_right.read()
    
    if not ret_l or not ret_r:
        print("Failed to read from cameras")
        break
    
    # Show either original or rectified view
    if show_rectified:
        _, _, check, _ = process_and_display(frame_l, frame_r, save_images=False)
        display_frame = check
        cv2.putText(display_frame, "RECTIFIED VIEW (Press 'r' for original)", 
                   (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    else:
        display_frame = np.hstack((frame_l, frame_r))
        cv2.line(display_frame, (w, 0), (w, h), (255, 0, 0), 2)
        cv2.putText(display_frame, "ORIGINAL VIEW (Press 'r' for rectified)", 
                   (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(display_frame, "Press SPACE to capture", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    cv2.imshow("Stereo View", display_frame)
    
    # Handle keyboard input
    key = cv2.waitKey(1) & 0xFF
    
    if key == 27 or key == ord('q'):  # ESC or 'q'
        print("\nQuitting...")
        break
    
    elif key == ord('r'):  # Toggle rectified view
        show_rectified = not show_rectified
        print(f"View mode: {'RECTIFIED' if show_rectified else 'ORIGINAL'}")
    
    elif key == 32 or key == ord('c'):  # SPACE or 'c' to capture
        capture_count += 1
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        print(f"\n{'='*60}")
        print(f"CAPTURING IMAGE PAIR #{capture_count}")
        print(f"{'='*60}")
        
        # Process and save
        orig, rect, check, disp = process_and_display(
            frame_l, frame_r, 
            save_images=True, 
            timestamp=timestamp
        )
        
        # Show results in separate windows
        cv2.namedWindow(f"Capture #{capture_count} - Original", cv2.WINDOW_NORMAL)
        cv2.namedWindow(f"Capture #{capture_count} - Rectified Check", cv2.WINDOW_NORMAL)
        cv2.namedWindow(f"Capture #{capture_count} - Disparity", cv2.WINDOW_NORMAL)
        
        cv2.imshow(f"Capture #{capture_count} - Original", orig)
        cv2.imshow(f"Capture #{capture_count} - Rectified Check", check)
        cv2.imshow(f"Capture #{capture_count} - Disparity", disp)
        
        print(f"\n✓ Capture #{capture_count} completed!")
        print("  Check the rectification - horizontal lines should align")
        print("  Press any key in the capture windows to close them")

# -------------------------------------------------
# Cleanup
# -------------------------------------------------
cap_left.release()
cap_right.release()
cv2.destroyAllWindows()

print("\n" + "="*60)
print(f"Session complete! Total captures: {capture_count}")
print(f"All images saved in: {OUTPUT_DIR}")
print("="*60)