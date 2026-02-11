import cv2
import numpy as np
import pickle
from collections import deque

# ===========================
# CONFIGURATION SETTINGS
# ===========================
CALIB_FILE = "stereo_calibration.pkl"
LEFT_CAM_ID = 0
RIGHT_CAM_ID = 2

# Red color HSV ranges (red wraps around in HSV)
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

# Yellow color HSV range
LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([40, 255, 255])

# Detection parameters
MIN_AREA = 800  # Minimum contour area to consider (pixels)

# Stereo matching parameters
NUM_DISPARITIES = 16 * 10  # Must be divisible by 16
BLOCK_SIZE = 5  # Odd number, typically 3-11

# Temporal smoothing
SMOOTHING_FRAMES = 5  # Number of frames to smooth over

# Tracking parameters
TRACKING_THRESHOLD = 0.2  # meters - max distance to match objects between frames
MAX_FRAMES_MISSING = 10  # frames - remove object if not seen this long

# ===========================
# LOAD CALIBRATION DATA
# ===========================
print("Loading calibration data...")
try:
    with open(CALIB_FILE, "rb") as f:
        calib = pickle.load(f)
    
    mtxL = calib["mtxL"]
    distL = calib["distL"]
    mtxR = calib["mtxR"]
    distR = calib["distR"]
    R1 = calib["R1"]
    R2 = calib["R2"]
    P1 = calib["P1"]
    P2 = calib["P2"]
    Q = calib["Q"]
    
    print("✓ Calibration loaded successfully")
    
except FileNotFoundError:
    print(f"ERROR: Calibration file '{CALIB_FILE}' not found!")
    print("Please run stereo calibration first.")
    exit(1)
except Exception as e:
    print(f"ERROR loading calibration: {e}")
    exit(1)

# ===========================
# INITIALIZE CAMERAS
# ===========================
print("Initializing cameras...")
capL = cv2.VideoCapture(LEFT_CAM_ID)
capR = cv2.VideoCapture(RIGHT_CAM_ID)

if not capL.isOpened() or not capR.isOpened():
    print("ERROR: Cannot open cameras!")
    print(f"Left camera ID: {LEFT_CAM_ID}, Right camera ID: {RIGHT_CAM_ID}")
    exit(1)

# Set resolution (optional - adjust as needed)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
capL.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
capL.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
capR.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
capR.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# Read one frame to get actual dimensions
ret_l, frame_l = capL.read()
ret_r, frame_r = capR.read()

if not ret_l or not ret_r:
    print("ERROR: Failed to read from cameras!")
    capL.release()
    capR.release()
    exit(1)

img_height, img_width = frame_l.shape[:2]
image_size = (img_width, img_height)

print(f"✓ Cameras initialized at {img_width}x{img_height}")

# ===========================
# COMPUTE RECTIFICATION MAPS
# ===========================
print("Computing rectification maps...")

# Use the projection matrices from calibration
map1_left, map2_left = cv2.initUndistortRectifyMap(
    mtxL, distL, R1, P1, image_size, cv2.CV_16SC2
)
map1_right, map2_right = cv2.initUndistortRectifyMap(
    mtxR, distR, R2, P2, image_size, cv2.CV_16SC2
)

print("✓ Rectification maps computed")

# ===========================
# CREATE STEREO MATCHER
# ===========================
print("Creating stereo matcher...")

# Left matcher (main)
stereo_left = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=NUM_DISPARITIES,
    blockSize=BLOCK_SIZE,
    P1=8 * 3 * BLOCK_SIZE * BLOCK_SIZE,
    P2=32 * 3 * BLOCK_SIZE * BLOCK_SIZE,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32,
    disp12MaxDiff=1,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

# Right matcher for WLS filtering
stereo_right = cv2.ximgproc.createRightMatcher(stereo_left)

# WLS filter for better disparity quality
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereo_left)
wls_filter.setLambda(8000)
wls_filter.setSigmaColor(1.5)

print("✓ Stereo matcher initialized with WLS filtering")

# ===========================
# OBJECT TRACKING CLASS
# ===========================
class TrackedObject:
    """Represents a tracked object with temporal smoothing"""
    
    def __init__(self, obj_id):
        self.id = obj_id
        self.positions_3d = deque(maxlen=SMOOTHING_FRAMES)
        self.centers_2d = deque(maxlen=SMOOTHING_FRAMES)
        self.frames_missing = 0
        # Random color for visualization
        self.color = tuple(np.random.randint(50, 255, 3).tolist())
    
    def update(self, x, y, z, center_2d):
        """Update object with new 3D position"""
        self.positions_3d.append((x, y, z))
        self.centers_2d.append(center_2d)
        self.frames_missing = 0
    
    def get_smoothed_position_3d(self):
        """Get temporally smoothed 3D position using median"""
        if len(self.positions_3d) == 0:
            return None
        positions = np.array(self.positions_3d)
        return np.median(positions, axis=0)
    
    def get_smoothed_center_2d(self):
        """Get temporally smoothed 2D center"""
        if len(self.centers_2d) == 0:
            return None
        centers = np.array(self.centers_2d)
        return np.median(centers, axis=0).astype(int)
    
    def increment_missing(self):
        """Increment frames missing counter"""
        self.frames_missing += 1
    
    def is_stale(self):
        """Check if object should be removed"""
        return self.frames_missing > MAX_FRAMES_MISSING

# ===========================
# TRACKING VARIABLES
# ===========================
tracked_objects = {}
next_object_id = 0
frame_count = 0

# ===========================
# MAIN LOOP
# ===========================
print("\n" + "="*60)
print("STEREO 3D OBJECT TRACKING")
print("="*60)
print("Controls:")
print("  'q' - Quit")
print("  'r' - Toggle raw camera view")
print("  'd' - Toggle disparity view")
print("  's' - Save current frame")
print("="*60 + "\n")

show_raw = False
show_disparity = True

try:
    while True:
        # Read frames
        ret_l, frame_l = capL.read()
        ret_r, frame_r = capR.read()
        
        if not ret_l or not ret_r:
            print("ERROR: Failed to read frames")
            break
        
        frame_count += 1
        
        # ===========================
        # RECTIFY IMAGES
        # ===========================
        rect_left = cv2.remap(frame_l, map1_left, map2_left, cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_r, map1_right, map2_right, cv2.INTER_LINEAR)
        
        # ===========================
        # COMPUTE DISPARITY MAP
        # ===========================
        gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)
        
        # Compute disparity with WLS filtering
        disp_left = stereo_left.compute(gray_left, gray_right)
        disp_right = stereo_right.compute(gray_right, gray_left)
        disp = wls_filter.filter(disp_left, gray_left, None, disp_right)
        
        # Convert to float and scale
        disp = disp.astype(np.float32) / 16.0
        
        # ===========================
        # COLOR-BASED OBJECT DETECTION
        # ===========================
        hsv = cv2.cvtColor(rect_left, cv2.COLOR_BGR2HSV)
        
        # Create masks for red (two ranges because red wraps around)
        mask_red_1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
        mask_red_2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        
        # Create mask for yellow
        mask_yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
        
        # Combine masks
        mask_combined = cv2.bitwise_or(mask_red, mask_yellow)
        
        # Clean up mask with morphological operations
        mask_combined = cv2.GaussianBlur(mask_combined, (5, 5), 0)
        kernel = np.ones((5, 5), np.uint8)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, kernel, iterations=2)
        mask_combined = cv2.morphologyEx(mask_combined, cv2.MORPH_CLOSE, kernel, iterations=3)
        
        # Find contours
        contours, _ = cv2.findContours(
            mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        # ===========================
        # PROCESS DETECTED OBJECTS
        # ===========================
        detected_objects = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < MIN_AREA:
                continue
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w // 2
            cy = y + h // 2
            
            # Extract disparity patch around center
            patch_size = 5
            y1 = max(0, cy - patch_size)
            y2 = min(img_height, cy + patch_size + 1)
            x1 = max(0, cx - patch_size)
            x2 = min(img_width, cx + patch_size + 1)
            
            disp_patch = disp[y1:y2, x1:x2]
            
            # Filter valid disparities
            valid_disp = disp_patch[(disp_patch > 0) & (disp_patch < NUM_DISPARITIES)]
            
            # Need enough valid points for reliable depth
            if valid_disp.size < 5:
                continue
            
            # Use median disparity for robustness
            d = np.median(valid_disp)
            
            # ===========================
            # REPROJECT TO 3D
            # ===========================
            # Method: Use reprojection matrix Q
            # [X Y Z W]^T = Q * [u v disparity 1]^T
            # Then: real_X = X/W, real_Y = Y/W, real_Z = Z/W
            
            point_4d = np.array([cx, cy, d, 1.0])
            coords_4d = Q.dot(point_4d)
            
            # Avoid division by zero
            if coords_4d[3] == 0:
                continue
            
            X = coords_4d[0] / coords_4d[3]
            Y = coords_4d[1] / coords_4d[3]
            Z = coords_4d[2] / coords_4d[3]
            
            # Sanity check: valid depth range
            if not (np.isfinite(Z) and 0.1 < Z < 5.0):
                continue
            
            # Store detected object
            detected_objects.append({
                'bbox': (x, y, w, h),
                'center': (cx, cy),
                'coords_3d': (X, Y, Z),
                'disparity': d,
                'area': area
            })
        
        # ===========================
        # OBJECT TRACKING
        # ===========================
        # Age all tracked objects
        for obj_id in list(tracked_objects.keys()):
            tracked_objects[obj_id].increment_missing()
            if tracked_objects[obj_id].is_stale():
                del tracked_objects[obj_id]
        
        # Match detected objects to tracked objects
        for obj in detected_objects:
            X, Y, Z = obj['coords_3d']
            cx, cy = obj['center']
            
            # Find closest tracked object
            best_match_id = None
            best_distance = float('inf')
            
            for obj_id, tracked in tracked_objects.items():
                smoothed_pos = tracked.get_smoothed_position_3d()
                if smoothed_pos is None:
                    continue
                
                # Euclidean distance in 3D space
                dx = smoothed_pos[0] - X
                dy = smoothed_pos[1] - Y
                dz = smoothed_pos[2] - Z
                dist = np.sqrt(dx*dx + dy*dy + dz*dz)
                
                if dist < TRACKING_THRESHOLD and dist < best_distance:
                    best_match_id = obj_id
                    best_distance = dist
            
            if best_match_id is not None:
                # Update existing tracked object
                tracked_objects[best_match_id].update(X, Y, Z, (cx, cy))
                obj['id'] = best_match_id
            else:
                # Create new tracked object
                tracked_objects[next_object_id] = TrackedObject(next_object_id)
                tracked_objects[next_object_id].update(X, Y, Z, (cx, cy))
                obj['id'] = next_object_id
                next_object_id += 1
        
        # ===========================
        # VISUALIZATION
        # ===========================
        output = rect_left.copy()
        
        # Draw all detected objects
        for obj in detected_objects:
            obj_id = obj['id']
            tracked = tracked_objects[obj_id]
            
            # Get smoothed values
            smoothed_pos = tracked.get_smoothed_position_3d()
            smoothed_center = tracked.get_smoothed_center_2d()
            
            if smoothed_pos is None or smoothed_center is None:
                continue
            
            X_smooth, Y_smooth, Z_smooth = smoothed_pos
            cx_smooth, cy_smooth = smoothed_center
            
            # Get bounding box
            x, y, w, h = obj['bbox']
            
            # Draw bounding box
            cv2.rectangle(output, (x, y), (x + w, y + h), tracked.color, 2)
            
            # Draw center point
            cv2.circle(output, (cx_smooth, cy_smooth), 5, (255, 0, 0), -1)
            
            # Prepare text
            text = f"ID{obj_id}: Z={Z_smooth:.2f}m X={X_smooth:.2f}m Y={Y_smooth:.2f}m"
            
            # Draw text with background
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            (text_w, text_h), _ = cv2.getTextSize(text, font, font_scale, thickness)
            
            # Background rectangle
            cv2.rectangle(output, (x, y - text_h - 10), (x + text_w, y), tracked.color, -1)
            
            # Text
            cv2.putText(output, text, (x, y - 5), font, font_scale, (255, 255, 255), thickness)
        
        # Draw frame info
        info = f"Frame: {frame_count} | Objects: {len(detected_objects)} | Tracked: {len(tracked_objects)}"
        cv2.putText(output, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # ===========================
        # DISPLAY WINDOWS
        # ===========================
        cv2.imshow("3D Object Tracking", output)
        cv2.imshow("Detection Mask", mask_combined)
        
        if show_raw:
            cv2.imshow("Raw Left Camera", frame_l)
            cv2.imshow("Raw Right Camera", frame_r)
        
        if show_disparity:
            # Normalize disparity for visualization
            disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
            disp_vis = np.clip(disp_vis, 0, 255).astype(np.uint8)
            disp_colored = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
            cv2.imshow("Disparity Map", disp_colored)
        
        # ===========================
        # KEYBOARD CONTROLS
        # ===========================
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\nQuitting...")
            break
        elif key == ord('r'):
            show_raw = not show_raw
            if not show_raw:
                cv2.destroyWindow("Raw Left Camera")
                cv2.destroyWindow("Raw Right Camera")
            print(f"Raw view: {'ON' if show_raw else 'OFF'}")
        elif key == ord('d'):
            show_disparity = not show_disparity
            if not show_disparity:
                cv2.destroyWindow("Disparity Map")
            print(f"Disparity view: {'ON' if show_disparity else 'OFF'}")
        elif key == ord('s'):
            filename = f"frame_{frame_count:05d}.png"
            cv2.imwrite(filename, output)
            print(f"Saved: {filename}")

except KeyboardInterrupt:
    print("\nInterrupted by user")
except Exception as e:
    print(f"\nERROR: {e}")
    import traceback
    traceback.print_exc()
finally:
    # ===========================
    # CLEANUP
    # ===========================
    print("Cleaning up...")
    capL.release()
    capR.release()
    cv2.destroyAllWindows()
    print("Done!")