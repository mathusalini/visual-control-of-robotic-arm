#!/usr/bin/env python3
"""
stereo_world_object_tracker.py

FULL end-to-end code for:
1) Stereo rectification + disparity + reprojectImageTo3D (LEFT camera frame XYZ)
2) ONE-TIME camera->world pose estimation using 4 floor markers on your black sheet (click)
3) Color detection on rectified LEFT frame -> object centroid pixel (u,v)
4) Get Pc = points_3d[v,u] and convert to world: Pw = Rwc*Pc + twc
5) Save / load Rwc, twc so you DO NOT click every time

Folder structure (same as your script):
- Stereo_pairs_1/
    left_000.png, right_000.png, ...
- stereo_calibration.pkl
- (generated) cam_to_world.npz

Run:
A) First time (compute pose by clicking 4 markers):
   python3 stereo_world_object_tracker.py --mode pose

B) Normal run (detect object and print world coords):
   python3 stereo_world_object_tracker.py --mode run --color red
"""

import cv2
import numpy as np
import pickle
import glob
import os
import sys
import argparse

# -------------------------------------------------
# Paths (same style as your script)
# -------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PAIR_DIR   = os.path.join(SCRIPT_DIR, "Stereo_pairs_1")
CALIB_FILE = os.path.join(SCRIPT_DIR, "stereo_calibration.pkl")
POSE_SAVE  = os.path.join(SCRIPT_DIR, "cam_to_world.npz")

# -------------------------------------------------
# Marker world coordinates (EDIT THIS!)
# -------------------------------------------------
# Measure 4 marker centers from your chosen origin corner (0,0,0) on black sheet.
# Units MUST match your stereo XYZ units (usually meters if Q was built with meters).
# Order here MUST match the order you CLICK them on the rectified LEFT image.
OBJP_WORLD = np.array([
    [0.00, 0.00, 0.0],   # Marker 1
    [0.40, 0.00, 0.0],   # Marker 2
    [0.00, 0.30, 0.0],   # Marker 3
    [0.40, 0.30, 0.0],   # Marker 4
], dtype=np.float64)

# -------------------------------------------------
# Utilities
# -------------------------------------------------
def require_file(path, msg):
    if not os.path.exists(path):
        raise FileNotFoundError(f"{msg}: {path}")

def require_dir(path, msg):
    if not os.path.isdir(path):
        raise FileNotFoundError(f"{msg}: {path}")

def load_calibration():
    require_file(CALIB_FILE, "Calibration file not found")
    with open(CALIB_FILE, "rb") as f:
        calib = pickle.load(f)
    # Using your known keys:
    mtxL, distL = calib["mtxL"], calib["distL"]
    mtxR, distR = calib["mtxR"], calib["distR"]
    R1, R2 = calib["R1"], calib["R2"]
    P1, P2 = calib["P1"], calib["P2"]
    Q = calib["Q"]
    return mtxL, distL, mtxR, distR, R1, R2, P1, P2, Q

def load_first_pair():
    require_dir(PAIR_DIR, "Stereo pair folder not found")

    left_images  = sorted(glob.glob(os.path.join(PAIR_DIR, "left_*.*")))
    right_images = sorted(glob.glob(os.path.join(PAIR_DIR, "right_*.*")))

    if len(left_images) == 0 or len(right_images) == 0:
        raise RuntimeError(
            f"No images found in {PAIR_DIR}\n"
            f"Found left={len(left_images)}, right={len(right_images)}\n"
            f"Expected names like left_000.png and right_000.png"
        )
    if len(left_images) != len(right_images):
        raise RuntimeError(f"Image count mismatch: left={len(left_images)} right={len(right_images)}")

    left_path, right_path = left_images[0], right_images[0]
    imgL = cv2.imread(left_path)
    imgR = cv2.imread(right_path)
    if imgL is None or imgR is None:
        raise RuntimeError(f"Failed to read:\n{left_path}\n{right_path}")

    return imgL, imgR, left_path, right_path

def rectify_pair(imgL, imgR, mtxL, distL, mtxR, distR, R1, R2, P1, P2):
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    h, w = grayL.shape[:2]
    image_size = (w, h)

    map1L, map2L = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, image_size, cv2.CV_16SC2)
    map1R, map2R = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, image_size, cv2.CV_16SC2)

    rectL = cv2.remap(grayL, map1L, map2L, cv2.INTER_LINEAR)
    rectR = cv2.remap(grayR, map1R, map2R, cv2.INTER_LINEAR)
    rectL_bgr = cv2.remap(imgL, map1L, map2L, cv2.INTER_LINEAR)  # for color detection

    return rectL, rectR, rectL_bgr

def compute_disparity(rectL, rectR, num_disp=16*8, block_size=7):
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
    return disp

def click_points_on_image(image_gray, n_points):
    has_display = ("DISPLAY" in os.environ) and (os.environ["DISPLAY"].strip() != "")
    if not has_display:
        raise RuntimeError("No DISPLAY detected. You need GUI to click markers.")

    img_show = cv2.cvtColor(image_gray, cv2.COLOR_GRAY2BGR)
    clicked = []

    win = "Click markers on RECTIFIED LEFT (press q when done)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def mouse_cb(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked.append([x, y])
            cv2.circle(img_show, (x, y), 6, (0, 255, 0), -1)
            cv2.putText(img_show, str(len(clicked)), (x+8, y-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.imshow(win, img_show)

    cv2.setMouseCallback(win, mouse_cb)

    print(f"\nClick {n_points} markers in SAME ORDER as OBJP_WORLD, then press 'q'")
    while True:
        cv2.imshow(win, img_show)
        key = cv2.waitKey(20) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyWindow(win)

    if len(clicked) != n_points:
        raise ValueError(f"Need {n_points} clicks, got {len(clicked)}")

    return np.array(clicked, dtype=np.float64).reshape(-1, 1, 2)

def compute_pose_from_markers(objp_world, imgp_pixels, K, dist):
    """
    solvePnP gives world->camera: Pc = Rcw*Pw + tcw
    invert to get camera->world: Pw = Rwc*Pc + twc
    """
    ok, rvec, tvec = cv2.solvePnP(objp_world, imgp_pixels, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed")

    Rcw, _ = cv2.Rodrigues(rvec)
    Rwc = Rcw.T
    twc = -Rwc @ tvec
    return Rwc, twc

def load_pose():
    require_file(POSE_SAVE, "Pose file not found. Run --mode pose first")
    data = np.load(POSE_SAVE)
    Rwc = data["Rwc"]
    twc = data["twc"].reshape(3, 1)
    return Rwc, twc

def save_pose(Rwc, twc):
    np.savez(POSE_SAVE, Rwc=Rwc, twc=twc)

def cam_to_world(Pc_xyz, Rwc, twc):
    Pc = np.array(Pc_xyz, dtype=np.float64).reshape(3, 1)
    Pw = Rwc @ Pc + twc
    return Pw.reshape(3,)

# -------------------------------------------------
# Color detection on rectified LEFT BGR image
# -------------------------------------------------
def get_hsv_mask(hsv, color_name):
    """
    Simple HSV masks. Tune if needed.
    """
    if color_name == "red":
        # red wraps hue -> two ranges
        lower1 = np.array([0, 120, 60])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([170, 120, 60])
        upper2 = np.array([179, 255, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        return mask

    if color_name == "blue":
        lower = np.array([95, 120, 60])
        upper = np.array([135, 255, 255])
        return cv2.inRange(hsv, lower, upper)

    if color_name == "yellow":
        lower = np.array([20, 120, 80])
        upper = np.array([35, 255, 255])
        return cv2.inRange(hsv, lower, upper)

    if color_name == "green":
        lower = np.array([40, 80, 60])
        upper = np.array([85, 255, 255])
        return cv2.inRange(hsv, lower, upper)

    raise ValueError("Unsupported color. Use: red/blue/yellow/green")

def find_largest_contour_centroid(mask):
    # clean mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < 150:  # ignore tiny noise (tune)
        return None, mask

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, mask

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy), mask

# -------------------------------------------------
# Main modes
# -------------------------------------------------
def mode_pose():
    print("\n=== MODE: pose (click markers and save Rwc, twc) ===")

    mtxL, distL, mtxR, distR, R1, R2, P1, P2, Q = load_calibration()
    imgL, imgR, left_path, right_path = load_first_pair()

    rectL, rectR, rectL_bgr = rectify_pair(imgL, imgR, mtxL, distL, mtxR, distR, R1, R2, P1, P2)

    print("\nUsing pair for pose:")
    print(" Left :", left_path)
    print(" Right:", right_path)

    imgp = click_points_on_image(rectL, n_points=len(OBJP_WORLD))
    Rwc, twc = compute_pose_from_markers(OBJP_WORLD, imgp, mtxL, distL)

    save_pose(Rwc, twc)

    print("\nSaved pose to:", POSE_SAVE)
    print("Rwc:\n", Rwc)
    print("twc:\n", twc.ravel())

def mode_run(color="red", show=True):
    print("\n=== MODE: run (detect object & output world coords) ===")
    print("Color:", color)

    Rwc, twc = load_pose()
    mtxL, distL, mtxR, distR, R1, R2, P1, P2, Q = load_calibration()

    imgL, imgR, left_path, right_path = load_first_pair()
    rectL, rectR, rectL_bgr = rectify_pair(imgL, imgR, mtxL, distL, mtxR, distR, R1, R2, P1, P2)

    disp = compute_disparity(rectL, rectR)
    points_3d = cv2.reprojectImageTo3D(disp, Q)  # Pc for each pixel (v,u)

    # Color detect on rectified LEFT BGR
    hsv = cv2.cvtColor(rectL_bgr, cv2.COLOR_BGR2HSV)
    mask = get_hsv_mask(hsv, color)
    centroid, mask_clean = find_largest_contour_centroid(mask)

    out_vis = rectL_bgr.copy()

    if centroid is None:
        print("No object found for that color (or too small).")
    else:
        u, v = centroid
        # Safety bounds
        h, w = rectL.shape[:2]
        u = np.clip(u, 0, w-1)
        v = np.clip(v, 0, h-1)

        Pc = points_3d[v, u, :]  # [Xc,Yc,Zc] in left camera frame

        # Validate depth (many stereo pixels can be invalid)
        Zc = Pc[2]
        if not np.isfinite(Zc) or Zc <= 0 or Zc > 10_000:
            print(f"Invalid depth at centroid pixel ({u},{v}). Try tuning disparity / choose another pixel.")
        else:
            Pw = cam_to_world(Pc, Rwc, twc)
            Xw, Yw, Zw = Pw

            print("\nObject centroid pixel (u,v):", (u, v))
            print("Pc (camera XYZ):", Pc)
            print("Pw (world  XYZ):", Pw)

            # Draw on image
            cv2.circle(out_vis, (u, v), 8, (0, 255, 0), -1)
            text = f"W: X={Xw:.3f}, Y={Yw:.3f}, Z={Zw:.3f}"
            cv2.putText(out_vis, text, (u+10, v-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # Save outputs always
    out_rectL = os.path.join(SCRIPT_DIR, "out_rectL.png")
    out_mask  = os.path.join(SCRIPT_DIR, f"out_mask_{color}.png")
    out_visp  = os.path.join(SCRIPT_DIR, f"out_world_{color}.png")

    cv2.imwrite(out_rectL, rectL)
    cv2.imwrite(out_mask, mask_clean)
    cv2.imwrite(out_visp, out_vis)

    print("\nSaved outputs:")
    print(" ", out_rectL)
    print(" ", out_mask)
    print(" ", out_visp)

    # Show (if display exists and user wants)
    has_display = ("DISPLAY" in os.environ) and (os.environ["DISPLAY"].strip() != "")
    if show and has_display:
        disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        cv2.namedWindow("Rectified Left (BGR)", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow("World Result", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Disparity (normalized)", cv2.WINDOW_NORMAL)

        cv2.imshow("Rectified Left (BGR)", rectL_bgr)
        cv2.imshow("Mask", mask_clean)
        cv2.imshow("World Result", out_vis)
        cv2.imshow("Disparity (normalized)", disp_vis)

        print("\nWindows opened. Press 'q' or ESC to quit.")
        while True:
            k = cv2.waitKey(30) & 0xFF
            if k == 27 or k == ord('q'):
                break
        cv2.destroyAllWindows()
    else:
        if not has_display:
            print("\nNo DISPLAY detected. Skipping cv2.imshow(). Open saved images instead.")

# -------------------------------------------------
# CLI
# -------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["pose", "run"], required=True,
                        help="pose = click 4 markers & save transform, run = detect object & print world coords")
    parser.add_argument("--color", default="red", choices=["red", "blue", "yellow", "green"],
                        help="color to detect in rectified left image")
    parser.add_argument("--no-gui", action="store_true", help="disable imshow even if display exists")
    args = parser.parse_args()

    # basic checks
    require_file(CALIB_FILE, "Calibration file not found")
    require_dir(PAIR_DIR, "Stereo pair folder not found")

    if args.mode == "pose":
        mode_pose()
    else:
        mode_run(color=args.color, show=(not args.no_gui))

if __name__ == "__main__":
    main()