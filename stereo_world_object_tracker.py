#!/usr/bin/env python3
import cv2
import numpy as np
import pickle
import glob
import os
import sys
import argparse

# ----------------------------
# Paths
# ----------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PAIR_DIR   = os.path.join(SCRIPT_DIR, "Stereo_pairs_2")
CALIB_FILE = os.path.join(SCRIPT_DIR, "stereo_calibration_2.pkl")
POSE_SAVE  = os.path.join(SCRIPT_DIR, "cam_to_world.npz")

# ----------------------------
# EDIT THIS: marker world coordinates (meters)
# Order MUST match click order.
# ----------------------------
OBJP_WORLD = np.array([
    [0.00, 0.00, 0.0],   # Marker 1
    [0.945, 0.00, 0.0],   # Marker 2
    [0.00, 0.49, 0.0],   # Marker 3
    [0.594, 0.787, 0.0],   # Marker 4
], dtype=np.float64)

# ----------------------------
# StereoSGBM params (you can tune)
# ----------------------------
NUM_DISP = 16 * 8
BLOCK   = 7

def require_file(path, msg):
    if not os.path.exists(path):
        raise FileNotFoundError(f"{msg}: {path}")

def load_calibration():
    require_file(CALIB_FILE, "Calibration file not found")
    with open(CALIB_FILE, "rb") as f:
        calib = pickle.load(f)

    mtxL, distL = calib["mtxL"], calib["distL"]
    mtxR, distR = calib["mtxR"], calib["distR"]
    R1, R2 = calib["R1"], calib["R2"]
    P1, P2 = calib["P1"], calib["P2"]
    Q = calib["Q"]
    return mtxL, distL, mtxR, distR, R1, R2, P1, P2, Q

def build_rectify_maps(mtxL, distL, mtxR, distR, R1, R2, P1, P2, image_size):
    w, h = image_size
    map1L, map2L = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, (w, h), cv2.CV_16SC2)
    map1R, map2R = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, (w, h), cv2.CV_16SC2)
    return (map1L, map2L, map1R, map2R)

def rectify_frames(frameL, frameR, map1L, map2L, map1R, map2R):
    rectL_bgr = cv2.remap(frameL, map1L, map2L, cv2.INTER_LINEAR)
    rectR_bgr = cv2.remap(frameR, map1R, map2R, cv2.INTER_LINEAR)
    rectL = cv2.cvtColor(rectL_bgr, cv2.COLOR_BGR2GRAY)
    rectR = cv2.cvtColor(rectR_bgr, cv2.COLOR_BGR2GRAY)
    return rectL, rectR, rectL_bgr, rectR_bgr

def compute_disparity(rectL, rectR):
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=NUM_DISP,
        blockSize=BLOCK,
        P1=8 * BLOCK * BLOCK,
        P2=32 * BLOCK * BLOCK,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
        disp12MaxDiff=1,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    disp = stereo.compute(rectL, rectR).astype(np.float32) / 16.0
    return disp

def click_points_on_gray(gray, n_points):
    has_display = ("DISPLAY" in os.environ) and (os.environ["DISPLAY"].strip() != "")
    if not has_display:
        raise RuntimeError("No DISPLAY detected. Need GUI to click points.")

    show = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    clicked = []
    win = "Click 4 markers on LIVE rectified LEFT (press q when done)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def mouse_cb(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked.append([x, y])
            cv2.circle(show, (x, y), 6, (0, 255, 0), -1)
            cv2.putText(show, str(len(clicked)), (x+8, y-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.imshow(win, show)

    cv2.setMouseCallback(win, mouse_cb)

    print(f"Click {n_points} markers in SAME ORDER as OBJP_WORLD, then press 'q'")
    while True:
        cv2.imshow(win, show)
        k = cv2.waitKey(20) & 0xFF
        if k == ord('q'):
            break

    cv2.destroyWindow(win)

    if len(clicked) != n_points:
        raise ValueError(f"Need {n_points} clicks, got {len(clicked)}")

    return np.array(clicked, dtype=np.float64).reshape(-1, 1, 2)

def compute_pose_from_markers(objp_world, imgp_pixels, K, dist):
    ok, rvec, tvec = cv2.solvePnP(objp_world, imgp_pixels, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed")

    Rcw, _ = cv2.Rodrigues(rvec)  # world->camera
    Rwc = Rcw.T
    twc = -Rwc @ tvec
    return Rwc, twc

def save_pose(Rwc, twc):
    np.savez(POSE_SAVE, Rwc=Rwc, twc=twc)

def load_pose():
    if not os.path.exists(POSE_SAVE):
        raise FileNotFoundError("cam_to_world.npz not found. Run --mode pose_live first.")
    d = np.load(POSE_SAVE)
    return d["Rwc"], d["twc"].reshape(3, 1)

def cam_to_world(Pc_xyz, Rwc, twc):
    Pc = np.array(Pc_xyz, dtype=np.float64).reshape(3, 1)
    Pw = Rwc @ Pc + twc
    return Pw.reshape(3,)

# ----------------------------
# Simple color detection
# ----------------------------
def hsv_mask(hsv, color):
    if color == "red":
        lower1 = np.array([0, 120, 60]); upper1 = np.array([10, 255, 255])
        lower2 = np.array([170,120, 60]); upper2 = np.array([179,255, 255])
        return cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1),
                              cv2.inRange(hsv, lower2, upper2))
    if color == "blue":
        return cv2.inRange(hsv, np.array([95,120,60]), np.array([135,255,255]))
    if color == "yellow":
        return cv2.inRange(hsv, np.array([20,120,80]), np.array([35,255,255]))
    if color == "green":
        return cv2.inRange(hsv, np.array([40,80,60]), np.array([85,255,255]))
    raise ValueError("color must be red/blue/yellow/green")

def centroid_from_mask(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 150:
        return None, mask
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, mask
    cx = int(M["m10"]/M["m00"])
    cy = int(M["m01"]/M["m00"])
    return (cx, cy), mask

# ----------------------------
# Open two cameras (common indices: 0 & 1)
# ----------------------------
def open_stereo_cams(left_index=0, right_index=1):
    capL = cv2.VideoCapture(left_index)
    capR = cv2.VideoCapture(right_index)

    if not capL.isOpened():
        raise RuntimeError(f"Cannot open LEFT camera index {left_index}")
    if not capR.isOpened():
        raise RuntimeError(f"Cannot open RIGHT camera index {right_index}")

    return capL, capR

# ----------------------------
# MODE: pose_live
# ----------------------------
def pose_live(left_index, right_index):
    print("\n=== MODE: pose_live (click 4 markers on LIVE rectified LEFT) ===")
    mtxL, distL, mtxR, distR, R1, R2, P1, P2, Q = load_calibration()

    capL, capR = open_stereo_cams(left_index, right_index)

    # Read one frame to get image size and build rectify maps
    retL, frameL = capL.read()
    retR, frameR = capR.read()
    if not retL or not retR:
        capL.release(); capR.release()
        raise RuntimeError("Failed to read initial frames from cameras")

    h, w = frameL.shape[:2]
    map1L, map2L, map1R, map2R = build_rectify_maps(mtxL, distL, mtxR, distR, R1, R2, P1, P2, (w, h))

    # Grab a fresh rectified frame for clicking
    retL, frameL = capL.read()
    retR, frameR = capR.read()
    if not retL or not retR:
        capL.release(); capR.release()
        raise RuntimeError("Failed to read frames for clicking")

    rectL, rectR, rectL_bgr, rectR_bgr = rectify_frames(frameL, frameR, map1L, map2L, map1R, map2R)

    # Click points on rectified LEFT (gray)
    imgp = click_points_on_gray(rectL, n_points=len(OBJP_WORLD))

    # Compute pose and save
    Rwc, twc = compute_pose_from_markers(OBJP_WORLD, imgp, mtxL, distL)
    save_pose(Rwc, twc)

    capL.release()
    capR.release()

    print("\nSaved pose to:", POSE_SAVE)
    print("Rwc:\n", Rwc)
    print("twc:\n", twc.ravel())

# ----------------------------
# MODE: run_live
# ----------------------------
def run_live(left_index, right_index, color, no_gui=False):
    print("\n=== MODE: run_live (LIVE object world coordinates) ===")
    print("Color:", color)

    Rwc, twc = load_pose()
    mtxL, distL, mtxR, distR, R1, R2, P1, P2, Q = load_calibration()

    capL, capR = open_stereo_cams(left_index, right_index)

    # Build rectify maps from first frame size
    retL, frameL = capL.read()
    retR, frameR = capR.read()
    if not retL or not retR:
        capL.release(); capR.release()
        raise RuntimeError("Failed to read initial frames from cameras")

    h, w = frameL.shape[:2]
    map1L, map2L, map1R, map2R = build_rectify_maps(mtxL, distL, mtxR, distR, R1, R2, P1, P2, (w, h))

    has_display = ("DISPLAY" in os.environ) and (os.environ["DISPLAY"].strip() != "")
    show = has_display and (not no_gui)

    print("Press 'q' to quit live run.")

    while True:
        retL, frameL = capL.read()
        retR, frameR = capR.read()
        if not retL or not retR:
            print("Frame grab failed. Exiting.")
            break

        rectL, rectR, rectL_bgr, rectR_bgr = rectify_frames(frameL, frameR, map1L, map2L, map1R, map2R)

        disp = compute_disparity(rectL, rectR)
        points_3d = cv2.reprojectImageTo3D(disp, Q)

        hsv = cv2.cvtColor(rectL_bgr, cv2.COLOR_BGR2HSV)
        mask = hsv_mask(hsv, color)
        centroid, mask_clean = centroid_from_mask(mask)

        out = rectL_bgr.copy()

        if centroid is not None:
            u, v = centroid
            u = int(np.clip(u, 0, w-1))
            v = int(np.clip(v, 0, h-1))

            Pc = points_3d[v, u, :]
            Zc = Pc[2]

            if np.isfinite(Zc) and Zc > 0:
                Pw = cam_to_world(Pc, Rwc, twc)
                Xw, Yw, Zw = Pw

                cv2.circle(out, (u, v), 8, (0,255,0), -1)
                cv2.putText(out, f"W: {Xw:.3f},{Yw:.3f},{Zw:.3f}",
                            (u+10, v-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                # Print live world coords
                print(f"Pixel({u},{v}) Pc={Pc}  Pw=[{Xw:.3f},{Yw:.3f},{Zw:.3f}]")

        if show:
            disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            cv2.imshow("Rectified Left", rectL_bgr)
            cv2.imshow("Mask", mask_clean)
            cv2.imshow("World Result", out)
            cv2.imshow("Disparity", disp_vis)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q') or k == 27:
                break
        else:
            # No GUI: allow quit with Ctrl+C
            pass

    capL.release()
    capR.release()
    if show:
        cv2.destroyAllWindows()

# ----------------------------
# Main
# ----------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", required=True, choices=["pose", "run", "pose_live", "run_live"],
                        help="pose/run use saved images, pose_live/run_live use webcams")
    parser.add_argument("--color", default="red", choices=["red","blue","yellow","green"])
    parser.add_argument("--left-cam", type=int, default=0, help="Left camera index")
    parser.add_argument("--right-cam", type=int, default=1, help="Right camera index")
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args()

    require_file(CALIB_FILE, "Calibration file not found")

    if args.mode == "pose_live":
        pose_live(args.left_cam, args.right_cam)
    elif args.mode == "run_live":
        run_live(args.left_cam, args.right_cam, args.color, no_gui=args.no_gui)
    else:
        # keep your old image-pair modes if you still want them
        print("Use pose_live/run_live for live camera. (pose/run not implemented in this live-only file.)")
        print("Run: python3 stereo_world_object_tracker_live.py --mode pose_live")
        print("Then: python3 stereo_world_object_tracker_live.py --mode run_live --color red")

if __name__ == "__main__":
    main()