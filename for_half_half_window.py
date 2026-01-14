# capture pairs (half-half window)
import cv2, os, time
import numpy as np

save_dir = "stereo_pairs"
os.makedirs(save_dir, exist_ok=True)

capL = cv2.VideoCapture(0)
capR = cv2.VideoCapture(2)

# Desired size for each camera frame
W, H = 640, 480

i = 0
print("Press 's' to save a pair, 'q' to quit.")

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()

    if not retL or not retR:
        print("Camera read failed")
        break

    # Resize frames
    frameL = cv2.resize(frameL, (W, H))
    frameR = cv2.resize(frameR, (W, H))

    # Combine frames side-by-side
    combined = np.hstack((frameL, frameR))

    cv2.imshow("Stereo View (Left | Right)", combined)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        left_path  = os.path.join(save_dir, f"left_{i:03d}.png")
        right_path = os.path.join(save_dir, f"right_{i:03d}.png")

        cv2.imwrite(left_path, frameL)
        cv2.imwrite(right_path, frameR)

        print("Saved", left_path, right_path)
        i += 1
        time.sleep(0.2)

    elif key == ord('q'):
        break

capL.release()
capR.release()
cv2.destroyAllWindows()
