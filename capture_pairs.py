# capture pairs
import cv2, os, time

save_dir = "stereo_pairs"
os.makedirs(save_dir, exist_ok=True)

capL = cv2.VideoCapture(0)
capR = cv2.VideoCapture(2)

i = 0
print("Press 's' to save a pair, 'q' to quit.")

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()
    if not retL or not retR:
        print("Camera read failed")
        break

    cv2.imshow("Left", frameL)
    cv2.imshow("Right", frameR)

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
