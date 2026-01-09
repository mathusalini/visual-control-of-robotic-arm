import cv2

url = "http://192.168.8.124:8080/video"  # <-- change to your phone's URL
cap = cv2.VideoCapture(url)

while True:
    ret, frame = cap.read()
    if not ret:
        print("No frame. Check Wi-Fi + URL.")
        break
    cv2.imshow("Phone Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
