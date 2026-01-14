import cv2

capL = cv2.VideoCapture(0)   # Camera on usb-...-1
capR = cv2.VideoCapture(2)   # Camera on usb-...-2

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()

    if retL:
        cv2.imshow("Left Camera", frameL)
    if retR:
        cv2.imshow("Right Camera", frameR)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
capL.release()
capR.release()
cv2.destroyAllWindows()
