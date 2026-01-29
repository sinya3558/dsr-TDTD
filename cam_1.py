import cv2

cap = cv2.VideoCapture(2)  # ← /dev/video2
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임 못 읽음")
        break

    cv2.imshow("C270 Fixed Cam", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
