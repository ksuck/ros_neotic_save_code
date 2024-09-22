import cv2

for i in range(3):
    cap = cv2.VideoCapture(1)
    if cap.isOpened():
        print(f"Camera {i} is available.")
        # ลองจับภาพจากกล้อง
        ret, frame = cap.read()
        if ret:
            print(f"Camera {i} is working.")
            cv2.imshow(f'Camera {i}', frame)
            cv2.waitKey(0)
        cap.release()
    else:
        print(f"Camera {i} is not available.")

cv2.destroyAllWindows()
