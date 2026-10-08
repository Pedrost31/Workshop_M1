import cv2

for i in range(4):
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    if cap.isOpened():
        ok, frame = cap.read()
        if ok:
            print(f"Caméra {i} : OK ({frame.shape[1]}x{frame.shape[0]})")
            cv2.imshow(f"Caméra {i} - appuyez sur une touche", frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    cap.release()