import tello
import time
import cv2

tello = tello.Tello()

time.sleep(1)

tello.start_sdk_mode(mode="video")
tello.streamon()

try:

    while True:
        frame = tello.receive_camera_image()
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Keyboard Interrupt")
    tello.streamoff()