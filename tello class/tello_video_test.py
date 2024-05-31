import tello
import time
import cv2

drone = tello.Tello()

time.sleep(1)

drone.start_sdk_mode(mode="local")
drone.streamon()

# frame_read = drone.get_frame_read()

# drone.takeoff()
# cv2.imwrite("picture.png", frame_read.frame)

# drone.land()

drone.receive_camera_image()