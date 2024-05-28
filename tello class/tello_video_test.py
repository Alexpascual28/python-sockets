import tello
import time

tello = tello.Tello()

time.sleep(1)

tello.start_sdk_mode(mode="video")
tello.streamon()

tello.receive_camera_image()