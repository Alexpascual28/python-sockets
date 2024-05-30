import tello
import time
import cv2

tello = tello.Tello()

time.sleep(1)

tello.start_sdk_mode(mode="state")

while True:
    z_acceleration = tello.get_state().agz

    print(f"agz: {z_acceleration}")
