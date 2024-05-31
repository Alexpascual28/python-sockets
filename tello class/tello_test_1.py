import tello
import time
import cv2

drone = tello.Tello()

time.sleep(1)

drone.start_sdk_mode(mode="state")

while True:
    z_acceleration = drone.get_state().agz

    print(f"agz: {z_acceleration}")
