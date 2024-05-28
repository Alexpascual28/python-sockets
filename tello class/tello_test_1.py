import tello
import time

tello = tello.Tello()

time.sleep(1)

tello.start_sdk_mode(mode="video")
tello.takeoff()

while True:
    tello_state = tello.get_state()

    vertical_acceleration = tello_state.agz

    print(vertical_acceleration)