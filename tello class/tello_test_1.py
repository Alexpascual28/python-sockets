import tello
import time

tello = tello.Tello()

time.sleep(1)

tello.start_sdk_mode(receive_state=True)
tello.takeoff()

while True:
    tello_state = tello.get_state()

    vertical_acceleration = tello_state.agz

    print(vertical_acceleration)