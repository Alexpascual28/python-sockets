#
# tello-client-server.py
#
# @author: Alejandro Pascual San Roman
#
# https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf
#

import threading
import logging
import socket
import sys
import time

from dataclasses import dataclass, fields

import cv2
import numpy as np
import subprocess
import av

from collections import deque
from threading import Thread, Lock
from typing import Optional, Union, Type, Dict

@dataclass
class TelloState:
    mid: str = "" # ID of Mission Pad detected. If no pad detected, mid = -1
    
    x: float = 0.0 # X coordinate of Pad. If no pad, "0"
    y: float = 0.0 # Y coordinate of Pad. If no pad, "0"
    z: float = 0.0 # Z coordinate of Pad. If no pad, "0"

    pitch: float = 0.0 # Degree of drone pitch
    roll: float = 0.0 # Degree of drone roll
    yaw: float = 0.0 # Degree of drone yaw

    vgx: float = 0.0 # Speed in x axis
    vgy: float = 0.0 # Speed in y axis
    vgz: float = 0.0 # Speed in z axis

    templ: float = 0.0 # Lowest temperature in Celsius
    temph: float = 0.0 # Highest temperature in Celsius

    tof: float = 0.0 # Time of Flight distance in CM
    h: float = 0.0 # Height in CM
    bat: float = 0.0 # Percentage of current battery level
    baro: float = 0.0 # Barometer measurement in CM
    time: float = 0.0 # Amount of time the motor has been used

    agx: float = 0.0 # Acceleration in the X axis
    agy: float = 0.0 # Acceleration in the Y axis
    agz: float = 0.0 # Acceleration in the Z axis

class TelloException(Exception):
    pass

class BackgroundFrameRead:
    """
    This class read frames using PyAV in background. Use
    backgroundFrameRead.frame to get the current frame.
    """

    def __init__(self, tello, address, with_queue = False, maxsize = 32):
        self.address = address
        self.lock = Lock()
        self.frame = np.zeros([300, 400, 3], dtype=np.uint8)
        self.frames = deque([], maxsize)
        self.with_queue = with_queue

        # Try grabbing frame with PyAV
        # According to issue #90 the decoder might need some time
        # https://github.com/damiafuentes/DJITelloPy/issues/90#issuecomment-855458905
        try:
            Tello.LOGGER.debug('trying to grab video frames...')
            self.container = av.open(self.address, timeout=(Tello.FRAME_GRAB_TIMEOUT, None))
        except av.error.ExitError:
            raise TelloException('Failed to grab video frames from video stream')

        self.stopped = False
        self.worker = Thread(target=self.update_frame, args=(), daemon=True)

    def start(self):
        """Start the frame update worker
        Internal method, you normally wouldn't call this yourself.
        """
        self.worker.start()

    def update_frame(self):
        """Thread worker function to retrieve frames using PyAV
        Internal method, you normally wouldn't call this yourself.
        """
        try:
            for frame in self.container.decode(video=0):
                if self.with_queue:
                    self.frames.append(np.array(frame.to_image()))
                else:
                    self.frame = np.array(frame.to_image())

                if self.stopped:
                    self.container.close()
                    break
        except av.error.ExitError:
            raise TelloException('Do not have enough frames for decoding, please try again or increase video fps before get_frame_read()')
    
    def get_queued_frame(self):
        """
        Get a frame from the queue
        """
        with self.lock:
            try:
                return self.frames.popleft()
            except IndexError:
                return None

    @property
    def frame(self):
        """
        Access the frame variable directly
        """
        if self.with_queue:
            return self.get_queued_frame()

        with self.lock:
            return self._frame

    @frame.setter
    def frame(self, value):
        with self.lock:
            self._frame = value

    def stop(self):
        """Stop the frame update worker
        Internal method, you normally wouldn't call this yourself.
        """
        self.stopped = True

class Tello:
    """Python wrapper to interact with the Ryze Tello drone using the official Tello api.
    Tello API documentation:
    [1.3](https://dl-cdn.ryzerobotics.com/downloads/tello/20180910/Tello%20SDK%20Documentation%20EN_1.3.pdf),
    [2.0 with EDU-only commands](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)
    """
    # Send and receive commands, client socket
    RESPONSE_TIMEOUT = 7  # in seconds
    TAKEOFF_TIMEOUT = 20  # in seconds
    FRAME_GRAB_TIMEOUT = 5
    TIME_BTW_COMMANDS = 0.1  # in seconds
    TIME_BTW_RC_CONTROL_COMMANDS = 0.001  # in seconds
    RETRY_COUNT = 3  # number of retries after a failed command
    TELLO_IP = '192.168.10.1'  # Tello IP address

    # Video stream, server socket
    VS_UDP_IP = '0.0.0.0'
    DEFAULT_VS_UDP_PORT = 11111
    VS_UDP_PORT = DEFAULT_VS_UDP_PORT

    CONTROL_UDP_PORT = 8889
    STATE_UDP_PORT = 8890

    # Constants for video settings
    BITRATE_AUTO = 0
    BITRATE_1MBPS = 1
    BITRATE_2MBPS = 2
    BITRATE_3MBPS = 3
    BITRATE_4MBPS = 4
    BITRATE_5MBPS = 5
    RESOLUTION_480P = 'low'
    RESOLUTION_720P = 'high'
    FPS_5 = 'low'
    FPS_15 = 'middle'
    FPS_30 = 'high'
    CAMERA_FORWARD = 0
    CAMERA_DOWNWARD = 1

    # Set up logger
    HANDLER = logging.StreamHandler()
    FORMATTER = logging.Formatter('[%(levelname)s] %(filename)s - %(lineno)d - %(message)s')
    HANDLER.setFormatter(FORMATTER)

    LOGGER = logging.getLogger('djitellopy')
    LOGGER.addHandler(HANDLER)
    LOGGER.setLevel(logging.INFO)
    # Use Tello.LOGGER.setLevel(logging.<LEVEL>) in YOUR CODE
    # to only receive logs of the desired level and higher

    # Conversion functions for state protocol fields
    INT_STATE_FIELDS = (
        # Tello EDU with mission pads enabled only
        'mid', 'x', 'y', 'z',
        # 'mpry': (custom format 'x,y,z')
        # Common entries
        'pitch', 'roll', 'yaw',
        'vgx', 'vgy', 'vgz',
        'templ', 'temph',
        'tof', 'h', 'bat', 'time'
    )
    FLOAT_STATE_FIELDS = ('baro', 'agx', 'agy', 'agz')

    state_field_converters: Dict[str, Union[Type[int], Type[float]]]
    state_field_converters = {key : int for key in INT_STATE_FIELDS}
    state_field_converters.update({key : float for key in FLOAT_STATE_FIELDS})

    # VideoCapture object
    background_frame_read: Optional['BackgroundFrameRead'] = None

    stream_on = False
    is_flying = False

    # PRIVATE MEMBERS

    def __init__(self):
        self.tello_address = ('192.168.10.1', 8889) # client

        local_address = ('', 9000) # server
        state_address = ('0.0.0.0', 8890) # server
        video_address = ('0.0.0.0', 11111) # server

        self.server_socket = ''

        self.local_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.local_socket.bind(local_address)

        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.data_socket.bind(state_address)

        #self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.video_socket.bind(video_address)

        self.data_buffersize = 1518
        self.video_buffersize = 2048

        # Start ffmpeg subprocess to decode the video stream
        self.ffmpeg_process = subprocess.Popen([
            'ffmpeg',
            '-i', 'udp://0.0.0.0:11111',  # Input from UDP stream
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        self.tello_state = TelloState()

    def _send_command(self, message):
        # If 'end' command received, close client and exit
        if 'end' in message:
            print ('...')
            self._close_sockets()

        # Send data to server
        message = message.encode(encoding="utf-8") 
        sent = self.server_socket.sendto(message, self.tello_address)

    def _close_sockets(self):
        print("Closing all sockets...")
        self.local_socket.close()
        self.data_socket.close()
        self.video_socket.close()
        print("Done.\n")

    def _read_socket(self, socket, buffersize):
        data, server = socket.recvfrom(buffersize)
        return data.decode(encoding="utf-8")

    # CONTROL COMMANDS

    def start_sdk_mode(self, mode="local"):
        print("Initialising SDK Mode...")
        self.server_socket = self.data_socket if mode=="state" else self.local_socket # else self.video_socket if mode=="video" else self.local_socket

        self._send_command("command")
        response = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Response: {response}")

    def end(self):
        print("Ending robot...")
        self.land()
        self._send_command("end")

    def takeoff(self):
        self._send_command("takeoff")
        response = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Response: {response}")

    def land(self):
        self._send_command("land")
        response = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Response: {response}")

    def streamon(self):
        self._send_command("streamon")
        response = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Response: {response}")

    def streamoff(self):
        self._send_command("streamoff")
        response = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Response: {response}")

    def emergency(self):
        self._send_command("emergency")
        response = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Response: {response}")

    def up(self, distance):
        if distance >= 20 and distance <= 500:
            self._send_command(f"up {distance}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Value must be between 20 and 500")

    def down(self, distance):
        if distance >= 20 and distance <= 500:
            self._send_command(f"down {distance}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Value must be between 20 and 500")

    def left(self, distance):
        if distance >= 20 and distance <= 500:
            self._send_command(f"left {distance}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Value must be between 20 and 500")

    def right(self, distance):
        if distance >= 20 and distance <= 500:
            self._send_command(f"right {distance}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Value must be between 20 and 500")

    def forward(self, distance):
        if distance >= 20 and distance <= 500:
            self._send_command(f"forward {distance}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Value must be between 20 and 500")

    def back(self, distance):
        if distance >= 20 and distance <= 500:
            self._send_command(f"back {distance}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Value must be between 20 and 500")

    def rotate_clockwise(self, angle):
        if angle >= 1 and angle <= 360:
            self._send_command(f"cw {angle}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Value must be between 1 and 360")

    def rotate_counterclockwise(self, angle):
        if angle >= 1 and angle <= 360:
            self._send_command(f"ccw {angle}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Value must be between 1 and 360")

    def flip(self, direction):
        possible_directions = {"left": "l", "right": "r", "forward": "f", "back": "b"}

        if direction in possible_directions:
            self._send_command(f"flip {possible_directions[direction]}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print(f"Direction value must be one of the following: {possible_directions.keys()}")

    def go(self, x, y, z, speed):
        if x >= 20 and x <= 500 and y >= 20 and y <= 500 and z >= 20 and z <= 500 and speed >= 10 and speed <= 100:
            self._send_command(f"go {x} {y} {z} {speed}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Values must be between 20 and 500 for x, y, z and between 10 and 100 for speed")

    def go(self, x, y, z, speed, mid):
        if x >= 20 and x <= 500 and y >= 20 and y <= 500 and z >= 20 and z <= 500 and speed >= 10 and speed <= 100:
            self._send_command(f"go {x} {y} {z} {speed} m{mid}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Values must be between 20 and 500 for x, y, z and between 10 and 100 for speed")

    def stop(self):
        self._send_command("stop")
        response = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Response: {response}")

    def curve(self, x1, y1, z1, x2, y2, z2, speed):
        if x1 >= 20 and x1 <= 500 and y1 >= 20 and y1 <= 500 and z1 >= 20 and z1 <= 500 and x2 >= 20 and x2 <= 500 and y2 >= 20 and y2 <= 500 and z2 >= 20 and z2 <= 500 and speed >= 10 and speed <= 60:
            self._send_command(f"curve {x1} {y1} {z1} {x2} {y2} {z2} {speed}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Values must be between 20 and 500 for x1, y1, z1, x2, y2, z2 and between 10 and 60 for speed")

    def curve(self, x1, y1, z1, x2, y2, z2, speed, mid):
        if x1 >= 20 and x1 <= 500 and y1 >= 20 and y1 <= 500 and z1 >= 20 and z1 <= 500 and x2 >= 20 and x2 <= 500 and y2 >= 20 and y2 <= 500 and z2 >= 20 and z2 <= 500 and speed >= 10 and speed <= 60:
            self._send_command(f"curve {x1} {y1} {z1} {x2} {y2} {z2} {speed} m{mid}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Values must be between 20 and 500 for x1, y1, z1, x2, y2, z2 and between 10 and 60 for speed")

    def jump(self, x, y, z, speed, yaw, mid1, mid2):
        if x >= 20 and x <= 500 and y >= 20 and y <= 500 and z >= 20 and z <= 500 and speed >= 10 and speed <= 100 and yaw >= 1 and yaw <= 360:
            self._send_command(f"jump {x} {y} {z} {speed} {yaw} m{mid1} m{mid2}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Values must be between 20 and 500 for x, y, z, between 10 and 100 for speed and between 1 and 360 for yaw")

    # SET COMMANDS

    def set_speed(self, speed):
        if speed >= 10 and speed <= 100:
            self._send_command(f"speed {speed}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Speed value must be between 10 and 100")

    def set_rc_control(self, a, b, c, d):
        if a >= -100 and a <= 100 and b >= -100 and b <= 100 and c >= -100 and c <= 100 and d >= -100 and d <= 100:
            self._send_command(f"rc {a} {b} {c} {d}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Values must be between -100 and 100 for a, b, c, d")

    def set_wifi(self, ssid, password):
        self._send_command(f"wifi {ssid} {password}")
        response = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Response: {response}")

    def set_mission_pad_detection(self, on_off):
        if on_off == "on":
            self._send_command(f"mon")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        elif on_off == "off":
            self._send_command(f"moff")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print("Value must be 'on' or 'off'")

    def set_mission_pad_detection_direction(self, direction):
        possible_directions = {"forward": "0", "downward": "1", "both": "2"}

        if direction in possible_directions:
            self._send_command(f"mdirection {possible_directions[direction]}")
            response = self._read_socket(self.server_socket, self.data_buffersize)
            print(f"Response: {response}")
        else:
            print(f"Direction value must be one of the following: {possible_directions.keys()}")

    def set_station_mode(self, ssid, password):
        self._send_command(f"ap {ssid} {password}")
        response = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Response: {response}")

    # READ COMMANDS

    def read_speed(self):
        self._send_command("speed?")
        speed = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Speed: {speed}")
        return speed
    
    def read_battery(self):
        self._send_command("battery?")
        battery = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Battery: {battery}")
        return battery
    
    def read_flight_time(self):
        self._send_command("time?")
        flight_time = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Flight Time: {flight_time}")
        return flight_time
    
    def read_wifi_snr(self):
        self._send_command("wifi?")
        wifi_snr = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Wifi SNR: {wifi_snr}")
        return wifi_snr
    
    def read_sdk_version(self):
        self._send_command("sdk?")
        sdk_version = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"SDK version: {sdk_version}")
        return sdk_version
    
    def read_serial_number(self):
        self._send_command("sn?")
        serial_number = self._read_socket(self.server_socket, self.data_buffersize)
        print(f"Tello Serial Number: {serial_number}")
        return serial_number
    
    # VIDEO FEED
    def get_udp_video_address(self) -> str:
        """Internal method, you normally wouldn't call this youself.
        """
        address_schema = 'udp://@{ip}:{port}'  # + '?overrun_nonfatal=1&fifo_size=5000'
        address = address_schema.format(ip=self.VS_UDP_IP, port=self.VS_UDP_PORT)
        return address

    def get_frame_read(self, with_queue = False, max_queue_len = 32) -> 'BackgroundFrameRead':
        """Get the BackgroundFrameRead object from the camera drone. Then, you just need to call
        backgroundFrameRead.frame to get the actual frame received by the drone.
        Returns:
            BackgroundFrameRead
        """
        if self.background_frame_read is None:
            address = self.get_udp_video_address()
            self.background_frame_read = BackgroundFrameRead(self, address, with_queue, max_queue_len)
            self.background_frame_read.start()
        return self.background_frame_read

    def receive_camera_image(self):
        cv2.namedWindow('Camera Image', cv2.WINDOW_NORMAL)

        print("Receiving video feed...")

        while True:
            try:
                # Read raw video frame from ffmpeg subprocess
                raw_frame = self.ffmpeg_process.stdout.read(1280 * 720 * 3)  # Assuming 720p video stream

                print(len(raw_frame))

                if len(raw_frame) != 1280 * 720 * 3:
                    print("Incomplete frame received")
                    continue

                # Convert raw frame to numpy array
                frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((720, 1280, 3))

                # Display the frame
                cv2.imshow('Camera Image', frame)
                cv2.waitKey(1)

            except KeyboardInterrupt:
                break
            except Exception as e:
                print("Error receiving video feed:", e)
                continue

        # Close the video socket
        # self.video_socket.close()
        cv2.destroyAllWindows()
        self.ffmpeg_process.terminate()
    
    # TELLO STATE

    def get_state(self):
        if self.server_socket == self.data_socket:
            state_message = self._read_socket(self.server_socket, self.data_buffersize)

            # mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:0;roll:0;yaw:-5;vgx:0;vgy:0;vgz:0;
            # templ:86;temph:88;tof:10;h:0;bat:78;baro:101.26;time:0;agx:-9.00;agy:-4.00;agz:-997.00;

            state_list = state_message.split(";")

            for item in range(len(state_list)):
                state_list[item] = state_list[item].split(":")

            i = 0
            for field in fields(self.tello_state):
                setattr(self.tello_state, field.name, state_list[i][1])
                i = i + 1

            # print(f"Tello State: {self.tello_state}")
            return self.tello_state
        else:
            print("Drone server socket must be pointed to state socket to get the Tello State. Returning last known state.")
            return self.tello_state
        