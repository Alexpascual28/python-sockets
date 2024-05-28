#
# Tello Python3 PC UDP Client 
#
# http://www.ryzerobotics.com/
#
# 20/03/2024
#

import threading 
import socket
import sys
import time

host = '0.0.0.0'
port = 8890

local_address = (host,port)

# Create a UDP server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(local_address) # Bind the client socket to the specified IP address and port

def receive_from_client():
    while True:
        try:
            data, client = server_socket.recvfrom(1518)
            print(data.decode(encoding="utf-8"))

        except Exception:
            print ('\nExit . . .\n')
            break

if __name__ == "__main__":

    print ('\r\n\r\nTello Python3 UDP Server.\r\n')

    # Create thread to receive data from client
    recieveThread = threading.Thread(target=receive_from_client)
    recieveThread.start()