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

host = ''
port = 9000

local_address = (host,port)
tello_address = ('192.168.10.1', 8889)

# Create a UDP client socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# client_socket.bind(local_address) # Bind the client socket to the specified IP address and port

# Read data from user and send to the UDP server
def send_to_server():
    while True:
        try:
            # Read input
            msg = input("");

            # If data is null, break loop
            if not msg:
                break  

            # If 'end' command received, close client and exit
            if 'end' in msg:
                print ('...')
                client_socket.close()  
                break

            # Send data to server
            msg = msg.encode(encoding="utf-8") 
            sent = client_socket.sendto(msg, tello_address)

        except KeyboardInterrupt:
            print ('\n . . .\n')
            client_socket.close() 
            break

def receive_from_server():
    while True:
        try:
            data, server = client_socket.recvfrom(1518)
            print(data.decode(encoding="utf-8"))

        except Exception:
            print ('\nExit . . .\n')
            break

if __name__ == "__main__":

    print ('\r\n\r\nTello Python3 UDP Client.\r\n')

    print ('Tello commands: command takeoff land flip forward back left right \r\n       up down cw ccw speed speed?\r\n')
    print ('end -- quit demo.\r\n')

    # Create thread to receive data from server
    recieveThread = threading.Thread(target=receive_from_server)
    recieveThread.start()

    # Send thread to send data to server
    sendThread = threading.Thread(target=send_to_server)
    sendThread.start()