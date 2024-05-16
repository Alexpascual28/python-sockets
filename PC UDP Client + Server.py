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

host = '' # '' # '0.0.0.0'
port = 9000 # 9000 # 8890

host2 = '0.0.0.0'
port2 = 8890

data_address = (host2, port2)

local_address = (host,port)
tello_address = ('192.168.10.1', 8889)

# Create a UDP client socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.bind(local_address) # Bind the client socket to the specified IP address and port
# client_socket.setblocking(False)

#data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#data_socket.bind(data_address)
#data_socket.setblocking(False)

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
                #data_socket.close()
                break

            # Send data to server
            msg = msg.encode(encoding="utf-8") 
            sent_local = client_socket.sendto(msg, tello_address)
            #sent_data = data_socket.sendto(msg, tello_address)

        except KeyboardInterrupt:
            print ('\n . . .\n')
            client_socket.close()
            #data_socket.close()
            break

def receive_from_server():
    while True:
        try:
            local_data, server = client_socket.recvfrom(1518)
            #state_data, server = data_socket.recvfrom(1518)
            print(local_data.decode(encoding="utf-8"))
            #print(state_data.decode(encoding="utf-8"))

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