# echo-client.py

import socket
import sys

# HOST = "0.0.0.0" # "127.0.0.1"  # Standard loopback interface address (localhost)
# PORT = 8890 # 65432  # Port to listen on (non-privileged ports are > 1023)

tello_address = ('192.168.10.1', 8889)

# Check for the correct number of arguments
if len(sys.argv) <= 3:
    print ('\nTello Python3 UDP Client.\n')
    print(f"Usage: {sys.argv[0]} <host> <port> <message>\n")
    print ('Tello commands: command takeoff land flip forward back left right \r\n       up down cw ccw speed speed?\r\n')
    print ('end -- quit demo.\r\n')
    sys.exit(1)

host, port, message = sys.argv[1:4]
message = sys.argv[1]

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    # s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, int(port)))

    try:
        # If 'end' command received, close client and exit
        if 'end' in message:
            print ('...')
            sys.exit(1)

        # Send data to server
        msg = message.encode(encoding="utf-8") 
        sent = s.sendto(msg, tello_address)

        data, server = s.recvfrom(1518)
        print(data.decode(encoding="utf-8"))

    except KeyboardInterrupt:
        print ('\n . . .\n')