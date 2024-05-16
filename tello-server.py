# echo-server.py

import socket

HOST = "0.0.0.0"  # '192.168.10.1' # "0.0.0.0" # "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 8890 # 8889 # 8890 # 65432  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    print ('\r\n\r\nTello Python3 UDP Server.\r\n')

    s.bind((HOST, PORT))

    try:
        data, server = s.recvfrom(1518)
        print(data.decode(encoding="utf-8"))

    except KeyboardInterrupt:
        print ('\nExit . . .\n')