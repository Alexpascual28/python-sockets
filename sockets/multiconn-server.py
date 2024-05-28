# multiconn-server.py
# Multi-Connection Server

import sys
import socket
import selectors
import types

sel = selectors.DefaultSelector()

def accept_wrapper(sock):
    # Because the listening socket was registered for the event selectors.EVENT_READ, it should be ready to read.
    # You call sock.accept() and then call conn.setblocking(False) to put the socket in non-blocking mode.
    conn, addr = sock.accept()  # Should be ready to read
    print(f"Accepted connection from {addr}")
    conn.setblocking(False)

    # Next, you create an object to hold the data that you want included along with the socket using a SimpleNamespace.
    data = types.SimpleNamespace(addr=addr, inb=b"", outb=b"")

    # Because you want to know when the client connection is ready for reading and writing, both of those events are set with the bitwise OR operator
    events = selectors.EVENT_READ | selectors.EVENT_WRITE

    # The events mask, socket, and data objects are then passed to sel.register().
    sel.register(conn, events, data=data)

# How a client connection is handled when it’s ready
# This is the heart of the simple multi-connection server.
# key is the namedtuple returned from .select() that contains the socket object (fileobj) and data object.
# mask contains the events that are ready.
def service_connection(key, mask):
    sock = key.fileobj
    data = key.data

    # If the socket is ready for reading, then mask & selectors.EVENT_READ will evaluate to True, so sock.recv() is called.
    # Any data that’s read is appended to data.outb so that it can be sent later.
    if mask & selectors.EVENT_READ:
        recv_data = sock.recv(1024)  # Should be ready to read
        if recv_data:
            data.outb += recv_data

        # Note the else: block to check if no data is received.
        # If no data is received, then the client has closed their socket, so the server should too.
        else:
            print(f"Closing connection to {data.addr}")

            # call sel.unregister() before closing, so it’s no longer monitored by .select().
            sel.unregister(sock)
            sock.close()
    
    # When the socket is ready for writing, which should always be the case for a healthy socket,
    # any received data stored in data.outb is echoed to the client using sock.send().
    # The bytes sent are then removed from the send buffer
    if mask & selectors.EVENT_WRITE:
        if data.outb:
            print(f"Echoing {data.outb!r} to {data.addr}")

            # The .send() method returns the number of bytes sent.
            # This number can then be used with slice notation on the .outb buffer to discard the bytes sent.
            sent = sock.send(data.outb)  # Should be ready to write
            data.outb = data.outb[sent:]

# The first thing you do is check that the correct number of arguments are passed to the script.
if len(sys.argv) != 3:
    print(f"Usage: {sys.argv[0]} <host> <port>")
    sys.exit(1)

# Sets up the listening socket
host, port = sys.argv[1], int(sys.argv[2])
lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
lsock.bind((host, port))
lsock.listen()
print(f"Listening on {(host, port)}")

# call to lsock.setblocking(False) to configure the socket in non-blocking mode. Calls made to this socket will no longer block.
# When it’s used with sel.select(), as you’ll see below, you can wait for events on one or more sockets and then read and write data when it’s ready.
lsock.setblocking(False)

# sel.register() registers the socket to be monitored with sel.select() for the events that you’re interested in.
# For the listening socket, you want read events: selectors.EVENT_READ.
sel.register(lsock, selectors.EVENT_READ, data=None) 

# Event loop
try:
    while True:

        # sel.select(timeout=None) blocks until there are sockets ready for I/O. It returns a list of tuples, one for each socket.
        events = sel.select(timeout=None)

        # Each tuple contains a key and a mask.
        # The key is a SelectorKey namedtuple that contains a fileobj attribute.
        # key.fileobj is the socket object, and mask is an event mask of the operations that are ready.
        for key, mask in events:

            # If key.data is None, then you know it’s from the listening socket and you need to accept the connection.
            if key.data is None:

                # You’ll call your own accept_wrapper() function to get the new socket object and register it with the selector.
                accept_wrapper(key.fileobj)
            
            # If key.data is not None, then you know it’s a client socket that’s already been accepted, and you need to service it.
            else:

                # service_connection() is then called with key and mask as arguments, and that’s everything you need to operate on the socket.
                service_connection(key, mask)

except KeyboardInterrupt:
    print("Caught keyboard interrupt, exiting")

finally:
    sel.close()