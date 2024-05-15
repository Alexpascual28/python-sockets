list = ['multiconn-client.py', '127.0.0.1', 65432, 2, 'message1', 'message2', 'message3']

host, port, num_conns = list[1:4]
messages = [str.encode(message) for message in list[4:]]
print(f"Details: {host}, {port}, {num_conns}")
print(f"Messages: {messages}")