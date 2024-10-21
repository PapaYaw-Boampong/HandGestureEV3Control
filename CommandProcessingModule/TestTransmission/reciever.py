#!/usr/bin/env python3

import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('', 59700))  # Ensure this is the same port as used in the PC code
server_socket.listen(1)
print("EV3 waiting for a connection...")

connection, client_address = server_socket.accept()
print("Connected to", client_address)

try:
    while True:
        data = connection.recv(1024).decode('utf-8')
        if data:
            print("Received:", data)
            connection.sendall("Command received".encode('utf-8'))
        else:
            break
finally:
    connection.close()







