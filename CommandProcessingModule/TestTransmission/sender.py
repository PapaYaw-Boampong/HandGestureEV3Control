import socket

# Define the server address and port
server_address = ('', 59700)  # Listen on all available interfaces
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(server_address)
server_socket.listen(1)

print("Waiting for a connection from EV3...")

# Wait for a connection
connection, client_address = server_socket.accept()

try:
    print("Connected to ",client_address)
    
    # Receive the data from the EV3
    data = connection.recv(1024).decode('utf-8')
    print("Received: ", data)
    
    # If the command is 'beep', respond to EV3
    if data == 'beep':
        response = "Command received"
        connection.sendall(response.encode('utf-8'))
finally:
    connection.close()


