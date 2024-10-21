import socket

class EV3CommandSender:
    def __init__(self, sender_port=59700):
        """
        Initializes the EV3CommandSender with the port to listen on.

        Parameters:
        - sender_port (int): The port on which the sender will bind and listen for connections (default is 59700).
        """
        self.sender_port = sender_port

    def send_command(self, command):
        """
        Waits for the EV3 to connect, then sends the command and waits for a response.

        Parameters:
        - command (str): The command string to send to the EV3.
        """
        command = command.strip()

        # Create a server socket to listen for the EV3 connection
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('', self.sender_port))
        server_socket.listen(1)

        print("Waiting for a connection on port " + str(self.sender_port) + "...")

        try:
            # Wait for EV3 to connect
            connection, client_address = server_socket.accept()
            print("Connected to EV3 at " + str(client_address))

            try:
                # Send the command to the EV3
                print("Sending command to EV3: " + command)
                connection.sendall(command.encode('utf-8'))

                # Wait for the response from EV3
                response = connection.recv(1024).decode('utf-8')
                print("Received response from EV3: " + response)

                if response == "Command received":
                    print("EV3 successfully received the command.")
                else:
                    print("Failed to confirm command on EV3.")

            finally:
                print("Closing connection.")
                connection.close()

        except socket.error as e:
            print("Socket error: " + str(e))

        finally:
            server_socket.close()
