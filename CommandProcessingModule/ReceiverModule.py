#!/usr/bin/env python3

import socket
from EV3InstructionDecoder import GestureToEV3Controller

class EV3CommandReceiver:
    def __init__(self, sender_ip, sender_port=59700):
        """
        Initializes the EV3CommandReceiver with the sender's IP address and port.

        Parameters:
        - sender_ip (str): The IP address of the sender (PC).
        - sender_port (int): The port on which the sender is listening (default is 59700).
        """
        self.sender_ip = sender_ip
        self.sender_port = sender_port
        self.controller = GestureToEV3Controller()  # Initialize the GestureToEV3Controller

    def receive_commands(self):
        """
        Continuously connects to the sender, receives commands, and processes them.
        """
        while True:
            try:
                # Connect to the sender's server
                server_address = (self.sender_ip, self.sender_port)
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                print("Connecting to ",self.sender_ip,":",self.sender_port,'......')
                sock.connect(server_address)

                try:
                    # Receive the command from the sender
                    command = sock.recv(1024).decode('utf-8')
                    if not command:
                        break
                    print("Command received:", command)

                    # Process the received command
                    self.execute_command(command)

                    # Send a confirmation response back to the sender
                    response = "Command received and processed."
                    sock.sendall(response.encode('utf-8'))

                finally:
                    print("Closing connection.")
                    sock.close()

            except socket.error as e:
                print("Socket error:", e)
                break  # Exit the loop if there is a socket error

    def execute_command(self, command):
        """
        Passes the received command to the GestureToEV3Controller for processing and execution.

        Parameters:
        - command (str): The command to execute.
        """
        self.controller.process_gesture(command)
        print("Command processed by GestureToEV3Controller:", command)


# Start the receiver on EV3
if __name__ == '__main__':
    sender_ip = '169.254.246.24'  # Replace with your PC's IP address
    ev3_receiver = EV3CommandReceiver(sender_ip=sender_ip, sender_port=59700)
    while True:
        ev3_receiver.receive_commands()
