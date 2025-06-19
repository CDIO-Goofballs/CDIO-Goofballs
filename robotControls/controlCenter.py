import socket
import time
from enum import Enum
from typing import Any

import keyboard

port = 12345
server_socket = None
conn = None
def connect():
    global server_socket
    global conn
    # Setup server socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('', port))
    server_socket.listen(1)
    
    print("Waiting for a client to connect...")
    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")
    
def reconnect():
    global conn
    counter = 0
    while True:
        try:
            print("Waiting for a client to connect...")
            conn, addr = server_socket.accept()
            print(f"Connected by {addr}")
            break
        except Exception as e:
            print(f"Connection attempt failed: {e}")
            if counter > 20:
                break
            counter += 1
            time.sleep(1)  # Wait before retrying"""

class Command(Enum):
    DRIVE = "drive"
    TURN = "turn"
    STOP = "stop"
    SERVO = "servo"
    TEST = "test"

def send_command(command: tuple[Command, Any]):
    def wait_for_done():
        global conn
        if server_socket is None:
            print("Serversocket is None")
            return
        msg = ""
        while msg != "Done":
            data = conn.recv(1024)
            if not data:
                continue
            print(data.decode())
            msg = data.decode()

    print(command)
    global conn
    name, val = command
    message = ','.join((name.value, str(val))) + ';'
    try:
        conn.send(message.encode())
    except (BrokenPipeError, ConnectionResetError, OSError):
        print("Connection lost. Reconnecting...")
        reconnect()
    wait_for_done()


def send_commands(commands):
    for command in commands:
        send_command(command)

keyboard.add_hotkey('e', send_command, args=((Command.STOP, None),))
keyboard.add_hotkey('t', send_command, args=((Command.SERVO, 30),))
keyboard.add_hotkey('y', send_command, args=((Command.SERVO, 0),))
keyboard.add_hotkey('u', send_command, args=((Command.SERVO, -80),))