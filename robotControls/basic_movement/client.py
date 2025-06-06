import socket
import keyboard

ev3_ip = '192.168.0.138'  # Replace with EV3's IP
port = 12346

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((ev3_ip, port))

# Function to send command to the server
def send_command(command):
    client_socket.send(command.encode())

# Setup hotkeys for key press events
keyboard.add_hotkey('w', send_command, args=('forward',))  # Move forward
keyboard.add_hotkey('e', send_command, args=('stop',))     # Stop
keyboard.add_hotkey('a', send_command, args=('left',))     # Turn left
keyboard.add_hotkey('d', send_command, args=('right',))    # Turn right
keyboard.add_hotkey('s', send_command, args=('backwards',)) # Move backward
keyboard.add_hotkey('q', send_command, args=('exit',))     # Quit
keyboard.add_hotkey('j', send_command, args=('speedup',)) # Speed up
keyboard.add_hotkey('k', send_command, args=('speeddown',)) # Speed down
keyboard.add_hotkey('u', send_command, args=('calibrateGyro',)) # Calibrate the gyro sensor
keyboard.add_hotkey('1', send_command, args=('toggle_servo',)) # Calibrate the gyro sensor

# Wait for hotkeys to be pressed
keyboard.wait('q')  # Keep waiting until 'q' is pressed to exit

client_socket.close()
