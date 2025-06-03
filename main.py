import threading
from ImageRecognition.RoboFlow.MainImageRecognition import start_image_recognition

# Start image recognition in a background thread
#image_thread = threading.Thread(target=start_image_recognition, daemon=True)
#image_thread.start()

# Main thread can now run pathfinding or other logic
while True:
    start_image_recognition()
    # Example: call your pathfinding function
    # You can fetch latest positions/objects using the get_* functions
    # vip = get_vip_ball()
    # balls = get_balls()
    # walls = get_walls()
    # path = plan_robot_path(vip, balls, walls)
    pass  # Replace with your logic