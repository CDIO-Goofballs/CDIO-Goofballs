from inference_sdk import InferenceHTTPClient
from inference import get_model
import supervision as sv
import cv2

#CLIENT = InferenceHTTPClient(
    #api_url="https://detect.roboflow.com",
    #api_key="VD9BLusLGWoKvrez3ufK"
#)

#results = CLIENT.infer("Test.jpg", model_id="pingpong-2/1")

cam = cv2.VideoCapture(1)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define the codec and create VideoWriter object
#fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))

while True:
    ret, frame = cam.read()

    # Write the frame to the output file
    #out.write(frame)

    
    # Display the captured frame
    cv2.imshow('Camera', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
#out.release()
cv2.destroyAllWindows()