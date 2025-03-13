from inference_sdk import InferenceHTTPClient
from inference import get_model
import supervision as sv
import cv2

#CLIENT = InferenceHTTPClient(
    #api_url="https://detect.roboflow.com",
    #api_key="VD9BLusLGWoKvrez3ufK"
#)

#results = CLIENT.infer("Test.jpg", model_id="pingpong-2/1")

# load a pre-trained yolov8n model
model = get_model(model_id="oob_cdio-kghp5/4", api_key="VD9BLusLGWoKvrez3ufK")

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
    # run inference on our chosen image, image can be a url, a numpy array, a PIL image, etc.
    results = model.infer(frame)[0]

    # load the results into the supervision Detections api
    detections = sv.Detections.from_inference(results)

    # create supervision annotators
    bounding_box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    # annotate the image with our inference results
    annotated_image = bounding_box_annotator.annotate(
        scene=frame, detections=detections)
    annotated_image = label_annotator.annotate(
        scene=annotated_image, detections=detections)

    # display the image
    #sv.plot_image(annotated_image)

    # Display the captured frame
    cv2.imshow('Camera', annotated_image)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
#out.release()
cv2.destroyAllWindows()