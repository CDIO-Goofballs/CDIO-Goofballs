from inference_sdk import InferenceHTTPClient
from inference import get_model
import supervision as sv
import cv2
from sympy.strategies.core import switch

#CLIENT = InferenceHTTPClient(
    #api_url="https://detect.roboflow.com",
    #api_key="VD9BLusLGWoKvrez3ufK"
#)

#results = CLIENT.infer("Test.jpg", model_id="pingpong-2/1")

# load a pre-trained yolov8n model
model = get_model(model_id="oob_cdio-kghp5/8", api_key="VD9BLusLGWoKvrez3ufK")

cam = cv2.VideoCapture(1)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

balls = []
vip_balls = []
walls = []

def object_recognition(img):
    # run inference on our chosen image, image can be a url, a numpy array, a PIL image, etc.
    results = model.infer(img)[0]
    predictions = results.predictions
    for prediction in predictions:
        match prediction.class_name:
            case "Ball":
                balls.append((prediction.x, prediction.y))
                print(f"x: {prediction.x}, y: {prediction.y}")
            case "Vip":
                vip_balls.append((prediction.x, prediction.y))
                print(f"x: {prediction.x}, y: {prediction.y}")
            case "Wall":
                walls.append(prediction.points)
                print(prediction.points)
            case "Cross":
                print(prediction.points)


    # load the results into the supervision Detections api
    detections = sv.Detections.from_inference(results)

    # create supervision annotators
    bounding_box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()
    polygon_annotater = sv.PolygonAnnotator()

    # annotate the image with our inference results
    #annotated_image = bounding_box_annotator.annotate(
    #    scene=img, detections=detections)
    annotated_image = polygon_annotater.annotate(
        scene=img, detections=detections)
    annotated_image = label_annotator.annotate(
        scene=annotated_image, detections=detections)

    return annotated_image
