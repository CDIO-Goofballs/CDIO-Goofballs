import math

from sklearn.linear_model import LinearRegression

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
model = get_model(model_id="oob_cdio-kghp5/9", api_key="VD9BLusLGWoKvrez3ufK")

cam = cv2.VideoCapture(1)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

balls = []
vip_balls = []
walls = []

ratio = 10

def longest_distance(points):
    max_dist = 0
    for i in range(len(points)):
        x1 = points[i].x
        y1 = points[i].y
        for j in range(i + 1, len(points)):
            x2 = points[j].x
            y2 = points[j].y
            dist = math.hypot(x2 - x1, y2 - y1)
            if dist > max_dist:
                max_dist = dist
    return max_dist

def object_recognition(img):
    # run inference on our chosen image, image can be a url, a numpy array, a PIL image, etc.
    global ratio
    balls.clear()
    vip_balls.clear()
    walls.clear()
    results = model.infer(img)[0]
    predictions = results.predictions
    for prediction in predictions:
        match prediction.class_name:
            case "Ball":
                #print(f"Ball diameter: {longest_distance(prediction.points) * ratio}")
                balls.append((prediction.x, prediction.y))
                #print(f"x: {prediction.x}, y: {prediction.y}")
            case "Vip":
                vip_balls.append((prediction.x, prediction.y))
                #print(f"Vip diameter: {longest_distance(prediction.points) * ratio}")
                #print(f"x: {prediction.x}, y: {prediction.y}")
            case "Wall":
                walls.append(prediction.points)
                print(f"Wall length: {longest_distance(prediction.points) * ratio}")
            case "Cross":
                print(prediction.points)

    if len(walls) == 4:
        distance = 0
        for wall in walls:
            distance += longest_distance(wall)
        print(distance)
        ratio = (112*2+172*2) / distance

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

    return annotated_image, ratio
