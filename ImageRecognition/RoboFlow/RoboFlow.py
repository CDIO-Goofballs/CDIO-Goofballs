import math

from sklearn.linear_model import LinearRegression

from inference_sdk import InferenceHTTPClient
from inference import get_model
import supervision as sv
import cv2
from sympy.strategies.core import switch

model = get_model(model_id="oob_cdio-kghp5/9", api_key="VD9BLusLGWoKvrez3ufK")

cam = cv2.VideoCapture(1)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

balls = []
vip_ball = None
walls = []
egg = None
cross = None
small_goal = None

def point_x(point):
  return point.x
def point_y(point):
  return point.y

def object_recognition(img, scale_factor):
    global vip_ball
    global cross
    global egg
    global small_goal
    # run inference on our chosen image, image can be a url, a numpy array, a PIL image, etc.
    balls.clear()
    vip_ball = None
    walls.clear()
    cross = []
    egg = None
    small_goal = None

    results = model.infer(img)[0]
    predictions = results.predictions

    for prediction in predictions:
        match prediction.class_name:
            case "Ball":
                balls.append((prediction.x * scale_factor, prediction.y * scale_factor))
            case "Vip":
                vip_ball = (prediction.x * scale_factor, prediction.y * scale_factor)
            case "Wall":
                walls.append([(scale_factor * point.x, scale_factor * point.y) for point in prediction.points])
            case "Cross":
                top = prediction.points.sort(key = point_y)[0]
                bottom = prediction.points.sort(key = point_y, reverse=True)[0]
                right = prediction.points.sort(key = point_x)[0]
                left = prediction.points.sort(key = point_x, reverse=True)[0]
                cross = (top, bottom, right, left)
            case "Eggman":
                egg = (prediction.x * scale_factor, prediction.y * scale_factor)
            case "Small-goal":
                small_goal = (prediction.x * scale_factor, prediction.y * scale_factor)

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

    return annotated_image, balls, vip_ball, walls, cross, egg, small_goal
