import supervision as sv
from inference import get_model

model = get_model(model_id="oob_cdio-kghp5/9", api_key="VD9BLusLGWoKvrez3ufK")

balls = []
vip_ball = None
wall_points = []
egg = None
cross = None
small_goal = None
wall_corners = None
big_goal = None

def point_x(point):
  return point.x
def point_y(point):
  return point.y
def point_xplusy(point):
    return point[0] + point[1]
def point_xminusy(point):
    return point[0] - point[1]

def object_recognition(img, scale_factor):
    global vip_ball
    global cross
    global egg
    global small_goal
    global big_goal
    global wall_corners
    # run inference on our chosen image, image can be a url, a numpy array, a PIL image, etc.
    balls.clear()
    vip_ball = None
    wall_points.clear()

    vips = []
    cross_objects = []

    results = model.infer(img)[0]
    predictions = results.predictions

    for prediction in predictions:
        match prediction.class_name:
            case "Ball":
                balls.append((prediction.x * scale_factor, prediction.y * scale_factor))
            case "Vip":
                vips.append(prediction)
            case "Wall":
                for point in prediction.points:
                    wall_points.append( (scale_factor * point.x, scale_factor * point.y) )
            case "Cross":
                cross_objects.append(prediction)
            case "Eggman":
                egg = (prediction.x * scale_factor, prediction.y * scale_factor)
            case "Small-goal":
                small_goal = (prediction.x * scale_factor, prediction.y * scale_factor)
            case "Big-goal":
                big_goal = (prediction.x * scale_factor, prediction.y * scale_factor)

    # Vip should be the one with the highest confidence, while the rest become normal balls
    if vips:
        vips.sort(key=lambda x: x.confidence, reverse=True)
        vip_ball = (vips[0].x * scale_factor, vips[0].y * scale_factor)
        vips = vips[1:]
    for vip in vips:
        balls.append((vip.x * scale_factor, vip.y * scale_factor))

    # Cross should be the one with the highest confidence, while the rest are ignored
    if cross_objects:
        cross_objects.sort(key=lambda x: x.confidence, reverse=True)
        selected_cross = cross_objects[0]
        y_sort = sorted(selected_cross.points, key=point_y)
        top = (y_sort[0].x * scale_factor, y_sort[0].y * scale_factor)
        bottom = (y_sort[-1].x * scale_factor, y_sort[-1].y * scale_factor)
        x_sort = sorted(selected_cross.points, key=point_x)
        left = (x_sort[0].x * scale_factor, x_sort[0].y * scale_factor)
        right = (x_sort[-1].x * scale_factor, x_sort[-1].y * scale_factor)
        cross = (top, bottom, right, left)


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

    if wall_points:
        xplusy_sort = sorted(wall_points, key=point_xplusy)
        xminusy_sort = sorted(wall_points, key=point_xminusy)
        top_left = xplusy_sort[0]
        bottom_right = xplusy_sort[-1]
        bottom_left = xminusy_sort[0]
        top_right = xminusy_sort[-1]
        wall_corners = (top_left, bottom_left, bottom_right, top_right)

    return annotated_image, balls, vip_ball, wall_corners, cross, egg, small_goal, big_goal
