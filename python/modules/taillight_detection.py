# This module detects the taillights of the vehicle and returns:
# - upper left and lower right corners for each one of the bounding boxes
#   [ [bb1_ul_x, bb1_ul_y, bb1_lr_x, bb1_lr_y], [bb2_ul_x, bb2_ul_y, bb2_lr_x, bb2_lr_y] ]
# - center of each one of the bounding boxes
#   [[bb1_x, bb1_y], [bb2_x, bb2_y]]

from roboflow import Roboflow
import supervision as sv
import cv2

def detect_taillights(path):

    image = cv2.imread(path) # load image

    # connect to roboflow and load the model
    rf = Roboflow(api_key="fKIPdjFkhzve37YlaXsC")
    model = rf.workspace().project("final-e4u4f").version("4").model

    # inference
    result = model.predict(path, confidence=45, overlap=10).json()
    # extract detection information
    detections = sv.Detections.from_inference(result)
    box_corners = detections.xyxy   # upper left and lower right corner of bounding boxes
                                    # [ [bb1_ul_x, bb1_ul_y, bb1_lr_x, bb1_lr_y], [bb2_ul_x, bb2_ul_y, bb2_lr_x, bb2_lr_y] ]

    # prepare annotators
    box_color = sv.ColorPalette.from_hex(['#FFDD4A'])
    point_color = sv.ColorPalette.from_hex(['#3BC14A'])

    label_annotator = sv.LabelAnnotator(color=box_color)
    bounding_box_annotator = sv.BoxAnnotator(color=box_color, thickness=3)
    dot_annotator = sv.DotAnnotator(color=point_color, radius=5)
    
    labels = [item["class"] for item in result["predictions"]]
    annotated_image = bounding_box_annotator.annotate(scene=image, detections=detections)
    #annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections, labels=labels)
    annotated_image = dot_annotator.annotate(scene=annotated_image, detections=detections)

    # center points of bounding box
    centers = []
    cnt = 1
    for bb in box_corners:
        cx = int((bb[0]+bb[2])/2)
        cy = int((bb[1]+bb[3])/2)
        centers.append([cx, cy])
        print( f"bounding box {cnt}:\n"+
                f"\tcenter: X={cx}, Y={cy}")
        cnt += 1
    
    return box_corners.astype(int), centers
