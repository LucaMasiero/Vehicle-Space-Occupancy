# This module detects the taillights of the vehicle and returns:
# - upper left and lower right corners for each one of the bounding boxes
#   [ [bb1_ul_x, bb1_ul_y, bb1_lr_x, bb1_lr_y], [bb2_ul_x, bb2_ul_y, bb2_lr_x, bb2_lr_y] ]
# - center of each one of the bounding boxes
#   [[bb1_x, bb1_y], [bb2_x, bb2_y]]

from roboflow import Roboflow
import supervision as sv
import cv2

def taillights_detection(path):

    # connect to roboflow and load the model
    rf = Roboflow(api_key="fKIPdjFkhzve37YlaXsC")
    model = rf.workspace().project("final-e4u4f").version("4").model

    # inference
    result = model.predict(path, confidence=45, overlap=10).json()
    # extract detection information
    detections = sv.Detections.from_inference(result)
    box_corners = detections.xyxy   # upper left and lower right corners of bounding boxes
                                    # [ [bb1_ul_x, bb1_ul_y, bb1_lr_x, bb1_lr_y], [bb2_ul_x, bb2_ul_y, bb2_lr_x, bb2_lr_y] ]

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
