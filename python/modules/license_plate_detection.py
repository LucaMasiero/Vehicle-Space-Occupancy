# This module detects the license plate of a vehicle and returns:
# - four corners of the license plate lcockwise starting from the upper left corner
#   [[ul_x, ul_y], [ur_x, ur_y], [br_x, br_y], [bl_x, bl_y]]

from roboflow import Roboflow
import supervision as sv
import cv2
import sys

def canny(img):
    # Blur the image for better edge detection
    img_blur = cv2.GaussianBlur(img, (3,3), 0) 
    # Canny Edge Detection
    edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)

    return edges

def license_plate_detection(path):
    
    img = cv2.imread(path) # load image
    
    # connect to roboflow and load the model
    rf = Roboflow(api_key="fKIPdjFkhzve37YlaXsC")
    model = rf.workspace().project("license-plate-recognition-8fvub").version("1").model
    result = model.predict(path, confidence=70, overlap=10).json() # inference
    detections = sv.Detections.from_inference(result) # extract detection information

    x1, y1, x2, y2 = map(int, detections.xyxy[0])  # bounding box coordinates
    plate_img = img.copy()[y1:y2, x1:x2]

    blue, green, red = cv2.split(plate_img) # split channels
    # Run canny edge detection on each channel
    blue_edges = canny(blue)
    green_edges = canny(green)    
    red_edges = canny(red)

    edges = blue_edges | green_edges | red_edges # join edges back into image
 
    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cnts=sorted(contours, key = cv2.contourArea, reverse = True) # sort contours based on area from larger to smaller

    hulls = [cv2.convexHull(cnt) for cnt in cnts]
    perims = [cv2.arcLength(hull, True) for hull in hulls]
    approxes = [cv2.approxPolyDP(hulls[i], 0.08 * perims[i], True) for i in range(len(hulls))]
    
    #for a in approxes:
    #    cv2.drawContours(img, [a+[x1,y1]], -1, (0, 0, 255), 2)
    #plt.figure()
    #plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    #plt.show()

    approx_cnts = sorted(approxes, key = cv2.contourArea, reverse = True) # sort based on area
    lengths = [len(cnt) for cnt in approx_cnts]

    #approx_idx = [index for (index, item) in enumerate(lengths)] 
    approx_idx = [index for (index, item) in enumerate(lengths) if item==4] # take all the polygons with 4 edges

    # find polygon with maximum area
    max_area = 0
    max_approx = None
    for idx in approx_idx:
        approx = approxes[idx]
        curr_area = cv2.contourArea(approx)
        if curr_area >= max_area:
            max_area = curr_area
            max_approx = approx 

    # transform the coordinates of the bounding box to be applied on the entire image, not just the license plate region
    max_approx = max_approx + [x1,y1]
    img = cv2.imread(path)
    img = cv2.rectangle(img, (x1, y1), (x2, y2), color=(0, 0, 0), thickness=2) # draw initial bounding box
    
    if len(max_approx) == 4:
        # sort plate's corners
        sorted_pts = sorted(max_approx, key=lambda x: x[0][0]) # keep the approx variable unchanged otherwise drawContours won't work
        sorted_pts[0:2] = sorted(sorted_pts[0:2], key=lambda x: x[0][1])
        sorted_pts[2:4] = sorted(sorted_pts[2:4], key=lambda x: x[0][1])
        # upper left = ul; upper right=ur; bottom left=bl; bottom right=br;
        ul, bl, ur, br = [p[0] for p in sorted_pts]

        print("\nLICENSE PLATE POINTS:\n"+
            f"Upper Left: {ul}\n"+
            f"Upper Right: {ur}\n"+
            f"Bottom Right: {br}\n"+
            f"Bottom Left: {bl}\n")
    else:
        raise Exception("It was not possible to find a four edged polygon to contain the license plate!!")
        #TODO: what to do in this case ?
    
    return [ul, ur, br, bl]

if __name__ == "__main__":
    license_plate_detection(sys.argv[1])
