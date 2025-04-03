# This script extracts and return feature points of a vehicle image:
# - one point per taillight (e.g. centers of bounding boxes)
#   [[bb1_x, bb1_y], [bb2_x, bb2_y]]
# - two points for the license plate (e.g. upper corners)
#   [[ul_x, ul_y], [ur_x, ur_y]]

import matplotlib.pyplot as plt
import math
import cv2
import sys
import os

from python.modules import *

def annotate_image(img_path, taillights_corners, taillights_centers, plate_corners):
    # annotate image
    # TODO: we can remove annotators from modules functions
    img = cv2.imread(img_path)
    cv2.rectangle(img,
                    taillights_corners[0][:2],
                    taillights_corners[0][2:4],
                    color=(0,235,255),
                    thickness=5) # first taillight box
    cv2.rectangle(img,
                    taillights_corners[1][:2],
                    taillights_corners[1][2:4],
                    color=(0,235,255),
                    thickness=5) # second taillight box
    cv2.circle(img, taillights_centers[0], radius=8, color=(1,208,6), thickness=-1) # first taillight center
    cv2.circle(img, taillights_centers[1], radius=8, color=(1,208,6), thickness=-1) # second taillight center

    cv2.line(img, plate_corners[0], plate_corners[1], color=(0,235,255), thickness=5) # license plate top segment
    cv2.line(img, plate_corners[1], plate_corners[2], color=(0,235,255), thickness=5) # license plate right segment
    cv2.line(img, plate_corners[2], plate_corners[3], color=(0,235,255), thickness=5) # license plate bottom segment
    cv2.line(img, plate_corners[3], plate_corners[0], color=(0,235,255), thickness=5) # license plate left segment
    cv2.circle(img, plate_corners[0], radius=8, color=(16,76,249), thickness=cv2.FILLED) # license plate upper left corner
    cv2.circle(img, plate_corners[1], radius=8, color=(16,76,249), thickness=cv2.FILLED) # license plate upper right corner

    return img

def main(path, dir=False):
    if dir:
        # for testing on multiple images
        paths = []
        for entry in os.scandir(path):  
            if entry.is_file():
                paths.append(entry.name)

        # display all images 
        plot_row_len = 5
        fig, axs = plt.subplots(math.ceil(len(paths)/plot_row_len), plot_row_len, squeeze=False)

        cnt = 0
        for image in paths:
            
            img_path = path+image
            img_path = check_format(img_path)

            print(f"Detecting on image {img_path}")

            # detect features
            taillights_corners, taillights_centers = detect_taillights(img_path)
            plate_corners = detect_license_plate(img_path)

            # annotate image
            annotated_img = annotate_image(img_path, taillights_corners, taillights_centers, plate_corners)
            # show image
            ax = axs[cnt//plot_row_len, cnt%plot_row_len]
            ax.imshow(cv2.cvtColor(annotated_img, cv2.COLOR_BGR2RGB))
            ax.grid(False)
            ax.axis("off")
            cnt += 1

        # remove grid and axis from remaining frames
        while cnt < plot_row_len*len(axs):
            ax = axs[cnt//plot_row_len, cnt%plot_row_len]
            ax.grid(False)
            ax.axis("off")
            cnt += 1

        fig.show()
        plt.show()


    else:
        path = check_format(path) # function in modules.heic_to_jpg

        # detect features
        taillights_corners, taillights_centers = detect_taillights(path)
        plate_corners = detect_license_plate(path)

        # annotate image
        annotated_img = annotate_image(path, taillights_corners, taillights_centers, plate_corners)

        plt.figure()
        plt.imshow(cv2.cvtColor(annotated_img, cv2.COLOR_BGR2RGB))
        plt.show()

        # return features
        return taillights_centers, plate_corners[:2]

if __name__ == "__main__":
    if len(sys.argv) > 0:
        if sys.argv[1] == "-file":
            #process single image, then the path refers to an image
            taillights_centers, plate_corners = main(sys.argv[2])
        elif sys.argv[1] == "-dir":
            #process all images in directory, then the path refers to a directory
            taillights_centers, plate_corners = main(sys.argv[2], dir=True)
