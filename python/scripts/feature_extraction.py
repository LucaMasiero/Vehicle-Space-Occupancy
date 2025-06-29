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
import numpy as np

from python.modules import *

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
            taillights_corners, taillights_centers = taillights_detection(img_path)
            plate_corners = license_plate_detection(img_path)

            # annotate image
            annotated_img = annotate_image(img_path, taillights_corners, taillights_centers, plate_corners)
            # show image
            ax = axs[cnt//plot_row_len, cnt%plot_row_len]
            ax.imshow(cv2.cvtColor(annotated_img, cv2.COLOR_BGR2RGB))
            ax.grid(False)
            ax.axis("off")
            cnt += 1

        # remove grid and axis for remaining frames
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
        taillights_corners, taillights_centers = taillights_detection(path)
        plate_corners = license_plate_detection(path)

        # Check SIFT capabilities
        SIFT(taillights_corners, path=path, image=None)
            
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
