import cv2
import numpy as np
from matplotlib import pyplot as plt

from .adjust_contrast_brightness import *

def SIFT(taillights_corners, path=None, image=None):
    sift = cv2.SIFT.create(contrastThreshold=0.3,
                           edgeThreshold=7,
                           sigma=1.6)
    # left taillight
    masked_image1 = focus_on_patch(taillights_corners[1][:2], taillights_corners[1][2:], path, image)
    masked_image1 = adjust_contrast_brightness(masked_image1, alpha=1.2, beta=0)
    kp1, des1 = sift.detectAndCompute(masked_image1,None)
    # raight taillight
    masked_image2 = focus_on_patch(taillights_corners[0][:2], taillights_corners[0][2:], path, image)
    masked_image2= adjust_contrast_brightness(masked_image2, alpha=1.2, beta=0)
    kp2, des2 = sift.detectAndCompute(masked_image2,None)
    
    img = cv2.imread(path) if path else image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    img=cv2.drawKeypoints(img,kp1,img)
    img=cv2.drawKeypoints(img,kp2,img)

    FLANNMatcher(path, kp1, des1, kp2, des2, masked_image1, masked_image2)

def FLANNMatcher(path, kp1, des1, kp2, des2, masked_image1, masked_image2):
    # FLANN parameters  
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=100)   # or pass empty dictionary
    
    flann = cv2.FlannBasedMatcher(index_params,search_params)
    
    matches = flann.knnMatch(des1,des2,k=2)
    
    # Need to draw only good matches, so create a mask
    matchesMask = [[0,0] for i in range(len(matches))]
    
    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.95*n.distance:
            matchesMask[i]=[1,0]
    
    draw_params = dict(matchColor = (0,255,0),
                    singlePointColor = (255,0,0),
                    matchesMask = matchesMask,
                    flags = cv2.DRAW_MATCHES_FLAGS_DEFAULT)
    
    img = cv2.imread(path)
    img3 = cv2.drawMatchesKnn(masked_image1,kp1,masked_image2,kp2,matches,None,**draw_params)
    
    plt.imshow(img3,),plt.show()

def focus_on_patch(ul,br, path=None, image=None):
    '''
        ul = [ul_x, ul_y]
        br = [br_x,  br_y] with x: width
                                y: height
    '''

    minx = ul[0]
    maxx = br[0]
    miny = ul[1]
    maxy = br[1]

    # read image
    img = cv2.imread(path) if path else image
    # gray image
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img

    # creat mask
    mask = np.zeros(gray.shape, dtype="uint8")
    for i in range(miny, maxy):
        for j in range(minx, maxx):
            mask[i,j] = 1

    # apply mask
    masked_img = cv2.bitwise_and(gray, gray, mask=mask)
    
    return masked_img