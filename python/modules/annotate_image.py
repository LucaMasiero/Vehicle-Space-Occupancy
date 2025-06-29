import cv2

def annotate_image(img_path, taillights_corners, taillights_centers, plate_corners):
    # annotate image
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
    cv2.circle(img, taillights_centers[0], radius=12, color=(1,208,6), thickness=-1) # first taillight center
    cv2.circle(img, taillights_centers[1], radius=12, color=(1,208,6), thickness=-1) # second taillight center

    cv2.line(img, plate_corners[0], plate_corners[1], color=(0,235,255), thickness=5) # license plate top segment
    cv2.line(img, plate_corners[1], plate_corners[2], color=(0,235,255), thickness=5) # license plate right segment
    cv2.line(img, plate_corners[2], plate_corners[3], color=(0,235,255), thickness=5) # license plate bottom segment
    cv2.line(img, plate_corners[3], plate_corners[0], color=(0,235,255), thickness=5) # license plate left segment
    cv2.circle(img, plate_corners[2], radius=12, color=(16,76,249), thickness=cv2.FILLED) # license plate upper left corner
    cv2.circle(img, plate_corners[3], radius=12, color=(16,76,249), thickness=cv2.FILLED) # license plate upper right corner

    return img