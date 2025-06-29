import cv2

def adjust_contrast_brightness(image, alpha, beta):
    ''' 
    For each pixel of the image do
        p = (alpha*p + beta) % 255

    alpha: adjusts the contrast
    beta : adjusts the brightness
    '''
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) if len(image.shape) == 3 else image
    
    for i in range(gray.shape[0]):
        for j in range(gray.shape[1]):
            gray[i,j] = (alpha*gray[i,j] + beta)

    return gray  