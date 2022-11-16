#!/usr/bin/env python3

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0
beta = np.abs(-4.48-(-4.58))/76.95959751157433

SPLIT = 190

tx = 66 
ty = 66 - SPLIT

# Function that converts image coord to world coord
def IMG2W(col, row):
    x = row - ty 
    y = col - tx
    
    x = beta*x
    y = beta*y
    
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    X = np.array([x,y])
    X = R@X
    
    x = X[0] * 1000
    y = X[1] * 1000

    # print(x,y)
    
    return x,y
# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    if color == "red":
        # Filter by Color
        params.filterByColor = False

        # Filter by Area.
        params.filterByArea = True
        params.minArea = (5) **2
        params.maxArea = (100) ** 2

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.01
        params.maxCircularity = 10

        # Filter by Inerita
        params.filterByInertia = False

        # Filter by Convexity
        params.filterByConvexity = False

        image_raw = image_raw[:SPLIT]
    elif color == "car":
                # Filter by Color
        params.filterByColor = False

        # Filter by Area.
        params.filterByArea = True
        params.minArea = (100) **2
        params.maxArea = (300) ** 2

        # Filter by Circularity
        params.filterByCircularity = False

        # Filter by Inerita
        params.filterByInertia = False

        # Filter by Convexity
        params.filterByConvexity = False


        image_raw = image_raw[SPLIT:]

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    hsv_image = cv2.GaussianBlur(hsv_image, (3,3), 1)

    # ========================= Student's code starts here =========================

    # sec = hsv_image[16:38, 211:237]
    # sec = hsv_image[116:280, 216:325]

    # print((sec[:, :, 0].min(), sec[:, :, 0].mean(), sec[:, :, 0].max()), 
    #         (sec[:, :, 1].min(), sec[:, :, 1].mean(), sec[:, :, 1].max()),  
    #         (sec[:, :, 2].min(), sec[:, :, 2].mean(), sec[:, :, 2].max()))

    # q = [.1, .9]

    # print(np.quantile(sec[: , : , 0], q), np.quantile(sec[: , : , 1], q) , np.quantile(sec[: , : , 2], q))

    if color == "red":
        # Red thresholds
        lower = (0, 100, 100)     # blue lower
        upper = (20, 147, 140)   # blue upper

        # Define a mask using the lower and upper bounds of the target color
        mask_image = cv2.inRange(hsv_image, lower, upper)
    elif color == "car":
        # Red thresholds
        lower = (0, 0, 50)     # blue lower
        upper = (10, 10, 70)   # blue upper

        # Define a mask using the lower and upper bounds of the target color
        mask_image = cv2.inRange(hsv_image, lower, upper)
    # else:
    #     mask_image = cv2.bitwise_or(mask_imageG, mask_image)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
        
    if num_blobs >= 2:
        blob1 = blob_image_center[0]
        blob2 = blob_image_center[1]
    
    
        dist = np.sqrt((blob1[0] -blob2[0])**2 + (blob1[1] - blob2[1])**2)
        deltay = (blob2[1] -blob1[1])
        deltax = (blob2[0] -blob1[0])
        
        if blob1[0] > blob2[0]:
            temp = blob1
            blob1 = blob2
            blob2 = temp

        # print(dist)
            
        
        theta = np.arctan(deltay/deltax)
        # print(theta,np.degrees(theta))
    # print(dist)
    
    
    # print(blob_image_center)

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0, (0, 0, 255))

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        # print("No block found!")
        pass
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
        # print(xw_yw)
        


    # cv2.namedWindow(f"Camera View color")
    # cv2.imshow("Camera View", image_raw)

    # cv2.namedWindow("Camera Temp View")
    # cv2.imshow("Camera Temp View", sec)

    cv2.namedWindow(f"Mask View {color}")
    cv2.imshow(f"Mask View {color}", mask_image)
    cv2.namedWindow(f"Keypoint View {color}")
    cv2.imshow(f"Keypoint View {color}", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
