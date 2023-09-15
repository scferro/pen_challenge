import pyrealsense2 as rs
import numpy as np
import cv2

class Cam:
    def __init__(self):
        pass

    def find_pen(self, pipeline, align):
        #default centroid
        cx = 320
        cy = 240
        cz = 500

        #set HSV range for purple
        hue_low = 110
        hue_high = 145
        sat_low = 15
        sat_high = 255
        val_low = 20
        val_high = 200

        # Get frameset of color and depth and align 
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame() 
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # define range of purple color in HSV
        lower_purple = np.array([hue_low, sat_low, val_low])
        upper_purple = np.array([hue_high, sat_high, val_high])

        # Threshold the HSV image to get only purple colors
        mask = cv2.inRange(hsv_image, lower_purple, upper_purple)
        masked_image = cv2.bitwise_and(color_image, color_image, mask= mask)    

        # clean up masked image
        kernel_1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(12,12))
        masked_image_opening = cv2.morphologyEx(masked_image, cv2.MORPH_OPEN, kernel_1)
        kernel_2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(40,40))
        masked_image_clean = cv2.morphologyEx(masked_image_opening, cv2.MORPH_CLOSE, kernel_2)

        #contour detection
        masked_image_gray = cv2.cvtColor(masked_image_clean, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(masked_image_gray, 10, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #find centroid
        try:
            edge = contours[0]
            moment = cv2.moments(edge)
            cx = int(moment['m10']/moment['m00'])
            cy = int(moment['m01']/moment['m00'])
            cz = depth_image[cy][cx]
            print([cx,cy,cz])
            pen_visible = True
        except IndexError as e:
            cx = 320
            cy = 240
            cz = 0
            pen_visible = False

        #render images: color on left, filtered image center, depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_HOT)
        if pen_visible == True:
            cv2.drawContours(color_image, contours, -1, (0,255,0), 2)
            cv2.circle(color_image, (cx,cy), radius=10, color=(0, 0, 255), thickness=2)
            cv2.drawContours(masked_image_clean, contours, -1, (0,255,0), 2)
            cv2.circle(masked_image_clean, (cx,cy), radius=10, color=(0, 0, 255), thickness=2)
            cv2.drawContours(depth_colormap, contours, -1, (0,255,0), 2)
            cv2.circle(depth_colormap, (cx,cy), radius=10, color=(0, 0, 255), thickness=2)
        images = np.hstack((color_image, masked_image_clean, depth_colormap))
        cv2.imshow('Video and Depth Output', images)
        return [cx,cy,cz, pen_visible]