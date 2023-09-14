#import the library
import pyrealsense2 as rs
import numpy as np
import cv2

def update_hue_low(val):
    global hue_low
    hue_low = val
def update_hue_high(val):
    global hue_high
    hue_high = val
def update_sat_low(val):
    global sat_low
    sat_low = val
def update_sat_high(val):
    global sat_high
    sat_high = val
def update_val_low(val):
    global val_low
    val_low = val
def update_val_high(val):
    global val_high
    val_high = val

#set HSV range for purple
hue_low = 110
hue_high = 145
sat_low = 85
sat_high = 255
val_low = 20
val_high = 200

#create pipeline and config
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

#enable depth and color stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#start stream
profile = pipeline.start(config)

#create align object
align_target = rs.stream.color
align = rs.align(align_target)

#add trackbars
window_name = "Sliders"
cv2.namedWindow(window_name)
cv2.createTrackbar("Hue Low", window_name, hue_low, 179, update_hue_low)   
cv2.createTrackbar("Hue High", window_name, hue_high, 179, update_hue_high)  
cv2.createTrackbar("Sat Low", window_name, sat_low, 255, update_sat_low)  
cv2.createTrackbar("Sat High", window_name, sat_high, 255, update_sat_high)  
cv2.createTrackbar("Val Low", window_name, val_low, 255, update_val_low)  
cv2.createTrackbar("Val High", window_name, val_high, 255, update_val_high)  

#loop
while True:
    # Get frameset of color and depth and align 
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Get aligned frames
    depth_frame = aligned_frames.get_depth_frame() 
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    if not depth_frame or not color_frame:
        continue

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
    kernel_1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
    masked_image_opening = cv2.morphologyEx(masked_image, cv2.MORPH_OPEN, kernel_1)
    kernel_2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(30,30))
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
    except IndexError as e:
        cx = 320
        cy = 240
    print([cx,cy])

    #render images: color on left, filtered image center, depth on right
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_HOT)
    cv2.drawContours(color_image, contours, -1, (0,255,0), 2)
    cv2.circle(color_image, (cx,cy), radius=10, color=(0, 0, 255), thickness=2)
    cv2.drawContours(masked_image_clean, contours, -1, (0,255,0), 2)
    cv2.circle(masked_image_clean, (cx,cy), radius=10, color=(0, 0, 255), thickness=2)
    cv2.drawContours(depth_colormap, contours, -1, (0,255,0), 2)
    cv2.circle(depth_colormap, (cx,cy), radius=10, color=(0, 0, 255), thickness=2)
    images = np.hstack((color_image, masked_image_clean, depth_colormap))
    cv2.imshow('Video and Depth Output', images)
    #cv2.imshow(window_name, masked_image_clean)

    #press esc to quit
    key = cv2.waitKey(1)
    if key == 27: 
        cv2.destroyAllWindows()
        break

pipeline.stop()