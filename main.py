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
sat_low = 70
sat_high = 255
val_low = 55
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

    #render images: color on left, depth on right
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_HOT)
    images = np.hstack((color_image, masked_image, depth_colormap))
    cv2.imshow('Video Output', images)

    #press esc to quit
    key = cv2.waitKey(1)
    if key == 27: 
        cv2.destroyAllWindows()
        break

    print("Variables:")
    print(hue_low)
    print(hue_high)

pipeline.stop()