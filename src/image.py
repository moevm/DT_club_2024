import numpy as np
import cv2
from src.const import *

writer_mask = cv2.VideoWriter(
    output_path + "/output_mask.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"),
    20,
    (640, 480), # width, height
)

writer_camera = cv2.VideoWriter(
    output_path + "/output.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"),
    20,
    (640, 480), # width, height
)

def get_bot_image(obs):
    cv2.imshow("mask yellow", get_mask(obs, 'yellow')) # show_mask 'yellow' / 'gray'
    cv2.imshow("mask gray", get_mask(obs, 'gray'))
    cv2.imshow("mask red", get_mask(obs, 'red'))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def get_mask(obs, mask_color):
    if mask_color == 'gray':
        image_bgr = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR) # convert from RGB to BGR

        mask_gray = cv2.inRange(image_bgr, np.array(lower_gray), np.array(upper_gray))
        mask = cv2.cvtColor(mask_gray, cv2.COLOR_GRAY2BGR) 
    elif mask_color == 'yellow':
        image_hsv = cv2.cvtColor(obs, cv2.COLOR_RGB2HSV) # convert from RGB to HSV
        
        mask_yellow = cv2.inRange(image_hsv, np.array(lower_yellow), np.array(upper_yellow))
        mask = cv2.cvtColor(mask_yellow, cv2.COLOR_GRAY2BGR)
    elif mask_color == 'red':
        image_bgr = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR) # convert from RGB to HSV

        mask_red = cv2.inRange(image_bgr, np.array(lower_red), np.array(upper_red))
        mask = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)
    return mask