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
    global contours_gray, contours_yellow
    camera_image = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR)


    image_bgr = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR) # convert from RGB to BGR
    mask_gray1, mask_gray2 = get_mask(obs, 'gray')


    cv2.imshow("mask yellow", get_mask(obs, 'yellow')) # show_mask 'yellow' / 'gray'
    cv2.imshow("for move", mask_gray1)
    cv2.imshow("for stop", mask_gray2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def get_mask(obs, mask_color):
    if mask_color == 'gray':
        image_bgr = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR) # convert from RGB to BGR
        lower_gray = np.array([155, 160, 155])
        upper_gray = np.array([230, 240, 230])


        mask = cv2.inRange(image_bgr, lower_gray, upper_gray)

        mask_for_move = mask.copy() 
        mask_for_move[0:230] = 0
        mask_for_move[0:, :image_width//2] = 0
        
        mask_for_stop = mask.copy() 
        mask_for_stop[0:340] = 0
        mask_for_stop[0:, :70] = 0
        mask_for_stop[0:, -70:] = 0

        return mask_for_move, mask_for_stop
    elif mask_color == 'yellow':
        image_hsv = cv2.cvtColor(obs, cv2.COLOR_RGB2HSV)  # convert from RGB to HSV
        lower_yellow = np.array([10, 100, 100])  
        upper_yellow = np.array([30, 255, 255]) 

        mask = cv2.inRange(image_hsv, lower_yellow, upper_yellow)
        mask[0:230] = 0
        return mask
    return None