import cv2
import numpy as np
import os.path
from src.const import output_path

if not os.path.exists(output_path):
    os.makedirs(output_path, exist_ok=True)
    print(f"Directory created: {output_path}")
else:
    print(f"Directory already exists: {output_path}")

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
    camera_image = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR)

    cv2.imshow("mask yellow", get_mask(obs, 'yellow')) # show_mask 'yellow' / 'gray'
    cv2.imshow("mask gray", get_mask(obs, 'gray'))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def get_mask(obs, mask_color):
    if mask_color == 'gray':
        image_bgr = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR) # convert from RGB to BGR
        lower_gray = np.array([156, 161, 156])
        upper_gray = np.array([185, 182, 185])

        mask_gray = cv2.inRange(image_bgr, lower_gray, upper_gray)
        mask = cv2.cvtColor(mask_gray, cv2.COLOR_GRAY2BGR) 
    else:
        image_hsv = cv2.cvtColor(obs, cv2.COLOR_RGB2HSV) # convert from RGB to HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([29, 254, 254])

        mask_yellow = cv2.inRange(image_hsv, lower_yellow, upper_yellow)
        mask = cv2.cvtColor(mask_yellow, cv2.COLOR_GRAY2BGR)
    return mask