from PIL import Image
import cv2
import argparse
import sys

import gym
import numpy as np
import pyglet
from pyglet.window import key

from gym_duckietown.envs import DuckietownEnv

CONST_UP_DN_MOVE= [0.44, 0]
CONST_LT_RT_MOVE = [0, 1]
CONST_STOP_MOVE = [0, 0]
DELTA_ANGLE = 3

parser = argparse.ArgumentParser()
parser.add_argument("--env-name", default="Duckietown-udem1-v0")
parser.add_argument("--map-name", default="udem1")
parser.add_argument("--distortion", default=False, action="store_true")
parser.add_argument("--camera_rand", default=False, action="store_true")
parser.add_argument("--draw-curve", action="store_true", help="draw the lane following curve")
parser.add_argument("--draw-bbox", action="store_true", help="draw collision detection bounding boxes")
parser.add_argument("--domain-rand", action="store_true", help="enable domain randomization")
parser.add_argument("--dynamics_rand", action="store_true", help="enable dynamics randomization")
parser.add_argument("--frame-skip", default=1, type=int, help="number of frames to skip")
parser.add_argument("--seed", default=42, type=int, help="seed")
args = parser.parse_args()

if args.env_name and args.env_name.find("Duckietown") != -1:
    env = DuckietownEnv(
        seed=args.seed,
        map_name=args.map_name,
        draw_curve=args.draw_curve,
        draw_bbox=args.draw_bbox,
        domain_rand=args.domain_rand,
        frame_skip=args.frame_skip,
        distortion=args.distortion,
        camera_rand=args.camera_rand,
        dynamics_rand=args.dynamics_rand,
    )
else:
    env = gym.make(args.env_name)

env.reset()
env.render()

turning_left = False
turning_right = False
turning_backward = False
turning_forward = False 
IMAGE_TAKEN = False
RENDER_PARAMS = ['human', 'top_down']
RENDER_MODE = RENDER_PARAMS[1]

writer_camera = cv2.VideoWriter(
    "output.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"),
    20,
    (640, 480), # width, height
) 
writer_mask = cv2.VideoWriter(
    "output_mask.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"),
    20,
    (640, 480), # width, height
)

@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """
    global turning_left, turning_right, turning_backward, turning_forward

    # RENDER_MODE SWITCH
    
    global RENDER_MODE
    if symbol == key.TAB:
        RENDER_MODE = RENDER_PARAMS[1] if RENDER_MODE == RENDER_PARAMS[0] else RENDER_PARAMS[0]

    # Toggle turning LEFT on 'J' key press
    if symbol == key.J:
        turning_left = not turning_left
    # Toggle turning RIGHT on 'L' key press
    if symbol == key.L:
        turning_right = not turning_right
    # Toggle turning BACKWARD on 'K' key press
    if symbol == key.K:
        turning_backward = not turning_backward
    # Toggle turning FORWARD on 'I' key press
    if symbol == key.I:
        turning_forward = not turning_forward
    
    if symbol == key.BACKSPACE or symbol == key.SLASH:
        print("RESET")
        env.reset()
        env.render()
    elif symbol == key.PAGEUP:
        env.unwrapped.cam_angle[0] = 0
    elif symbol == key.ESCAPE:
        writer_camera.release()
        writer_mask.release() 
        env.close()
        sys.exit(0)

        
# Register a keyboard handler
key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)


def realistic_move(action):
    wheel_distance = 0.102
    min_rad = 0.08

    v1 = action[0]
    v2 = action[1]
    # Limit radius of curvature
    if v1 == 0 or abs(v2 / v1) > (min_rad + wheel_distance / 2.0) / (min_rad - wheel_distance / 2.0):
        # adjust velocities evenly such that condition is fulfilled
        delta_v = (v2 - v1) / 2 - wheel_distance / (4 * min_rad) * (v1 + v2)
        v1 += delta_v
        v2 -= delta_v

    action[0] = v1
    action[1] = v2

    return action

def get_bot_image(obs):
    camera_image = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR) # convert from RGB to BGR

    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    hsv_image = cv2.cvtColor(obs, cv2.COLOR_RGB2HSV) # convert from RGB to HSV
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    
    cv2.imshow("camera image", camera_image)
    cv2.imshow("mask yellow", mask_yellow)
    cv2.waitKey(0)

def get_gray_mask_image(obs): 
    camera_image = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR) # convert from RGB to BGR
    
    lower_gray = np.array([155, 160, 155])
    upper_gray = np.array([185, 180, 185])
    mask_gray = cv2.inRange(camera_image, lower_gray, upper_gray)
    mask = cv2.cvtColor(mask_gray, cv2.COLOR_GRAY2BGR) 

    return mask

def move_left(current_angle):
    action = [0, 0]
    angle_deg = np.rad2deg(current_angle)

    if (angle_deg < 0 and np.abs(angle_deg + 180) < DELTA_ANGLE) or (angle_deg > 0 and np.abs(angle_deg - 180) < DELTA_ANGLE):
        #if the angle within the delta_angle is -180 or 180 degrees
        action = np.array(CONST_UP_DN_MOVE)
    else:
        if angle_deg >= 0: 
            #if the angle of rotation of the bot is positive, then it is faster and better to turn it to the left
            action = np.array([0, CONST_LT_RT_MOVE[1] / 2]) 
        else: 
            #if the angle of rotation of the bot os negative, then it is faster and better to turn it ti the right
            action = -np.array([0, CONST_LT_RT_MOVE[1] / 2]) 

    return action


def move_up(current_angle):
    action = [0, 0]
    angle_deg = np.rad2deg(current_angle)

    if np.abs(angle_deg - 90) <= DELTA_ANGLE:
        action = np.array(CONST_UP_DN_MOVE)
        #if the angle within the delta_angle is 90 degrees
    else:
        if np.abs(angle_deg) > 90:
            #if the angle of rotation of the bot is greater than 90 degrees, then it is faster and better to turn it to the right
            action = -np.array([0, CONST_LT_RT_MOVE[1] / 2]) 
        else: 
            #if the angle of rotation of the bot is less than 90 degrees, then it is faster and better to turn it to the left
            action = np.array([0, CONST_LT_RT_MOVE[1] / 2]) 

    return action


def move_down(current_angle):
    action = [0, 0]
    angle_deg = np.rad2deg(current_angle)

    if np.abs(angle_deg + 90) <= DELTA_ANGLE:
        #if the angle within the  delta_angle is -90 degrees
        action = np.array(CONST_UP_DN_MOVE)
    else:
        if np.abs(angle_deg) > 90:
            #if the angle of rotation of the bot is greater than 90 degrees, then it is faster and better to turn it to the left
            action = np.array([0, CONST_LT_RT_MOVE[1] / 2]) 
        else: 
            #if the angle of rotation of the bot is less than 90 degrees, then it is faster and better to turn it to the right
            action = -np.array([0, CONST_LT_RT_MOVE[1] / 2]) 

    return action


def move_right(current_angle):
    action = [0, 0]
    angle_deg = np.rad2deg(current_angle)
    
    if np.abs(angle_deg) <= DELTA_ANGLE:
        #if the angle within the delta_angle is 0 degrees
        action = np.array(CONST_UP_DN_MOVE)
    else:
        if angle_deg > 0: 
            #if the angle of rotation of the bot is positive, then it is faster and better to turn it to the right
            action = -np.array([0, CONST_LT_RT_MOVE[1] / 2]) 
        else: 
            #if the angle of rotation of the bot is negative, then it is faster and better to turn it to the left
            action = np.array([0, CONST_LT_RT_MOVE[1] / 2]) 

    return action


def update(dt):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """
    global turning_left, turning_right, turning_backward, turning_forward, IMAGE_TAKEN

    action = np.array([0.0, 0.0])

    # Movement handling

    if key_handler[key.UP] or key_handler[key.W]:
        action += np.array(CONST_UP_DN_MOVE)
    if key_handler[key.DOWN] or key_handler[key.S]:
        action -= np.array(CONST_UP_DN_MOVE)
    if key_handler[key.LEFT] or key_handler[key.A]:
        action += np.array(CONST_LT_RT_MOVE)
    if key_handler[key.RIGHT] or key_handler[key.D]: 
        action -= np.array(CONST_LT_RT_MOVE)

    if turning_left:
        action = move_left(env.cur_angle)
    if turning_right:
        action = move_right(env.cur_angle)
    if turning_backward:
        action = move_down(env.cur_angle)
    if turning_forward:
        action = move_up(env.cur_angle)
        
    if key_handler[key.SPACE]:
        action = np.array(CONST_STOP_MOVE)
    
    """
    Here you can set the movement for the duckiebot using action
    """
    action = realistic_move(action)

    # Speed boost
    if key_handler[key.LSHIFT]:
        action *= 1.5

    obs, reward, done, info = env.step(action) # RGB
    print("step_count = %s, reward=%.3f" % (env.unwrapped.step_count, reward))
    print("bot position = ", env.cur_pos)

    if key_handler[key.F]:
        if not IMAGE_TAKEN:
            get_bot_image(obs)
            IMAGE_TAKEN = True
    else:
        IMAGE_TAKEN = False
        
    bgr_image = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR)
    writer_camera.write(bgr_image)
    mask = get_gray_mask_image(obs) 
    writer_mask.write(mask)

    env.render(RENDER_MODE)


pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop
pyglet.app.run()

env.close()