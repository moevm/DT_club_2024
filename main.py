from PIL import Image
import cv2
import argparse
import sys

import gym
import numpy as np
import pyglet
from pyglet.window import key

from gym_duckietown.envs import DuckietownEnv

DOWN_UP_MOVE= [0.44, 0]
RIGHT_LEFT_MOVE = [0, 1]
CONST_STOP_MOVE = [0, 0]
delta = 3

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


@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """

    # RENDER_MODE SWITCH

    global RENDER_MODE
    
    if key_handler[key.TAB]:
        if RENDER_MODE == RENDER_PARAMS[0]:
            RENDER_MODE = RENDER_PARAMS[1]
        else:
            RENDER_MODE = RENDER_PARAMS[0]
    if symbol == key.BACKSPACE or symbol == key.SLASH:
        print("RESET")
        env.reset()
        env.render()
    elif symbol == key.PAGEUP:
        env.unwrapped.cam_angle[0] = 0
    elif symbol == key.ESCAPE:
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


RENDER_PARAMS = ['human', 'top_down']
RENDER_MODE = RENDER_PARAMS[0]
def update(dt):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """

    action = np.array([0.0, 0.0])

    # Moovement

    if key_handler[key.W]:
        action += np.array(DOWN_UP_MOVE)
    if key_handler[key.S]:
        action -= np.array(DOWN_UP_MOVE)
    if  key_handler[key.A]:
        action += np.array(RIGHT_LEFT_MOVE)
    if key_handler[key.D]: 
        action -= np.array(RIGHT_LEFT_MOVE)
    if key_handler[key.SPACE]:
        action = np.array(CONST_STOP_MOVE)
    if key_handler[key.LEFT]: 
        action = move_left(env.cur_angle)
    if key_handler[key.RIGHT]:
        action = move_right(env.cur_angle)
    if key_handler[key.UP]: 
        action = move_up(env.cur_angle)
    if key_handler[key.DOWN]:
        action = move_down(env.cur_angle)

    """
    Here you can set the movement for the duckiebot using action
    """
    action = realistic_move(action)

    # Speed boost
    if key_handler[key.LSHIFT]:
        action *= 1.5

    obs, reward, done, info = env.step(action)
    print("step_count = %s, reward=%.3f" % (env.unwrapped.step_count, reward))
    print("bot position = ", env.cur_pos)

    env.render(RENDER_MODE)



def move_left(current_angle):
    action = [0, 0]
    angle_deg = np.rad2deg(current_angle)

    if (angle_deg < 0 and np.abs(angle_deg + 180) < delta) or (angle_deg > 0 and np.abs(angle_deg - 180) < delta):
        action = np.array(DOWN_UP_MOVE)
    else:
        if angle_deg >= 0: 
            action = np.array([0, RIGHT_LEFT_MOVE[1] / 2]) 
        else: 
            action = -np.array([0, RIGHT_LEFT_MOVE[1] / 2]) 

    return action


def move_up(current_angle):
    action = [0, 0]
    angle_deg = np.rad2deg(current_angle)

    if np.abs(angle_deg - 90) <= delta:
        action = np.array(DOWN_UP_MOVE)
    else:
        if np.abs(angle_deg) > 90:
            action = -np.array([0, RIGHT_LEFT_MOVE[1] / 2]) 
        else: 
            action = np.array([0, RIGHT_LEFT_MOVE[1] / 2]) 

    return action


def move_down(current_angle):
    action = [0, 0]
    angle_deg = np.rad2deg(current_angle)

    if np.abs(angle_deg + 90) <= delta:
        action = np.array(DOWN_UP_MOVE)
    else:
        if np.abs(angle_deg) > 90:
            action = np.array([0, RIGHT_LEFT_MOVE[1] / 2]) 
        else: 
            action = -np.array([0, RIGHT_LEFT_MOVE[1] / 2]) 

    return action


def move_right(current_angle):
    action = [0, 0]
    angle_deg = np.rad2deg(current_angle)
    
    if np.abs(angle_deg) <= delta:
        action = np.array(DOWN_UP_MOVE)
    else:
        if angle_deg > 0: 
            action = -np.array([0, RIGHT_LEFT_MOVE[1] / 2]) 
        else: 
            action = np.array([0, RIGHT_LEFT_MOVE[1] / 2]) 

    return action



    




pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop
pyglet.app.run()

env.close()