from PIL import Image
import argparse
import sys
import gym
import pyglet
import os.path
from pyglet.window import key
from gym_duckietown.envs import DuckietownEnv
from src.const import *
from src.move import *
from src.image import *

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
    
# Create output folder
if not os.path.exists(output_path):
    os.makedirs(output_path, exist_ok=True)
    print(f"Directory created: {output_path}")
else:
    print(f"Directory already exists: {output_path}")


env.reset()
env.render()

def set_false(state_dict, key_to_keep):
    for key in state_dict:
        if key != key_to_keep:
            state_dict[key] = False
    return state_dict

@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
    """
    This handler processes keyboard commands that
    control the simulation
    """

    global RENDER_MODE
    
    if symbol == key.TAB:
        RENDER_MODE = RENDER_PARAMS[1] if RENDER_MODE == RENDER_PARAMS[0] else RENDER_PARAMS[0]

    # Toggle turning states using the keyboard
    if symbol == key.J:
        turning_states['turning_left'] = not turning_states['turning_left']
        set_false(turning_states, 'turning_left')
    if symbol == key.L:
        turning_states['turning_right'] = not turning_states['turning_right']
        set_false(turning_states, 'turning_right')
    if symbol == key.K:
        turning_states['turning_backward'] = not turning_states['turning_backward']
        set_false(turning_states, 'turning_backward')
    if symbol == key.I:
        turning_states['turning_forward'] = not turning_states['turning_forward']
        set_false(turning_states, 'turning_forward')
    
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

def update(dt):
    """
    This function is called at every frame to handle
    movement/stepping and redrawing
    """
    
    global TAKE_IMAGE
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

    if turning_states['turning_left']:
        action = move_left(env.cur_angle)
    if turning_states['turning_right']:
        action = move_right(env.cur_angle)
    if turning_states['turning_backward']:
        action = move_down(env.cur_angle)
    if turning_states['turning_forward']:
        action = move_up(env.cur_angle)
        
    if key_handler[key.SPACE]:
        action = np.array(CONST_STOP_MOVE)
        set_false(turning_states, 'all')
    
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

    # image
    if key_handler[key.F]:
        if not TAKE_IMAGE:
            get_bot_image(obs)
            TAKE_IMAGE = True
    else:
        TAKE_IMAGE = False
    
    image_bgr = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR)
    writer_camera.write(image_bgr)

    mask_gray = get_mask(obs, 'gray')  # запись маски 'gray' / 'yellow'
    writer_mask.write(mask_gray)
    
    env.render(RENDER_MODE)

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop
pyglet.app.run()

env.close()