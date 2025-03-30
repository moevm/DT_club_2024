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

RENDER_PARAMS = ['human', 'top_down']
RENDER_MODE = RENDER_PARAMS[1]
TAKE_IMAGE = False
MAX_CONTOUR_AREA = 307200

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
    global TAKE_IMAGE,contours_gray_for_stop,contours_gray_for_move,contours_yellow, lx_y, lx_g, last_steering_angle

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


    if contours_gray_for_stop:
        gray_contourArea = max(contours_gray_for_stop, key=cv2.contourArea)
        contour_area = cv2.contourArea(gray_contourArea)

        if contour_area > MAX_CONTOUR_AREA // 33:
            action -= np.array(CONST_UP_DN_MOVE)
            set_false(turning_states, 'all')
                     
    action = realistic_move(action)

    # Speed boost
    if key_handler[key.LSHIFT]:
        action *= 1.5

    obs, reward, done, info = env.step(action) # RGB
    print("step_count = %s, reward=%.3f" % (env.unwrapped.step_count, reward))
    print("bot position = ", env.cur_pos)

    # Получение контуров желтой и серой разметки каждые 8 кадров
    steps = env.unwrapped.step_count
    if steps % 6 == 0:
        mask_gray_for_move, mask_gray_for_stop = get_mask(obs, 'gray')
        contours_gray_for_move, _ = cv2.findContours(image=mask_gray_for_move, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        contours_gray_for_stop, _ = cv2.findContours(image=mask_gray_for_stop, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)


        mask_yellow = get_mask(obs, 'yellow')
        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    
    if contours_yellow and contours_gray_for_move:
        filtered_contours_yellow = [cnt for cnt in contours_yellow if cv2.contourArea(cnt) > min_contour_area]
        filtered_contours_gray = [cnt for cnt in contours_gray_for_move if cv2.contourArea(cnt) > min_contour_area]
        
        if filtered_contours_yellow and filtered_contours_gray:
            largest_contour_yellow = max(filtered_contours_yellow, key=cv2.contourArea)
            largest_contour_gray = max(filtered_contours_gray, key=cv2.contourArea)
            
            # Вычисляем моменты ж и с линий 
            M_Y = cv2.moments(largest_contour_yellow)
            M_G = cv2.moments(largest_contour_gray)
            
            if M_Y["m00"] != 0 and M_G["m00"] != 0:
                lx_y, lx_g= int(M_Y["m10"] / M_Y["m00"]), int(M_G["m10"] / M_G["m00"]) # X-координата ж и с линий
                
                # Центр между двумя линиями 
                center_between_lines = (lx_y + lx_g) / 2

                image_center = image_width / 2
                
                # насколько смещены от центра
                delta_centre = center_between_lines - image_center

                # Чем больше, тем резче повороты
                Kp = 0.02
                
                # минус, чтобы поворачиваться в нужню сторону
                steering_angle = -Kp * delta_centre
                
                obs, _, _, _ = env.step([0.2, steering_angle])
                last_steering_angle = steering_angle  # Запоминаем угол

    elif last_steering_angle != 0 :
        obs, _, _, _ = env.step([0.2, last_steering_angle])
                
    # image
    if key_handler[key.F]:
        if not TAKE_IMAGE:
            get_bot_image(obs)
            TAKE_IMAGE = True
    else:
        TAKE_IMAGE = False
    
    image_bgr = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR)
    writer_camera.write(image_bgr)

    _, mask_gray = get_mask(obs, 'gray')  # запись маски 'gray' / 'yellow'
    writer_mask.write(mask_gray)
    
    env.render(RENDER_MODE)

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop
pyglet.app.run()

env.close()