# Constants
CONST_UP_DN_MOVE= [0.44, 0]
UP_MOVE = [0.2, 0]
CONST_LT_RT_MOVE = [0, 1]
CONST_STOP_MOVE = [0.0, 0.0]
DELTA_ANGLE = 3
lower_gray = [156, 161, 156]
upper_gray = [220, 220, 220]
lower_yellow = [20, 100, 100]
upper_yellow = [29, 254, 254]
lower_red = [0, 0, 150]
upper_red = [100, 100, 255]
lx_y, ly_y,lx_g, ly_g = None, None, None, None
contours_gray_for_stop = False
contours_gray_for_move = False
contours_yellow = False
image_width = 640
image_height = 480
MAX_CONTOUR_AREA = 640 * 480
min_contour_area = 10
last_steering_angle = 0
TAKE_IMAGE = False
Enable_Movement = False
last_action = -1
count_stop_time = 0
count_move_time = 0
# States
turning_states = {
    'turning_left': False,
    'turning_right': False,
    'turning_backward': False,
    'turning_forward': False
}
TAKE_IMAGE = False

# Simulation renders
RENDER_PARAMS = ['human', 'top_down']
RENDER_MODE = RENDER_PARAMS[1]

# Paths
output_path = "output/"