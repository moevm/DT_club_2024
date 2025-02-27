CONST_UP_DN_MOVE= [0.44, 0]
CONST_LT_RT_MOVE = [0, 1]
CONST_STOP_MOVE = [0, 0]
DELTA_ANGLE = 3
contours = False
turning_states = {
    'turning_left': False,
    'turning_right': False,
    'turning_backward': False,
    'turning_forward': False
}
RENDER_PARAMS = ['human', 'top_down']
RENDER_MODE = RENDER_PARAMS[1]
TAKE_IMAGE = False
directory_path = "src/output"