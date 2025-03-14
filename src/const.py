# Constants
CONST_UP_DN_MOVE= [0.44, 0]
CONST_LT_RT_MOVE = [0, 1]
CONST_STOP_MOVE = [0, 0]
DELTA_ANGLE = 3

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