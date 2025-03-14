import numpy as np
from src.const import *

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