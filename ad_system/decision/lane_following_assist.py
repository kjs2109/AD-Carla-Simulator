import math
import numpy as np
from itertools import combinations

LFA_MODE = 0
LOW_SPEED_MODE = 0
HIGH_SPEED_MODE = 1

HIGH_SPEED_MODE_PREV_ERROR = 0
HIGH_SPEED_MODE_INTEGRAL = 0
LOW_SPEED_MODE_PREV_ERROR = 0
LOW_SPEED_MODE_INTEGRAL = 0

NUMBER_TWO = 2
LANE_WIDTH_THRESHOLD = 2
MODE_CONVERSION_SPEED_KPH = 70

PREV_EGO_LANE = None 

def lane_following_assist_purepursuit(lane_info : list, ego_velocity : list):
    target_lane = find_ego_lane(lane_info)
    
    steer = 0

    if target_lane:
        low_speed_steer = calculate_steer_in_low_speed_purepursuit(target_lane)

        high_speed_steer = calculate_steer_in_high_speed_purepursuit(target_lane)

        steer = lfa_mode_selection(low_speed_steer, high_speed_steer, ego_velocity)
        
    return steer


def lfa_mode_selection(low_speed_steer, high_speed_steer, ego_velocity):
    global LFA_MODE, HIGH_SPEED_MODE_PREV_ERROR, HIGH_SPEED_MODE_INTEGRAL, LOW_SPEED_MODE_PREV_ERROR, LOW_SPEED_MODE_INTEGRAL

    EGO_SPEED_KPH = ego_velocity[0]

    if LFA_MODE == LOW_SPEED_MODE:
        if EGO_SPEED_KPH >= MODE_CONVERSION_SPEED_KPH:
            LFA_MODE = HIGH_SPEED_MODE
    else:
        if EGO_SPEED_KPH < MODE_CONVERSION_SPEED_KPH:
            LFA_MODE = LOW_SPEED_MODE

    if LFA_MODE == LOW_SPEED_MODE:
        steer = low_speed_steer
        HIGH_SPEED_MODE_PREV_ERROR = 0
        HIGH_SPEED_MODE_INTEGRAL = 0
    else:
        steer = high_speed_steer
        LOW_SPEED_MODE_PREV_ERROR = 0
        LOW_SPEED_MODE_INTEGRAL = 0

    return steer


def calculate_steer_in_low_speed_purepursuit(target_lane):
    wheelbase = 5
    lookahead_distance = 30

    target_lane_y = sum([(lookahead_distance-n)/m for m, n in target_lane])/2

    steer = math.atan2(2 * wheelbase * target_lane_y, lookahead_distance ** 2)
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    return steer

def calculate_steer_in_high_speed_purepursuit(target_lane):
    wheelbase = 5
    lookahead_distance = 60

    target_lane_y = sum([(lookahead_distance - n) / m for m, n in target_lane]) / 2

    steer = math.atan2(2 * wheelbase * target_lane_y, lookahead_distance ** 2)
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    return steer

def find_ego_lane(lane_info : list):
    
    global PREV_EGO_LANE 
    ego_lane = None

    if len(lane_info) >= NUMBER_TWO:
        # 각 차선들의 y절편 구하기
        lane_start_points = [-n/m for m, n in lane_info]

        # 절편간 거리가 2m 이하면 경로 삭제
        filter_lane_info = [lane_info[i] for i in range(len(lane_start_points)-1) if abs(lane_start_points[i]-lane_start_points[i+1]) >= LANE_WIDTH_THRESHOLD]
        if abs(lane_start_points[-1]-lane_start_points[-2]) >= LANE_WIDTH_THRESHOLD:       # 마지막 차선이 빠지는지 확인
            filter_lane_info.append(lane_info[-1])

        filterd_lane_start_points = [-n/m for m, n in filter_lane_info]

        if len(filterd_lane_start_points) >= NUMBER_TWO:
            min_abs_indices = [filterd_lane_start_points.index(value) for value in sorted(filterd_lane_start_points, key = abs)[:2]]
            ego_lane = [filter_lane_info[i] for i in min_abs_indices]

    if ego_lane is None:
        ego_lane = PREV_EGO_LANE 
    else:
        PREV_EGO_LANE = ego_lane

    return ego_lane
