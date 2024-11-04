from itertools import combinations
from collections import defaultdict
from collections import deque 

LANE_WIDTH_THRESHOLD = 2.7
NUMBER_TWO = 2
PREDICTION_TIME = 3 
PREV_EGO_LANE = None 
TARGET_MEMORY = deque([None], maxlen=10)

def target_selection_preceding(target_candidates, lane_info, ego_velocity):
    target = None
    if target_candidates:
        preceding_candidates = list()                       # 전방 차량 후보군
        abs_vel_x_ego, abs_vel_y_ego = ego_velocity         # 제어차량 속도
        ego_lane = find_ego_lane(lane_info)                 # 인접 차선

        for rel_pos_x_tar, rel_pos_y_tar, rel_vel_x_tar, rel_vel_y_tar in target_candidates.values():
            abs_vel_x_tar, abs_vel_y_tar = rel_vel_x_tar + abs_vel_x_ego, rel_vel_y_tar + abs_vel_y_ego

            predict_pos_x_tar = rel_pos_x_tar + abs_vel_x_tar * PREDICTION_TIME
            predict_pos_y_tar = rel_pos_y_tar + abs_vel_y_tar * PREDICTION_TIME

            if ego_lane:
                left_lane_y, right_lane_y  = sorted([(predict_pos_x_tar-n)/m for m, n in ego_lane])
                if left_lane_y <= predict_pos_y_tar <= right_lane_y:
                    preceding_candidates.append([rel_pos_x_tar, rel_pos_y_tar, rel_vel_x_tar, rel_vel_y_tar, predict_pos_x_tar])

        if preceding_candidates:
            target = sorted(preceding_candidates, key = lambda x : x[-1])[0][0:-1]  # 예측 종방향 거리가 가장 짧은 후보 선택

    if target is None and len(TARGET_MEMORY) > 0: 
        target = TARGET_MEMORY.pop() 
    else: 
        TARGET_MEMORY.append(target)

    return target

def find_ego_lane(lane_info):
    global PREV_EGO_LANE 

    ego_lane = None

    if len(lane_info) >= NUMBER_TWO:
        # 각 차선들의 x절편 구하기 
        lane_start_points = [-n/m for m, n in lane_info]

        # 절편간 거리가 2m 이하면 경로 삭제
        filter_lane_info = [lane_info[i] for i in range(len(lane_start_points)-1) if abs(lane_start_points[i]-lane_start_points[i+1]) >= LANE_WIDTH_THRESHOLD]
        # 마지막 차선 확인
        if abs(lane_start_points[-1]-lane_start_points[-2]) >= LANE_WIDTH_THRESHOLD:       
            filter_lane_info.append(lane_info[-1])

        filterd_lane_start_points = [-n/m for m, n in filter_lane_info]

        if len(filterd_lane_start_points) >= NUMBER_TWO:
            # 카메라 기준 종 방향으로 가장 가까운 2개 lane 선택
            min_abs_indices = [filterd_lane_start_points.index(value) for value in sorted(filterd_lane_start_points, key = abs)[:2]]
            ego_lane = [filter_lane_info[i] for i in min_abs_indices] 

    if ego_lane is None:
        ego_lane = PREV_EGO_LANE 
    else:
        PREV_EGO_LANE = ego_lane

    return ego_lane

