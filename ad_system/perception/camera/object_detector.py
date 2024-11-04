import torch
import numpy as np
import cv2, os
import math
from collections import defaultdict
from numpy.matlib import repmat

from ultralytics import YOLO
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider 


class PVDetector: 
    """
        preceding vehicle detector 
    """
    FRAME = 0 
    TIME_STEP = 0.5 

    def __init__(self, intrinsic_params, memory=5, diff_threshold=5): 

        self.intrinsic_params = intrinsic_params
        self.memory = memory
        self.diff_threshold = diff_threshold  # 같은 객체로 인식하기 위한 최소한의 거리
        self.object_list = defaultdict(list)  # list : [FRAME, Z, X, REL_Vz, REL_Vx, MEMORY] 

        # object detector
        self._model = torch.hub.load(os.path.abspath("resources") + "/yolov5", "yolov5s", source='local') 

    def get_preceding_vehicles(self, _rgb_img, _depth_img, actor, target=None, camera=None): 
        # if target == None: 
        #     return self.object_detection(_rgb_img, _depth_img, actor)
        # else: 
        #     return self.gt_object_detection(_rgb_img, _depth_img, actor, target, camera) 
        return self.object_detection(_rgb_img, _depth_img, actor, target, camera)


    def _build_projection_matrix(self, w, h, fov, is_behind_camera=False):
        focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
        K = np.identity(3)

        if is_behind_camera:
            K[0, 0] = K[1, 1] = -focal
        else:
            K[0, 0] = K[1, 1] = focal

        K[0, 2] = w / 2.0
        K[1, 2] = h / 2.0
        return K


    def _get_image_point(self, loc, K, w2c):
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(w2c, point)

        # New we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth componebonent also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]


    def gt_object_detection(self, _rgb_img, _depth_img, actor, target, camera):

        # get_transform(): 차량의 현재 위치와 회전 정보
        # get_forward_vector(): 해당 변환에서의 전방 방향을 단위 벡터로 반환

        # 월드 좌표계에서 특정 객체나 점의 위치를 카메라 좌표계로 변환하기 위해 이 역행렬을 사용
        world_2_camera = np.array(camera.get_transform().get_inverse_matrix()) 
        K = self._build_projection_matrix(1000, 400, 90) 
        
        bbox_3d = target.bounding_box 
        distance = target.get_transform().location.distance(actor.get_transform().location) 

        results = [] 
        if distance < 130: 
            forward_vec = actor.get_transform().get_forward_vector()  
            ray = target.get_transform().location - actor.get_transform().location 

            # 전방 차량이면
            if forward_vec.dot(ray) > 0: 
                center_point = self._get_image_point(bbox_3d.location, K, world_2_camera)

                verts = [v for v in bbox_3d.get_world_vertices(target.get_transform())]
                x_max = -10000
                x_min = 10000
                y_max = -10000
                y_min = 10000

                for vert in verts:
                    p = self._get_image_point(vert, K, world_2_camera)
                    if p[0] > x_max:
                        x_max = p[0]
                    if p[0] < x_min:
                        x_min = p[0]
                    if p[1] > y_max:
                        y_max = p[1]
                    if p[1] < y_min:
                        y_min = p[1]

                if abs(x_max - x_min) < 999 and abs(y_max - y_min) < 399: 
                    results.append([x_min, y_min, x_max, y_max, 0.99, 2])

        return results

    
    def object_detection(self, _rgb_img, _depth_img, actor, target=None, camera=None):

        if target == None:
            outputs = self._model(_rgb_img) 
            results = outputs.xyxy[0] 
        else: 
            results = self.gt_object_detection(_rgb_img, _depth_img, actor, target, camera)

        for result in results:
            xmin, ymin, xmax, ymax, confidence, cls = result 
            xmin, ymin, xmax, ymax = map(int, [xmin, ymin, xmax, ymax])

            # 차량이 탐지된 경우
            if int(cls) == 2: 
                center_x, center_y = int((xmin + xmax) / 2), int((ymin + ymax) / 2)  

                pv_ego_coord = self._camera_to_ego(center_x, center_y, _depth_img[center_y][center_x], actor) 
                self._object_tracking(pv_ego_coord[0], pv_ego_coord[1])
                
                cv2.rectangle(_rgb_img, (xmin, ymin), (xmax, ymax), (255, 0, 0), 1) 
                cv2.circle(_rgb_img, (center_x, center_y), 3, (0, 255, 0), -1)
                text = text = f"CLASS: {int(cls)} Distance: {round(np.linalg.norm(pv_ego_coord), 2)}" 
                cv2.putText(_rgb_img, text, (xmin, ymin), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA) 
        
        self.FRAME += 1 
        self._reduce_memory() 

        return _rgb_img, {i: self.object_list[i][1:-1] for i in list(self.object_list.keys()) if len(self.object_list[i]) != 0}

    def _reduce_memory(self): 

        # 리스트에 들어있는 object를 모두 순회하며
        for i in list(self.object_list.keys()):  

            # id가 i인 object의 정보가 없는 경우
            if len(self.object_list[i]) == 0: 
                continue 
            else: 
                self.object_list[i][5] -= 1 

            # memory가 0인 object의 정보는 지워준다
            if self.object_list[i][5] == 0: 
                self.object_list[i] = []  

    def _camera_to_ego(self, center_x, center_y, d, actor): 
        # camera intrinsic parameters 
        c_x, c_y, f = self.intrinsic_params 

        # pixel to camera coordinate 
        y = c_y - center_y 
        x = c_x - center_x  

        # camera to ego coordinate 
        Y = -x * d / f  # 횡방향 거리
        X = d           # 종방향 거리

        # self._object_tracking(X, Y)

        return [X, Y] 

    def _object_tracking(self, X, Y): 
        X = round(X, 2) 
        Y = round(Y, 2) 

        if len(list(self.object_list.keys())) == 0:  
            # initialize 
            self.object_list[0] = [self.FRAME, X, Y, 0, 0, self.memory]
        else: 

            min_loc_diff = float('inf') 
            same_obj_key = -1 

            # 모든 object 후보를 순회하며 
            for i in list(self.object_list.keys()): 

                # 비어있는 key면 다음 loop
                if len(self.object_list[i]) == 0: 
                    continue 

                frame, x, y, v_x, v_y, memory = self.object_list[i] 

                # 같은 frame에 찍힌 물체면 다음 loop (이전 프레임의 object만 비교)
                if frame == self.FRAME: 
                    continue  
                
                # 위치 차이가 최소인 객체 key값 선택
                rel_dist = np.linalg.norm((X - x, Y - y)) 
                if rel_dist <= min_loc_diff and rel_dist <= self.diff_threshold: 
                    min_loc_diff = rel_dist 
                    same_obj_key = i 

            # 현재 object가 tracking 상태라면, 현재 object 상태로 update 
            if same_obj_key != -1: 
                frame, x, y, v_x, v_y, memory = self.object_list[same_obj_key]  # 이전 object 정보 
                v_x = round((X - x) / self.TIME_STEP, 2) 
                v_y = round((Y - y) / self.TIME_STEP, 2) 
                self.object_list[same_obj_key] = [self.FRAME, X, Y, v_x, v_y, self.memory] 

            else: 
                self.object_list[i + 1] = [self.FRAME, X, Y, 0, 0, self.memory]
