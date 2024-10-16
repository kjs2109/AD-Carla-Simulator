#!/usr/bin/env python

# Copyright (c) 2020-2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>. 
 
import os 
from distutils.util import strtobool
import math
import time
import numpy as np
import pandas as pd 
from matplotlib import cm
import carla
import cv2
import open3d as o3d
import copy
from itertools import combinations 

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from perception import PVDetector, LaneDetector  
from decision import ACCSystem, arbitration, target_selection_preceding, lane_following_assist_purepursuit 
from controller import engine_control, brake_control, steer_control 



class AutonomousDrivingSystem(BasicControl): 

    def __init__(self, actor, args=None): 
        super(AutonomousDrivingSystem, self).__init__(actor) 
        print(f"======================= autonomous system is running ... =======================") 
        self.tick = time.time()
        self.last_frame_time = None

        # synchronous mode 
        self.dt = 0.02  # 50 FPS
        self._set_synchronous_mode(fixed_delta_seconds=self.dt)  

        # driving log 
        self.ego_velocity_logging = [] 
        self.target_velocity_logging = [] 
        self.ego_acceleration_logging = [] 
        self.ego_target_distance_logging = [] 
        self.desired_acceleration_logging = []
        self.time_to_collision_logging = [] 
        self.headway_time_logging = []
        self.acc_mode_logging = [] 

        # initialization 
        self._init_control() 
        self._actor.show_debug_telemetry(True) 

        self.farthest_depth = 1000 # m
        self.image_width = 1000 
        self.image_height = 400 
        self.image_fov = 90 

        self.c_x = int(self.image_width // 2)
        self.c_y = int(self.image_height // 2) 
        self.focal = self.image_width / (2*math.tan(self.image_fov*math.pi/360))
        self.intrinsic_parameters = [self.c_x, self.c_y, self.focal]

        self.rgb_image = None 
        self.depth_image = None 
        self.depth_vis = None 
        self.imu_acceleration = None 

        self.spectator = CarlaDataProvider.get_world().get_spectator() 

        self._set_camera_sensor()
        self._set_depth_sensor()
        self._set_imu_sensor()   

        self.pv_detector = PVDetector(self.intrinsic_parameters) 
        self.lane_detector = LaneDetector(self.intrinsic_parameters) 
        self.acc_system = ACCSystem(desired_velocity_kph=50, desired_distance=30, maximum_acc_distance=80, dt=self.dt) 
        # self.pid_controller = PidController(headway_time=2, distance_gains=[0.5, 0, 0], velocity_gains=[0.1, 0, 0], dt=self.dt) 

    def _set_synchronous_mode(self, fixed_delta_seconds=0.05):

        world = CarlaDataProvider.get_world()
        settings = world.get_settings()

        settings.synchronous_mode = True  # 동기 모드 활성화
        settings.fixed_delta_seconds = fixed_delta_seconds  # 고정 시간 스텝 설정
        world.apply_settings(settings)

    def _init_control(self): 

        self.control = carla.VehicleControl() 
        self.control.steer = 0 
        self.control.throttle = 0 
        self.control.brake = 0 

    # imu sensor 
    def _set_imu_sensor(self): 

        bp = CarlaDataProvider.get_world().get_blueprint_library().find('sensor.other.imu')
        imu_location = carla.Location(x=0.0, y=0.0, z=0.0)
        imu_rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        imu_transform = carla.Transform(imu_location, imu_rotation)

        self._imu_sensor = CarlaDataProvider.get_world().spawn_actor(bp, imu_transform, attach_to = self._actor)
        self._imu_sensor.listen(lambda data: self._on_imu_update(data)) 

    def _on_imu_update(self, data):
        self.imu_acceleration = data.accelerometer 

    # depth camera sensor 
    def _set_depth_sensor(self): 

        bp = CarlaDataProvider.get_world().get_blueprint_library().find('sensor.camera.depth')
        bp.set_attribute('image_size_x', str(self.image_width))
        bp.set_attribute('image_size_y', str(self.image_height))
        bp.set_attribute('fov', str(self.image_fov)) 

        self._depth_actor = CarlaDataProvider.get_world().spawn_actor(bp, carla.Transform(carla.Location(x=2.3, z=1.4)), attach_to=self._actor)
        self._depth_actor.listen(lambda depth: self._on_depth_update(depth)) 

    def _on_depth_update(self, depth):

        if not depth:
            return

        depth_data = np.frombuffer(depth.raw_data, dtype=np.dtype("uint8"))
        np_image = np.reshape(depth_data, (depth.height, depth.width, 4))
        np_image = np_image[:, :, :3]
        np_image = np_image[:, :, ::-1]

        np_image = np_image[:,:,0]+np_image[:,:,1]*256.0+np_image[:,:,2]*256.0*256.0
        np_image = (np_image) / (256.0*256.0*256.0-1.0)

        np_image = (np_image)*self.farthest_depth

        self.depth_image = np_image 
        self.depth_vis = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET) 

    # carmera sensor 
    def _set_camera_sensor(self):
        bp = CarlaDataProvider.get_world().get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x',  str(self.image_width))
        bp.set_attribute('image_size_y', str(self.image_height)) 
        bp.set_attribute('fov', str(self.image_fov))

        self._camera_actor = CarlaDataProvider.get_world().spawn_actor(bp, carla.Transform(carla.Location(x=2.3, z=1.4)), attach_to=self._actor)
        self._camera_actor.listen(lambda image: self._on_camera_update(image)) 

    def _on_camera_update(self, image):

        if not image:
            return

        image_data = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        np_image = np.reshape(image_data, (image.height, image.width, 4))
        np_image = np_image[:, :, :3]
        np_image = np_image[:, :, ::-1]

        self.rgb_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB) 


    def save_log(self, saved_folder): 
        if self.ego_velocity_logging == [] or self.target_velocity_logging == []: 
            return 
        
        date_time = time.strftime('%Y%m%d-%H%M%S', time.localtime(time.time()))
        saved_folder = os.path.join(saved_folder, 'temp', date_time)

        os.makedirs(saved_folder, exist_ok=True) 
        saved_path = os.path.join(saved_folder, 'log_data.csv')
        
        velocity_df = pd.DataFrame({
            'ego_velocity': self.ego_velocity_logging, 
            'target_velocity': self.target_velocity_logging, 
            'ego_acceleration': self.ego_acceleration_logging, 
            'ego_target_distance': self.ego_target_distance_logging, 
            'desired_acceleration': self.desired_acceleration_logging,
            'time_to_collision': self.time_to_collision_logging, 
            'headway_time': self.headway_time_logging,
            'acc_mode': self.acc_mode_logging, 
        })

        velocity_df.to_csv(saved_path, index=False)

    def _log_driving_data(self, ego_vehicle_speed, target_vehicle_speed, ego_acceleration, ego_target_distance, acceleration, acc_mode=-1): 
        
        time_to_collision = ego_target_distance / abs((ego_vehicle_speed - target_vehicle_speed + 1e-5))
        headway_time = ego_target_distance / (ego_vehicle_speed + 1e-5)
        
        self.ego_velocity_logging.append(ego_vehicle_speed)
        self.target_velocity_logging.append(target_vehicle_speed) 
        self.ego_acceleration_logging.append(ego_acceleration)
        self.ego_target_distance_logging.append(ego_target_distance) 
        self.desired_acceleration_logging.append(acceleration) 
        self.time_to_collision_logging.append(time_to_collision) 
        self.headway_time_logging.append(headway_time)
        self.acc_mode_logging.append(acc_mode)

    def _visualize_camera_info(self, object_detected_image, line_detected_image, ego_vehicle_speed, target_vehicle_speed, ego_target_distance):   

        # concated_image = np.vstack((object_detected_image, line_detected_image, self.depth_vis))
        concated_image = np.vstack((object_detected_image, line_detected_image)) 

        ego_vehicle_speed_kph = ego_vehicle_speed * 3.6
        target_vehicle_speed_kph = target_vehicle_speed * 3.6 

        ego_info_text = f'Ego Speed: {round(ego_vehicle_speed, 2)} m/s [{round(ego_vehicle_speed_kph, 2)} km/h]' 
        target_info_text = f'Target Speed: {round(target_vehicle_speed, 2)} m/s [{round(target_vehicle_speed_kph, 2)} km/h]' 
        distance_text = f'Distance: {round(ego_target_distance, 2)} m' 

        for i, text_line in enumerate([ego_info_text, target_info_text, distance_text]):
            cv2.putText(concated_image, text_line, (10, 30+i*18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        cv2.imshow('AD System Camera', concated_image)
        cv2.waitKey(1) 

    def _spectator_update(self):

        spectator_transform = self._actor.get_transform()
        spectator_transform.location.x += 0
        spectator_transform.location.z += 13
        spectator_transform.location.y += 10  
        spectator_transform.rotation.pitch = -30
        self.spectator.set_transform(spectator_transform) 


    def reset(self):
        self.save_log('./saved/relative_speed_headway_distance-17')
        print('======================= autonomous system is terminated ... =======================') 
        print('simulation time: ', round(time.time() - self.tick, 2), 's')

        # synchronous mode termination
        world = CarlaDataProvider.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        if self._actor and self._actor.is_alive:
            self._actor = None
            # self._actor.destroy() 

    def run_step(self):

        world = CarlaDataProvider.get_world()
        world.tick()  # 프레임 수동으로 조절 (동기 모드)

        # 프레임 간 시간 출력
        # snapshot = world.get_snapshot()
        # current_simulation_time = snapshot.timestamp.elapsed_seconds  # elapsed time 
        # delta_time = snapshot.timestamp.delta_seconds  # dt
        # print(f"Frame Time Delta (from CARLA): {round(delta_time, 4)} seconds")
        # print(f"Current Simulation Time: {round(current_simulation_time, 4)} seconds")
        
        target = None
        for actor in CarlaDataProvider.get_world().get_actors():
            # find target vehicle
            if actor.type_id.endswith('mkz_2017'):   
                target = actor
                break 

        # postion (m)
        ego_position = [self._actor.get_location().x, self._actor.get_location().y]
        target_position = [target.get_location().x, target.get_location().y] 

        # velocity (m/s) 
        ego_speed = self._actor.get_velocity() 
        ego_vehicle_velocity = [-ego_speed.y, -ego_speed.x]  # carla 좌표로 변환 
        ego_vehicle_speed = np.linalg.norm(ego_vehicle_velocity)  
        
        target_speed = target.get_velocity() 
        target_vehicle_velocity = [-target_speed.y, -target_speed.x] 
        target_vehicle_speed = np.linalg.norm(target_vehicle_velocity)

        # acceleration (m/s^2)
        ego_acceleration = -self._actor.get_acceleration().y

        # relative distance (m)
        # ego_target_distance = target.get_transform().location.distance(self._actor.get_transform().location)
        ego_target_distance = np.linalg.norm([ego_position[0] - target_position[0], ego_position[1] - target_position[1]]) - 5  # vehicle length
        
        # yaw angle (degree)
        # target_transform = target.get_transform()  # Get the transformation of the target vehicle
        # target_yaw = target_transform.rotation.yaw 

        # ego_transform = self._actor.get_transform()  # Get the transformation of the ego vehicle
        # ego_yaw = ego_transform.rotation.yaw

        # print('target_yaw:', np.deg2rad(target_yaw), 'ego_yaw:', np.deg2rad(ego_yaw)) 

        """ Perception """
        
        lane_info = list()
        objects_positions = list()

        # camera object detection
        if self.rgb_image is not None and self.depth_image is not None:
            # object_detected_image, objects_positions = self.pv_detector.get_preceding_vehicles(self.rgb_image, self.depth_image, self._actor) 
            object_detected_image, objects_positions = self.pv_detector.get_preceding_vehicles(self.rgb_image, self.depth_image, self._actor, target, self._camera_actor)
        # print(objects_positions, end=' ')

        # camera lane detection
        if self.rgb_image is not None and self.depth_image is not None:
            line_detected_image, lane_info = self.lane_detector.get_lanes(self.rgb_image, self.depth_image)


        if objects_positions is None or lane_info is None:
            # get_lanes()는 왼,오 하나라도 None이면 None 반환 (허프변환의 결과가 없는 경우) 
            return

        # camera visualization (debugging)
        if objects_positions or lane_info:
            self._visualize_camera_info(
                object_detected_image, line_detected_image,  # image 
                ego_vehicle_speed, target_vehicle_speed, ego_target_distance,  # gt info 
            )  # TODO: predict info 


        """ Decision """

        # target selection
        target_preceding = target_selection_preceding(objects_positions, lane_info, ego_vehicle_velocity) 

        # adaptive_cruise_control
        # target_preceding = [rel_pos_x_tar, rel_pos_y_tar, rel_vel_x_tar, rel_vel_y_tar]
        acceleration_by_acc = self.acc_system.adaptive_cruise_control(target_preceding, target_vehicle_velocity, ego_vehicle_velocity) 
        # acceleration_by_pid_controller = self.pid_controller.get_acceleration(target_preceding, target_vehicle_velocity, ego_vehicle_velocity) 
        # print('acc: ', acceleration_by_acc, 'pid: ', acceleration_by_pid_controller)

        # lane_following_assist
        steer_by_lfa = lane_following_assist_purepursuit(lane_info, ego_vehicle_velocity) 

        # arbitration
        acceleration_by_aeb = None 
        desired_acceleration, desired_steer = arbitration(acceleration_by_acc, acceleration_by_aeb, steer_by_lfa)

        # logging
        self._log_driving_data(ego_vehicle_speed, target_vehicle_speed, ego_acceleration, ego_target_distance, desired_acceleration, self.acc_system.ACC_MODE) 


        """ Control """

        self.control.throttle = engine_control.calc_engine_control_command(desired_acceleration)
        self.control.brake = brake_control.calc_brake_command(desired_acceleration)
        self.control.steer = steer_control.calc_steer_command(desired_steer)
        self._actor.apply_control(self.control)


        self._spectator_update()

