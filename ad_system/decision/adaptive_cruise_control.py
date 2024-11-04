import numpy as np 
from controller import (SpeedPidController, DistancePidController, RelativeSpeedPidController,
                        RelativeSpeedDistancePidController, RelativeSpeedHeadwayDistancePidController, 
                        LinearQuadraticRegulator, LinearQuadraticRegulatorDelay, ModelPredictiveControl, ModelPredictiveControlDelay)   


class ACCSystem: 

    ACC_MODE = 1 

    CUSTOM_MODE = 0 
    SPEED_MODE = 1 
    LOW_SPEED_MODE = 2

    LOW_SPEED_THRESHOLD = 20 / 3.6  # m/s 
    MINIMUM_DISTANCE = 11

    def __init__(self, desired_velocity_kph=50, headway_time=1.8, desired_distance=30, maximum_acc_distance=110, dt=0.2): 
        self.headway_time = headway_time 
        self.desired_velocity = desired_velocity_kph / 3.6 
        self.maximum_acc_distance = maximum_acc_distance 

        self.speed_pid_controller = SpeedPidController(desired_velocity_kph, dt=dt) 
        self.low_speed_pid_controller = RelativeSpeedPidController(dt=dt)
        # self.low_speed_pid_controller = RelativeSpeedDistancePidController(self.MINIMUM_DISTANCE, dt=dt) 

        """ select one of the following longitudinal controllers """
        # self.custom_pid_controller = DistancePidController(desired_distance, dt=dt) 
        # self.custom_pid_controller = RelativeSpeedPidController(dt=dt) 
        # self.custom_pid_controller = RelativeSpeedDistancePidController(desired_distance, dt=dt) 
        # self.custom_pid_controller = RelativeSpeedHeadwayDistancePidController(headway_time=headway_time, dt=dt)
        self.custom_pid_controller = LinearQuadraticRegulator(headway_time=headway_time, Q=np.diag([2, 4]), R=np.diag([5]), dt=dt)
        # self.custom_pid_controller = LinearQuadraticRegulatorDelay(headway_time=headway_time, Q=np.diag([2, 4, 1]), R=np.diag([5]), dt=dt)
        # TODO: self.custom_pid_controller = ModelPredictiveControl(headway_time=headway_time, Q=np.diag([25, 40]), R=np.diag([2]), Rd=np.diag([2]), time_horizon=10, dt=dt)
        # self.custom_pid_controller = ModelPredictiveControlDelay(headway_time=headway_time, Q=np.diag([25, 40, 0.1]), R=np.diag([2]), Rd=np.diag([2]), time_horizon=10, dt=dt)
    
    def adaptive_cruise_control(self, target, target_velocity, ego_velocity): 

        accleration_speed_mode = self.speed_pid_controller.get_acceleration(target, target_velocity, ego_velocity)
        acceleration_low_speed_mode = self.low_speed_pid_controller.get_acceleration(target, target_velocity, ego_velocity) 
        accleration_custom_mode = self.custom_pid_controller.get_acceleration(target, target_velocity, ego_velocity) 

        acceleration_for_acc = self._acc_mode_selection(target, ego_velocity, accleration_speed_mode, acceleration_low_speed_mode, accleration_custom_mode)

        return acceleration_for_acc 
    

    def _acc_mode_selection(self, target, ego_velocity, accleration_speed_mode, acceleration_low_speed_mode, accleration_custom_mode): 

        if target: 
            rel_pos_x_tar, rel_pos_y_tar, rel_vel_x_tar, rel_vel_y_tar = target 

        # CUSTOM_MODE -> LOW_SPEED_MODE로 전환
        if target and (self.ACC_MODE == self.CUSTOM_MODE) and (ego_velocity[0] < self.LOW_SPEED_THRESHOLD) and (rel_pos_x_tar < self.MINIMUM_DISTANCE): 
            self.ACC_MODE = self.LOW_SPEED_MODE

        # CUSTOM_MODE -> SPEED_MODE로 전환 
        elif self.ACC_MODE == self.CUSTOM_MODE: 
            # target 없음 or TTC가 3초 이상이고 최대 acc 작동 거리 이상 -> SPEED_MODE로 변경
            if not target or ((rel_pos_x_tar / (ego_velocity[0]+ 1e-4)) >= (self.headway_time + 0.6) and rel_pos_x_tar >= self.maximum_acc_distance): 
                self.ACC_MODE = self.SPEED_MODE 
        
        else: 
            if target:
                if ((rel_pos_x_tar / (ego_velocity[0]+ 1e-4)) < (self.headway_time + 0.6)):
                    self.ACC_MODE = self.CUSTOM_MODE
                if (self.ACC_MODE == self.LOW_SPEED_MODE) and (self.MINIMUM_DISTANCE + 7) < rel_pos_x_tar:  
                    self.ACC_MODE = self.CUSTOM_MODE 


        if self.ACC_MODE == self.CUSTOM_MODE: 
            # print('CUSTOM_MODE', end=' ')
            self.speed_pid_controller.reset() 
            acceleration = accleration_custom_mode 

        elif self.ACC_MODE == self.LOW_SPEED_MODE: 
            # print('LOW_SPEED_MODE', end=' ')
            self.low_speed_pid_controller.reset() 
            acceleration = acceleration_low_speed_mode 
        else: 
            # print('SPEED_MODE', end=' ')
            self.custom_pid_controller.reset() 
            acceleration = accleration_speed_mode 


        return acceleration 