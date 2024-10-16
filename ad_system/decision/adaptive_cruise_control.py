import numpy as np 
from controller import (SpeedPidController, DistancePidController, RelativeSpeedPidController,
                        RelativeSpeedDistancePidController, RelativeSpeedHeadwayDistancePidController, 
                        LinearQuadraticRegulator, LinearQuadraticRegulatorDelay, ModelPredictiveControl, ModelPredictiveControlDelay)   


class ACCSystem: 

    ACC_MODE = 0 
    CUSTOM_MODE = 0 
    SPEED_MODE = 1 

    def __init__(self, desired_velocity_kph=50, desired_distance=30, maximum_acc_distance=60, dt=0.2): 
        self.maximum_acc_distance = maximum_acc_distance 

        self.speed_pid_controller = SpeedPidController(desired_velocity_kph, dt=dt) 

        """ select one of the following longitudinal controllers """
        # self.custom_pid_controller = DistancePidController(desired_distance, dt=dt) 
        # self.custom_pid_controller = RelativeSpeedPidController(dt=dt) 
        # self.custom_pid_controller = RelativeSpeedDistancePidController(desired_distance, dt=dt) 
        self.custom_pid_controller = RelativeSpeedHeadwayDistancePidController(headway_time=1.7, dt=dt)
        # self.custom_pid_controller = LinearQuadraticRegulator(headway_time=2, Q=np.diag([2, 4]), R=np.diag([5]), dt=dt)
        # self.custom_pid_controller = LinearQuadraticRegulatorDelay(headway_time=0, Q=np.diag([2, 4, 1]), R=np.diag([5]), dt=dt)
        # TODO: self.custom_pid_controller = ModelPredictiveControl(headway_time=0, Q=np.diag([25, 40]), R=np.diag([2]), Rd=np.diag([2]), time_horizon=10, dt=dt)
        # self.custom_pid_controller = ModelPredictiveControlDelay(headway_time=2, Q=np.diag([25, 40, 1]), R=np.diag([2]), Rd=np.diag([2]), time_horizon=10, dt=dt)
    
    def adaptive_cruise_control(self, target, target_velocity, ego_velocity): 

        accleration_speed_mode = self.speed_pid_controller.get_acceleration(target, target_velocity, ego_velocity)
        accleration_custom_mode = self.custom_pid_controller.get_acceleration(target, target_velocity, ego_velocity) 

        acceleration_for_acc = self._acc_mode_selection(target, accleration_speed_mode, accleration_custom_mode)

        return acceleration_for_acc 
    
    def _acc_mode_selection(self, target, accleration_speed_mode, accleration_custom_mode): 

        if target: 
            rel_pos_x_tar, _, _, _ = target 

        if self.ACC_MODE == self.CUSTOM_MODE: 
            if not target or rel_pos_x_tar >= self.maximum_acc_distance: 
                self.ACC_MODE = self.SPEED_MODE 
        else: 
            if target and rel_pos_x_tar < self.maximum_acc_distance: 
                self.ACC_MODE = self.CUSTOM_MODE

        if self.ACC_MODE == self.CUSTOM_MODE: 
            self.speed_pid_controller.reset() 
            acceleration = accleration_custom_mode 
        else: 
            self.custom_pid_controller.reset() 
            acceleration = accleration_speed_mode 

        return acceleration 