import numpy as np 

class PidModule: 

    def __init__(self, kp, ki, kd, dt): 
        self.kp = kp 
        self.ki = ki 
        self.kd = kd 
        self.dt = dt 

        self.integral = 0 
        self.prev_error = 0 

    def update(self, error): 
        self.integral += error * self.dt 
        derivative = (error - self.prev_error) / self.dt 
        self.prev_error = error 

        return self.kp * error + self.ki * self.integral + self.kd * derivative 
    


class DistancePidController:

    def __init__(self, desired_distance, dt=0.2): 
        self.desired_distance = desired_distance 
        self.distance_pid = PidModule(kp=0.4, ki=0.001, kd=0.5, dt=dt)

        self.distance_error = 0

    def get_acceleration(self, target, target_velocity, ego_velocity): 
        acceleration = 0 
        if target: 
            self.distance_error = target[0] - self.desired_distance 
            acceleration = self.distance_pid.update(self.distance_error) 
            # print('relative distance:', target[0], 'distance error:', self.distance_error, 'integrated error:', self.distance_pid.integral, 'acceleration:', acceleration)

        return acceleration

    def reset(self): 
        self.distance_error = 0 
        self.distance_pid.integral = 0 
        self.distance_pid.prev_error = 0


class SpeedPidController: 

    def __init__(self, desired_velocity_kph, dt=0.2):
        self.desired_velocity = desired_velocity_kph / 3.6 
        self.velocity_pid = PidModule(kp=1, ki=0.001, kd=0.1, dt=dt)

        self.velocity_error = 0  

    def get_acceleration(self, target, target_velocity, ego_velocity): 

        # desired_velocity > ego_velocity 일때, acceleration은 +
        self.velocity_error = self.desired_velocity - ego_velocity[0]  
        acceleration = self.velocity_pid.update(self.velocity_error)
        # print('velocity error:', self.velocity_error, 'acceleration:', acceleration, 'integrated error:', self.velocity_pid.integral)
        return acceleration 
    
    def reset(self): 
        self.velocity_error = 0 
        self.velocity_pid.integral = 0 
        self.velocity_pid.prev_error = 0 


class RelativeSpeedPidController: 

    def __init__(self, dt): 
        self.relative_velocity_pid = PidModule(kp=0.5, ki=0.001, kd=0.05, dt=dt)  # PidModule(kp=1.0, ki=0.001, kd=0.1, dt=dt) 

        self.relative_velocity_error = 0

    def get_acceleration(self, target, target_velocity, ego_velocity): 

        acceleration = 0
        if target: 
            self.relative_velocity_error = target_velocity[0] - ego_velocity[0] 
            acceleration = self.relative_velocity_pid.update(self.relative_velocity_error)

        return acceleration 
    
    def reset(self): 
        self.relative_velocity_error = 0 
        self.relative_velocity_pid.integral = 0 
        self.relative_velocity_pid.prev_error = 0 


class RelativeSpeedDistancePidController: 

    def __init__(self, desired_distance, dt): 
        self.desired_distance = desired_distance 

        # self.distance_pid = PidModule(kp=0.3, ki=0.05, kd=0.1, dt=dt) 
        # self.relative_velocity_pid = PidModule(kp=0.1, ki=0.05, kd=0.05, dt=dt) 
        self.distance_pid = PidModule(kp=0.4, ki=0.001, kd=0.5, dt=dt) 
        self.relative_velocity_pid = PidModule(kp=1, ki=0.001, kd=0.1, dt=dt)

        self.distance_error = 0 
        self.relative_velocity_error = 0 
    
    def get_acceleration(self, target, target_velocity, ego_velocity): 

        acceleration = 0 
        if target:
            self.distance_error = target[0] - self.desired_distance  
            self.relative_velocity_error = target_velocity[0] - ego_velocity[0]  

            acceleration = self.distance_pid.update(self.relative_velocity_error) + self.relative_velocity_pid.update(self.distance_error)
            # print('distance error:', self.distance_error, 'velocity error:', self.relative_velocity_error, 'acceleration:', acceleration)

        return acceleration
    
    def reset(self): 
        self.distance_error = 0 
        self.relative_velocity_error = 0 
        self.distance_pid.integral = 0 
        self.distance_pid.prev_error = 0 
        self.relative_velocity_pid.integral = 0 
        self.relative_velocity_pid.prev_error = 0


class RelativeSpeedHeadwayDistancePidController: 
    
        def __init__(self, headway_time, dt): 
            self.headway_time = headway_time 
    
            self.distance_pid = PidModule(kp=0.3, ki=0.05, kd=0.1, dt=dt) 
            self.relative_velocity_pid = PidModule(kp=0.1, ki=0.05, kd=0.05, dt=dt) 
    
            self.distance_error = 0 
            self.relative_velocity_error = 0 
        
        def get_acceleration(self, target, target_velocity, ego_velocity): 
    
            acceleration = 0 
            if target:
                target_distance = self.headway_time * ego_velocity[0] 
                # self.distance_error = target_distance - target[0]
                # self.relative_velocity_error = target_velocity[0] - ego_velocity[0]  
                self.distance_error = target[0] - target_distance  
                self.relative_velocity_error = target_velocity[0] - ego_velocity[0] 
    
                acceleration = self.distance_pid.update(self.relative_velocity_error) + self.relative_velocity_pid.update(self.distance_error) 
                # print('target distance', target_distance, 'distance error:', self.distance_error, 'velocity error:', self.relative_velocity_error, 'acceleration:', acceleration)
            
            return acceleration
        
        def reset(self): 
            self.distance_error = 0 
            self.relative_velocity_error = 0 
            self.distance_pid.integral = 0 
            self.distance_pid.prev_error = 0 
            self.relative_velocity_pid.integral = 0 
            self.relative_velocity_pid.prev_error = 0
