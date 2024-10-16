import numpy as np
import scipy


class LinearQuadraticRegulator:

    def __init__(self, headway_time=0, Q=np.diag([2, 4]), R=np.diag([5]), dt=0.02):
        self.headway_time = headway_time
        self.dt = dt

        # Control Gain
        self.Q = Q
        # Regulator Gain
        self.R = R

        self.state_vector: np.array = np.array([[0.0], [0.0]])

    def get_acceleration(self, target, target_velocity, ego_velocity):

        acceleration = 0 
        if target:
            acceleration = self._lqr_update(target, target_velocity, ego_velocity)

        return acceleration


    def _lqr_update(self, target, target_velocity, ego_velocity):

        self.state_vector = self._get_state_vector(target, target_velocity, ego_velocity)  
        A, B = self._get_dynamics_model()
        K = self._calculate_regulator_gain(A, B)
        acceleration = -K * self.state_vector 
        return acceleration.item()

    def _get_state_vector(self, target, target_velocity, ego_velocity):

        if self.headway_time == 0:
            target_distance = 30 
        else: 
            target_distance = ego_velocity[0] * self.headway_time  
        relative_distance = target[0]  

        distance_error = target_distance - relative_distance
        velocity_error = target_velocity[0] - ego_velocity[0]  

        return np.matrix([
            [distance_error],
            [velocity_error],
        ])

    def _get_dynamics_model(self):
        A_c = np.matrix([
            [0, -1],
            [0, 0]
        ])

        B_c = np.matrix([
            [self.headway_time],
            [-1]
        ])

        A = np.eye(2) + self.dt * A_c
        B = self.dt * B_c

        return A, B

    def _calculate_regulator_gain(self, A, B):
        P = np.matrix(scipy.linalg.solve_discrete_are(A, B, self.Q, self.R))
        K = np.matrix(scipy.linalg.inv(B.T * P * B + self.R) * (B.T * P * A))
        return K

    def reset(self):
        self.state_vector = np.array([[0.0], [0.0]]) 



class LinearQuadraticRegulatorDelay:

    def __init__(self, headway_time=0, Q=np.diag([2, 4, 1]), R=np.diag([5]), dt=0.02):
        self.headway_time = headway_time
        self.dt = dt

        # Control Gain
        self.Q = Q
        # Regulator Gain
        self.R = R

        self.prev_ego_vehicle_velocity = 0 
        self.state_vector: np.array = np.array([[0.0], [0.0], [0.0]])


    def get_acceleration(self, target, target_velocity, ego_velocity):

        acceleration = 0 
        if target:
            acceleration = self._lqr_update(target, target_velocity, ego_velocity)

        return acceleration


    def _lqr_update(self, target, target_velocity, ego_velocity):

        self.state_vector = self._get_state_vector(target, target_velocity, ego_velocity)  
        A, B = self._get_dynamics_model()
        K = self._calculate_regulator_gain(A, B)
        acceleration = -K * self.state_vector 
        self.prev_ego_vehicle_velocity = ego_velocity[0]
        return acceleration.item()

    def _get_state_vector(self, target, target_velocity, ego_velocity):

        if self.headway_time == 0:
            target_distance = 30 
        else: 
            target_distance = ego_velocity[0] * self.headway_time  
        relative_distance = target[0]  

        distance_error = target_distance - relative_distance
        velocity_error = target_velocity[0] - ego_velocity[0]  

        return np.matrix([
            [distance_error],
            [velocity_error],
            [(ego_velocity[0] - self.prev_ego_vehicle_velocity) / self.dt]
        ])

    def _get_dynamics_model(self):

        time_constant = 0.45
        A_c = np.matrix([
            [0, -1, 2 if self.headway_time == 0 else self.headway_time],
            [0, 0, -1], 
            [0, 0, -1 / time_constant] 
        ])

        B_c = np.matrix([
            [0],
            [0], 
            [1 / time_constant]
        ])

        A = np.eye(3) + self.dt * A_c
        B = self.dt * B_c

        return A, B

    def _calculate_regulator_gain(self, A, B):
        P = np.matrix(scipy.linalg.solve_discrete_are(A, B, self.Q, self.R))
        K = np.matrix(scipy.linalg.inv(B.T * P * B + self.R) * (B.T * P * A))
        return K

    def reset(self):
        self.state_vector = np.array([[0.0], [0.0]])
