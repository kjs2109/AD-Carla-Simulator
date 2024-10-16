import cvxpy as cp 
import numpy as np 


class ModelPredictiveControl: 
    pass 


class ModelPredictiveControlDelay: 

    def __init__(self, headway_time=0, Q=np.diag([25, 40, 1]), R=np.diag([2]), Rd=np.diag([2]), time_horizon=10, dt=0.02): 

        self.headway_time = headway_time 
        self.dt = dt 

        self.x_number = 3 
        self.u_number = 1 

        self.time_horizon = time_horizon 
        self.Q = Q 
        self.R = R
        self.Rd = Rd
        self.acceleration_constraint = 9.81 

        self.ego_vehicle_acceleration = 0 
        self.target_vehicle_acceleration = 0.0 


    def get_acceleration(self, target, target_velocity, ego_velocity): 
        
        acceleration = 0 
        if target is not None: 
            acceleration = self._update_mpc(target, target_velocity, ego_velocity) 

        return acceleration 
    
    def _update_mpc(self, target, target_velocity, ego_velocity): 
        x0 = self._get_state(target, target_velocity, ego_velocity) 

        x = cp.Variable((self.x_number, self.time_horizon + 1)) 
        u = cp.Variable((self.u_number, self.time_horizon)) 

        xref = np.zeros((self.x_number, self.time_horizon + 1)) 

        # mpc cost function & constraint 
        cost = 0 
        constraints = [] 
        for t in range(self.time_horizon - 1): 
            cost += cp.quad_form(xref[:, t + 1] - x[:, t + 1], self.Q) 
            cost += cp.quad_form(u[:, t], self.R) 
            cost += cp.quad_form(u[:, t + 1] - u[:, t], self.Rd) 

            A, B, C = self._get_dynamics_model() 
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + (C @ [self.target_vehicle_acceleration]).A1] 

        constraints += [x[:, 0] == x0] 
        constraints += [cp.abs(u[0, :]) <= self.acceleration_constraint] 

        prob = cp.Problem(cp.Minimize(cost), constraints) 
        prob.solve(solver=cp.ECOS, verbose=False) 

        if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE: 
            acceleration_inputs = np.array(u.value[0, :]).flatten() 

        else: 
            print("Failed to find the optimal solution") 
            acceleration_inputs = np.array(u.value[0, :]).flatten() 

        acceleration = acceleration_inputs[0] 
        return acceleration 
    
    def _get_state(self, target, target_velocity, ego_velocity):  

        if self.headway_time == 0: 
            target_distance = 30 
        else: 
            target_distance = ego_velocity[0] * self.headway_time 
        relative_distance = target[0] 
        
        distance_error = target_distance - relative_distance 
        velocity_error = target_velocity[0] - ego_velocity[0]

        return [distance_error, velocity_error, self.ego_vehicle_acceleration] 
    
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

        # disurbance model 
        C_c = np.matrix([
            [0], 
            [1], 
            [0]
        ])

        A = np.eye(self.x_number) + self.dt * A_c 
        B = self.dt * B_c 
        C = self.dt * C_c 

        return A, B, C 
    
    def reset(self): 
        self.ego_vehicle_acceleration = 0 
        self.target_vehicle_acceleration = 0.0


