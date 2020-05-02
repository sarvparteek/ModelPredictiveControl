import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = 5
        self.y_obs = 0.1

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t       = prev_state[3]
        a_t       = pedal
        beta      = steering
        wheelbase = 2.5
        
        x_t_1   = x_t   + v_t * np.cos(psi_t) * dt
        y_t_1   = y_t   + v_t * np.sin(psi_t) * dt
        psi_t_1 = psi_t + v_t * np.tan(beta) / wheelbase * dt
        v_t_1   = v_t   + a_t * dt - v_t/25

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        
        for i in range(self.horizon):
            state = self.plant_model(state, self.dt, u[2*i], u[2*i+1])
            cost += (ref[0] - state[0]) ** 2 # x error cost 
            cost += (ref[1] - state[1]) ** 2 # y error cost
            cost += (ref[2] - state[2]) ** 2 # psi error cost
            cost += self.obstacle_cost(state[0], state[1])
        return cost
    
    def obstacle_cost(self, x, y):
        dist = np.sqrt( (self.x_obs - x) ** 2 + (self.y_obs - y) ** 2 )
        if (dist > 2):
            return 15 # to smoothen the cost
        else:
            return 30/dist
        
sim_run(options, ModelPredictiveControl)
