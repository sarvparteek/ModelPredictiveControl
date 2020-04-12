import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        # state = [x, y, psi, vel]
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s
        a_t = pedal
        x_t_1 = x_t + v_t * dt
        v_t_1 = v_t + a_t * dt - v_t/25.0
        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self,u, *args):
        # u = [p1, s1, p2, s2, p3, s3 ....pN, sN].transpose()
        # where p = pedal, s = steering
        state = args[0]
        ref   = args[1]
        cost  = 0.0
        for i in range(self.horizon):
            state = self.plant_model(state, self.dt, u[i*2], 0.0)
            cost += (ref[0] - state[0])**2
            
            vel_kph = state[3] * 3.6
            if (vel_kph > 10):
                cost += 1000* (vel_kph - 10)
        return cost

sim_run(options, ModelPredictiveControl)
