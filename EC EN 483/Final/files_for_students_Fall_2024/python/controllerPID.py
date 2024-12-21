import numpy as np
import massParam as P

class controllerPID:

    def __init__(self):
        zeta = 0.707
        tr = 0.6998
        wn = 2.2/tr
        self.kp = 5
        self.ki = 2.25  # tune this
        self.kd = zeta*wn - 0.1 
        self.limit = P.F_max
        self.beta = (2.0 * P.sigma - P.Ts) / (2.0 * P.sigma + P.Ts)
        self.Ts = P.Ts
        
        self.z_d1 = 0.0         
        self.z_dot = 0.0
        self.error_d1 = 0.0
        self.integrator = 0.0
        self.Fe = P.Fe

    def update(self, z_r, y):
        z = y[0][0]

        error = z_r - z

        # integrate error
        self.integrator = self.integrator + (P.Ts / 2) * (error + self.error_d1)
        
        # dirty deriv
        self.z_dot = self.beta * self.z_dot + (1 - self.beta) * ((z - self.z_d1) / P.Ts)

        
        # compute the linearized torque using PD control
        F_unsat = self.kp * error + self.ki * self.integrator - self.kd * self.z_dot
        F = saturate(P.F_max, F_unsat)

        # integrator anti - windup
        if np.abs(self.z_dot) < 0.5:
            self.integrator = self.integrator + (P.Ts / 2) * (error + self.error_d1)
            
        self.error_d1 = error
        self.z_d1 = z
        return F

def saturate(self, u):
    if abs(u) > P.F_max:
        u = P.F_max*np.sign(u)
    return u