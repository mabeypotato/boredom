import numpy as np
import rodMassParam as P

class controllerPID:

    def __init__(self):
        self.kp = P.kp
        self.ki = P.ki
        self.kd = P.kd
        self.limit = P.tau_max
        self.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)
        self.Ts = P.Ts
        self.theta_d1 = 0
        self.theta_dot = 0
        self.error_d1 = 0
        self.integrator = 0
        self.tau_eq = P.tau_eq

    def update(self, theta_r, y):

        # this is for PD control #

        theta = y[0][0]
        thetadot = y[1][0]
        # compute the linearized torque using PD control
        tau_unsat = self.kp * (theta_r - theta) - self.kd * thetadot

        tau = saturate(tau_unsat, P.tau_max)

        # this is for PID control #

        return tau

def saturate(self, u):
    if abs(u) > self.limit:
        u = self.limit*np.sign(u)
    return u







