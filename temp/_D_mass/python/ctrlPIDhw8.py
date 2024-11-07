import numpy as np
import massParam as P


class ctrlPID:
    def __init__(self):
        #  tuning parameters
        tr = 0.6  
        zeta = 0.90
        self.ki = 0.8  # integrator gain

        # desired natural frequency
        wn = 2.2 / tr
        alpha1 = 2.0 * zeta * wn
        alpha0 = wn**2

        b0 = 1/P.m
        a0 = P.k/P.m
        a1 = P.b/P.m
        self.kp = (alpha0 - a0) / b0
        self.kd = (alpha1 - a1) / b0
        
        # PD gains
        print('kp: ', self.kp)
        print('kd: ', self.kd)
        print('kp: ', self.kp)
        print('ki: ', self.ki)
        print('kd: ', self.kd)       
       
        # dirty derivative gains
        self.sigma = 0.05
        self.beta = (2 * self.sigma - P.Ts) \
            / (2 * self.sigma + P.Ts)
        #----------------------------------------------------------
        # variables for integrator and differentiator
        self.z_dot = P.zdot0  # estimated derivative of theta
        self.theta_d1 = P.z0  # theta delayed by one sample
        self.error_dot = 0.0  # estimated derivative of error
        self.error_d1 = 0.0  # Error delayed by one sample
        self.integrator = 0.0  # integrator

    def update(self, theta_r, y):
        theta = y[0][0]

        # compute the linearized torque using PID
        # Compute the current error
        error = theta_r - theta
        # differentiate theta
        self.z_dot = self.beta * self.z_dot \
            + (2.0 / (2.0*self.sigma + P.Ts)) * ((theta - self.theta_d1))
        # Anti-windup scheme: only integrate theta when z_dot is small            @@@@@ change 0.08 for tuning if necessary @@@@@
        if abs(self.z_dot < 0.04): 
            self.integrator = self.integrator \
                + (P.Ts / 2) * (error + self.error_d1)
        # PID control
        tau_tilde = self.kp * error \
            + self.ki * self.integrator \
                - self.kd * self.z_dot
        
        # compute total torque
        tau_unsat = tau_tilde
        tau = saturate(tau_unsat, P.F_max)

        # update delayed variables
        self.error_d1 = error
        self.theta_d1 = theta
        return tau


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u








