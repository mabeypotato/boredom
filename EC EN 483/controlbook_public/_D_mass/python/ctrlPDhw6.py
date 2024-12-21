import numpy as np
import massParam as P

class ctrlPD:
    def __init__(self):
        # Declare pole locations
        p1 = -1
        p2 = -1.5

        tr = 2 # tuned for faster rise time before saturation.
        zeta = 0.707

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

        self.use_feedback_linearization = True

    def update(self, theta_r, x):
        theta = x[0][0]
        thetadot = x[1][0]
        # compute the linearized torque using PD control
        tau_tilde = self.kp * (theta_r - theta) - self.kd * thetadot

        tau_sat = saturate(tau_tilde, P.F_max)
        return tau_sat

def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u