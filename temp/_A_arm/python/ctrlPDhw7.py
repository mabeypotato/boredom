import numpy as np
import armParam as P

class ctrlPD:
    def __init__(self):
        # Declare pole locations
        p1 = -3
        p2 = -4
        alpha0 = (-p1) * (-p2) # Coefficient in front of s^0 in desired characteristic equation
        alpha1 = (-p1) + (-p2) # Coefficient in front of s^1 in desired characteristic equation
        b0 = 3 / (P.m * P.ell**2)
        a0 = 0.0
        a1 = 3 * P.b / (P.m * P.ell**2)
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

        # Include feadback linearization torque or equilibrium torque around theta_e
        if self.use_feedback_linearization:
            # Feedback linearized torque
            tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)
            # compute total torque
            tau = tau_fl + tau_tilde
        else:
            # equilibrium torque around theta_e = 0
            theta_e = 0.0
            tau_e = P.m * P.g * P.ell / 2.0 * np.cos(theta_e)
            # compute total torque
            tau = tau_e + tau_tilde

        return tau

