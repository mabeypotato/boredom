import numpy as np
import rodMassParam as P

class controllerObsv:
    def __init__(self):
        self.observer_state = np.array([
            [0.0],  # estimate of theta
            [0.0],  # estimate of theta_hat
            [0.0],  # estimate of disturbance
        ])
        self.tau_d1 = 0.0  # control torque, delayed by one sample
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = P.K  # state feedback gain
        self.ki = P.ki2  # Input gain
        self.L = P.L2  # observer gain
        self.A = P.A2  # system model
        self.B = P.B2
        self.C = P.C2
        self.limit = P.tau_max
        self.tau_eq = P.tau_eq
        self.Ts = P.Ts  # sample rate of controller

    def update(self, theta_r, y):
        return tau, x_hat, d_hat

    def update_observer(self, y):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.observer_state, y)
        F2 = self.observer_f(self.observer_state + self.Ts / 2 * F1, y)
        F3 = self.observer_f(self.observer_state + self.Ts / 2 * F2, y)
        F4 = self.observer_f(self.observer_state + self.Ts * F3, y)
        self.observer_state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        x_hat = np.array([[self.observer_state.item(0)],[self.observer_state.item(1)]])
        d_hat = self.observer_state.item(2)
        return x_hat, d_hat

    def observer_f(self, x_hat, y):
        return xhat_dot

    def integrateError(self, error):

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

