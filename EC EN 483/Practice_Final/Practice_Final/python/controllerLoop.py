import rodMassParam as P
import loopshape_rodMass as L
import numpy as np

class controllerLoop:
    def __init__(self):
        self.A_C = L.Css.A
        self.B_C = L.Css.B
        self.C_C = L.Css.C
        self.D_C = L.Css.D
        self.A_F = L.Fss.A
        self.B_F = L.Fss.B
        self.C_F = L.Fss.C
        self.D_F = L.Fss.D
        n = self.A_C.shape[0]
        self.x_C = np.zeros((n, 1))
        n = self.A_F.shape[0]
        self.x_F = np.zeros((n, 1))
        self.limit = P.tau_max
        self.tau_eq = P.tau_eq
        self.Ts = P.Ts

    def update(self, theta_r, y):
        theta = y.item(0)

        #prefilter the reference command
        # solve differential equation defining prefilter
        if self.x_F.shape[0] == 0:
            theta_c_filtered = self.D_F * theta_r
        else:
            N = 10  # number of Euler integration steps
            for i in range(0, N):
                self.x_F = self.x_F + self.Ts / N * \
                           (self.A_F @ self.x_F + self.B_F @ theta_r )
            # output equation for the prefilter
            theta_c_filtered = self.C_F * self.x_F + self.D_F * theta_r

        # error signal
        error = theta_c_filtered - theta

        # solve differential equation defining controller
        N = 10  #number of Euler integration steps
        for i in range(0, N):
            self.x_C = self.x_C + self.Ts / N * \
                       (self.A_C @ self.x_C + self.B_C * error )
        # output equation for the controller
        tau_tilde = self.C_C @ self.x_C + self.D_C * error

        # compute total torque
        # tau = self.saturate(self.tau_eq + tau_tilde)
        tau = self.tau_eq + tau_tilde

        return tau.item(0)
