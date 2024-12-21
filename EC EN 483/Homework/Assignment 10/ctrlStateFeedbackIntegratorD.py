import numpy as np
import control as cnt
import massParam as P

class ctrlStateFeedbackIntegrator:
    # dirty derivatives to estimate thetadot
    def __init__(self):
        #  tuning parameters
        tr = 2
        zeta = 0.707
        int_pole = -3

        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x

        A = np.array([[0.0, 1.0],
                      [-1*P.k/P.m, -1*P.b/P.m]])
        B = np.array([[0.0],
                      [1.0 / P.m]])        
        Cr = np.array([[1.0, 0.0]])

        A1 = np.vstack((np.hstack((A, np.zeros((np.size(A,1),1)))), 
                        np.hstack((-Cr, np.array([[0.0]]))) ))
        B1 = np.vstack( (B, 0.0) )

        # gain calculation
        wn = 2.2 / tr  # natural frequency
        des_char_poly = np.convolve([1, 2 * zeta * wn, wn**2], [1, -int_pole])
        des_poles = np.roots(des_char_poly)

        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 2:
            print("The system is not controllable")
        else:
            K1 = (cnt.place(A1, B1, des_poles)) # use augmented matrices
            self.K = K1[0][0:2]
            self.ki = K1[0][1]
        print('K: ', self.K)
        print('ki: ', self.ki)
        print('Desired poles:', des_poles)

        self.integrator = 0.0
        self.error_d1 = 0.0

    def update(self, z_r, x):
        z = x[0][0]
        z_dot = x[1][0]
        error = z_r - z

        # anti-windup
        if abs(z_dot < 0.65):
            self.integrator = self.integrator + (P.Ts/2)*(error + self.error_d1)

        # Compute the state feedback controller
        tau_tilde = -self.K @ x + self.ki * self.integrator
    
        # compute total torque
        tau = saturate(tau_tilde[0], P.F_max)
        self.error_d1 = error
        return tau


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u


