import numpy as np
import control as cnt
import blockbeamParam as P

class ctrlStateFeedbackIntegrator:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        # tuning parameters
        tr_z = 2.5       # rise time for position
        tr_theta = .25    # rise time for angle
        zeta_z = 0.707  # damping ratio position
        zeta_th = 0.707  # damping ratio angle
        integrator_pole = -3

        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
        A = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, -1*P.g, 0.0, 0.0],
            [-1*P.m1*P.g/((P.m2*P.length**2)/3 + P.m1*P.ze**2), 0.0, 0.0, 0.0]
            ])
        
        B = np.array([[0.0],
                      [0.0],
                      [0.0],
                      [P.length/((P.m2*P.length**2)/3 + P.m1*P.ze**2)]])
        C = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]])
        
        Cr = np.array([[1.0, 0.0, 0.0, 0.0]])

        A1 = np.vstack((np.hstack((A, np.zeros((4,1)))),
                        np.hstack((-Cr, np.zeros((1,1))))))
        B1 = np.vstack((B, np.zeros((1,1))))
        
        # gain calculation
        wn_th = 2.2 / tr_theta  # natural frequency for angle
        wn_z = 2.2 / tr_z  # natural frequency for position
        des_char_poly = np.convolve(
            
            np.convolve([1, 2 * zeta_z * wn_z, wn_z**2],
                        [1, 2 * zeta_th * wn_th, wn_th**2]), 
            
            np.poly([integrator_pole]))
        
        des_poles = np.roots(des_char_poly)

        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            print("The system is not controllable")
        else:
            K1 = cnt.acker(A1, B1, des_poles)
            self.K = K1[0][0:4]
            self.ki = K1[0][4]
        # print gains to terminal
        print('K: ', self.K)
        print('ki: ', self.ki)
        print('desired poles', des_poles)

        self.integrator_z = 0.0
        self.error_d1 = 0.0

    def update(self, z_r, x):
        # Compute the state feedback controller
        z = x[0,0]
        z_dot = x[2,0]

        error_z = z_r - z

        if abs(z_dot) < 1:
            self.integrator_z = self.integrator_z + (P.Ts/2.0)*(error_z + self.error_d1)

        x_tilde = x - np.array([[P.ze], [0.0], [0.0], [0.0]]) 

        F_tilde = -self.K @ x_tilde - self.ki * self.integrator_z
        F_unsat = F_tilde + P.Fe
        F = saturate(F_unsat[0], P.F_max)

        self.error_d1 = error_z
        return F


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

