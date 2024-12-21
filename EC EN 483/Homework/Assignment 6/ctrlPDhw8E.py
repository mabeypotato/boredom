import numpy as np
import blockbeamParam as P

class ctrlPD:
    def __init__(self):
        ####################################################
        #       PD Control: Time Design Strategy
        ####################################################
        # tuning parameters
        tr_th = 1         # Rise time for inner loop (theta)
        zeta_th = 0.707       # inner loop Damping Coefficient

        # saturation limits
        F_max = 5             		  # Max Force, N
        error_max = 1        		  # Max step size,m
        theta_max = 30.0 * np.pi / 180.0  # Max theta, rads

        #---------------------------------------------------
        #                    Inner Loop
        #---------------------------------------------------
        # parameters of the open loop transfer function
        A_th = (P.length)/(P.m2*P.length**2/3 + P.m1*P.ze**2)

        # coefficients for desired inner loop
        wn_th = 2.2 / tr_th     # Natural frequency

        # compute gains
        self.kp_th = wn_th**2/A_th
        self.kd_th = 2*zeta_th*wn_th/A_th
        DC_gain = 1

        #---------------------------------------------------
        #                    Outer Loop
        #---------------------------------------------------
        # coefficients for desired outer loop
        M = 10.0           # Time scale separation 
        zeta_z = 0.707     # outer loop Damping Coefficient
        tr_z = M * tr_th   # desired rise time, s
        wn_z = 2.2 / tr_z  # desired natural frequency

        # compute gains
        A_z = -9.8
        self.kd_z = wn_z*2*zeta_z/A_z
        self.kp_z = wn_z**2/A_z

        # print control gains to terminal        
        print('DC_gain: ', DC_gain)
        print('kp_th:   ', self.kp_th)
        print('kd_th:   ', self.kd_th)
        print('kp_z:   ', self.kp_z)
        print('kd_z:   ', self.kd_z)

    def update(self, z_r, state):
        z = state[0][0]
        theta = state[1][0]
        zdot = state[2][0]
        thetadot = state[3][0]

        # the reference angle for theta comes from the
        # outer loop PD control
        tmp = self.kp_z * (z_r - z) - self.kd_z * zdot

        # the force applied to the cart comes from the
        # inner loop PD control
        F_tilde = self.kp_th * (tmp - theta) - self.kd_th * thetadot
        F_e = (P.m1*P.g*z)/P.length + P.m2*P.g/2

        F_tmp = F_tilde + F_e
        F_sat = saturate(F_tmp, P.F_max)
        return F_sat

def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u



