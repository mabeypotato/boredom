import numpy as np
import VTOLParam as P

class ctrlPD:
    def __init__(self):
        ####################################################
        #       PD Control: Time Design Strategy
        ####################################################
        # tuning parameters
        tr_h = 8.0 # rise time for inner loop
        tr_z = 8.0
        zeta_h = 0.707  # inner loop damping ratio 
        zeta_th = 0.707  # outer loop damping ratio
        zeta_z = 0.707

        # saturation limits
        self.theta_max = 10.0*np.pi/180.0
        self.Fe = (P.mc + 2.0*P.mr) * P.g
        # maximum commanded base angle

        #---------------------------------------------------
        #                    Inner Loop
        #---------------------------------------------------
        
        # PD design for longitudinal inner loop
        wn_h = 2.2 / tr_h
        A_h = 1/(P.mc + 2*P.mr)
        self.kp_h = wn_h**2 / A_h
        self.kd_h = 2*zeta_h*wn_h/A_h

        # lateral inner 
        tr_th = 0.8
        wn_th = 2.2/tr_th

        A_th = 1/(P.Jc + 2*P.mr*P.d**2)

        self.kd_th = 2*zeta_th*wn_th/A_th
        self.kp_th = wn_th**2/A_th

        #---------------------------------------------------
        #                    Outer Loop
        #---------------------------------------------------
        # PD design for outer loop
        b1 = -self.Fe/(P.mc + 2.*P.mr)
        a1 = P.mu/(P.mc + 2*P.mr)
        wn_z = 2.2/tr_z

        self.kp_z = wn_z*2/b1
        self.kd_z = (2*zeta_z*wn_z - a1)/b1
        
        # print control gains to terminal        
        # print('k_DC_phi', k_DC_phi)
        print('kp_z:  ', self.kp_z)
        print('kd_z:  ', self.kd_z)        
        print('kp_h:  ', self.kp_h)
        print('kd_h:  ', self.kd_h)
        print('kp_th: ', self.kp_th)
        print('kd_th: ', self.kd_th)

    def update(self, reference, state):
        z_r = reference[0][0]
        h_r = reference[1][0]

        z = state[0][0]
        h = state[1][0]
        theta = state[2][0]
        zdot = state[3][0]
        hdot = state[4][0]
        thetadot = state[5][0]

        # control for longitude
        F_tilde = self.kp_h * (h_r - h) - self.kd_h * hdot
        F = saturate(F_tilde + self.Fe, 2*P.F_max_thrust)

        # theta_r of outer
        theta_r = saturate(self.kp_z*(z_r - z) - self.kd_th*zdot, self.theta_max)

        tau = saturate(self.kp_th*(theta_r - theta) - self.kd_th*thetadot, 2*P.F_max_thrust*P.d)

        motor_thrust = P.mixing @ np.array([[F], [tau]])

        return motor_thrust
    
def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u
