import numpy as np
import hummingbirdParam as P


class ctrlLatPD:
    def __init__(self):
        
        #### ROLL & YAW ####

        # tuning parameters
        M = 15.0
        tr_yaw = 1
        zeta_yaw = 0.707
        tr_roll = 1/M
        wn_yaw = 2.2/tr_yaw
        zeta_roll = 0.707
        wn_roll = 2.2/tr_roll

        # gain calculation
        b_phi = 1/P.J1x
        self.kd_roll = 2*zeta_roll*wn_roll/b_phi
        self.kp_roll = wn_roll**2/b_phi

        b_psi = P.ellT*P.Fe/(P.J1z + P.JT)
        self.kd_yaw = 2*zeta_yaw*wn_yaw/b_psi
        self.kp_yaw = wn_yaw**2/b_psi

        print('kd_roll:   ', self.kd_roll)
        print('kp_roll:   ', self.kp_roll)
        print('kd_yaw:    ', self.kd_yaw)
        print('kp_yaw:    ', self.kp_yaw)
        
        self.phi_dot = 0.0
        self.psi_dot = 0.0
        self.phi_d1 = 0.0
        self.psi_d1 = 0.0
           
        sigma = 0.005
        self.beta = (2*sigma - P.Ts)/(2*sigma + P.Ts)
    
    def update(self, r: np.ndarray, y: np.ndarray):
        phi = y[0][0]
        psi_ref = r[2][0] 
        psi = y[2][0]
        theta = y[1][0]


        # compute errors
        psi_error = psi_ref - psi

        self.psi_dot = self.beta*self.psi_dot + (1 + self.beta)*(psi - self.psi_d1)/P.Ts
        phi_ref = self.kp_yaw*(psi_error) - self.kd_yaw*self.psi_dot

        phi_error = phi_ref - phi
        self.phi_dot = self.beta*self.phi_dot + (1 + self.beta)*(phi - self.phi_d1)/P.Ts
        torque = self.kp_roll*phi_error - self.kd_roll*self.phi_dot

        force = P.km * np.cos(theta)

        # convert force and torque to pwm signals
        pwm = np.array([[force + torque/P.d],               # how is this output supposed to work? return torque, 
                      [force - torque/P.d]])/(2*P.Fe)   # other PD controller returns force then that's combined in my sim file?          
        pwm = saturate(pwm, 0, 1)

        # update all delayed variables
        self.phi_d1 = phi
        self.psi_d1 = psi


        # return pwm plus reference signals
        return pwm#, np.array([[phi_ref], [0], [psi_ref]])


def saturate(u, low_limit, up_limit):
    if isinstance(u, float) is True:
        if u > up_limit:
            u = up_limit
        if u < low_limit:
            u = low_limit
    else:
        for i in range(0, u.shape[0]):
            if u[i][0] > up_limit:
                u[i][0] = up_limit
            if u[i][0] < low_limit:
                u[i][0] = low_limit
    return u




