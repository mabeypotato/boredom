import numpy as np
import blockbeamParam as P

class ctrlPID:
    def __init__(self):
        # dirty derivative parameters
        self.sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * self.sigma - P.Ts) \
            / (2 * self.sigma + P.Ts)  
        ####################################################
        #       PD Control: Time Design Strategy
        ####################################################
        # tuning parameters
        tr_th = 0.12         # Rise time for inner loop (theta)
        zeta_th = 0.707       # inner loop Damping Coefficient

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
        A_z = P.g
        self.kd_z = -1*wn_z*2*zeta_z/A_z
        self.kp_z = -1*wn_z**2/A_z
        self.ki_z = -0.1
        # print control gains to terminal        
        print('DC_gain', DC_gain)
        print('kp_th: ', self.kp_th)
        print('kd_th: ', self.kd_th)
        print('kp_z: ', self.kp_z)
        print('kd_z: ', self.kd_z)
        print('ki_z: ', self.ki_z)
        #---------------------------------------------------
        # initialize variables for integrator and differentiators
        #---------------------------------------------------
        self.integrator_z = 0.0
        self.error_z_d1 = 0.0
        self.z_dot = P.zdot0
        self.z_d1 = P.z0
        self.theta_dot = P.thetadot0
        self.theta_d1 = P.theta0
        self.theta_max = 60*np.pi/180
        
    def update(self, z_r, y):
        z = y[0][0]
        theta = y[1][0]
        #---------------------------------------------------
        # Update Outer Loop (z-control)
        #---------------------------------------------------
        # Compute the error in z
        error_z = z_r - z
        # differentiate z
        self.z_dot = self.beta * self.z_dot \
            + (2.0 / (2.0*self.sigma + P.Ts)) * ((z - self.z_d1))
        # if z_dot is small, integrate z
        if np.abs(self.z_dot) < 0.07:
            self.integrator_z = self.integrator_z \
                + (P.Ts / 2) * (error_z + self.error_z_d1)
        
        # PID control - unsaturated
        theta_r_unsat = self.kp_z * error_z \
                + self.ki_z * self.integrator_z \
                - self.kd_z * self.z_dot
        # saturate theta_r
        theta_r = saturate(theta_r_unsat, self.theta_max)

        # integrator anti - windup
        if np.abs(self.ki_z) <= 0.08:
            self.integrator_z = self.integrator_z \
                + P.Ts / self.ki_z * (theta_r - theta_r_unsat)

       #---------------------------------------------------
        # Update Inner Loop (theta-control)
        #---------------------------------------------------
        # Compute the error in theta
        error_th = theta_r - theta
        # differentiate theta
        self.theta_dot = (2.0*self.sigma - P.Ts) / (2.0*self.sigma + P.Ts) * self.theta_dot \
            + (2.0 / (2.0*self.sigma + P.Ts)) * ((theta - self.theta_d1))
         # PD control on theta

        F_lin = P.m1*P.g*(z/P.length) + P.m2*P.g/2

        F_unsat = self.kp_th * error_th \
            - self.kd_th * self.theta_dot
        # saturate the force
        F_total = saturate(F_unsat + F_lin, P.F_max)
        # update delayed variables
        self.error_z_d1 = error_z
        self.z_d1 = z
        self.theta_d1 = theta
        # return computed force
        return F_total

def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u





