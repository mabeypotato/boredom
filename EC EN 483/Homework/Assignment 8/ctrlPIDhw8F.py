import numpy as np
import VTOLParam as P


class ctrlPID:
    def __init__(self):
        self.sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * self.sigma - P.Ts) \
            / (2 * self.sigma + P.Ts)  
        ####################################################
        #       PD Control: Time Design Strategy
        ####################################################
        # tuning parameters
        tr_h = 8.0 # rise time for inner loop
        M = 10
        tr_z = tr_h * M
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
        # PD design for inner loop
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
        self.ki_h = 0.4
        # DC gain for inner loop
        k_DC_th = 1
        #---------------------------------------------------
        #                    Outer Loop
        #---------------------------------------------------
        # PD design for outer loop
        b1 = -self.Fe/(P.mc + 2.*P.mr)
        a1 = P.mu/(P.mc + 2*P.mr)
        wn_z = 2.2/tr_z

        self.kp_z = wn_z*2/b1
        self.kd_z = (2*zeta_z*wn_z - a1)/b1
        self.ki_z = 0.1
        # DC gain for outer loop
        # k_DC_z = P.k * k_DC_th * self.kp_z \
        #     / (P.k + P.k * k_DC_th * self.kp_z)
        # print control gains to terminal        
        # print('k_DC_phi', k_DC_phi)
        print('kp_z:  ', self.kp_z)
        print('kd_z:  ', self.kd_z)        
        print('kp_h:  ', self.kp_h)
        print('kd_h:  ', self.kd_h)
        print('kp_th: ', self.kp_th)
        print('kd_th: ', self.kd_th)
        #---------------------------------------------------
        # initialize variables for integrator & differentiators
        #---------------------------------------------------
        self.z_dot = P.zdot0  # estimated derivative of theta
        self.z_d1 = P.z0  # theta delayed by one sample
        self.h_dot = P.hdot0
        self.h_d1 = P.h0
        self.theta_dot = P.thetadot0
        self.theta_d1 = P.theta0
        self.error_h_dot = 0.0  # estimated derivative of error
        self.error_z_dot = 0.0
        self.error_theta_dot = 0.0
        self.error_h_d1 = 0.0  # Error delayed by one sample
        self.error_z_d1 = 0.0
        self.error_theta_d1 = 0.0
        self.integrator_h = 0.0  # integrator
        self.integrator_z = 0.0
        self.integrator_theta = 0.0

        self.beta = (2 * self.sigma - P.Ts) \
            / (2 * self.sigma + P.Ts)

    def update(self, reference, y):
        z_r = reference[0][0]
        h_r = reference[1][0]

        z = y[0][0]
        h = y[1][0]
        theta = y[2][0]
        # zdot = y[3][0]
        # hdot = y[4][0]
        # thetadot = y[5][0]

        #---------------------------------------------------
        # Update longitude
        #---------------------------------------------------
        # Compute the error in h
        error_h = h_r - h
        # integrate error in h
        self.integrator_h = self.kp_h \
            + (P.Ts / 2) * (error_h + self.error_h_d1)        
        # Compute derivative of phi
        self.h_dot = self.beta * self.h_dot \
            + (2.0 / (2.0*self.sigma + P.Ts)) * ((h - self.h_d1))
        # PID control - unsaturated
        force_unsat = self.kp_h * error_h \
            + self.ki_h * self.integrator_h \
                - self.kd_h * self.h_dot
        # saturate h_r
        force_sat = saturate(force_unsat, P.F_max)
        # Integrator anti - windup
        if abs(self.ki_h) < 0.08:
            self.integrator_h = self.integrator_h \
                + P.Ts / self.ki_h * (force_sat - force_unsat)

        #---------------------------------------------------
        # Update Inner Loop (theta-control)
        #---------------------------------------------------
        # Compute the error in z
        error_z = z_r - z
        # differentiate z
        self.z_dot = self.beta * self.z_dot \
            + (2.0 / (2.0*self.sigma + P.Ts))*((z - self.z_d1))
        
        # control for longitude
        if abs(self.z_dot) <= 0.08:
            self.integrator_z = self.kp_z \
             + (P.Ts / 2)*(error_z + self.error_z_d1) 

        theta_r = self.kp_z*error_z - self.kd_z*self.z_dot - self.ki_z*self.integrator_z

        # differentiate theta
        
        error_th = theta_r - theta
        
        if abs(self.theta_dot) <= 0.08:
            self.integrator_theta = self.kp_th \
            + (P.Ts/2) * (error_th + self.error_theta_d1)
        
        self.theta_dot = self.beta*self.theta_dot \
        + (2.0 / (2.0*self.sigma + P.Ts))*((z - self.theta_d1))

        tau_unsat = self.kp_th*error_th - self.kd_th*self.theta_dot

        # theta_r of outer
        tau_max = 2*P.F_max*P.d
        tau = saturate(tau_unsat, tau_max) 

        motor_thrust = P.mixing @ np.array([[force_sat], [tau]])

    
        # update delayed variables
        self.error_z_d1 = error_z
        self.error_h_d1 = error_h
        self.error_theta_d1 = error_th
        self.z_d1
        self.h_d1 = h
        self.theta_d1 = theta
        # return computed force

        return motor_thrust

def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u






