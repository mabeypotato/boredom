import numpy as np
import control as cnt
from scipy import signal
import VTOLParam as P


class ctrlObserver:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        # tuning parameters
        wn_h = 1.0
        zeta_h = 0.95
        wn_z = 0.9905
        zeta_z = 0.95
        wn_th = 13.3803
        zeta_th = 0.95
        integrator_h = 1.0
        integrator_z = 1.0
        # pick observer poles
        wn_th_obs = 10.0*wn_th
        wn_z_obs = 10.0*wn_z
        wn_h_obs = 10.0*wn_h
        
        # State Space Equations
        self.Fe = (P.mc + 2.0 * P.mr) * P.g  # equilibrium force

        self.A_lon = np.array([[0.0, 1.0],
                               [0.0, 0.0]])
        self.B_lon = np.array([[0.0],
                        [1.0 / (P.mc + 2.0 * P.mr)]])
        self.C_lon = np.array([[1.0, 0.0]])
        
        self.A_lat = np.array([[0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                        [0.0, -self.Fe / (P.mc + 2.0 * P.mr), -(P.mu / (P.mc + 2.0 * P.mr)), 0.0],
                        [0.0, 0.0, 0.0, 0.0]])
        self.B_lat = np.array([[0.0],
                        [0.0],
                        [0.0],
                        [1.0 / (P.Jc + 2 * P.mr * P.d ** 2)]])
        self.C_lat = np.array([[1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0]])
        
        # form augmented system
        A1_lon = np.vstack((
                np.hstack((self.A_lon, np.zeros((2,1)))),
                np.hstack((-self.C_lon, np.zeros((1,1))))))
        B1_lon = np.vstack((self.B_lon, np.zeros((1,1))))

        A1_lat = np.vstack((
                np.hstack((self.A_lat, np.zeros((4,1)))),
                np.hstack((-self.C_lat[0:1], np.zeros((1,1))))))
        B1_lat = np.vstack((self.B_lat, np.zeros((1,1))))

        # gain calculation
        des_char_poly_lon = np.convolve([1.0, 2.0 * zeta_h * wn_h, wn_h ** 2],
                                        [1, integrator_h])
        
        des_poles_lon = np.roots(des_char_poly_lon)

        des_char_poly_lat = np.convolve(
            np.convolve([1.0, 2.0 * zeta_z * wn_z, wn_z ** 2],
                        [1.0, 2.0 * zeta_th * wn_th, wn_th ** 2]),
            [1, integrator_z])
        
        des_poles_lat = np.roots(des_char_poly_lat)

        # Compute the gains if the system is controllable
        if np.linalg.matrix_rank(cnt.ctrb(A1_lon, B1_lon)) != 3:
            print("The longitudinal system is not controllable")
        else:
            K1_lon = cnt.place(A1_lon, B1_lon, des_poles_lon)
            self.K_lon = K1_lon[0][0:2]
            self.ki_lon = K1_lon[0][2]
        if np.linalg.matrix_rank(cnt.ctrb(A1_lat, B1_lat)) != 5:
            print("The lateral system is not controllable")
        else:
            K1_lat = cnt.place(A1_lat, B1_lat, des_poles_lat)
            self.K_lat = K1_lat[0][0:4]
            self.ki_lat = K1_lat[0][4]

        # compute observer gains
        # latitude
        des_lat_obs_char_poly = np.convolve(
            [1, 2*zeta_z*wn_z_obs, wn_z_obs**2],
            [1, 2*zeta_th*wn_th_obs, wn_th_obs**2])
        des_lat_obs_poles = np.roots(des_lat_obs_char_poly)
        # Compute the gains if the system is observable
        if np.linalg.matrix_rank(cnt.ctrb(self.A_lat.T, self.C_lat.T)) != 4:
            print("The system is not observable")
        else:
            # place_poles returns an object with various properties.
            # The gains are accessed through .gain_matrix
            # .T transposes the matrix
            self.L_obs_lat = cnt.place(self.A_lat.T, self.C_lat.T, des_lat_obs_poles).T

        # longitude
        des_lon_obs_poles = np.roots([1.0, 2.0*zeta_h*wn_h_obs, wn_h_obs**2])

        if np.linalg.matrix_rank(cnt.ctrb(self.A_lon.T, self.C_lon.T)) !=2:
            print("The system is not observable")
        else:
            self.L_obs_lon = cnt.place(self.A_lon.T, self.C_lon.T, des_lon_obs_poles).T


        # print gains to terminal

        print('K_lon: ', self.K_lon)
        print('ki_lon: ', self.ki_lon)
        print('K_lat: ', self.K_lat)
        print('ki_lat: ', self.ki_lat)
        print('L^T: ', self.L_obs_lat.T)
        print('L^T: ', self.L_obs_lon)

        self.integrator_z = 0.0  # integrator on position z
        self.error_z_d1 = 0.0  # error signal delayed by 1 sample
        self.integrator_h = 0.0  # integrator on altitude h
        self.error_h_d1 = 0.0  # error signal delayed by 1 sample

        # this is a bit clunky, but is an attempt to handle saturation
        # limits without having to define them as a function of xhat_lat[1]
        # (which they probably are) or in the motor/thrust frame.
        self.F_limit = P.F_max_thrust * 2.0
        self.tau_limit = P.F_max_thrust * P.d * 2.0
        #--------------------------------------------------
        # variables to implement integrator
        self.integrator_phi = 0.0  # integrator
        self.error_phi_d1 = 0.0  # error signal delayed by 1 sample
        # estimated state variables

        self.tau_d1 = 0.0      # Computed torque delayed 1 sample
        self.f_d1 = 0.0
        
        self.xhat_lon = np.array([[0.0], [0.0]])
        self.xhat_lat = np.array([[0.0], [0.0], [0.0], [0.0]])

    def update(self, phi_r, y):
        # update the observer and extract z_hat
        y_lat = np.array([[y[0][0]], [y[2][0]]])

        y_lon = np.array([[y[1][0]]])

        xhat_lat = self.update_observer_lat(y_lat)
        xhat_lon = self.update_observer_lon(y_lon)

        z_r = phi_r[0][0]
        h_r = phi_r[1][0]

        # integrate error
        error_z = z_r - xhat_lat[0]
        self.integrator_z += (P.Ts/2.0)*(error_z + self.error_z_d1)
        self.error_z_d1 = error_z

        error_h = h_r - xhat_lon
        self.integrator_h += (P.Ts/2.0)*(error_h + self.error_h_d1)
        self.error_h_d1 = error_h

        # Compute the state feedback controllers
        F_tilde = -self.K_lon @ xhat_lon - self.ki_lon*self.integrator_h
        F = self.Fe/np.cos(xhat_lat[1]) + F_tilde[0]
        F_sat = saturate(F[0], self.F_limit)

        tau = -self.K_lat @ xhat_lat - self.ki_lat*self.integrator_z
        tau_sat = saturate(tau[0], self.tau_limit)

        motor_thrust = np.array([[F_sat], [tau_sat]])

        return motor_thrust, xhat_lat, xhat_lon

    def update_observer_lat(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lat(self.xhat_lat, y_m)
        F2 = self.observer_f_lat(self.xhat_lat + P.Ts / 2 * F1, y_m)
        F3 = self.observer_f_lat(self.xhat_lat + P.Ts / 2 * F2, y_m)
        F4 = self.observer_f_lat(self.xhat_lat + P.Ts * F3, y_m)
        self.xhat_lat = self.xhat_lat + P.Ts / 6 * (F1 + 2*F2 + 2*F3 + F4)
        return self.xhat_lat
    
    def update_observer_lon(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lon(self.xhat_lon, y_m)
        F2 = self.observer_f_lon(self.xhat_lon + P.Ts / 2 * F1, y_m)
        F3 = self.observer_f_lon(self.xhat_lon + P.Ts / 2 * F2, y_m)
        F4 = self.observer_f_lon(self.xhat_lon + P.Ts * F3, y_m)
        self.xhat_lon = self.xhat_lon + P.Ts / 6 * (F1 + 2*F2 + 2*F3 + F4)
        return self.xhat_lon

    def observer_f_lat(self, xhat_lat, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        xhat_dot = self.A_lat @ xhat_lat \
                   + self.B_lat * self.tau_d1 \
                   + self.L_obs_lat @ (y_m - self.C_lat @ xhat_lat)
        return xhat_dot
    
    def observer_f_lon(self, xhat_lon, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        xhat_dot = self.A_lon @ xhat_lon \
                   + self.B_lon * (self.f_d1 - self.Fe)\
                   + self.L_obs_lon @ (y_m - self.C_lon @ xhat_lon)
        return xhat_dot

def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u