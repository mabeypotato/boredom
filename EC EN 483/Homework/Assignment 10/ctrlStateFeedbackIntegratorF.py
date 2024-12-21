import numpy as np
import control as cnt
import VTOLParam as P

class ctrlStateFeedbackIntegrator:
    def __init__(self):
        #--------------------------------------------------
        # State Feedback Control Design
        #--------------------------------------------------
        # tuning parameters
        tr_h = 2
        tr_th = 0.2
        tr_z = 2 # try ~ 3 or 4ish??
        zeta_h = 0.707  # damping ratio position
        zeta_th = 0.707  # damping ratio angle
        zeta_z = 0.707
        lat_int_pole = -3
        lon_int_pole = -1


        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x

        #### LATITUDINAL ####

        A_lat = np.array([[0.0,       0.0,                       1.0,                      0.0],
                          [0.0,       0.0,                       0.0,                      1.0],
                          [0.0,      -P.Fe/(P.mc + 2*P.mr),     -P.mu/(P.mc + 2*P.mr),     0.0],
                          [0.0,       0.0,                       0.0,                     0.0]])
        B_lat = np.array([[0.0],
                          [0.0],
                          [0.0],
                          [1/(P.Jc + 2*P.mr*P.d**2)]])
        C_lat = np.array([[1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0]])
        
        Cr_lat = np.array([[1.0, 1.0, 0.0, 0.0]])
        A1_lat = np.vstack((np.hstack((A_lat, np.zeros((np.size(A_lat,1),1)))), 
                        np.hstack((-Cr_lat, np.array([[0.0]]))) ))
        B1_lat = np.vstack( (B_lat, 0.0) )
       
        # gain calculation
        wn_th = 2.2 / tr_th
        wn_z = 2.2 / tr_z
        
        des_lat_char_poly = np.convolve(
                        np.convolve([1, 2 * zeta_th * wn_th, wn_th**2],
                                    [1, 2 * zeta_z * wn_z, wn_z**2]),

                        np.poly([lat_int_pole]))
        
        des_lat_poles = np.roots(des_lat_char_poly)
    
        if np.linalg.matrix_rank(cnt.ctrb(A_lat, B_lat)) != 4:
            print("The lat system is not controllable")
        else:
            K1lat = cnt.acker(A_lat, B_lat, des_lat_poles)
            self.K_lat = K1lat[0][0:3]
            self.ki_lat = K1lat[0][3]
            Cr_lat = np.array([[1.0, 0.0, 0.0, 0.0]])
        # print gains to terminal
        print('K_lat: ', self.K_lat)
        print('ki_lat: ', self.ki_lat)

        #### LONGITUDINAL ####

        A_lon = np.array([[0.0, 1.0],
                          [0.0, 0.0]])
        B_lon = np.array([[0], [1/(P.mc + 2*P.mr)]])
        C_lon = np.array([1, 0])

        Cr_lon = np.array([[1.0, 1.0, 0.0, 0.0]])
        A1_lon = np.vstack((np.hstack((A_lon, np.zeros((np.size(A_lon,1),1)))), 
                        np.hstack((-Cr_lon, np.array([[0.0]])))))
        B1_lon = np.vstack( (B_lon, 0.0) )

        # gain calculation
        wn_h = 2.2 / tr_h

        des_lon_char_poly = np.convolve([1, 2 * zeta_h * wn_h, wn_h**2], np.poly([lon_int_pole]))
        des_lon_poles = np.roots(des_lon_char_poly)
        # Compute the gains of longitudinal system if it's controllable
        if np.linalg.matrix_rank(cnt.ctrb(A_lon, B_lon)) != 3:
            print("The lon system is not controllable")
        else:
            K1lon = cnt.acker(A_lon, B_lon, des_lon_poles)
            self.K_lon = K1lon[0][0:2]
            self.ki_lon = K1lon[0][2]
            Cr_lon = np.array([[1.0, 0.0]])
        # print gains to terminal
        print('K_lon: ', self.K_lon)
        print('ki_lon: ', self.ki_lon)

        self.integrator_lat = 0.0
        self.integrator_lon = 0.0
        self.error_d1_lat = 0.0
        self.error_d1_lon = 0.0

    def update(self, r, x):
        # Compute the state feedback controller

       # x_lon is my h and hdot terms
       # x_lat is my z, theta, zdot, thetadot 

        x_lon = np.array([x[1], x[4]])
        x_lat = np.array([x[0], x[2], x[3], x[5]])

        error_lon = r[1] - x_lon[0]
        erorr_lat = r[0] - x_lat[0]

        if abs(x_lon[1]) < 0.8:
            self.integrator_lon = self.integrator_lon + (P.Ts/2.0)*(error_lon + self.error_d1_lon)

        if abs(x_lat[2]) < 0.8:
            self.integrator_lat = self.integrator_lat + (P.Ts/2.0)*(erorr_lat + self.error_d1_lat)
       
        F_tilde = -self.K_lon @ x_lon + self.ki_lon * self.integrator_lon
        F_unsat = F_tilde + P.Fe
        # F_sat = saturate(F_unsat[0][0], P.F_max_thrust)

        tau_tilde = -self.K_lat @ x_lat + self.ki_lat * self.integrator_lat
        tau_unsat = tau_tilde
        # tau_sat = saturate(tau_unsat[0][0], P.F_max_thrust*P.d*2)

        motor_thrust = P.mixing @ np.array([[F_unsat[0][0]], [tau_unsat[0][0]]]) # 2x2 @ 2x1
        return motor_thrust


def saturate(u, limit):


    if abs(u) > limit:
        u = limit * np.sign(u)
    return u


