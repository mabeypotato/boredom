import numpy as np
import control as cnt
import VTOLParam as P

class ctrlStateFeedback:
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
        # State Space Equations
        # xdot = A*x + B*u
        # y = C*x
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
        
        A_lon = np.array([[0.0, 1.0],
                          [0.0, 0.0]])
        B_lon = np.array([[0], [1/(P.mc + 2*P.mr)]])
        C_lon = np.array([1, 0])

        # gain calculation
        wn_th = 2.2 / tr_th
        wn_z = 2.2 / tr_z
        des_lat_char_poly = np.convolve([1, 2 * zeta_th * wn_th, wn_th**2],
                                        [1, 2 * zeta_z * wn_z, wn_z**2])
        des_lat_poles = np.roots(des_lat_char_poly)
    
        # Compute the gains of latitudinal system if it's controllable
        if np.linalg.matrix_rank(cnt.ctrb(A_lat, B_lat)) != 4:
            print("The lat system is not controllable")
        else:
            self.K_lat = cnt.acker(A_lat, B_lat, des_lat_poles)
            Cr_lat = np.array([[1.0, 0.0, 0.0, 0.0]])
            self.kr_lat = -1.0 / (Cr_lat @ np.linalg.inv(A_lat - B_lat @ self.K_lat) @ B_lat)
        # print gains to terminal
        print('K_lat: ', self.K_lat)
        print('kr_lat: ', self.kr_lat)

        wn_h = 2.2 / tr_h
        des_lon_poles = np.roots([1, 2 * zeta_h * wn_h, wn_h**2])
        # Compute the gains of longitudinal system if it's controllable
        if np.linalg.matrix_rank(cnt.ctrb(A_lon, B_lon)) != 2:
            print("The lon system is not controllable")
        else:
            self.K_lon = cnt.acker(A_lon, B_lon, des_lon_poles)
            Cr_lon = np.array([[1.0, 0.0]])
            self.kr_lon = -1.0 / (Cr_lon @ np.linalg.inv(A_lon - B_lon @ self.K_lon) @ B_lon)
        # print gains to terminal
        print('K_lon: ', self.K_lon)
        print('kr_lon: ', self.kr_lon)

    def update(self, r, x):
        # Compute the state feedback controller

       # x_lon is my h and hdot terms
       # x_lat is my z, theta, zdot, thetadot 

        x_lon = np.array([x[1], x[4]])
        x_lat = np.array([x[0], x[2], x[3], x[5]])
       
        F_tilde = -self.K_lon @ x_lon + self.kr_lon * r[1]
        F_unsat = F_tilde + P.Fe
        F_sat = saturate(F_unsat[0][0], P.F_max_thrust)

        tau_tilde = -self.K_lat @ x_lat + self.kr_lat * r[0]
        tau_unsat = tau_tilde
        tau_sat = saturate(tau_unsat[0][0], P.F_max_thrust*P.d*2)

        motor_thrust = P.mixing @ np.array([[F_unsat[0][0]], [tau_unsat[0][0]]]) # 2x2 @ 2x1
        return motor_thrust


def saturate(u, limit):


    if abs(u) > limit:
        u = limit * np.sign(u)
    return u


