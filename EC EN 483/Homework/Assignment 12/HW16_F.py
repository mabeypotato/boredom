# satellite parameter file
import  VTOLParam as P
from ctrlPIDhw10 import ctrlPID
import HW15_F as P15
from control import tf, bode
import matplotlib.pyplot as plt
import numpy as np

P10 = ctrlPID()

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P15.dB_flag

# Compute inner and outer open-loop transfer functions
A_tau = 1/(P.Jc + 2*P.mr*P.d**2)

num_theta = -P.Fe/(P.mc + 2*P.mr)
den_theta =  P.mu/(P.mc + 2*P.mr)

P_in = tf([A_tau],
          [1, 0, 0])
P_out = tf([num_theta], 
           [1, den_theta, 0])

# Compute controller transfer functions

# PD control xfer function for inner loop
C_in = tf([(P10.kd_h+P10.sigma*P10.kp_h), P10.kp_h], [P10.sigma, 1])

# PID xfer function for outer loop
C_out = tf([(P10.kd_z+P10.kp_z*P10.sigma),
            (P10.kp_z+P10.ki_z*P10.sigma),
            P10.ki_z],
           [P10.sigma, 1, 0])

if __name__ == '__main__':

    omegas = np.logspace(-2, 3, 1000)

    # display bode plots of transfer functions
    fig1 = plt.figure()
    bode([P_in, P_in*C_in, tf([1.0], [1.0, 0.0, 0.0])],
         omega=omegas, dB=dB_flag)
    plt.legend(['$P_{in}(s)$', '$C_{in}(s)P_{in}(s)$', '$\\frac{1}{s^2}$'])
    fig1.axes[0].set_title('VTOL, Inner Loop')

    fig2 = plt.figure()
    bode([P_out, P_out*C_out], omega=omegas, dB=dB_flag)
    plt.legend(['$P_{out}(s)$', '$C_{out}(s)P_{out}(s)$'])
    fig2.axes[0].set_title('VTOL, Outer Loop')

    print('Close window(s) to end program')
    plt.show()
