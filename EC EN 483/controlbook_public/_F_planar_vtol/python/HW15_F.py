# Inverted Pendulum Parameter File
import VTOLParam as P
from control import tf, bode
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = True

# Compute inner and outer open-loop transfer functions
A_tau = 1/(P.Jc + 2*P.mr*P.d**2)

num_theta = -P.Fe/(P.mc + 2*P.mr)
den_theta =  P.mu/(P.mc + 2*P.mr)

P_in = tf([A_tau],
          [1, 0, 0])
P_out = tf([num_theta], 
           [1, den_theta, 0])

if __name__ == '__main__':

    # Plot the open loop bode plots for the inner loop
    fig1 = plt.figure()
    bode(P_in, dB=dB_flag)
    fig1.axes[0].set_title('$P_{in}(s)$')

    fig2 = plt.figure()
    bode(P_out, dB=dB_flag)
    fig2.axes[0].set_title('$P_{out}(s)$')

    print('Close window to end program')
    plt.show()
