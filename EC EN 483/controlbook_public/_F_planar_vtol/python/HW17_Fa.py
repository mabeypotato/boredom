# Satellite Parameter File
import VTOLParam as P
import HW16_F as P16
import HW15_F as P15
from ctrlPIDhw10 import ctrlPID
from control import *
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = True
PID = ctrlPID()

# Compute the controller transfer functions from HW10
# (to make sure we don't introduce additional errors)
P_num = 1/(P.mc + 2*P.mr)
P_alt = tf([P_num],
          [1, 0, 0])

C_alt = tf([(PID.kd_h+P.sigma*PID.kp_h), PID.kp_h], [P.sigma, 1])

if __name__ == '__main__':
    
    # we have to define the frequencies, or we get non-smooth
    # bode plots.
    omegas = np.logspace(-2, 3, 1000)

    ##################################################
    ########### Inner loop ###########################
    # Calculate the phase and gain margins
    if dB_flag:
        gm, pm, Wcg, Wcp = margin(P_alt * C_alt)
        gm = mag2db(gm)
        print("Inner Loop:", "gm: ", gm,
              " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    else:
        gm, pm, Wcg, Wcp = margin(P_alt * C_alt)
        print("Inner Loop:", "gm: ", gm,
              " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    # display bode plots of transfer functions
    fig1 = plt.figure()

    # this makes two bode plots for open and closed loop
    bode(P_alt * C_alt / (1 + P_alt * C_alt),
         omega=omegas, dB=dB_flag,
         label=r'$\frac{P_{in}C_{in}}{1+P_{in}C_{in}}$'
               + '- Closed-loop')

    # now we can add lines to show where we calculated the GM and PM
    gm_line = fig1.axes[0].plot([Wcg, Wcg],
                                plt.ylim(), 'k--', label='GM')
    gm_line[0].set_label('GM')
    fig1.axes[0].legend()
    fig1.axes[1].plot([Wcp, Wcp], plt.ylim(), 'b--', label='PM')
    plt.legend()

    # setting axis title
    fig1.axes[0].set_title(r'Satellite Inner Loop ($\theta$) - ' +
                           'GM:' + str(round(gm, 2)) +
                           ', PM:' + str(round(pm, 2)))


    # now we can add lines to show where we calculated the GM and PM
    gm_line = fig1.axes[0].plot([Wcg, Wcg],
                                plt.ylim(), 'k--', label='GM')
    gm_line[0].set_label('GM')
    fig1.axes[0].legend()
    fig1.axes[1].plot([Wcp, Wcp], plt.ylim(), 'b--', label='PM')
    plt.legend()

    # setting axis title
    fig1.axes[0].set_title(r'Satellite Outer Loop ($\phi$) - ' +
                           'GM:' + str(round(gm, 2)) +
                           ', PM:' + str(round(pm, 2)))

    print('Close window to end program')
    plt.show()
