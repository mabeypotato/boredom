import pendulumParam as P
import hw16 as P16
from control import bode, margin, mag2db
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# Assign inner and outer open-loop transfer functions from
# previous HW solution
P_in = P16.P_in
P_out = P16.P_out

# Compute inner and outer closed-loop transfer functions
C_in = P16.C_in
C_out = P16.C_out

if __name__=="__main__":
      ##################################################
      ########### Inner loop ###########################
      # Calculate the phase and gain margins
      if dB_flag:
            gm, pm, Wcg, Wcp = margin(P_in*C_in)
            gm = mag2db(gm)
            print("Inner Loop:", "gm: ",gm,
                  " pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

      else:
            gm, pm, Wcg, Wcp = margin(P_in * C_in)
            print("Inner Loop:", "gm: ", gm,
                  " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

      # display bode plots of transfer functions
      fig1 = plt.figure()

      # this makes two bode plots for open and closed loop
      bode(P_in * C_in, dB=dB_flag,
            label='$C_{in}P_{in}$ - Open-loop')
      bode(P_in*C_in/(1+P_in*C_in), dB=dB_flag, 
            label=r'$\frac{P_{in}C_{in}}{1+P_{in}C_{in}}$'
                  +'- Closed-loop')

      # now we can add lines to show where we calculated the GM and PM
      gm_line = fig1.axes[0].plot([Wcg, Wcg],
                                    plt.ylim(), 'k--', label='GM')
      gm_line[0].set_label('GM')
      fig1.axes[0].legend()
      fig1.axes[1].plot([Wcp, Wcp], plt.ylim(), 'b--', label='PM')
      plt.legend()

      # setting axis title
      fig1.axes[0].set_title('Pendulum Inner Loop - '+
                              'GM:'+str(round(gm, 2))+
                              ', PM:'+str(round(pm, 2)))

      ##################################################
      ########### Outer loop ###########################
      # Calculate the phase and gain margins
      if dB_flag:
            gm, pm, Wcg, Wcp = margin(P_out * C_out)
            gm = mag2db(gm)
            print("Outer Loop:", "gm: ", gm,
                  " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

      else:
            gm, pm, Wcg, Wcp = margin(P_out * C_out)
            print("Outer Loop:", "gm: ", gm,
                  " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

      # display bode plots of transfer functions
      fig2 = plt.figure()

      # this makes two bode plots for open and closed loop
      bode(P_out * C_out, dB=dB_flag,
            label='$C_{out}P_{out}$ - Open-loop')
      bode(P_out*C_out/(1+P_out*C_out), dB=dB_flag, 
            label=r'$\frac{P_{out}C_{out}}{1+P_{out}C_{out}}$'
                  +' - Closed-loop')

      # now we can add lines to show where we calculated the GM and PM
      gm_line = fig2.axes[0].plot([Wcg, Wcg],
                                    plt.ylim(), 'k--', label='GM')
      gm_line[0].set_label('GM')
      fig2.axes[0].legend()
      fig2.axes[1].plot([Wcp, Wcp], plt.ylim(), 'b--', label='PM')
      plt.legend()

      # setting axis title
      fig2.axes[0].set_title('Pendulum Outer Loop - '+
                              'GM:'+str(round(gm, 2))+
                              ', PM:'+str(round(pm, 2)))

      print('Close window to end program')
      plt.show()
