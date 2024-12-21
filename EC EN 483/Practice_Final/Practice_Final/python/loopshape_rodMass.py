import rodMassParam as P
import matplotlib.pyplot as plt
from control import TransferFunction as tf
import control as cnt
import numpy as np

# ----------- noise specification --------
# attenuate noise above omega_n by gamma_n
def add_spec_noise(gamma, omega, flag):
    w = np.logspace(np.log10(omega), np.log10(omega) + 4)
    if flag==True:
        plt.subplot(211)
        plt.plot(w,
                 (20 * np.log10(gamma)) * np.ones(len(w)),
                 color='g',
                 label='noise spec')

#----------- input disturbance specification --------
# reject disturbance above omega by gamma
def add_spec_disturbance(gamma, omega, flag):
    w = np.logspace(np.log10(omega)-4, np.log10(omega))
    if flag==True:
        plt.subplot(211)
        plt.plot(w,
                 20*np.log10(1.0/gamma)*np.ones(len(w)),
                 color='g',
                 label='disturbance spec')

#----------- general tracking specification --------
# track references below omega by gamma
def add_spec_tracking(gamma, omega, flag):
    w = np.logspace(np.log10(omega) - 2, np.log10(omega))
    if flag==True:
        plt.subplot(211)
        plt.plot(w,
                 20*np.log10(1/gamma)*np.ones(len(w)),
                 color='g',
                 label='tracking spec')

#----------- steady state tracking of step --------
# track step by gamma
def add_spec_tracking_step(gamma, flag):
    omega = 0.01
    w = np.logspace(np.log10(omega)-4, np.log10(omega))
    if flag==True:
        plt.subplot(211)
        plt.plot(w,
                 20*np.log10(1.0/gamma),
                 color='g',
                 label='tracking spec')

#----------- steady state tracking of ramp --------
# track ramp by gamma
def add_spec_tracking_ramp(gamma, flag):
    omega = 0.01
    w = np.logspace(np.log10(omega)-4, np.log10(omega))
    if flag==True:
        plt.subplot(211)
        plt.plot(w,
                 20*np.log10(1.0/gamma)-20*np.log10(w),
                 color='g',
                 label='tracking spec')

# proportional control: change cross over frequency
def add_control_proportional(C, kp):
    proportional = tf([kp], [1])
    return C * proportional

# integral control: increase steady state tracking and dist rejection
# ki: frequency at which integral action ends
def add_control_integral(C, ki):
    integrator = tf([1, ki], [1, 0])
    return C * integrator

# phase lag: add gain at low frequency
# z: frequency at which gain ends
# M: separation between pole and zero
def add_control_lag(C, z, M):
    Lag = tf([1, z], [1, z/M])
    return C * Lag

# low pass filter: decrease gain at high frequency (noise)
# p: lpf cutoff frequency
def add_control_lpf(C, p):
    LPF = tf(p, [1, p])
    return C * LPF

# phase lead: increase PM (stability)
# w_L: location of maximum frequency bump
# M: separation between zero and pole
def add_control_lead(C, w_L, M):
    gain = (1.0+np.sqrt(M))/(1.0+1.0/np.sqrt(M))
    Lead = tf([gain * 1.0, gain * w_L / np.sqrt(M)],
              [1.0, w_L * np.sqrt(M)])
    return C * Lead

# Compute plant transfer functions
Plant =
C_pid =

PLOT = True
#PLOT = False

# calculate bode plot and gain and phase margin
mag, phase, omega = cnt.bode(Plant, dB=True, omega=np.logspace(-3, 5), Plot=False)
gm, pm, Wcg, Wcp = cnt.margin(Plant*C_pid)
print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

if PLOT:
    plt.figure(3), plt.clf()
    plt.subplot(211), plt.grid(True)
    plantMagPlot, = plt.semilogx(omega, 20*np.log10(mag), label='Plant')
    plt.subplot(212), plt.grid(True)
    plantPhasePlot, = plt.semilogx(omega, phase, label='Plant')

#########################################
#   Define Design Specifications
#add_spec_noise(gamma=0.1, omega=10, flag=PLOT)
#add_spec_disturbance(gamma=0.1, omega=0.1, flag=PLOT)
#add_spec_tracking(gamma=0.03, omega=0.1, flag=PLOT)
#add_spec_tracking_ramp(gamma=0.03, flag=PLOT)

#########################################
#   Control Design
#C = C_pid
#C = add_control_lag(C, z=, M=)
#C = add_control_lpf(C, p=)
#C = add_control_lead(C, w_L=, M=)
#C = add_control_integral(C, ki=)
#C = add_control_proportional(C, kp=)


mag, phase, omega = cnt.bode(Plant*C, dB=True, omega=np.logspace(-5, 5), Plot=False)
gm, pm, Wcg, Wcp = cnt.margin(Plant*C)
print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)
if PLOT:
    plt.subplot(211),
    plantMagPlot, = plt.semilogx(omega, 20*np.log10(mag), label='PC')
    plt.subplot(212),
    plantPhasePlot, = plt.semilogx(omega, phase, label='PC')


###########################################################
# add a prefilter to eliminate the overshoot
F = tf(1, 1)
#F = add_control_lpf(F, p=)

############################################
#  Create Plots
############################################
# Closed loop transfer function from R to Y - no prefilter
CLOSED_R_to_Y = (Plant*C/(1.0+Plant*C))
# Closed loop transfer function from R to Y - with prefilter
CLOSED_R_to_Y_with_F = (F*Plant*C/(1.0+Plant*C))
# Closed loop transfer function from R to U
CLOSED_R_to_U = (C/(1.0+Plant*C))

if PLOT:
    plt.figure(4), plt.clf()

    plt.subplot(311),  plt.grid(True)
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y, dB=True, Plot=False)
    plt.semilogx(omega, mag, color='b')
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y_with_F, dB=True, Plot=False)
    plt.semilogx(omega, mag, color='g')
    plt.title('Close Loop Bode Plot')

    plt.subplot(312), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_Y, T)
    plt.plot(T, yout, color='b')
    plt.ylabel('Step Response')

    plt.subplot(313), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_U, T)
    plt.plot(T, yout, color='b')
    plt.ylabel('Control Effort')

    # Keeps the program from closing until the user presses a button.
    plt.pause(0.0001)  # not sure why this is needed for both figures to display
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()

##############################################
#  Convert Controller to State Space Equations
##############################################
C_num = np.asarray(C.num[0])
C_den = np.asarray(C.den[0])
F_num = np.asarray(F.num[0])
F_den = np.asarray(F.den[0])

Css=cnt.tf2ss(C)
Fss=cnt.tf2ss(F)



