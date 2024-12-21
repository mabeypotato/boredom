import matplotlib.pyplot as plt
import numpy as np
import blockbeamParam as P
from signalGenerator import signalGenerator
from blockbeamAnimation import blockbeamAnimation
from dataPlotter import dataPlotter
from blockbeamDynamics import blockBeamDynamics
from ctrlStateFeedback import ctrlStateFeedback
import matplotlib
matplotlib.use('tkagg')

# instantiate beam, controller, and reference classes
beam = blockBeamDynamics(alpha=0.0)
controller = ctrlStateFeedback()
reference = signalGenerator(amplitude=0.15, frequency=0.05, y_offset=0.25)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = blockbeamAnimation()

t = P.t_start  # time starts at t_start
y = beam.h()  # output of system at start of simulation

# for part e), we can uncomment below
#beam.state[1,0] = 10.0*np.pi/180.0
#reference = signalGenerator(amplitude = 0.0, frequency=0.0)


while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        r = reference.square(t)  # reference input
        x = beam.state  # use state instead of output
        u = controller.update(r, x)  # update controller
        y = beam.update(u)  # propagate system
        t += P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(beam.state)
    dataPlot.update(t, beam.state, u, r)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
