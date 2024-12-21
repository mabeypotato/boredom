import matplotlib
matplotlib.use('tkAgg')
import matplotlib.pyplot as plt
import numpy as np
import blockbeamParam as P
from signalGenerator import signalGenerator
from blockbeamAnimation import blockbeamAnimation
from dataPlotter import dataPlotter
from blockbeamDynamics import blockBeamDynamics
from ctrlObserverE import ctrlObserver
from dataPlotterObserver import dataPlotterObserver

# instantiate blockbeam, controller, and reference classes
blockbeam = blockBeamDynamics(alpha=0.0)
controller = ctrlObserver()
reference = signalGenerator(amplitude=0.125, frequency=0.05, y_offset=0.25)
disturbance = signalGenerator(amplitude=0.05)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
dataPlotObserver = dataPlotterObserver()
animation = blockbeamAnimation()

t = P.t_start  # time starts at t_start
y = blockbeam.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        r = reference.square(t)
        d = disturbance.step(t)
        u, xhat = controller.update(r, y)
        y = blockbeam.update(u)  # propagate system
        t += P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(blockbeam.state)
    dataPlot.update(t, blockbeam.state, u, r)
    dataPlotObserver.update(t, blockbeam.state, xhat, d, 0.0)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()