import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter
from massDynamics import massDynamics
from ctrlObserverD import ctrlObserver
from dataPlotterObserver import dataPlotterObserver

# instantiate mass, controller, and reference classes
mass = massDynamics(alpha=0.0)
controller = ctrlObserver()
reference = signalGenerator(amplitude= 1, 
                            frequency=0.05, y_offset=1)
disturbance = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
dataPlotObserver = dataPlotterObserver()
animation = massAnimation()

t = P.t_start  # time starts at t_start
y = mass.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot: 
        r = reference.square(t)
        d = disturbance.step(t) # start with no d,
                                # then use this for part e)

        u, xhat = controller.update(r, y)  # update controller
        y = mass.update(u)  # propagate system
        t += P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(mass.state)
    dataPlot.update(t, mass.state, u, r)
    dataPlotObserver.update(t, mass.state, xhat)

    # the pause causes the figure to display during simulation
    plt.pause(0.0001)  

# Keeps the program from closing until user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
