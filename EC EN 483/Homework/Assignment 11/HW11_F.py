import matplotlib.pyplot as plt
import numpy as np
import VTOLParam as P
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter
from VTOLDynamics import VTOLDynamics
from ctrlObserverF import ctrlObserver
from dataPlotterObserver import dataPlotterObserver

# instantiate VTOL, controller, and reference classes
VTOL = VTOLDynamics(alpha=0.0)
controller = ctrlObserver()
z_reference = signalGenerator(amplitude=2.5, frequency=0.04, y_offset=3.0)
h_reference = signalGenerator(amplitude=3.0, frequency=0.03, y_offset=5.0)
disturbance = signalGenerator(amplitude=1.0)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
# dataPlotObserver = dataPlotterObserver()
animation = VTOLAnimation()

t = P.t_start  # time starts at t_start
y = VTOL.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:  
        h_ref = h_reference.square(t)  # reference input
        z_ref = z_reference.square(t)
        r = np.array([[z_ref], [h_ref]])
        u, xhat_lat, xhat_lon = controller.update(r, y)  # update controller, using y instead of x because this uses observer
        y = VTOL.update(P.mixing @ u)  # propagate system
        t += P.Ts  # advance time by Ts

    # update animation and data plots
    dataPlot.update(t, VTOL.state, u, z_ref, h_ref)
    animation.update(VTOL.state, z_ref)    
    # dataPlotObserver.update(t, VTOL.state, xhat_lat, xhat_lon)

    # the pause causes the figure to display during simulation
    plt.pause(0.0001)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
