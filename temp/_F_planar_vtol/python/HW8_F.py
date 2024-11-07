import matplotlib.pyplot as plt
import numpy as np
import VTOLParam as P
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter
from VTOLDynamics import VTOLDynamics
from ctrlPIDhw10 import ctrlPID
import matplotlib
matplotlib.use('tkagg')

# instantiate vtol, controller, and reference classes
vtol = VTOLDynamics(alpha=0.2)
controller = ctrlPID()
z_reference = signalGenerator(amplitude=2.5, frequency=0.04, y_offset=3.0)
h_reference = signalGenerator(amplitude=3.0, frequency=0.03, y_offset=5.0)



# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = VTOLAnimation()

t = P.t_start  # time starts at t_start
y = vtol.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:  
        h_ref = h_reference.square(t)  # reference input
        z_ref = z_reference.square(t)
        r = np.array([[z_ref], [h_ref]])
        d = np.array([[0.0], [0.0]])
        x = vtol.state
        u = controller.update(r, y)  # update controller
        y = vtol.update(u + d)  # propagate system
        t += P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(vtol.state)
    
    dataPlot.update(t, vtol.state, u, z_ref, h_ref)

    # the pause causes the figure to display for the simulation.
    plt.pause(0.0001)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
