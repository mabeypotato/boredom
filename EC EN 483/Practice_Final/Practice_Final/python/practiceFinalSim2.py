import matplotlib.pyplot as plt
import rodMassParam as P
from rodMassAnimation import rodMassAnimation
from dataPlotter import dataPlotter
from rodMassDynamics import rodMassDynamics

# instantiate arm, controller, and reference classes
rodMass = rodMassDynamics()

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = rodMassAnimation()

t = P.t_start
while t < P.t_end:
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        u = P.tau_eq
        y = rodMass.update(u)
        t = t + P.Ts
    # update animation and data plots
    animation.update(rodMass.state)
    dataPlot.update(t, 0, rodMass.state, u)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
