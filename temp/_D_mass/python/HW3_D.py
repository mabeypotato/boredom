import matplotlib.pyplot as plt
import massParam as P
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter
from massDynamics import massDynamics

# instantiate arm, controller, and reference classes
block = massDynamics()
torque = signalGenerator(amplitude=0.2, frequency=0.05)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = massAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:  
        # Get referenced inputs from signal generators
        u = torque.square(t)        
        y = block.update(u)  # Propagate the dynamics
        t += P.Ts  # advance time by Ts
    
    # update animation and data plots
    animation.update(block.state)
    dataPlot.update(t, block.state, u)

    # the pause causes the figure to be displayed during the simulation
    plt.pause(0.0001)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
