import matplotlib.pyplot as plt
import numpy as np
import blockbeamParam as P
from signalGenerator import signalGenerator
from blockbeamAnimation import blockbeamAnimation
from dataPlotter import dataPlotter

# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1)
zRef = signalGenerator(amplitude=0.5, frequency=0.1)
thetaRef = signalGenerator(amplitude=.25*np.pi, frequency=.5)
fRef = signalGenerator(amplitude=5, frequency=.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = blockbeamAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    r = reference.square(t)
    z = zRef.sin(t) + P.z0
    theta = thetaRef.sin(t) + P.theta0
    f = fRef.sawtooth(t) + P.Fmax
    # update animation
    state = np.array([[z], [theta], [0.0], [0.0]])
    animation.update(state)
    dataPlot.update(t, r, state, f)
    # advance time by t_plot    
    t = t + P.t_plot  
    plt.pause(0.05)  # allow time for animation to draw

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()