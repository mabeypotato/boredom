import matplotlib.pyplot as plt
import numpy as np
import hummingbirdParam as P
from signalGenerator import SignalGenerator
from hummingbirdAnimation import HummingbirdAnimation
from hummingbirdDynamics import HummingbirdDynamics
from dataPlotter import DataPlotter
from ctrlEquilibrium import ctrlEquilibrium

# instantiate reference input classes
hummingbird = HummingbirdDynamics(alpha=0.0)
equilibrium = ctrlEquilibrium()
phi_ref = SignalGenerator(amplitude=1.5, frequency=0.05)
theta_ref = SignalGenerator(amplitude=0.5, frequency=0.05)
psi_ref = SignalGenerator(amplitude=0.5, frequency=.05)
torque = SignalGenerator(amplitude=0.5, frequency=.05)
force = SignalGenerator(amplitude=0.5, frequency=.05)

# instantiate the simulation plots and animation
dataPlot = DataPlotter()
animation = HummingbirdAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    t_next_plot = t + P.t_plot  # advance time by t_plot
    while t < t_next_plot:
        # update animation
        ref = np.array([[0], [0], [0]])

        motor_thrust = equilibrium.update(hummingbird.state)
        y = hummingbird.update(motor_thrust)
        t += P.Ts

    animation.update(t, hummingbird.state)
    dataPlot.update(t, hummingbird.state, motor_thrust, ref)

    plt.pause(0.05)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()