# mass-spring-damper Parameter File
import numpy as np

# Physical parameters of the arm known to the controller
m = 5 # mass kg
k = 3 # spring constant Kg/s^2
b = 0.5 # damping coefficient Kg/s
g = 9.8 # gravity

# parameters for animation
length = 5.0
width = 1.0

# Initial Conditions
z0 = 1  # initial position of mass, m
zdot0 = 0.5  # initial velocity of mass m/s

# Simulation Parameters
t_start = 0 # Start time of simulation
t_end = 50  # End time of simulation
Ts = 0.001  # sample time for simulation
t_plot = 0.2 # the plotting and animation is updated at this rate

# dirty derivative parameters
sigma =  0.05 # cutoff freq for dirty derivative

# saturation limits
F_max = 10  # Max force, N

