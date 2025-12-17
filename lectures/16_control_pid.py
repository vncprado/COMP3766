# -*- coding: utf-8 -*-
"""16 - Control - PID
Pseudo code from video
https://www.youtube.com/watch?v=taSlxgvvrBM&list=PLggLP4f-rq02ecj0q0VB9jEbfua8lOc_E&index=1

eint = 0
eprev =  0

repeart every dt seconds:
    e = desired - read_sensor()
    edot = (e - eprev)/dt
    eint = eint + e*dt
    u = kp*e + ki*eint + kd*edot
    eprev = e
    send_control(u)
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lti, lsim

# Define a simple first-order plant G(s) = K / (tau * s + 1)
K = 1.0   # Process gain
tau = 2.0  # Time constant
plant = lti([K], [tau, 1])

def system_response(t, u):
    _, y, _ = lsim(plant, U=u, T=t)
    return y

# PID parameters
kp = 1  # Proportional gain
ki = 0.0  # Integral gain
kd = 0.0 # Derivative gain

# Simulation parameters
dt = 0.001  # Time step
total_time = 5  # Total simulation time

# Initialization
eint = 0
eprev = 0
pv = 0  # Process variable, initially 0

# Desired value (step function at 1 second)
desired = np.zeros(int(total_time / dt))  # Initialize to 0
desired[int(1 / dt):] = 1  # Set to 1 after 1 second

# Store data for plotting
time_values = []
pv_values = []
desired_values = []

# Simulation loop
for i in range(int(total_time / dt)):
    # Calculate error
    e = desired[i] - pv

    # Calculate derivative of error
    edot = (e - eprev) / dt

    # Calculate integral of error
    eint = eint + e * dt

    # Calculate control output
    u = kp * e + ki * eint + kd * edot

    # Update process variable (simple model)
    pv = system_response(u, dt)

    # Store data for plotting
    time_values.append(i * dt)
    pv_values.append(pv)
    desired_values.append(desired[i])

    # Update previous error
    eprev = e

# Plotting
plt.plot(time_values, pv_values, label='Process Variable (PV)')
plt.plot(time_values, desired_values, label='Desired Value')
plt.xlabel('Time (seconds)')
plt.ylabel('Value')
plt.title('PID Controller Response (Step Desired)')
plt.legend()
plt.grid(True)
plt.show()

