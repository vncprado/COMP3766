"""
This example is based on
Pseudo code from video
https://www.youtube.com/watch?v=taSlxgvvrBM&list=PLggLP4f-rq02ecj0q0VB9jEbfua8lOc_E&index=1
And in the equations 11.21 and 11.23 from the text book

eint = 0
eprev =  0

repeat every dt seconds:
    e = desired - read_sensor()
    edot = (e - eprev)/dt
    eint = eint + e*dt
    u = kp*e + ki*eint + kd*edot
    eprev = e
    send_control(u)
"""

import numpy as np
import matplotlib.pyplot as plt

# Initial conditions
theta = 0.0
thetadot = 0.0
dt = 0.01  # Time step

def send_control(tau):
    """
    Updates theta and thetadot using numerical integration based on the control input (torque).
    """
    global theta, thetadot

    # Physical constants
    M = 0.5
    m = 0.1
    g = 9.81
    r = 0.1
    b = 0.1

    # Compute angular acceleration using the full equation
    thetadotdot = (tau - m * g * r * np.cos(theta) - b * thetadot) / M

    # Numerical integration
    thetadot += thetadotdot * dt  # Update angular velocity
    theta += thetadot * dt        # Update angular position

def read_sensor():
    """
    Simulates a sensor reading of theta with noise.
    """
    noise = np.random.normal(0, 0.01)  # Gaussian noise with mean=0, std=0.01
    return theta + noise
    # return theta

# PID parameters
Kp = 0  # Proportional
Ki = 0 # Integral
Kd = 0  # Derivative

# Simulation parameters
total_time = 5  # Total time

eint = 0
eprev = 0

# Desired final angle (can be any value you choose, e.g., np.pi or 2)
desired_angle = np.pi/2 #1  # Change this to any desired angle (e.g., 2, np.pi)

# Time array
time_values = np.arange(0, total_time, dt)

# Desired values: step from 0 to the desired angle
desired_values = np.zeros_like(time_values)
desired_values[int(1 / dt):] = desired_angle  # Step at t = 1s

position_values = []
for t, desired in zip(time_values, desired_values):
    # Simulate sensor reading
    sensor_value = read_sensor()  # Read from sensor with noise

    # Error calculation
    e = desired - sensor_value
    edot = (e - eprev) / dt
    eint = eint + e*dt

    # Control action calculation (acting as force/torque)
    u = Kp * e + Ki * eint + Kd * edot
    eprev = e
    
    send_control(u)

    position_values.append(sensor_value)

# Plotting
plt.plot(time_values, position_values, label='Position')
plt.plot(time_values, desired_values, label='Desired Position')
plt.xlabel('Time (seconds)')
plt.ylabel('Position')
plt.title('PID Controller Response for a Robotic Arm')
plt.legend()
plt.grid(True)
plt.show()