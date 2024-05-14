import numpy as np
from scipy.integrate import odeint

# Parameters
m = 1.5  # mass (kg)
g = 9.81  # gravity (m/s^2)
T = 15.0  # thrust (N)
theta_v = 0.1  # vectoring angle (radians)
psi_v = 0.0  # vectoring angle (radians)

# Define the equations of motion
def equations(state, t):
    x, y, z, vx, vy, vz = state
    # Thrust components
    Tx = T * np.sin(theta_v) * np.cos(psi_v)
    Ty = T * np.sin(theta_v) * np.sin(psi_v)
    Tz = T * np.cos(theta_v)
    
    # Equations of motion
    ax = Tx / m
    ay = Ty / m
    az = (Tz / m) - g
    
    return [vx, vy, vz, ax, ay, az]

# Initial state [x, y, z, vx, vy, vz]
state0 = [0, 0, 0, 0, 0, 0]

# Time vector
t = np.linspace(0, 10, 100)

# Solve the equations
state = odeint(equations, state0, t)

# Extract the position data
x, y, z = state[:, 0], state[:, 1], state[:, 2]

# Plot the results
import matplotlib.pyplot as plt
plt.plot(t, z)
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.title('Drone Altitude Over Time')
plt.show()
