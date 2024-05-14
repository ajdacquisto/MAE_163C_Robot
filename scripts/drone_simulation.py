import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# PID Controller Class
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
    
    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# Parameters
m = 1.5  # mass (kg)
g = 9.81  # gravity (m/s^2)
theta_v = 0.0  # vectoring angle (radians)
psi_v = 0.0  # vectoring angle (radians)

# Desired setpoint for altitude (z)
setpoint_z = 5.0  # desired altitude (m)

# PID Controller for altitude
pid_z = PID(kp=2.0, ki=0.1, kd=0.5)

# Define the equations of motion
def equations(state, t, u):
    x, y, z, vx, vy, vz = state
    # Thrust components
    Tx = u * np.sin(theta_v) * np.cos(psi_v)
    Ty = u * np.sin(theta_v) * np.sin(psi_v)
    Tz = u * np.cos(theta_v)
    
    # Equations of motion
    ax = Tx / m
    ay = Ty / m
    az = (Tz / m) - g
    
    return [vx, vy, vz, ax, ay, az]

# Initial state [x, y, z, vx, vy, vz]
state0 = [0, 0, 0, 0, 0, 0]

# Time vector
t = np.linspace(0, 10, 100)

# Simulation loop
states = []
state = state0
for i in range(1, len(t)):
    dt = t[i] - t[i-1]
    # Update control input using PID controller
    thrust = pid_z.update(setpoint_z, state[2], dt)
    
    # Limit thrust to realistic values (e.g., 0 to 20 N)
    thrust = max(0, min(thrust, 20))
    
    # Solve the equations of motion
    state = odeint(equations, state, [t[i-1], t[i]], args=(thrust,))[-1]
    states.append(state)

# Convert to array for plotting
states = np.array(states)

# Plot results
plt.plot(t[1:], states[:, 2])
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.title('Drone Altitude Over Time with PID Control')
plt.show()
