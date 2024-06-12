# dynamics_simulator.py

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec


# Function to update the drone's position and handle bouncing
def update_position(state, dt, limits):
    x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state

    # Update position
    x_new = x + vx * dt
    y_new = y + vy * dt
    z_new = z + vz * dt

    # Handle bouncing
    if not (0 <= x_new <= limits[0]):
        vx = -vx
        x_new = x + vx * dt  # Recalculate position after bounce
    if not (0 <= y_new <= limits[1]):
        vy = -vy
        y_new = y + vy * dt  # Recalculate position after bounce
    if not (0 <= z_new <= limits[2]):
        vz = -vz
        z_new = z + vz * dt  # Recalculate position after bounce

    # Update angles
    phi_new = phi + p * dt
    theta_new = theta + q * dt
    psi_new = psi + r * dt

    return [x_new, y_new, z_new, vx, vy, vz, phi_new, theta_new, psi_new, p, q, r]


# Function to create a 3D cylinder (drone)
def create_cylinder(radius, height, resolution=10):
    z = np.linspace(0, height, resolution)
    theta = np.linspace(0, 2 * np.pi, resolution)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x_grid = radius * np.cos(theta_grid)
    y_grid = radius * np.sin(theta_grid)
    return x_grid, y_grid, z_grid


# Function to rotate the cylinder
def rotate_cylinder(x, y, z, phi, theta, psi):
    # Rotation matrices
    R_x = np.array(
        [[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]]
    )

    R_y = np.array(
        [
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)],
        ]
    )

    R_z = np.array(
        [[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]]
    )

    R = np.dot(R_z, np.dot(R_y, R_x))

    xyz = np.array([x.flatten(), y.flatten(), z.flatten()])
    rotated_xyz = np.dot(R, xyz)

    x_rotated = rotated_xyz[0].reshape(x.shape)
    y_rotated = rotated_xyz[1].reshape(y.shape)
    z_rotated = rotated_xyz[2].reshape(z.shape)

    return x_rotated, y_rotated, z_rotated


# Initialize the drone's state
# [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
initial_state = [0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0, 0.1, 0.1, 0.1]
dt = 0.1  # Time step
num_steps = 100
limits = [2, 2, 2]  # Bounds for bouncing

# Store the trajectory
trajectory = [initial_state]

# Simulate the motion
for _ in range(num_steps):
    new_state = update_position(trajectory[-1], dt, limits)
    trajectory.append(new_state)

# Extract positions and angles for plotting
x_data = [state[0] for state in trajectory]
y_data = [state[1] for state in trajectory]
z_data = [state[2] for state in trajectory]
phi_data = [state[6] for state in trajectory]
theta_data = [state[7] for state in trajectory]
psi_data = [state[8] for state in trajectory]
time_data = np.linspace(0, dt * num_steps, num_steps + 1)

# Create the figure and subplots
fig = plt.figure(figsize=(12, 8))
gs = gridspec.GridSpec(3, 2, height_ratios=[2, 1, 1])

# 3D plot for the drone animation
ax_3d = fig.add_subplot(gs[0, :], projection="3d")

# Create the cylinder representing the drone
radius = 0.1
height = 0.5
x_cyl, y_cyl, z_cyl = create_cylinder(radius, height)

# Set the plot limits
ax_3d.set_xlim(0, limits[0])
ax_3d.set_ylim(0, limits[1])
ax_3d.set_zlim(0, limits[2])
ax_3d.set_xlabel("X")
ax_3d.set_ylabel("Y")
ax_3d.set_zlabel("Z")

# Create the initial plot elements
(path,) = ax_3d.plot([], [], [], "b-")
(drone_pos,) = ax_3d.plot([], [], [], "ro")

# Subplots for angles over time
ax_phi = fig.add_subplot(gs[1, 0])
ax_theta = fig.add_subplot(gs[1, 1])
ax_psi = fig.add_subplot(gs[2, 0])

ax_phi.set_xlim(0, time_data[-1])
ax_theta.set_xlim(0, time_data[-1])
ax_psi.set_xlim(0, time_data[-1])

ax_phi.set_ylim(-np.pi, np.pi)
ax_theta.set_ylim(-np.pi, np.pi)
ax_psi.set_ylim(-np.pi, np.pi)

ax_phi.set_ylabel("Phi (rad)")
ax_theta.set_ylabel("Theta (rad)")
ax_psi.set_ylabel("Psi (rad)")
ax_psi.set_xlabel("Time (s)")

(phi_line,) = ax_phi.plot([], [], "r-")
(theta_line,) = ax_theta.plot([], [], "g-")
(psi_line,) = ax_psi.plot([], [], "b-")

# Keep references to the plotted objects to remove them later
drone_objects = []


# Animation function
def animate(i):
    global drone_objects
    # Remove previous drone objects
    for obj in drone_objects:
        obj.remove()
    drone_objects = []

    # Rotate and plot the drone
    x_rot, y_rot, z_rot = rotate_cylinder(
        x_cyl, y_cyl, z_cyl, phi_data[i], theta_data[i], psi_data[i]
    )
    drone_body = ax_3d.plot_surface(
        x_rot + x_data[i],
        y_rot + y_data[i],
        z_rot + z_data[i],
        color="white",
        alpha=0.6,
    )
    drone_objects.append(drone_body)

    # Update path and position
    path.set_data(x_data[: i + 1], y_data[: i + 1])
    path.set_3d_properties(z_data[: i + 1])
    drone_pos.set_data([x_data[i]], [y_data[i]])
    drone_pos.set_3d_properties([z_data[i]])

    # Update angle plots
    phi_line.set_data(time_data[: i + 1], phi_data[: i + 1])
    theta_line.set_data(time_data[: i + 1], theta_data[: i + 1])
    psi_line.set_data(time_data[: i + 1], psi_data[: i + 1])

    return path, drone_pos, phi_line, theta_line, psi_line


# Create the animation
ani = FuncAnimation(fig, animate, frames=num_steps, interval=100, blit=False)

# Show the animation
plt.tight_layout()
plt.show()
