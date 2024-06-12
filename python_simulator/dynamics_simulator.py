import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec

# Constants
DT = 0.1
NUM_STEPS = 100
RADIUS = 0.1
HEIGHT = 0.5


class DroneSimulation:
    def __init__(self):
        self.initial_state = [0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0, 0.05, 0.05, 0.05]
        self.trajectory = [self.initial_state]
        self.num_steps = NUM_STEPS
        self.dt = DT
        self.radius = RADIUS
        self.height = HEIGHT
        self.limits = [-np.pi / 3, np.pi / 3]

        self.simulate()

    def update_drone_state(state, control_input, dt):
        """
        Update the state of the drone given the current state and control input.

        Args:
        - state: The current state of the drone.
        - control_input: The control input (e.g., thrust, torque).
        - dt: The time step.

        Returns:
        - new_state: The updated state of the drone.
        """
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
        thrust, torque_phi, torque_theta, torque_psi = control_input

        # Apply dynamics
        # Update linear velocities
        vx_new = vx + thrust * dt
        vy_new = vy + thrust * dt
        vz_new = vz + thrust * dt

        # Update angular velocities
        p_new = p + torque_phi * dt
        q_new = q + torque_theta * dt
        r_new = r + torque_psi * dt

        # Update positions
        x_new = x + vx_new * dt
        y_new = y + vy_new * dt
        z_new = z + vz_new * dt

        # Update angles with bounded movements
        phi_new = phi + p_new * dt
        theta_new = theta + q_new * dt
        psi_new = psi + r_new * dt

        return [
            x_new,
            y_new,
            z_new,
            vx_new,
            vy_new,
            vz_new,
            phi_new,
            theta_new,
            psi_new,
            p_new,
            q_new,
            r_new,
        ]

    def control_input(t):
        """
        Generate control inputs for the drone.

        Args:
        - t: The current time.

        Returns:
        - control_input: A tuple (thrust, torque_phi, torque_theta, torque_psi).
        """
        # Simple control inputs for demonstration
        thrust = 0.1  # Constant thrust
        torque_phi = np.sin(t) * 0.01  # Oscillating torque around phi
        torque_theta = np.cos(t) * 0.01  # Oscillating torque around theta
        torque_psi = 0.0  # No torque around psi

        return thrust, torque_phi, torque_theta, torque_psi

    def update_position(self, state):
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state

        # No linear position change
        x_new, y_new, z_new = 0, 0, 0

        # Apply simple dynamics: damping and noise
        p *= 0.99  # Damping
        q *= 0.99
        r *= 0.99
        p += np.random.normal(0, 0.005)  # Noise
        q += np.random.normal(0, 0.005)
        r += np.random.normal(0, 0.005)

        # Update angles with bounded movements
        phi_new = phi + p * self.dt * 5
        theta_new = theta + q * self.dt * 5
        psi_new = psi + r * self.dt * 5

        # Restrict angles to ±60 degrees (±π/3 radians) from vertical
        if abs(phi_new) > np.pi / 3:
            p = -p  # Reverse direction
            phi_new = np.clip(phi_new, -np.pi / 3, np.pi / 3)

        if abs(theta_new) > np.pi / 3:
            q = -q  # Reverse direction
            theta_new = np.clip(theta_new, -np.pi / 3, np.pi / 3)

        psi_new = (psi_new + np.pi) % (2 * np.pi) - np.pi  # Keep psi within ±π

        return [x_new, y_new, z_new, vx, vy, vz, phi_new, theta_new, psi_new, p, q, r]

    def simulate(self):
        for _ in range(self.num_steps):
            new_state = self.update_position(self.trajectory[-1])
            self.trajectory.append(new_state)

    def get_trajectory_data(self):
        x_data = [state[0] for state in self.trajectory]
        y_data = [state[1] for state in self.trajectory]
        z_data = [state[2] for state in self.trajectory]
        phi_data = [state[6] for state in self.trajectory]
        theta_data = [state[7] for state in self.trajectory]
        psi_data = [state[8] for state in self.trajectory]
        time_data = np.linspace(0, self.dt * self.num_steps, self.num_steps + 1)

        return x_data, y_data, z_data, phi_data, theta_data, psi_data, time_data

    @staticmethod
    def create_cylinder(radius, height, resolution=10):
        z = np.linspace(0, height, resolution)
        theta = np.linspace(0, 2 * np.pi, resolution)
        theta_grid, z_grid = np.meshgrid(theta, z)
        x_grid = radius * np.cos(theta_grid)
        y_grid = radius * np.sin(theta_grid)
        return x_grid, y_grid, z_grid

    @staticmethod
    def rotate_cylinder(x, y, z, phi, theta, psi):
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

    @staticmethod
    def format_pi_axis(ax):
        ax.set_yticks(np.linspace(-np.pi / 3, np.pi / 3, 5))
        ax.set_yticklabels(["$-\\pi/3$", "$-\\pi/6$", "0", "$\\pi/6$", "$\\pi/3$"])

    @staticmethod
    def add_thrust_vector(ax, origin, direction):
        ax.quiver(*origin, *direction, color="orange", length=0.5, normalize=True)

    def animate(
        self,
        i,
        x_data,
        y_data,
        z_data,
        phi_data,
        theta_data,
        psi_data,
        time_data,
        ax_3d,
        ax_phi,
        ax_theta,
        ax_psi,
        x_cyl,
        y_cyl,
        z_cyl,
        path,
        drone_pos,
        phi_line,
        theta_line,
        psi_line,
        drone_objects,
    ):
        for obj in drone_objects:
            obj.remove()
        drone_objects.clear()

        x_rot, y_rot, z_rot = self.rotate_cylinder(
            x_cyl, y_cyl, z_cyl, phi_data[i], theta_data[i], psi_data[i]
        )
        drone_body = ax_3d.plot_surface(x_rot, y_rot, z_rot, color="white", alpha=0.6)
        drone_objects.append(drone_body)

        # Add thrust vector
        self.add_thrust_vector(ax_3d, (0, 0, 0), (0, 0, -1))

        path.set_data(x_data[: i + 1], y_data[: i + 1])
        path.set_3d_properties(z_data[: i + 1])
        drone_pos.set_data([x_data[i]], [y_data[i]])
        drone_pos.set_3d_properties([z_data[i]])

        phi_line.set_data(time_data[: i + 1], phi_data[: i + 1])
        theta_line.set_data(time_data[: i + 1], theta_data[: i + 1])
        psi_line.set_data(time_data[: i + 1], psi_data[: i + 1])

        return path, drone_pos, phi_line, theta_line, psi_line

    def plot_simulation(self):
        x_data, y_data, z_data, phi_data, theta_data, psi_data, time_data = (
            self.get_trajectory_data()
        )

        fig = plt.figure(figsize=(12, 8))
        gs = gridspec.GridSpec(3, 2, height_ratios=[2, 1, 1])

        ax_3d = fig.add_subplot(gs[0, :], projection="3d")
        ax_3d.set_xlim(-1, 1)
        ax_3d.set_ylim(-1, 1)
        ax_3d.set_zlim(-1, 1)
        ax_3d.set_xlabel("X")
        ax_3d.set_ylabel("Y")
        ax_3d.set_zlabel("Z")

        x_cyl, y_cyl, z_cyl = self.create_cylinder(self.radius, self.height)
        (path,) = ax_3d.plot([], [], [], "b-")
        (drone_pos,) = ax_3d.plot([], [], [], "ro")

        ax_phi = fig.add_subplot(gs[1, 0])
        ax_theta = fig.add_subplot(gs[1, 1])
        ax_psi = fig.add_subplot(gs[2, 0])

        ax_phi.set_xlim(0, time_data[-1])
        ax_theta.set_xlim(0, time_data[-1])
        ax_psi.set_xlim(0, time_data[-1])
        ax_phi.set_ylim(-np.pi / 3, np.pi / 3)
        ax_theta.set_ylim(-np.pi / 3, np.pi / 3)
        ax_psi.set_ylim(-np.pi, np.pi)
        self.format_pi_axis(ax_phi)
        self.format_pi_axis(ax_theta)
        self.format_pi_axis(ax_psi)
        ax_phi.set_ylabel("Phi (rad)")
        ax_theta.set_ylabel("Theta (rad)")
        ax_psi.set_ylabel("Psi (rad)")
        ax_psi.set_xlabel("Time (s)")

        (phi_line,) = ax_phi.plot([], [], "r-")
        (theta_line,) = ax_theta.plot([], [], "g-")
        (psi_line,) = ax_psi.plot([], [], "b-")

        drone_objects = []

        ani = FuncAnimation(
            fig,
            self.animate,
            frames=self.num_steps,
            interval=100,
            blit=False,
            fargs=(
                x_data,
                y_data,
                z_data,
                phi_data,
                theta_data,
                psi_data,
                time_data,
                ax_3d,
                ax_phi,
                ax_theta,
                ax_psi,
                x_cyl,
                y_cyl,
                z_cyl,
                path,
                drone_pos,
                phi_line,
                theta_line,
                psi_line,
                drone_objects,
            ),
        )

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    simulation = DroneSimulation()
    simulation.plot_simulation()
