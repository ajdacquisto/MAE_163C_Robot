import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
import random

# Constants
DT = 0.1
NUM_STEPS = 200
RADIUS = 0.1
HEIGHT = 0.5


class DroneSimulation:
    def __init__(self):
        """
        Initialize the DroneSimulation class with initial state, simulation parameters, and trajectory.
        """
        self.initial_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.trajectory = [self.initial_state]
        self.num_steps = NUM_STEPS
        self.dt = DT
        self.radius = RADIUS
        self.height = HEIGHT

        self.simulate()

    import random

    def add_random_disturbance(self, state):
        """
        Add a random disturbance to the drone's state.

        Args:
        - state: The current state of the drone.

        Returns:
        - disturbed_state: The state of the drone after applying the disturbance.
        """
        disturbance_intensity = 0.05  # Adjust the intensity of the disturbance
        disturbed_state = np.copy(state)

        # Apply random disturbance to position (x, y, z)
        disturbed_state[0] += disturbance_intensity * (random.random() - 0.5)
        disturbed_state[1] += disturbance_intensity * (random.random() - 0.5)
        disturbed_state[2] += disturbance_intensity * (random.random() - 0.5)

        # Apply random disturbance to orientation (phi, theta, psi)
        disturbed_state[6] += disturbance_intensity * (random.random() - 0.5)
        disturbed_state[7] += disturbance_intensity * (random.random() - 0.5)
        disturbed_state[8] += disturbance_intensity * (random.random() - 0.5)

        return disturbed_state

    def compute_control_action(self, pos_err, vel, gravity, gyro):
        """
        Compute the control action based on position error, velocity, gravity compensation, and gyro effects.

        Args:
        - pos_err: Position error (3D vector).
        - vel: Velocity (3D vector).
        - gravity: Gravity compensation (3D vector).
        - gyro: Gyro effects (3D vector).

        Returns:
        - control_action: Desired forces and torques (3D force vector, 3D torque vector).
        """
        Kp = np.array([1.0, 1.0, 1.0])  # Proportional gains
        Kd = np.array([0.1, 0.1, 0.1])  # Derivative gains

        desired_force = Kp * pos_err + Kd * vel + gravity
        desired_torque = gyro  # Simplified for now, needs actual calculation

        return desired_force, desired_torque

    def update_drone_state(self, state, pos_err, vel, gravity, gyro, dt):
        """
        Update the state of the drone given the current state and control input.

        Args:
        - state: The current state of the drone.
        - pos_err: Position error (3D vector).
        - vel: Velocity (3D vector).
        - gravity: Gravity compensation (3D vector).
        - gyro: Gyro effects (3D vector).
        - dt: The time step.

        Returns:
        - new_state: The updated state of the drone.
        """
        # Compute control action
        desired_force, desired_torque = self.compute_control_action(
            pos_err, vel, gravity, gyro
        )

        # Unpack the state variables
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state

        # Constants (to be defined based on your drone's specifications)
        m = 1.0  # Mass of the drone
        I_x = 0.1  # Moment of inertia around x-axis
        I_y = 0.1  # Moment of inertia around y-axis
        I_z = 0.1  # Moment of inertia around z-axis

        # Linear motion
        ax = (desired_force[0] - gravity[0]) / m
        ay = (desired_force[1] - gravity[1]) / m
        az = (desired_force[2] - gravity[2]) / m

        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        vz_new = vz + az * dt

        x_new = x + vx_new * dt
        y_new = y + vy_new * dt
        z_new = z + vz_new * dt

        # Angular motion
        p_new = p + (desired_torque[0] - (I_y - I_z) * q * r) / I_x * dt
        q_new = q + (desired_torque[1] - (I_z - I_x) * p * r) / I_y * dt
        r_new = r + (desired_torque[2] - (I_x - I_y) * p * q) / I_z * dt

        phi_new = (
            phi
            + (p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta))
            * dt
        )
        theta_new = theta + (q * np.cos(phi) - r * np.sin(phi)) * dt
        psi_new = psi + (q * np.sin(phi) + r * np.cos(phi)) / np.cos(theta) * dt

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

    def convert_to_servo_angles(self, desired_force, desired_torque):
        """
        Convert the desired forces and torques into servo fin angles.

        Args:
        - desired_force: Desired force vector.
        - desired_torque: Desired torque vector.

        Returns:
        - servo_angles: Angles for the servo fins to achieve the desired forces and torques.
        """
        # Placeholder implementation: map forces and torques to servo angles
        servo_angles = desired_force + desired_torque  # Simplified for now
        return servo_angles

    def control_input(self, t):
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

    def simulate(self):
        """
        Simulate the drone's trajectory over the specified number of steps.
        """
        disturbance_interval = 5  # Apply disturbance every 5 steps
        for i in range(self.num_steps):
            t = i * self.dt
            control = self.control_input(t)

            # Compute position error (for simplicity, using zero position error)
            pos_err = np.array([0.0, 0.0, 0.0])

            # Get current state
            current_state = self.trajectory[-1]

            # Compute velocity
            vel = np.array(current_state[3:6])

            # Gravity compensation (for simplicity, using a constant gravity vector)
            gravity = np.array([0.0, 0.0, 9.81])

            # Gyro effects (for simplicity, using zero gyro effects)
            gyro = np.array([0.0, 0.0, 0.0])

            new_state = self.update_drone_state(
                current_state, pos_err, vel, gravity, gyro, self.dt
            )

            # Apply random disturbance at intervals
            if i % disturbance_interval == 0 and i != 0:
                new_state = self.add_random_disturbance(new_state)

            self.trajectory.append(new_state)

    def get_trajectory_data(self):
        """
        Retrieve the trajectory data for plotting.

        Returns:
        - x_data: List of x positions over time.
        - y_data: List of y positions over time.
        - z_data: List of z positions over time.
        - phi_data: List of phi (roll) angles over time.
        - theta_data: List of theta (pitch) angles over time.
        - psi_data: List of psi (yaw) angles over time.
        - time_data: List of time steps.
        """
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
        """
        Create a 3D cylinder representing the drone.

        Args:
        - radius: The radius of the cylinder.
        - height: The height of the cylinder.
        - resolution: The resolution of the cylinder mesh.

        Returns:
        - x_grid, y_grid, z_grid: The coordinates of the cylinder mesh.
        """
        z = np.linspace(0, height, resolution)
        theta = np.linspace(0, 2 * np.pi, resolution)
        theta_grid, z_grid = np.meshgrid(theta, z)
        x_grid = radius * np.cos(theta_grid)
        y_grid = radius * np.sin(theta_grid)
        return x_grid, y_grid, z_grid

    @staticmethod
    def rotate_cylinder(x, y, z, phi, theta, psi):
        """
        Rotate the 3D cylinder to represent the drone's orientation.

        Args:
        - x, y, z: The coordinates of the cylinder mesh.
        - phi: The roll angle.
        - theta: The pitch angle.
        - psi: The yaw angle.

        Returns:
        - x_rotated, y_rotated, z_rotated: The rotated coordinates of the cylinder mesh.
        """
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
        """
        Format the y-axis of a plot to display in terms of pi.

        Args:
        - ax: The matplotlib axis object.
        """
        ax.set_yticks(np.linspace(-np.pi / 3, np.pi / 3, 5))
        ax.set_yticklabels(["$-\\pi/3$", "$-\\pi/6$", "0", "$\\pi/6$", "$\\pi/3$"])

    @staticmethod
    def add_thrust_vector(ax, origin, thrust_vector):
        """
        Add a thrust vector to the 3D plot.

        Args:
        - ax: The matplotlib axis object.
        - origin: The origin of the thrust vector.
        - thrust_vector: The thrust vector direction and magnitude.

        Returns:
        - thrust_vec: The quiver object representing the thrust vector.
        """
        thrust_vec = ax.quiver(
            *origin,
            *thrust_vector,
            color="orange",
            length=np.linalg.norm(thrust_vector),
            normalize=True
        )
        return thrust_vec

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
        thrust_vectors,
    ):
        """
        Animate the drone's trajectory and orientation over time.

        Args:
        - i: The current frame index.
        - x_data, y_data, z_data: The positional data.
        - phi_data, theta_data, psi_data: The angular data.
        - time_data: The time steps.
        - ax_3d: The 3D axis for the drone plot.
        - ax_phi, ax_theta, ax_psi: The 2D axes for the angle plots.
        - x_cyl, y_cyl, z_cyl: The cylinder mesh coordinates.
        - path: The line object for the drone's path.
        - drone_pos: The line object for the drone's position.
        - phi_line, theta_line, psi_line: The line objects for the angle plots.
        - drone_objects: The list of drone objects to be animated.
        - thrust_vectors: The list of thrust vector objects to be animated.
        """
        # Clear previous drone objects
        for obj in drone_objects:
            obj.remove()
        drone_objects.clear()

        # Clear previous thrust vectors
        for vec in thrust_vectors:
            vec.remove()
        thrust_vectors.clear()

        # Rotate the cylinder to match the current orientation
        x_rot, y_rot, z_rot = self.rotate_cylinder(
            x_cyl, y_cyl, z_cyl, phi_data[i], theta_data[i], psi_data[i]
        )
        drone_body = ax_3d.plot_surface(x_rot, y_rot, z_rot, color="white", alpha=0.6)
        drone_objects.append(drone_body)

        # Compute thrust vector based on control inputs (for simplicity, using a constant thrust vector)
        thrust_vector = np.array([0.0, 0.0, -1.0])

        # Add thrust vector
        thrust_vec = self.add_thrust_vector(
            ax_3d, (x_data[i], y_data[i], z_data[i]), thrust_vector
        )
        thrust_vectors.append(thrust_vec)

        # Update the 3D path and position
        path.set_data(x_data[: i + 1], y_data[: i + 1])
        path.set_3d_properties(z_data[: i + 1])
        drone_pos.set_data([x_data[i]], [y_data[i]])
        drone_pos.set_3d_properties([z_data[i]])

        # Update the 2D angle plots
        phi_line.set_data(time_data[: i + 1], phi_data[: i + 1])
        theta_line.set_data(time_data[: i + 1], theta_data[: i + 1])
        psi_line.set_data(time_data[: i + 1], psi_data[: i + 1])

        return path, drone_pos, phi_line, theta_line, psi_line

    def plot_simulation(self):
        """
        Plot the simulation results including the 3D drone plot and the angle plots.
        """
        # Retrieve trajectory data
        x_data, y_data, z_data, phi_data, theta_data, psi_data, time_data = (
            self.get_trajectory_data()
        )

        # Create figure and grid layout
        fig = plt.figure(figsize=(12, 8))
        gs = gridspec.GridSpec(3, 2, height_ratios=[2, 1, 1])

        # Create 3D plot
        ax_3d = fig.add_subplot(gs[0, :], projection="3d")
        ax_3d.set_xlim(-1, 1)
        ax_3d.set_ylim(-1, 1)
        ax_3d.set_zlim(-1, 1)
        ax_3d.set_xlabel("X")
        ax_3d.set_ylabel("Y")
        ax_3d.set_zlabel("Z")

        # Create cylinder mesh
        x_cyl, y_cyl, z_cyl = self.create_cylinder(self.radius, self.height)
        (path,) = ax_3d.plot([], [], [], "b-")
        (drone_pos,) = ax_3d.plot([], [], [], "ro")

        # Create 2D angle plots
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
        thrust_vectors = []

        # Create animation
        ani = FuncAnimation(
            fig,
            self.animate,
            frames=self.num_steps,
            interval=20,
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
                thrust_vectors,
            ),
        )

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    simulation = DroneSimulation()
    simulation.plot_simulation()
