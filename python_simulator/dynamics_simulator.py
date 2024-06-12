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
        self.phi_data = []
        self.theta_data = []
        self.psi_data = []

        self.simulate()

    def calculate_thrust_vector(self, servo_angles):
        """
        Calculate the thrust vector based on the servo angles.

        Args:
        - servo_angles: Array of servo angles for the drone's fins.

        Returns:
        - thrust_vector: The resulting thrust vector.
        """
        if len(servo_angles) != 2:
            raise ValueError(
                "servo_angles should contain exactly two elements: [pitch_servo, roll_servo]"
            )

        # Extract servo angles
        pitch_servo, roll_servo = servo_angles

        # Define the base thrust vector (pointing upward in the world frame)
        base_thrust_magnitude = 9.81  # Example thrust magnitude to counteract gravity
        base_thrust_vector = np.array(
            [0, 0, base_thrust_magnitude]
        )  # Base thrust along +z axis in world frame

        # Calculate rotation matrices for pitch and roll adjustments
        pitch_rotation_matrix = np.array(
            [
                [1, 0, 0],
                [0, np.cos(pitch_servo), -np.sin(pitch_servo)],
                [0, np.sin(pitch_servo), np.cos(pitch_servo)],
            ]
        )

        roll_rotation_matrix = np.array(
            [
                [np.cos(roll_servo), 0, np.sin(roll_servo)],
                [0, 1, 0],
                [-np.sin(roll_servo), 0, np.cos(roll_servo)],
            ]
        )

        # Combine rotations to get the final rotation matrix
        rotation_matrix = np.dot(roll_rotation_matrix, pitch_rotation_matrix)

        # Apply the rotation to the base thrust vector
        thrust_vector = np.dot(rotation_matrix, base_thrust_vector)

        # Debug information
        print(f"Servo angles: {servo_angles}")
        print(f"Rotation matrix:\n{rotation_matrix}")
        print(f"Thrust vector: {thrust_vector}")

        return thrust_vector

    def convert_to_servo_angles(self, desired_force, desired_torque):
        """
        Convert the desired forces and torques into servo fin angles.

        Args:
        - desired_force: Desired force vector.
        - desired_torque: Desired torque vector.

        Returns:
        - servo_angles: Angles for the servo fins to achieve the desired forces and torques.
        """
        max_servo_angle = np.pi / 6  # Maximum servo angle (30 degrees)

        # Normalize the desired torque to get the servo angles
        servo_pitch_angle = np.clip(
            desired_torque[1] / 10.0, -max_servo_angle, max_servo_angle
        )
        servo_roll_angle = np.clip(
            desired_torque[0] / 10.0, -max_servo_angle, max_servo_angle
        )

        # Return the servo angles as an array with exactly two elements
        servo_angles = np.array([servo_pitch_angle, servo_roll_angle])

        return servo_angles

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
        Compute the control action based on the current error and state.

        Args:
        - pos_err: Position error (3D vector).
        - vel: Velocity (3D vector).
        - gravity: Gravity compensation (3D vector).
        - gyro: Gyro effects (3D vector).

        Returns:
        - desired_force: Desired force vector to achieve the control action.
        - desired_torque: Desired torque vector to achieve the control action.
        """
        # PD controller gains
        Kp = np.array([1.0, 1.0, 1.0])
        Kd = np.array([0.5, 0.5, 0.5])

        # Compute the desired force using PD control
        desired_force = Kp * pos_err - Kd * vel + gravity

        # Assume no desired torque for simplicity
        desired_torque = np.array([0.0, 0.0, 0.0])

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

        # Convert control actions to servo angles
        servo_angles = self.convert_to_servo_angles(desired_force, desired_torque)

        # Calculate thrust vector based on servo angles
        thrust_vector = self.calculate_thrust_vector(servo_angles)

        # Unpack the state variables
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state

        # Constants (to be defined based on your drone's specifications)
        m = 1.0  # Mass of the drone
        I_x = 0.1  # Moment of inertia around x-axis
        I_y = 0.1  # Moment of inertia around y-axis
        I_z = 0.1  # Moment of inertia around z-axis

        # Linear motion
        ax = (thrust_vector[0] - gravity[0]) / m
        ay = (thrust_vector[1] - gravity[1]) / m
        az = (thrust_vector[2] - gravity[2]) / m

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

    def simulate(self):
        """
        Simulates the drone's behavior over time.
        """
        state = np.zeros(12)  # Assuming a 12-element state vector
        gravity = np.array([0, 0, -9.81])
        gyro = np.zeros(3)  # Assuming no initial gyroscopic effects

        for _ in range(self.num_steps):
            pos_err = -np.array(state[:3])  # Assuming desired position is origin
            vel_err = -np.array(state[3:6])  # Assuming desired velocity is zero
            new_state = self.update_drone_state(
                state, pos_err, vel_err, gravity, gyro, self.dt
            )
            state = new_state
            self.trajectory.append(state)

            self.phi_data.append(state[6])
            self.theta_data.append(state[7])
            self.psi_data.append(state[8])

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
        z = np.linspace(-height / 2, height / 2, resolution)
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

        # Combined rotation matrix
        R = np.dot(R_z, np.dot(R_y, R_x))

        # Flatten the grid arrays for matrix multiplication
        coords = np.vstack((x.flatten(), y.flatten(), z.flatten()))

        # Apply rotation
        rotated_coords = np.dot(R, coords)

        # Reshape back to grid shape
        x_rotated = rotated_coords[0, :].reshape(x.shape)
        y_rotated = rotated_coords[1, :].reshape(y.shape)
        z_rotated = rotated_coords[2, :].reshape(z.shape)

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
    def add_thrust_vector(ax, origin, thrust_vector, scale=0.05):
        """
        Add a thrust vector to the 3D plot.

        Args:
        - ax: The matplotlib axis object.
        - origin: The origin of the thrust vector.
        - thrust_vector: The thrust vector direction and magnitude.
        - scale: Scaling factor for the thrust vector length for visualization.

        Returns:
        - thrust_vec: The quiver object representing the thrust vector.
        """
        length = np.linalg.norm(thrust_vector) * scale
        if length == 0:
            normalized_vector = np.zeros_like(thrust_vector)
        else:
            normalized_vector = thrust_vector / np.linalg.norm(thrust_vector) * length

        thrust_vec = ax.quiver(
            *origin, *normalized_vector, color="orange", length=1, normalize=True
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
        Update the animation at each frame.
        """
        if i < len(phi_data):
            # Clear existing plots
            ax_3d.cla()

            # Set axis limits
            ax_3d.set_xlim(-1, 1)
            ax_3d.set_ylim(-1, 1)
            ax_3d.set_zlim(-1, 1)
            ax_3d.set_xlabel("X")
            ax_3d.set_ylabel("Y")
            ax_3d.set_zlabel("Z")

            # Update cylinder position and orientation
            x_rotated, y_rotated, z_rotated = self.rotate_cylinder(
                x_cyl, y_cyl, z_cyl, phi_data[i], theta_data[i], psi_data[i]
            )
            ax_3d.plot_surface(x_rotated, y_rotated, z_rotated, color="b", alpha=0.6)

            # Update path and drone position
            path.set_data([x_data[:i], y_data[:i]])
            path.set_3d_properties(z_data[:i])
            drone_pos.set_data([x_data[i], y_data[i]])
            drone_pos.set_3d_properties(z_data[i])

            # Update angle plots
            phi_line.set_data(time_data[:i], phi_data[:i])
            theta_line.set_data(time_data[:i], theta_data[:i])
            psi_line.set_data(time_data[:i], psi_data[:i])

            # Add thrust vector
            servo_angles = [0.1, 0.1]  # Use small servo angles for testing
            thrust_vector = self.calculate_thrust_vector(servo_angles)
            print("z_data[i]:", z_data[i])
            thrust_vector_origin = [
                x_data[i],
                y_data[i],
                z_data[i] + 0.5 * self.height,
            ]  # Adjust origin based on drone's position and height
            self.add_thrust_vector(
                ax_3d, thrust_vector_origin, thrust_vector, scale=0.05
            )

            return path, drone_pos, phi_line, theta_line, psi_line

        return []

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
