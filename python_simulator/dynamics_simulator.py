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
        base_thrust_vector = np.array(
            [0, 0, 1]
        )  # Unit vector along +z axis in world frame

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
        max_servo_angle = np.pi / 6
        servo_pitch_angle = np.clip(
            desired_torque[1] / 10.0, -max_servo_angle, max_servo_angle
        )
        servo_roll_angle = np.clip(
            desired_torque[0] / 10.0, -max_servo_angle, max_servo_angle
        )
        servo_angles = np.array([servo_pitch_angle, servo_roll_angle])

        return servo_angles

    def add_random_disturbance(self, state):
        disturbance_intensity = 0.05
        disturbed_state = np.copy(state)
        disturbed_state[0] += disturbance_intensity * (random.random() - 0.5)
        disturbed_state[1] += disturbance_intensity * (random.random() - 0.5)
        disturbed_state[2] += disturbance_intensity * (random.random() - 0.5)
        disturbed_state[6] += disturbance_intensity * (random.random() - 0.5)
        disturbed_state[7] += disturbance_intensity * (random.random() - 0.5)
        disturbed_state[8] += disturbance_intensity * (random.random() - 0.5)

        return disturbed_state

    def compute_control_action(self, pos_err, vel, gravity, gyro):
        Kp = np.array([1.0, 1.0, 1.0])
        Kd = np.array([0.5, 0.5, 0.5])
        desired_force = Kp * pos_err - Kd * vel + gravity
        desired_force[2] += 9.81
        desired_torque = np.array([0.0, 0.0, 0.0])

        return desired_force, desired_torque

    def update_drone_state(self, state, thrust_vector, desired_torque, dt):
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
        m = 1.0
        I_x, I_y, I_z = 0.1, 0.1, 0.1

        ax = thrust_vector[0] / m
        ay = thrust_vector[1] / m
        az = (thrust_vector[2] - 9.81) / m

        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        vz_new = vz + az * dt

        x_new = x + vx_new * dt
        y_new = y + vy_new * dt
        z_new = z + vz_new * dt

        p_new = p + (desired_torque[0] - (I_y - I_z) * q * r) / I_x * dt
        q_new = q + (desired_torque[1] - (I_z - I_x) * p * r) / I_y * dt
        r_new = r + (desired_torque[2] - (I_x - I_y) * p * q) / I_z * dt

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

    def simulate(self):
        state = np.zeros(12)
        state[2] = 0.5

        for _ in range(self.num_steps):
            pos_err = -np.array(state[:3])
            vel_err = -np.array(state[3:6])
            gravity = np.array([0, 0, 9.81])

            desired_force, desired_torque = self.compute_control_action(
                pos_err, vel_err, gravity, [0, 0, 0]
            )
            servo_angles = self.convert_to_servo_angles(desired_force, desired_torque)
            thrust_vector = self.calculate_thrust_vector(servo_angles)

            state[6] += random.uniform(-0.1, 0.1)
            state[7] += random.uniform(-0.1, 0.1)
            state[8] += random.uniform(-0.1, 0.1)

            new_state = self.update_drone_state(
                state, thrust_vector, desired_torque, self.dt
            )
            state = new_state

            self.trajectory.append(state)
            self.phi_data.append(state[6])
            self.theta_data.append(state[7])

    def get_trajectory_data(self):
        x_data = [state[0] for state in self.trajectory]
        y_data = [state[1] for state in self.trajectory]
        z_data = [state[2] for state in self.trajectory]
        phi_data = [state[6] for state in self.trajectory]
        theta_data = [state[7] for state in self.trajectory]
        psi_data = [state[8] for state in self.trajectory]
        time_data = np.linspace(0, self.dt * self.num_steps, self.num_steps + 1)

        return x_data, y_data, z_data, phi_data, theta_data, psi_data, time_data

    def update_cylinder(
        self, ax, x_cyl, y_cyl, z_cyl, x_pos, y_pos, z_pos, phi, theta, psi
    ):
        """
        Update the position and orientation of the drone cylinder in the 3D plot.

        Args:
        - ax: The matplotlib axis object.
        - x_cyl, y_cyl, z_cyl: The coordinates of the cylinder mesh.
        - x_pos, y_pos, z_pos: The position of the drone.
        - phi: The roll angle.
        - theta: The pitch angle.
        - psi: The yaw angle.
        """
        # Rotate the cylinder mesh to match the drone's orientation
        x_rot, y_rot, z_rot = self.rotate_cylinder(x_cyl, y_cyl, z_cyl, phi, theta, psi)

        # Update the cylinder position
        x_rot += x_pos
        y_rot += y_pos
        z_rot += z_pos

        # Update the cylinder plot
        ax.plot_surface(x_rot, y_rot, z_rot, color="b", alpha=0.7)

    @staticmethod
    def create_cylinder(radius, height, resolution=10):
        z = np.linspace(-height / 2, height / 2, resolution)
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
        coords = np.vstack((x.flatten(), y.flatten(), z.flatten()))
        rotated_coords = np.dot(R, coords)
        x_rotated = rotated_coords[0, :].reshape(x.shape)
        y_rotated = rotated_coords[1, :].reshape(y.shape)
        z_rotated = rotated_coords[2, :].reshape(z.shape)

        return x_rotated, y_rotated, z_rotated

    @staticmethod
    def format_pi_axis(ax):
        ax.set_yticks(np.linspace(-np.pi / 3, np.pi / 3, 5))
        ax.set_yticklabels(["$-\\pi/3$", "$-\\pi/6$", "0", "$\\pi/6$", "$\\pi/3$"])

    def add_thrust_vector(self, ax, position, thrust_vector):
        """
        Add a thrust vector to the plot, flipped to point in the direction of force.
        """
        origin = position
        end = position - thrust_vector  # Negate the thrust vector to flip it
        thrust_vec = ax.quiver(
            origin[0],
            origin[1],
            origin[2],
            -thrust_vector[0],  # Negate each component
            -thrust_vector[1],  # Negate each component
            -thrust_vector[2],  # Negate each component
            color="orange",
        )
        return thrust_vec

    def animate(self, i):
        """
        Update the animation at each frame.
        """
        if i < len(self.phi_data):
            self.ax_3d.cla()  # Clear the previous plot

            # Recalculate and update the cylinder position and orientation
            x_cyl, y_cyl, z_cyl = self.create_cylinder(self.radius, self.height)
            self.update_cylinder(
                self.ax_3d,
                x_cyl,
                y_cyl,
                z_cyl,
                self.x_data[i],
                self.y_data[i],
                self.z_data[i],
                self.phi_data[i],
                self.theta_data[i],
                self.psi_data[i],
            )

            # Calculate the current rotation matrix based on the current orientation
            phi = self.phi_data[i]
            theta = self.theta_data[i]
            psi = self.psi_data[i]

            R_x = np.array(
                [
                    [1, 0, 0],
                    [0, np.cos(phi), -np.sin(phi)],
                    [0, np.sin(phi), np.cos(phi)],
                ]
            )

            R_y = np.array(
                [
                    [np.cos(theta), 0, np.sin(theta)],
                    [0, 1, 0],
                    [-np.sin(theta), 0, np.cos(theta)],
                ]
            )

            R_z = np.array(
                [
                    [np.cos(psi), -np.sin(psi), 0],
                    [np.sin(psi), np.cos(psi), 0],
                    [0, 0, 1],
                ]
            )

            # Combined rotation matrix
            rotation_matrix = np.dot(R_z, np.dot(R_y, R_x))

            # Calculate the current thrust vector in the global frame
            current_thrust_vector = self.calculate_thrust_vector(
                [self.phi_data[i], self.theta_data[i]]
            )
            thrust_vector_global = np.dot(rotation_matrix, current_thrust_vector)

            # Add the updated thrust vector to the plot, negated to flip it
            thrust_vec = self.add_thrust_vector(
                self.ax_3d,
                [self.x_data[i], self.y_data[i], self.z_data[i]],
                thrust_vector_global,
            )
            self.thrust_vectors.append(thrust_vec)

            # Set plot limits dynamically based on the current position
            self.ax_3d.set_xlim(self.x_data[i] - 1, self.x_data[i] + 1)
            self.ax_3d.set_ylim(self.y_data[i] - 1, self.y_data[i] + 1)
            self.ax_3d.set_zlim(self.z_data[i] - 1, self.z_data[i] + 1)
            self.ax_3d.set_xlabel("X")
            self.ax_3d.set_ylabel("Y")
            self.ax_3d.set_zlabel("Z")

            self.line_phi.set_data(np.arange(i), self.phi_data[:i])
            self.line_theta.set_data(np.arange(i), self.theta_data[:i])
            self.line_psi.set_data(np.arange(i), self.psi_data[:i])

            return self.line_phi, self.line_theta, self.line_psi

        return []

    def plot_simulation(self):
        """
        Plot the simulation results including the 3D drone plot and the angle plots.
        """
        # Retrieve trajectory data
        (
            self.x_data,
            self.y_data,
            self.z_data,
            self.phi_data,
            self.theta_data,
            self.psi_data,
            self.time_data,
        ) = self.get_trajectory_data()

        # Create figure and grid layout
        fig = plt.figure(figsize=(12, 8))
        gs = gridspec.GridSpec(3, 2, height_ratios=[2, 1, 1])

        # Create 3D plot
        self.ax_3d = fig.add_subplot(gs[0, :], projection="3d")
        self.ax_3d.set_xlim(-1, 1)
        self.ax_3d.set_ylim(-1, 1)
        self.ax_3d.set_zlim(-1, 1)
        self.ax_3d.set_xlabel("X")
        self.ax_3d.set_ylabel("Y")
        self.ax_3d.set_zlabel("Z")

        # Create 2D angle plots
        self.ax_phi = fig.add_subplot(gs[1, 0])
        self.ax_theta = fig.add_subplot(gs[1, 1])
        self.ax_psi = fig.add_subplot(gs[2, 0])

        self.ax_phi.set_xlim(0, self.time_data[-1])
        self.ax_theta.set_xlim(0, self.time_data[-1])
        self.ax_psi.set_xlim(0, self.time_data[-1])
        self.ax_phi.set_ylim(-np.pi / 3, np.pi / 3)
        self.ax_theta.set_ylim(-np.pi / 3, np.pi / 3)
        self.ax_psi.set_ylim(-np.pi, np.pi)
        self.format_pi_axis(self.ax_phi)
        self.format_pi_axis(self.ax_theta)
        self.format_pi_axis(self.ax_psi)
        self.ax_phi.set_ylabel("Phi (rad)")
        self.ax_theta.set_ylabel("Theta (rad)")
        self.ax_psi.set_ylabel("Psi (rad)")
        self.ax_psi.set_xlabel("Time (s)")

        (self.line_phi,) = self.ax_phi.plot([], [], "r-")
        (self.line_theta,) = self.ax_theta.plot([], [], "g-")
        (self.line_psi,) = self.ax_psi.plot([], [], "b-")

        self.thrust_vectors = []

        # Create animation
        ani = FuncAnimation(
            fig, self.animate, frames=len(self.time_data), interval=20, blit=False
        )

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    simulation = DroneSimulation()
    simulation.plot_simulation()
