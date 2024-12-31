from arm_dynamics_base import ArmDynamicsBase
import numpy as np
from geometry import rot, xaxis, yaxis

class ArmDynamicsStudent(ArmDynamicsBase):

    def dynamics_step(self, state, action, dt):
        """
        Args:
            state: A NumPy array representing the current state of the robot.
                   The format is [q_0, ..., q_(n-1), qdot_0, ..., qdot_(n-1)], where:
                     - q_i is the joint angle of the i-th joint.
                     - qdot_i is the joint velocity of the i-th joint.
                     - n is the number of links.
            action: A NumPy array representing the joint torques applied to the robot.
                    The format is [mu_0, ..., mu_(n-1)], where mu_i is the torque 
                    applied to the i-th joint.
            dt: The time step (in seconds) for the simulation.

        Returns:
            A NumPy array representing the updated state of the robot after one time step.
            The format is the same as the input 'state'.
        """

        # State and parameters
        num_links = self.num_links
        q = state[:num_links, 0]
        qd = state[num_links:, 0]
        mu = action[:, 0]
        b = self.joint_viscous_friction  # Viscous friction coefficient
        m = self.link_masses  # Link masses
        L = self.link_lengths  # Link lengths
        I = self.link_inertias  # Link inertias
        g = 9.8 if self.gravity else 0.0  # Gravitational acceleration

        # Joint angles using Euler integration
        q_new = q + qd * dt

        # Dynamics for 1, 2, and 3 links
        if num_links == 1:
            # --- 1-Link ---
            qdd = (mu[0] - b * qd[0] - 0.5 * m[0] * g * L[0] * np.cos(q[0])) / (I[0] + 0.25 * m[0] * L[0] ** 2)
            qd_new = qd + qdd * dt

        elif num_links == 2:
            # --- 2-Link ---
            M11 = I[0] + I[1] + m[1] * L[0] ** 2 + 0.25 * m[0] * L[0] ** 2 + 0.25 * m[1] * L[1] ** 2 + m[1] * L[0] * L[1] * np.cos(q[1])
            M12 = I[1] + 0.25 * m[1] * L[1] ** 2 + 0.5 * m[1] * L[0] * L[1] * np.cos(q[1])
            M21 = M12
            M22 = I[1] + 0.25 * m[1] * L[1] ** 2
            h1 = -m[1] * L[0] * L[1] * np.sin(q[1]) * qd[1] ** 2 - 2 * m[1] * L[0] * L[1] * np.sin(q[1]) * qd[0] * qd[1]
            h2 = m[1] * L[0] * L[1] * np.sin(q[1]) * qd[0] ** 2
            g1 = 0.5 * m[0] * g * L[0] * np.cos(q[0]) + m[1] * g * L[0] * np.cos(q[0]) + 0.5 * m[1] * g * L[1] * np.cos(q[0] + q[1])
            g2 = 0.5 * m[1] * g * L[1] * np.cos(q[0] + q[1])

            M = np.array([[M11, M12],
                          [M21, M22]])
            h = np.array([h1, h2])
            grav = np.array([g1, g2])
            qdd = np.linalg.solve(M, mu - h - grav - b * qd)
            qd_new = qd + qdd * dt

        elif num_links == 3:
            # --- 3-Link ---
            # Lagrangian derivation:
            M11 = I[0] + I[1] + I[2] + m[1] * L[0]**2 + m[2] * (L[0]**2 + L[1]**2 + 2 * L[0] * L[1] * np.cos(q[1])) + 0.25 * m[0] * L[0]**2 + 0.25 * m[1] * L[1]**2 + 0.25 * m[2] * L[2]**2 + m[2] * (L[0] * L[2] * np.cos(q[1] + q[2]) + L[1] * L[2] * np.cos(q[2]))
            M12 = I[1] + I[2] + 0.25 * m[1] * L[1]**2 + m[2] * (L[1]**2 + L[0] * L[1] * np.cos(q[1]) + 0.5 * L[1] * L[2] * np.cos(q[2])) + 0.25 * m[2] * L[2]**2 + 0.5 * m[2] * L[0] * L[2] * np.cos(q[1] + q[2])
            M13 = I[2] + 0.25 * m[2] * L[2]**2 + 0.5 * m[2] * L[0] * L[2] * np.cos(q[1] + q[2]) + 0.5 * m[2] * L[1] * L[2] * np.cos(q[2])
            M21 = M12
            M22 = I[1] + I[2] + 0.25 * m[1] * L[1]**2 + m[2] * (L[1]**2 + L[1] * L[2] * np.cos(q[2])) + 0.25 * m[2] * L[2]**2
            M23 = I[2] + 0.25 * m[2] * L[2]**2 + 0.5 * m[2] * L[1] * L[2] * np.cos(q[2])
            M31 = M13
            M32 = M23
            M33 = I[2] + 0.25 * m[2] * L[2]**2

            h1 = -m[2] * L[0] * L[1] * np.sin(q[1]) * (2 * qd[0] * qd[1] + qd[1]**2) - m[2] * L[0] * L[2] * np.sin(q[1] + q[2]) * (qd[0] + qd[1] + qd[2])**2 + m[2] * L[0] * L[2] * np.sin(q[1] + q[2]) * qd[0]**2 - m[2] * L[1] * L[2] * np.sin(q[2]) * (qd[1] + qd[2])**2 + m[2] * L[1] * L[2] * np.sin(q[2]) * qd[1]**2
            h2 = m[2] * L[0] * L[1] * np.sin(q[1]) * qd[0]**2 - m[2] * L[1] * L[2] * np.sin(q[2]) * (2 * qd[1] * qd[2] + qd[2]**2)
            h3 = m[2] * L[0] * L[2] * np.sin(q[1] + q[2]) * qd[0]**2 + m[2] * L[1] * L[2] * np.sin(q[2]) * qd[1]**2

            g1 = 0.5 * g * (m[0] * L[0] * np.cos(q[0]) + m[1] * (2 * L[0] * np.cos(q[0]) + L[1] * np.cos(q[0] + q[1])) + m[2] * (2 * L[0] * np.cos(q[0]) + 2 * L[1] * np.cos(q[0] + q[1]) + L[2] * np.cos(q[0] + q[1] + q[2])))
            g2 = 0.5 * g * (m[1] * L[1] * np.cos(q[0] + q[1]) + m[2] * (2 * L[1] * np.cos(q[0] + q[1]) + L[2] * np.cos(q[0] + q[1] + q[2])))
            g3 = 0.5 * g * m[2] * L[2] * np.cos(q[0] + q[1] + q[2])

            M = np.array([[M11, M12, M13],
                          [M21, M22, M23],
                          [M31, M32, M33]])
            h = np.array([h1, h2, h3])
            grav = np.array([g1, g2, g3])

            # print("--- Time Step ---")
            # print("q:", q)
            # print("qd:", qd)
            # print("M:", M)
            # print("h:", h)
            # print("grav:", grav)
            # print("mu:", mu)
            # print("b:", b)
            # print("qd_term:", b * qd)
            # print("mu - h - grav - b*qd", mu - h - grav - b * qd)

            # joint accelerations
            qdd = np.linalg.solve(M, mu - h - grav - b * qd)
            # print("qdd:", qdd)

            # Euler integration for velocity
            qd_new = qd + qdd * dt

        else:
            raise ValueError("Invalid number of links. Only 1, 2, and 3 links are supported.")

        # --- Velocity Clamping (to prevent numerical instability) ---
        qd_max = 5.0  # rad/s
        qd_new = np.clip(qd_new, -qd_max, qd_max)

        # New state vector
        new_state = np.zeros((2 * num_links, 1))
        new_state[:num_links, 0] = q_new
        new_state[num_links:, 0] = qd_new

        return new_state