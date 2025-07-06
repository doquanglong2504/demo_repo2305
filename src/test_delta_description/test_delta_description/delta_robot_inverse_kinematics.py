#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np
import math

class DeltaRobotKinematics:
    def __init__(self):
        # Robot parameters (adjusted to match URDF)
        self.e = 0.1   # End-effector platform side (sP)
        self.f = 0.211 # Base platform side (sB)
        self.re = 0.293 # Lower arm length (l)
        self.rf = 0.18  # Upper arm length (L)

        # Auxiliary variables
        self.E = [0, 0, 0]
        self.F = [0, 0, 0]
        self.G = [0, 0, 0]

    def inverse(self, x, y, z):
        """Compute inverse kinematics for the delta robot."""
        self._initialize_auxiliary_parameters(x, y, z)

        theta = [0, 0, 0]
        for i in range(3):
            delta = self.E[i]**2 + self.F[i]**2 - self.G[i]**2
            if delta < 0:
                raise ValueError(f"Delta {i} less than 0!")
            t_half_angle = (-self.F[i] - np.sqrt(delta)) / (self.G[i] - self.E[i])
            theta[i] = 2 * np.arctan(t_half_angle)
        return theta

    def _initialize_auxiliary_parameters(self, x, y, z):
        wB = (np.sqrt(3)/6) * self.f
        uB = (np.sqrt(3)/3) * self.f
        wP = (np.sqrt(3)/6) * self.e
        uP = (np.sqrt(3)/3) * self.e

        a = -uP + wB
        b = self.e/2 - (np.sqrt(3)/2) * wB
        c = wP - wB/2

        self.E[0] = 2 * self.rf * (y + a)
        self.F[0] = 2 * self.rf * z
        self.G[0] = x**2 + y**2 + z**2 + a**2 + self.rf**2 + 2*y*a - self.re**2

        self.E[1] = -self.rf * (np.sqrt(3)*(x + b) + y + c)
        self.F[1] = 2 * self.rf * z
        self.G[1] = x**2 + y**2 + z**2 + b**2 + c**2 + self.rf**2 + 2*(x*b + y*c) - self.re**2

        self.E[2] = self.rf * (np.sqrt(3)*(x - b) - y - c)
        self.F[2] = 2 * self.rf * z
        self.G[2] = x**2 + y**2 + z**2 + b**2 + c**2 + self.rf**2 + 2*(-x*b + y*c) - self.re**2

    def calculate_passive_joints(self, theta1, theta2, theta3, x, y, z):
        """Calculate the passive joint angles (universal joints) based on active joint angles and end-effector position."""
        wB = (np.sqrt(3)/6) * self.f
        uB = (np.sqrt(3)/3) * self.f
        wP = (np.sqrt(3)/6) * self.e
        uP = (np.sqrt(3)/3) * self.e
        
        # Base connection points
        B1 = np.array([0.11406, -0.04276, 0.03])
        B2 = np.array([-0.094063, -0.077401, 0.03])
        B3 = np.array([-0.02, 0.12016, 0.03])
        
        # End-effector connection points in end_effector frame
        P1 = np.array([0, -uP, 0])
        P2 = np.array([self.e/2, wP, 0])
        P3 = np.array([-self.e/2, wP, 0])
        
        # Rotation matrices for upper arms
        R1 = np.array([
            [np.cos(theta1), 0, np.sin(theta1)],
            [0, 1, 0],
            [-np.sin(theta1), 0, np.cos(theta1)]
        ])
        R2 = np.array([
            [np.cos(theta2 - 2*np.pi/3), -np.sin(theta2 - 2*np.pi/3), 0],
            [np.sin(theta2 - 2*np.pi/3), np.cos(theta2 - 2*np.pi/3), 0],
            [0, 0, 1]
        ])
        R3 = np.array([
            [np.cos(theta3 + 2*np.pi/3), -np.sin(theta3 + 2*np.pi/3), 0],
            [np.sin(theta3 + 2*np.pi/3), np.cos(theta3 + 2*np.pi/3), 0],
            [0, 0, 1]
        ])
        
        # Upper arm end points
        A1 = B1 + R1 @ np.array([0, 0, -self.rf])
        A2 = B2 + R2 @ np.array([0, 0, -self.rf])
        A3 = B3 + R3 @ np.array([0, 0, -self.rf])
        
        # Transform end-effector points to world frame (assuming end_effector is at (x, y, z))
        T_end = np.eye(4)
        T_end[:3, 3] = [x, y, z]
        P1_world = T_end @ np.append(P1, 1)[:3]
        P2_world = T_end @ np.append(P2, 1)[:3]
        P3_world = T_end @ np.append(P3, 1)[:3]
        
        # Vectors from A to P
        V1 = P1_world - A1
        V2 = P2_world - A2
        V3 = P3_world - A3
        
        # Verify length constraint
        if not (np.isclose(np.linalg.norm(V1), self.re, rtol=1e-2) and
                np.isclose(np.linalg.norm(V2), self.re, rtol=1e-2) and
                np.isclose(np.linalg.norm(V3), self.re, rtol=1e-2)):
            raise ValueError("Lower arm length constraint violated")
        
        def calc_universal_angles(V, A, theta):
            V_norm = V / np.linalg.norm(V)
            # First angle (around Y-axis)
            local_y = np.array([0, np.cos(theta), np.sin(theta)])
            proj_V = V_norm - np.dot(V_norm, local_y) * local_y / np.dot(local_y, local_y)
            phi1 = np.arccos(np.clip(np.dot(proj_V, np.array([1, 0, 0])), -1.0, 1.0))
            if np.dot(np.cross(proj_V, np.array([1, 0, 0])), local_y) < 0:
                phi1 = -phi1
            # Second angle (around X-axis after first rotation)
            R_y = np.array([
                [np.cos(phi1), 0, np.sin(phi1)],
                [0, 1, 0],
                [-np.sin(phi1), 0, np.cos(phi1)]
            ])
            V_rot = R_y.T @ V_norm
            phi2 = np.arccos(np.clip(V_rot[1], -1.0, 1.0))
            if V_rot[2] > 0:
                phi2 = -phi2
            return phi1, phi2

        phi1_1, phi1_2 = calc_universal_angles(V1, A1, theta1)
        phi2_1, phi2_2 = calc_universal_angles(V2, A2, theta2)
        phi3_1, phi3_2 = calc_universal_angles(V3, A3, theta3)
        
        return [phi1_1, phi1_2, phi2_1, phi2_2, phi3_1, phi3_2]

class DeltaInverseKinematics(Node):
    def __init__(self):
        super().__init__('delta_inverse_kinematics')
        self.subscription = self.create_subscription(
            Point,
            'target_position',
            self.listener_callback,
            10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.kinematics = DeltaRobotKinematics()
        
        self.joint_names = [
            'upper_arm_joint_1',
            'upper_arm_joint_2',
            'upper_arm_joint_3',
            'lower_arm_joint_1',
            'lower_arm_joint_2',
            'lower_arm_joint_3',
            'lower_to_universal_1_1',
            'lower_to_universal_1_2',
            'lower_to_universal_2_1',
            'lower_to_universal_2_2',
            'lower_to_universal_3_1',
            'lower_to_universal_3_2'
        ]
        
        self.get_logger().info("Delta Inverse Kinematics Node Started")

    def listener_callback(self, msg):
        x, y, z = msg.x, msg.y, msg.z
        try:
            theta1, theta2, theta3 = self.kinematics.inverse(x, y, z)
            lower_angles = self.kinematics.calculate_passive_joints(theta1, theta2, theta3, x, y, z)
            joint_angles = [theta1, theta2, theta3] + lower_angles
            self.publish_joint_state(joint_angles)
        except Exception as e:
            self.get_logger().error(f"Error in IK calculation: {str(e)}")
            self.publish_joint_state([0.0] * 12)

    def publish_joint_state(self, angles):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = angles
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeltaInverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
