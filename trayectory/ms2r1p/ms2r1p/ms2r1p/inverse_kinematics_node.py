#!/usr/bin/env python3
# oop:object oriented programming
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import PointStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__("inverse_kinematics_node")
        
        # Publisher to send the calculated joint values
        self.joint_state_pub = self.create_publisher(Twist, 'scara_conf', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.goal_pose_sub = self.create_subscription(
            PointStamped,
            'goal_pose_trajectory',
            self.goal_pose_callback,
            qos)

        # Parameters (Geometric) 
        self.declare_parameter("scale", 100.0)
        self.scale = self.get_parameter("scale").get_parameter_value().double_value

        self.declare_parameter("a_12", 160.0/self.scale)
        self.declare_parameter("a_23", 120.0/self.scale)
        self.declare_parameter("s_1", 82.0/self.scale)
        self.declare_parameter("s_3", 18.0/self.scale)
        
    def goal_pose_callback(self, msg: PointStamped):
        """
        Callback function to process the incoming goal pose and calculate
        the inverse kinematics using an algebraic approach.
        """
        # Get desired position from the message
        x_d = msg.point.x
        y_d = msg.point.y
        z_d = msg.point.z

        # Get geometric parameters from the parameter server
        a_12 = self.get_parameter("a_12").get_parameter_value().double_value
        a_23 = self.get_parameter("a_23").get_parameter_value().double_value
        s_1 = self.get_parameter("s_1").get_parameter_value().double_value
        s_3 = self.get_parameter("s_3").get_parameter_value().double_value

        # --- Inverse Kinematics Calculations ---
        
        s_4 = -(0.072-z_d)

        rx = x_d
        ry = y_d
        L_sq = rx**2 + ry**2
        
        # Check for reachability
        if L_sq > (a_12 + a_23)**2:
            self.get_logger().warn("Goal position is out of reach! "
                                   f"L={math.sqrt(L_sq):.2f} > (a12+a23)={a_12+a_23}")
            return 
            
        try:
            cos_theta2_num = L_sq - a_12**2 - a_23**2
            cos_theta2_den = 2 * a_12 * a_23
            cos_theta2 = cos_theta2_num / cos_theta2_den
            
            # Ensure value is within the domain of arccos
            cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
            
            theta_2 = math.acos(cos_theta2)
        except ValueError:
            self.get_logger().error("Math domain error: Could not calculate theta_2. "
                                   "The robot cannot reach this position.")
            return 

        # Solve for theta_1 using the algebraic method
        
        # sin(theta_1+theta_2) = (ry - r1*sin(theta_1))/r2
        # cos(theta_1+theta_2) = (rx - r1*cos(theta_1))/r2
        
        k1 = a_12 + a_23 * math.cos(theta_2)
        k2 = a_23 * math.sin(theta_2)

        
        # [cos(theta1)] = [k1 -k2] [rx]
        # [sin(theta1)]   [k2  k1] [ry]
        A = [
            [k1, -k2],
            [k2, k1]
            ]

        B = [[rx],[ry]]

        X = np.linalg.inv(A) @ B
        cos_theta1_num = X[0][0]
        sin_theta1_num = X[1][0]
        theta_1 = math.atan2(sin_theta1_num, cos_theta1_num)
        
        # --- Publish the results ---
        joint_cmd_msg = Twist()
        joint_cmd_msg.linear.x = theta_1
        joint_cmd_msg.linear.y = theta_2
        joint_cmd_msg.linear.z = s_4
        
        self.get_logger().info(f"Calculated joint angles (deg) and prismatic value: "
                               f"theta_1={theta_1:.2f}, "
                               f"theta_2={theta_2:.2f}, "
                               f"s_4={s_4:.2f}")

        self.joint_state_pub.publish(joint_cmd_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
