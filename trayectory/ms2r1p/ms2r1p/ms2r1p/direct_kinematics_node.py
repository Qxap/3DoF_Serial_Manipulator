
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import PointStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DirectKinematicsNode(Node): 
    def __init__(self):
        super().__init__("direct_kinematics_node")
        
        # Publisher to visualize the end-effector position
        self.end_effector_pub = self.create_publisher(PointStamped, 'end_effector_pose', 10)

        # Parameters (Geometric)
        self.declare_parameter("scale", 100.0)
        self.scale = self.get_parameter("scale").get_parameter_value().double_value

        self.declare_parameter("a_12", 160.0/self.scale)
        self.declare_parameter("a_23", 120.0/self.scale)
        self.declare_parameter("alpha_23", math.pi)
        self.declare_parameter("s_1", 82.0/self.scale)
        self.declare_parameter("s_3", 18.0/self.scale)  

        # Joint values, initialized to 0.0
        self.theta_1 = 0.0
        self.theta_2 = 0.0
        self.s_4 = 0.0
        
        # Define QoS for the subscription
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber for joint commands
        self.subscription = self.create_subscription(Twist, '/scara_conf', self._on_cmd_vel, qos)

        # Timer to publish the pose periodically
        self.timer = self.create_timer(0.1, self.timer_callback)

    def _on_cmd_vel(self, msg: Twist):
        # Update joint values from the incoming message
        self.theta_1 = msg.linear.x
        self.theta_2 = msg.linear.y
        self.s_4 = msg.linear.z

    def timer_callback(self):
        # Get parameter values
        a_12 = self.get_parameter("a_12").get_parameter_value().double_value
        a_23 = self.get_parameter("a_23").get_parameter_value().double_value
        alpha_23 = self.get_parameter("alpha_23").get_parameter_value().double_value
        s_1 = self.get_parameter("s_1").get_parameter_value().double_value
        s_3 = self.get_parameter("s_3").get_parameter_value().double_value
        
        # Denavit-Hartenberg table [a, alpha, d, theta]
        DH = np.array([
            [0, 0, s_1, self.theta_1],
            [a_12, 0, 0, self.theta_2],
            [a_23,alpha_23, s_3, 0],
            [0, 0, self.s_4, 0]
        ])
        
        # Calculate the final transformation matrix
        T_final = self.transform_matrix(DH)
        
        # Extract end-effector position (x, y, z)
        end_effector_x = T_final[0, 3]
        end_effector_y = T_final[1, 3]
        end_effector_z = T_final[2, 3]
        
        self.get_logger().info(f"End-effector position (x,y,z): ({end_effector_x:.2f}, {end_effector_y:.2f}, {end_effector_z:.2f})")
        
        # Publish the position
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "base_link"
        point_msg.point.x = end_effector_x
        point_msg.point.y = end_effector_y
        point_msg.point.z = end_effector_z
        self.end_effector_pub.publish(point_msg)

    def transform_matrix(self, DH_matrix):
        T_ij = []
        for i in range(DH_matrix.shape[0]):
            a, alpha, d, theta = DH_matrix[i, :]
            
            T = np.array([
                [math.cos(theta), -math.sin(theta), 0, a],
                [math.sin(theta)*math.cos(alpha), math.cos(theta)*math.cos(alpha), -math.sin(alpha), -d*math.sin(alpha)],
                [math.sin(theta)*math.sin(alpha), math.cos(theta)*math.sin(alpha), math.cos(alpha), d*math.cos(alpha)],
                [0, 0, 0, 1]
            ])
            T_ij.append(T)
            
        T_final = np.identity(4)
        for T in T_ij:
            T_final = T_final @ T # Python matrix multiplication operator
            
        return T_final

def main(args=None):
    rclpy.init(args=args)
    node = DirectKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()