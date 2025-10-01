#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState # referencia

class Ms2R1PStatePublisher_amg(Node): # <--- CHANGE ME
    def __init__(self):
        super().__init__("ms2r1p_state_publisher") # <--- CHANGE ME
        # ---- Parameters (declare + get) ----
        self.declare_parameter('arm_1', 'base_to_arm1')
        self.declare_parameter('arm_2', 'arm_1_arm_2_joint')
        self.declare_parameter('prismatic', 'arm_2_prismatic_joint')

        self.declare_parameter('upper_limit_angular_1',math.pi/2)
        self.declare_parameter('lower_limit_angular_1',-math.pi/2)

        self.declare_parameter('upper_limit_angular_2',2*math.pi/3)
        self.declare_parameter('lower_limit_angular_2',-2*math.pi/3)

        self.declare_parameter('upper_limit_linear',0)
        self.declare_parameter('lower_limit_linear',-6.085/10)

        self.declare_parameter('publish_rate', 50.0)      # Hz

        self.joint_1 = self.get_parameter('arm_1').get_parameter_value().string_value
        self.joint_2 = self.get_parameter('arm_2').get_parameter_value().string_value
        self.joint_3 = self.get_parameter('prismatic').get_parameter_value().string_value

        self.up_l_a_1 = self.get_parameter('upper_limit_angular_1').get_parameter_value().double_value
        self.low_l_a_1 = self.get_parameter('lower_limit_angular_1').get_parameter_value().double_value
        self.up_l_a_2 = self.get_parameter('upper_limit_angular_2').get_parameter_value().double_value
        self.low_l_a_2 = self.get_parameter('lower_limit_angular_2').get_parameter_value().double_value
        self.up_l_l = self.get_parameter('upper_limit_linear').get_parameter_value().double_value
        self.low_l_l = self.get_parameter('lower_limit_linear').get_parameter_value().double_value


        self.rate_hz = float(self.get_parameter('publish_rate').value)

        # ---- Joint angles ----
        self.cmd_th1 = 0.0
        self.cmd_th2 = 0.0 
        self.cmd_linear = 0.0
      
        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, 'scara_conf', self._on_cmd_vel, qos)
        self.pub_js = self.create_publisher(JointState, '/joint_states', qos)

        # ---- Timer ----
        self.last_time = None
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f"Joints: {self.joint_1}, {self.joint_2},{self.joint_3},rate={self.rate_hz} Hz\n limits: {self.low_l_a_1},{self.up_l_a_2} rad, {self.low_l_l},{self.up_l_l} m"
        )
    
    def _on_cmd_vel(self, msg: Twist):
        self.cmd_th1 = msg.linear.x
        self.cmd_th2 = msg.linear.y
        self.cmd_linear = msg.linear.z
    
    def _on_timer(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

    
        th1_des = self.cmd_th1
        th2_des = self.cmd_th2   
        z_des = self.cmd_linear

        if z_des >= self.up_l_l:
            z_des = self.up_l_l
        if z_des <= self.low_l_l:
            z_des = self.low_l_l
        
        if th1_des >= self.up_l_a_1:
            th1_des = self.up_l_a_1
        if th1_des <= self.low_l_a_1:
            th1_des = self.low_l_a_1
        if th2_des >= self.up_l_a_2:
            th2_des = self.up_l_a_2
        if th2_des <= self.low_l_a_2:
            th2_des = self.low_l_a_2


        # Publish joint_states 
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [self.joint_1, self.joint_2,self.joint_3]
        js.position = [th1_des,th2_des,z_des]
        js.velocity = [0.0,0.0,0.0]
        self.pub_js.publish(js)

def main():
    rclpy.init()
    node = Ms2R1PStatePublisher_amg() # <--- CHANGE ME
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()