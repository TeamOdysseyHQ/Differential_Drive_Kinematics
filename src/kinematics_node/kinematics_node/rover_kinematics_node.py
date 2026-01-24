#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class RoverKinematicsNode(Node):
    def __init__(self):
        super().__init__('rover_kinematics_node')
        
        # Declare parameters
        self.declare_parameters(
            '',
            [
                ('wheel_base', 0.15),          # meters
                ('wheel_diameter', 0.10),      # meters
                ('max_linear_speed', 0.8),     # m/s (physical max)
                ('max_angular_speed', 2.0),    # rad/s (physical max)
                ('linear_usage', 0.5),         # 0..1 fraction
                ('angular_usage', 0.5),        # 0..1 fraction
                ('max_rpm', 200.0),            # clamp RPM
                ('cmd_vel_timeout', 1.0),      # seconds
                ('input_cmd_topic', '/cmd_vel'),
                ('output_topic', '/teensy_motor_rpms'),
                ('update_rate', 50.0),         # Hz
            ],
        )
        
        # Load parameters
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.linear_usage = float(self.get_parameter('linear_usage').value)
        self.angular_usage = float(self.get_parameter('angular_usage').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.update_rate = float(self.get_parameter('update_rate').value)
        
        self.wheel_circumference = math.pi * self.wheel_diameter
        
        self.get_logger().info(
            f'RoverKinematicsNode: wheel_base={self.wheel_base:.3f} m, '
            f'wheel_diameter={self.wheel_diameter:.3f} m, '
            f'max_lin={self.max_linear_speed:.3f} m/s, '
            f'max_ang={self.max_angular_speed:.3f} rad/s, '
            f'linear_usage={self.linear_usage:.2f}, '
            f'angular_usage={self.angular_usage:.2f}'
        )
        
        # Subscribers / publishers
        self.cmd_sub = self.create_subscription(
            Twist,
            self.input_cmd_topic,
            self.cmd_callback,
            10,
        )
        self.rpm_pub = self.create_publisher(
            Float32MultiArray,
            self.output_topic,
            10,
        )
        
        # State
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        
        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        
        self.get_logger().info(f'Listening on {self.input_cmd_topic}, publishing to {self.output_topic}')

    def cmd_callback(self, msg: Twist):
        # Store last command
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        # Compute dt since last cmd
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        
        if dt > self.cmd_vel_timeout:
            # Safety stop: no recent command
            u_lin = 0.0
            u_ang = 0.0
        else:
            # Normalized inputs in [-1, 1]
            u_lin = max(-1.0, min(1.0, self.last_cmd.linear.x))
            u_ang = max(-1.0, min(1.0, self.last_cmd.angular.z))
        
        # Scale to real velocities
        v = u_lin * self.linear_usage * self.max_linear_speed       # m/s
        w = u_ang * self.angular_usage * self.max_angular_speed     # rad/s
        
        # Differential drive track velocities
        v_left = v - w * self.wheel_base / 2.0
        v_right = v + w * self.wheel_base / 2.0
        
        # Convert to RPM
        if self.wheel_circumference <= 0.0:
            rpm_left = 0.0
            rpm_right = 0.0
        else:
            rpm_left = (v_left / self.wheel_circumference) * 60.0
            rpm_right = (v_right / self.wheel_circumference) * 60.0
        
        # Clamp RPM
        rpm_left = max(-self.max_rpm, min(self.max_rpm, rpm_left))
        rpm_right = max(-self.max_rpm, min(self.max_rpm, rpm_right))
        
        # Map to 6 wheels: 0,2,4 = left; 1,3,5 = right
        data = [
            rpm_left,   # wheel 0
            rpm_right,  # wheel 1
            rpm_left,   # wheel 2
            rpm_right,  # wheel 3
            rpm_left,   # wheel 4
            rpm_right,  # wheel 5
        ]
        
        msg_out = Float32MultiArray()
        msg_out.data = data
        self.rpm_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = RoverKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
