#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class RPMMonitor(Node):
    def __init__(self):
        super().__init__('rpm_monitor')
        
        # Subscribe to the motor RPM output
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/teensy_motor_rpm',
            self.rpm_callback,
            10
        )
        
        self.get_logger().info('RPM Monitor Started - Listening to /teensy_motor_rpm')
        print("\n" + "="*60)
        print("Monitoring Motor RPMs (6-wheel differential drive)")
        print("Wheels: 0,2,4 = LEFT | 1,3,5 = RIGHT")
        print("="*60 + "\n")
    
    def rpm_callback(self, msg: Float32MultiArray):
        """Display RPM values for all 6 wheels"""
        if len(msg.data) >= 6:
            print(f"\rL: [{msg.data[0]:+7.1f}, {msg.data[2]:+7.1f}, {msg.data[4]:+7.1f}] | "
                  f"R: [{msg.data[1]:+7.1f}, {msg.data[3]:+7.1f}, {msg.data[5]:+7.1f}] RPM", 
                  end='', flush=True)
        else:
            self.get_logger().warn(f'Received {len(msg.data)} values, expected 6')


def main(args=None):
    rclpy.init(args=args)
    node = RPMMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[STOPPED] RPM Monitor")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
