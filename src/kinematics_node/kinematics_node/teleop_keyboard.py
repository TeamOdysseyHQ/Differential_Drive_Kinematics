#!/usr/bin/env python3

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Publisher to cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current velocity state
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Target velocity (what user wants)
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        # Velocity increments per key press
        self.linear_step = 0.05
        self.angular_step = 0.05
        
        # Ramping parameters (smooth acceleration over 1 second)
        self.publish_rate = 50.0  # Hz - higher rate for smoother ramping
        self.ramp_time = 1.0  # seconds to reach target velocity
        self.ramp_rate = 1.0 / self.ramp_time  # velocity change per second
        
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_twist)
        
        self.get_logger().info('Teleop Keyboard Node Started')
        self.print_instructions()
    
    def print_instructions(self):
        print("\n" + "="*50)
        print("Rover Teleop Keyboard Control")
        print("="*50)
        print("Controls:")
        print("  w/s : increase/decrease linear velocity")
        print("  a/d : increase/decrease angular velocity")
        print("  space : stop (zero all velocities)")
        print("  q : quit")
        print("="*50 + "\n")
    
    def get_key(self, timeout=0.1):
        """Get a single keypress from terminal with timeout"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            # Use select to check if input is available
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
                return key
            else:
                return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def publish_twist(self):
        """Publish current velocity as Twist message (called by timer)"""
        # Smooth ramping: gradually move current velocity toward target
        dt = 1.0 / self.publish_rate
        max_change = self.ramp_rate * dt
        
        # Ramp linear velocity
        linear_diff = self.target_linear_vel - self.linear_vel
        if abs(linear_diff) > max_change:
            self.linear_vel += max_change if linear_diff > 0 else -max_change
        else:
            self.linear_vel = self.target_linear_vel
        
        # Ramp angular velocity
        angular_diff = self.target_angular_vel - self.angular_vel
        if abs(angular_diff) > max_change:
            self.angular_vel += max_change if angular_diff > 0 else -max_change
        else:
            self.angular_vel = self.target_angular_vel
        
        # Publish
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.pub.publish(msg)
    
    def display_status(self):
        """Display current velocity status"""
        print(f"\rTarget - Linear: {self.target_linear_vel:+.2f} m/s | Angular: {self.target_angular_vel:+.2f} rad/s", end='', flush=True)
    
    def run(self):
        """Main teleop loop"""
        try:
            while rclpy.ok():
                # Check for key press with timeout
                key = self.get_key(timeout=0.05)
                
                if key:
                    if key == 'w':
                        self.target_linear_vel += self.linear_step
                    elif key == 's':
                        self.target_linear_vel -= self.linear_step
                    elif key == 'a':
                        self.target_angular_vel += self.angular_step
                    elif key == 'd':
                        self.target_angular_vel -= self.angular_step
                    elif key == ' ':
                        self.target_linear_vel = 0.0
                        self.target_angular_vel = 0.0
                        print("\n[STOP] All velocities zeroed")
                    elif key == 'q':
                        print("\n[QUIT] Exiting teleop...")
                        break
                    elif key == '\x03':  # Ctrl+C
                        break
                    
                    # Clamp target velocities to reasonable range
                    self.target_linear_vel = max(-1.0, min(1.0, self.target_linear_vel))
                    self.target_angular_vel = max(-1.0, min(1.0, self.target_angular_vel))
                    
                    self.display_status()
                
                # Spin once to process callbacks (including timer)
                rclpy.spin_once(self, timeout_sec=0.01)
        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        
        finally:
            # Send stop command before exiting
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            msg = Twist()
            self.pub.publish(msg)
            print("\n[STOPPED] Sent zero velocity command")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
