#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import math

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_axis', 1),      # Typically left stick vertical
                ('angular_axis', 0),     # Typically left stick horizontal
                ('scale_linear', 1.0),   # Max linear velocity (m/s)
                ('scale_angular', 1.0),  # Max angular velocity (rad/s)
                ('deadzone', 0.1),       # Joystick deadzone
                ('enable_button', 0),    # Button index for enable (typically A)
                ('turbo_button', 4),     # Button index for turbo (typically LB)
                ('turbo_multiplier', 2.0), # Turbo speed multiplier
            ]
        )
        
        # Get parameters
        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value
        self.deadzone = self.get_parameter('deadzone').value
        self.enable_button = self.get_parameter('enable_button').value
        self.turbo_button = self.get_parameter('turbo_button').value
        self.turbo_multiplier = self.get_parameter('turbo_multiplier').value
        
        # State variables
        self.enabled = False
        self.last_enabled_state = False
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Timer for safety (stop if no messages received)
        self.timer = self.create_timer(0.5, self.safety_check)
        self.last_joy_time = self.get_clock().now()
        
        self.get_logger().info('Joy Teleop Node Started')
        self.get_logger().info(f'Linear axis: {self.linear_axis}, Angular axis: {self.angular_axis}')
        self.get_logger().info('Press the enable button to start controlling')
    
    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()
        
        # # Check enable button
        # if len(msg.buttons) > self.enable_button:
        #     if msg.buttons[self.enable_button] == 1:
        #         self.enabled = True
        #         if not self.last_enabled_state:
        #             self.get_logger().info('Control ENABLED')
        #     else:
        #         self.enabled = False
        #         if self.last_enabled_state:
        #             self.get_logger().info('Control DISABLED')
        #             self.publish_zero_velocity()  # Stop when disabled
        #     self.last_enabled_state = self.enabled
        
        # # If not enabled, don't process further
        # self.get_logger().info('joy_callback1')
        # if not self.enabled:
        #     return
        
        # Check turbo button
        turbo_active = False
        if len(msg.buttons) > self.turbo_button and msg.buttons[self.turbo_button] == 1:
            turbo_active = True
        
        # Get axis values with deadzone
        linear_val = self.apply_deadzone(msg.axes[self.linear_axis])
        angular_val = self.apply_deadzone(msg.axes[self.angular_axis])
        
        # Apply scaling
        scale_linear = self.scale_linear * self.turbo_multiplier if turbo_active else self.scale_linear
        scale_angular = self.scale_angular * self.turbo_multiplier if turbo_active else self.scale_angular
        
        linear_cmd = linear_val * scale_linear
        angular_cmd = angular_val * scale_angular
        #self.get_logger().info('joy_callback2')    
        # Create and publish twist message
        twist = TwistStamped()        
        twist.twist.linear.x = linear_cmd
        twist.twist.angular.z = angular_cmd
        
        self.cmd_vel_pub.publish(twist)
        
        # Debug logging (throttled to avoid spam)
        self.get_logger().debug(
            f'Linear: {linear_cmd:.2f} m/s, Angular: {angular_cmd:.2f} rad/s'
            f'{" [TURBO]" if turbo_active else ""}',
            throttle_duration_sec=1.0
        )
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the value to compensate for deadzone
        return math.copysign((abs(value) - self.deadzone) / (1.0 - self.deadzone), value)
    
    def publish_zero_velocity(self):
        """Publish zero velocity command"""
        twist = TwistStamped()
        self.cmd_vel_pub.publish(twist)
    
    def safety_check(self):
        """Safety check - stop if no joy messages received for a while"""
        time_since_last_joy = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if time_since_last_joy > 1.0 and self.enabled:  # 1 second timeout
            self.get_logger().warn('No joy messages received for 1 second, stopping')
            self.publish_zero_velocity()
            self.enabled = False

def main(args=None):
    rclpy.init(args=args)
    joy_teleop = JoyTeleop()
    try:
        rclpy.spin(joy_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        joy_teleop.publish_zero_velocity()
        joy_teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()