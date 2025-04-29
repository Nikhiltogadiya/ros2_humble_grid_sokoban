#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf_transformations

class TurtleBotTurnController(Node):
    def __init__(self):
        super().__init__('turtlebot_turn_controller')
        
        # Create subscribers
        self.command_subscriber = self.create_subscription(
            String,
            '/turn_command',
            self.command_callback,
            10
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/turn_status',
            10
        )
        
        # Initialize variables
        self.current_yaw = 0.0
        self.target_yaw = None
        self.is_turning = False
        self.turn_direction = 1  # 1 for clockwise, -1 for counter-clockwise
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('TurtleBot Turn Controller has been initialized')
    
    def odom_callback(self, msg):
        # Extract orientation from odometry message
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        
        # Normalize yaw to be between -pi and pi
        self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
    
    def command_callback(self, msg):
        if not self.is_turning:
            command = msg.data.strip().upper()
            
            if command in ['RIGHT', 'LEFT', 'UTURN']:
                self.get_logger().info(f'Received command: {command}')
                
                # Set the target yaw based on the command
                if command == 'RIGHT':
                    self.target_yaw = self.normalize_angle(self.current_yaw - math.pi/2)
                    self.turn_direction = -1
                elif command == 'LEFT':
                    self.target_yaw = self.normalize_angle(self.current_yaw + math.pi/2)
                    self.turn_direction = 1
                elif command == 'UTURN':
                    self.target_yaw = self.normalize_angle(self.current_yaw + math.pi)
                    self.turn_direction = 1
                
                self.is_turning = True
                self.get_logger().info(f'Starting turn from {self.current_yaw} to {self.target_yaw}')
            else:
                self.get_logger().warn(f'Invalid command: {command}')
    
    def normalize_angle(self, angle):
        # Normalize angle to be between -pi and pi
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def control_loop(self):
        if self.is_turning and self.target_yaw is not None:
            # Calculate the difference between current and target yaw
            yaw_diff = self.normalize_angle(self.target_yaw - self.current_yaw)
            
            # Check if we've reached the target with a small tolerance
            if abs(yaw_diff) < 0.05:  # ~3 degrees tolerance
                # Stop turning
                self.stop_robot()
                self.is_turning = False
                self.target_yaw = None
                
                # Publish turn status
                status_msg = String()
                status_msg.data = "finished"
                self.status_publisher.publish(status_msg)
                self.get_logger().info('Turn completed, published status: finished')
            else:
                # Continue turning
                cmd_vel = Twist()
                
                # Set angular velocity based on how far we are from the target
                angular_velocity = 0.25 if abs(yaw_diff) > 0.5 else 0.3
                cmd_vel.angular.z = self.turn_direction * angular_velocity if yaw_diff * self.turn_direction > 0 else -self.turn_direction * angular_velocity
                
                self.cmd_vel_publisher.publish(cmd_vel)
    
    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    
    node = TurtleBotTurnController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()